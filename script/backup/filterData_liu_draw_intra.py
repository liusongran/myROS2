#! /usr/bin/env python

from cProfile import label
from ctypes import sizeof
import re

import matplotlib.pyplot as plt
import pandas as pd
import plotly as py
import plotly.express as px
import plotly.figure_factory as ff
import seaborn as sns
import numpy as np
from functools import reduce

plt.style.use('ggplot')
pd.set_option('display.max_rows', None)
pd.set_option('display.max_columns', None)

# ---------------------
# -------读取数据-------
# ---------------------

with open('./data/trace_code/test_print.log', 'r', encoding='utf-8') as f:
    content = f.readlines()

# translate PROFILE data
def tranPrfData (lst_index, lst_list):
    global content
    for i in lst_index:
        result      = re.findall(r'\d+', content[i])
        start_time  = int(result[-1])
        msg_value   = int(result[-2])
        thread_id   = int(result[-3])
        behavior    = "publisher"
        sublist     = [start_time, msg_value, thread_id, behavior]
        lst_list.append(sublist)
    return lst_list

def clctPrfData (lst_index, colum_index):
    global content
    global lst_pub_sub_info
    for i in lst_index:
        result      = re.findall(r'\d+', content[i])
        start_time  = int(result[-1])
        msg_value   = int(result[-2])
        lst_pub_sub_info[msg_value, 0] = msg_value
        lst_pub_sub_info[msg_value, colum_index] = start_time

# translate TRACE data

# ->Prepare: step1
# --> Data interprets: PROFILE
# | pub: pb1 | intra_sub: sb1 | inter_sub: sb2 |
idx_pub_start   = [x for x in range(len(content)) if ('pb6') in content[x].lower()]
idx_sub1_start  = [x for x in range(len(content)) if ('sb1') in content[x].lower()]
idx_sub2_start  = [x for x in range(len(content)) if ('sb2') in content[x].lower()]

# --> Data interprets: TRACE
# -----> level 0
idx_doNumComp       = [x for x in range(len(content)) if ('|do num compare|') in content[x].lower()]
# -----> level 1: branch-2
idx_br2_doIntra     = [x for x in range(len(content)) if ('[do_intra_process_ros_message_publish]') in content[x].lower()]
# -----> level 1: branch-1
idx_br1_doIntra     = [x for x in range(len(content)) if ('[do_intra_process_ros_message_publish_and_return_shared]') in content[x].lower()]
idx_br1_doInter     = [x for x in range(len(content)) if ('[do_inter_process_publish]') in content[x].lower()]

# ->Prepare: step2
# --> Data interprets: PROFILE |msg_value|pub_start|sub1_start|sub2_start|
lst_pub_sub_info= np.zeros([len(idx_pub_start), 4], dtype = int)
# --> Data interprets: TRACE |msg_value|start_time|end_time|
lst_doNumComp   = np.zeros([len(idx_pub_start), 3], dtype = int)   # level 0
lst_br1_doIntra = np.zeros([len(idx_pub_start), 3], dtype = int)   # level 1
lst_br1_doInter = np.zeros([len(idx_pub_start), 3], dtype = int)   # level 1
lst_br2_doIntra = np.zeros([len(idx_pub_start), 3], dtype = int)   # level 1


clctPrfData (idx_pub_start, 1)
clctPrfData (idx_sub1_start, 2)
clctPrfData (idx_sub2_start, 3)

temp_count = 0
for i in idx_doNumComp:
    result = re.findall(r'\d+', content[i])
    if (re.findall(r"start-", content[i])):
        res = re.findall(r'\d+', content[i-2])
        msg_value   = int(res[-2])
        start_time  = int(result[-1])
        lst_doNumComp[temp_count,0] = msg_value
        lst_doNumComp[temp_count,1] = start_time
    if (re.findall(r"end-", content[i])):
        end_time    = int(result[-1])
        lst_doNumComp[temp_count,2] = end_time
        temp_count = temp_count+1

temp_count = 0
for i in idx_br1_doIntra:
    result = re.findall(r'\d+', content[i])
    if (re.findall(r"start-", content[i])):
        res = re.findall(r'\d+', content[i-6])
        msg_value   = int(res[-2])
        start_time  = int(result[-1])
        lst_br1_doIntra[temp_count,0] = msg_value
        lst_br1_doIntra[temp_count,1] = start_time
    if (re.findall(r"end-", content[i])):
        end_time    = int(result[-1])
        lst_br1_doIntra[temp_count,2] = end_time
        temp_count = temp_count+1

temp_count = 0
for i in idx_br1_doInter:
    result = re.findall(r'\d+', content[i])
    if (re.findall(r"start-", content[i])):
        res = re.findall(r'\d+', content[i-10])
        msg_value   = int(res[-2])
        start_time  = int(result[-1])
        lst_br1_doInter[temp_count,0] = msg_value
        lst_br1_doInter[temp_count,1] = start_time
    if (re.findall(r"end-", content[i])):
        end_time    = int(result[-1])
        lst_br1_doInter[temp_count,2] = end_time
        temp_count = temp_count+1

temp_count = 0
for i in idx_br2_doIntra:
    result = re.findall(r'\d+', content[i])
    if (re.findall(r"start-", content[i])):
        res = re.findall(r'\d+', content[i-6])
        msg_value   = int(res[-2])
        start_time  = int(result[-1])
        lst_br2_doIntra[temp_count,0] = msg_value
        lst_br2_doIntra[temp_count,1] = start_time
    if (re.findall(r"end-", content[i])):
        end_time    = int(result[-1])
        lst_br2_doIntra[temp_count,2] = end_time
        temp_count = temp_count+1


# ->Calculate: step3
ltcy_sub1       = lst_pub_sub_info[:,2] - lst_pub_sub_info[:,1]
ltcy_doNumComp  = lst_doNumComp[:,2] - lst_doNumComp[:,1]
ltcy_br1DoIntra = lst_br1_doIntra[:,2] - lst_br1_doIntra[:,1]
ltcy_br1DoInter = lst_br1_doInter[:,2] - lst_br1_doInter[:,1]
ltcy_br2DoIntra = lst_br2_doIntra[:,2] - lst_br2_doIntra[:,1]


whole_timeline = np.zeros([len(idx_pub_start), 6], dtype = int)
for i in range(len(idx_pub_start)):
    whole_timeline[i,0] = lst_pub_sub_info[i,1]                         # pub_start
    whole_timeline[i,1] = lst_doNumComp[i,1]                            # doNumComp_start
    whole_timeline[i,2] = lst_doNumComp[i,2]                            # doNumComp_end
    if (lst_br1_doIntra[i,1]>0):
        whole_timeline[lst_br1_doIntra[i,0],3] = lst_br1_doIntra[i,1]   # doIntra_start
    if (lst_br2_doIntra[i,1]>0):
        whole_timeline[lst_br2_doIntra[i,0],3] = lst_br2_doIntra[i,1]
    if (lst_br1_doInter[i,2]>0):
        whole_timeline[lst_br1_doInter[i,0],4] = lst_br1_doInter[i,2]   # doIntra_end
    if (lst_br2_doIntra[i,2]>0):
        whole_timeline[lst_br2_doIntra[i,0],4] = lst_br2_doIntra[i,2]
    whole_timeline[i,5] = lst_pub_sub_info[i,2]                         # sub_end

fig, ax0 = plt.subplots()
t0_t1 = whole_timeline[:,1]-whole_timeline[:,0]
ax0.plot(lst_pub_sub_info[:,0], t0_t1, label='publish->doNumCmp')
plt.legend()
print("t0->t1: average", int(np.mean(t0_t1)))
print("t0->t1: normal", np.mean(t0_t1[0:70]), np.mean(t0_t1[113:153]), np.mean(t0_t1[196:232]), np.mean(t0_t1[274:311]) )
print("t0->t1: interference", np.mean(t0_t1[70:123]), np.mean(t0_t1[204:257]), np.mean(t0_t1[323:389]), np.mean(t0_t1[432:490]) )

fig, ax1 = plt.subplots()
t1_t2 = whole_timeline[:,2]-whole_timeline[:,1]
ax1.plot(lst_doNumComp[:,0], t1_t2, label='doNumCmp_start->doNumCmp_end')
plt.legend()
print("doNumComp| t1->t2: average: ", int(np.mean(t1_t2)))
print("doNumComp| t1->t2: normal", np.mean(t1_t2[0:70]), np.mean(t1_t2[113:153]), np.mean(t1_t2[196:232]), np.mean(t1_t2[274:311]) )
print("doNumComp| t1->t2: interference", np.mean(t1_t2[70:123]), np.mean(t1_t2[204:257]), np.mean(t1_t2[323:389]), np.mean(t1_t2[432:490]) )

fig, ax2 = plt.subplots()
t2_t3 = whole_timeline[:,3]-whole_timeline[:,2]
ax2.plot(lst_doNumComp[:,0], t2_t3, label='doNumCmp_end->doIntra_start')
plt.legend()
print("t2->t3: average", int(np.mean(t2_t3)))
print("t2->t3: normal", np.mean(t2_t3[0:70]), np.mean(t2_t3[113:153]), np.mean(t2_t3[196:232]), np.mean(t2_t3[274:311]) )
print("t2->t3: interference", np.mean(t2_t3[70:123]), np.mean(t2_t3[204:257]), np.mean(t2_t3[323:389]), np.mean(t2_t3[432:490]) )


fig, ax3 = plt.subplots()
t3_t4 = whole_timeline[:,4]-whole_timeline[:,3]
ax3.plot(lst_doNumComp[:,0], t3_t4, label='doIntra_start->doIntra_end')
plt.legend()
print("doIntra| t3->t4: average", int(np.mean(t3_t4)))
print("doIntra| t3->t4: normal", np.mean(t3_t4[0:70]), np.mean(t3_t4[113:153]), np.mean(t3_t4[196:232]), np.mean(t3_t4[274:311]) )
print("doIntra| t3->t4: interference", np.mean(t3_t4[70:123]), np.mean(t3_t4[204:257]), np.mean(t3_t4[323:389]), np.mean(t3_t4[432:490]) )

fig, ax4 = plt.subplots()
t4_t5 = whole_timeline[:,5]-whole_timeline[:,4]
ax4.plot(lst_doNumComp[:,0], t4_t5, label='doIntra_end->sub_start')
plt.legend()
print("doSub| t4->t5: average", int(np.mean(t4_t5)))
print("doSub| t4->t5: normal", np.mean(t4_t5[0:70]), np.mean(t4_t5[113:153]), np.mean(t4_t5[196:232]), np.mean(t4_t5[274:311]) )
print("doSub| t4->t5: interference", np.mean(t4_t5[70:123]), np.mean(t4_t5[204:257]), np.mean(t4_t5[323:389]), np.mean(t4_t5[432:490]) )

# time-cost percentage of all phases
fig, ax_perct = plt.subplots()
ax_perct.plot(lst_pub_sub_info[:,0], t0_t1/(whole_timeline[:,5]-whole_timeline[:,0]), label='publish->doNumCmp')
ax_perct.plot(lst_doNumComp[:,0], t1_t2/(whole_timeline[:,5]-whole_timeline[:,0]), label='doNumCmp_start->doNumCmp_end')
ax_perct.plot(lst_doNumComp[:,0], t2_t3/(whole_timeline[:,5]-whole_timeline[:,0]), label='doNumCmp_end->doIntra_start')
ax_perct.plot(lst_doNumComp[:,0], t3_t4/(whole_timeline[:,5]-whole_timeline[:,0]), label='doIntra_start->doIntra_end')
ax_perct.plot(lst_doNumComp[:,0], t4_t5/(whole_timeline[:,5]-whole_timeline[:,0]), label='doIntra_end->sub_start')
plt.legend()
'''
# |ltcy_sub1|doNumComp|ltcy_br1DoIntra|ltcy_br1DoInter|ltcy_br2DoIntra|
whole = np.zeros([len(idx_pub_start), 5], dtype = int)
for i in range(len(idx_pub_start)):
    whole[i,0] = ltcy_sub1[i]
    whole[i,1] = ltcy_doNumComp[i]
    if (ltcy_br1DoIntra[i]>0):
        whole[lst_br1_doIntra[i,0],2] = ltcy_br1DoIntra[i]
    if (ltcy_br1DoInter[i]>0):
        whole[lst_br1_doInter[i,0],3] = ltcy_br1DoInter[i]
    if (ltcy_br2DoIntra[i]>0):
        whole[lst_br2_doIntra[i,0],4] = ltcy_br2DoIntra[i]


whole_perct = np.zeros([len(idx_pub_start), 5], dtype = float)
for i in range(len(idx_pub_start)):
    whole_perct[i,:]=100*whole[i,:]/whole[i,0]

# ->Draw: step4

fig, ax1 = plt.subplots()
ax1.plot(lst_pub_sub_info[:,0], lst_pub_sub_info[:,2] - lst_pub_sub_info[:,1])


fig, ax2 = plt.subplots()
ax2.scatter(lst_doNumComp[:,0], ltcy_doNumComp, color='blue')
ax2.scatter(lst_br1_doIntra[:,0], ltcy_br1DoIntra, color='limegreen')
ax2.scatter(lst_br1_doInter[:,0], ltcy_br1DoInter, color='red')
ax2.scatter(lst_br2_doIntra[:,0], ltcy_br2DoIntra, color='orange')

fig, ax3 = plt.subplots()
ax3.scatter(lst_doNumComp[:,0], whole_perct[:,1], color='blue')
ax3.scatter(lst_doNumComp[:,0], whole_perct[:,2], color='limegreen')
ax3.scatter(lst_doNumComp[:,0], whole_perct[:,3], color='red')
ax3.scatter(lst_doNumComp[:,0], whole_perct[:,4], color='orange')

'''
plt.ioff()      #关闭interactive mode
plt.show()
plt.close("all")