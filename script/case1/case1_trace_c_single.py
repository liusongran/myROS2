#! /usr/bin/env python

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

# ----------------------------------
# -------读取数据-------use "c.trace"
# ----------------------------------
with open('./ubuntu_data/case1/c.trace', 'r', encoding='utf-8') as f:
    content = f.readlines()

def clctPrfData (lst_index, colum_index):
    global content
    global lst_pub_sub_info
    for i in lst_index:
        result      = re.findall(r'\d+', content[i])
        start_time  = int(result[-1])
        msg_value   = int(result[-2])
        lst_pub_sub_info[msg_value, 0] = msg_value
        lst_pub_sub_info[msg_value, colum_index] = start_time


# NOTE: translate TRACE data
# MARK:->Prepare: step1
# --> Data interprets: PROFILE
# | pub: pb1 | intra_sub: sb1 | intra_sub: sb5 |
idx_pub_start   = [x for x in range(len(content)) if ('pb1') in content[x].lower()]
idx_sub1_start  = [x for x in range(len(content)) if ('sb1') in content[x].lower()]
idx_sub5_start  = [x for x in range(len(content)) if ('sb5') in content[x].lower()]
# MARK:->Prepare: step2
# --> Data interprets: PROFILE |msg_value|pub_start|sub1_start|sub2_start|
lst_pub_sub_info= np.zeros([len(idx_pub_start), 3+1], dtype = int)
clctPrfData (idx_pub_start, 1)
clctPrfData (idx_sub1_start, 2)
clctPrfData (idx_sub5_start, 3)
latency_pub_sub     = lst_pub_sub_info[:,2] - lst_pub_sub_info[:,1]
latency_pub_sub5    = lst_pub_sub_info[:,3] - lst_pub_sub_info[:,1]

# MARK:->Prepare: step3
# --> Data interprets: TRACE |msg_value|pub_start|sub1_start|sub2_start|
# -----> level 0
idx_doNumComp       = [x for x in range(len(content)) if ('|do num compare|') in content[x].lower()]
# -----> level 1: branch-2
idx_br2_doIntra     = [x for x in range(len(content)) if ('[do_intra_process_ros_message_publish]') in content[x].lower()]
# -----> level 1: branch-1
idx_br1_doIntra     = [x for x in range(len(content)) if ('[do_intra_process_ros_message_publish_and_return_shared]') in content[x].lower()]
idx_br1_doInter     = [x for x in range(len(content)) if ('[do_inter_process_publish]') in content[x].lower()]
# MARK:->Prepare: step4
# --> Data interprets: TRACE |msg_value|start_time|end_time|
lst_doNumComp   = np.zeros([len(idx_pub_start), 3], dtype = int)   # level 0
lst_br1_doIntra = np.zeros([len(idx_pub_start), 3], dtype = int)   # level 1
lst_br1_doInter = np.zeros([len(idx_pub_start), 3], dtype = int)   # level 1
lst_br2_doIntra = np.zeros([len(idx_pub_start), 3], dtype = int)   # level 1
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
        idx_j = i
        for k in range(100):
            idx_j = idx_j-1
            if (re.findall(r"=> Publishing", content[idx_j])):
                break
        res = re.findall(r'\d+', content[idx_j])
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
        idx_j = i
        for k in range(100):
            idx_j = idx_j-1
            if (re.findall(r"=> Publishing", content[idx_j])):
                break
        res = re.findall(r'\d+', content[idx_j])
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
        idx_j = i
        for k in range(100):
            idx_j = idx_j-1
            if (re.findall(r"=> Publishing", content[idx_j])):
                break
        res = re.findall(r'\d+', content[idx_j])
        msg_value   = int(res[-2])
        start_time  = int(result[-1])
        lst_br2_doIntra[temp_count,0] = msg_value
        lst_br2_doIntra[temp_count,1] = start_time
    if (re.findall(r"end-", content[i])):
        end_time    = int(result[-1])
        lst_br2_doIntra[temp_count,2] = end_time
        temp_count = temp_count+1


# MARK:->Prepare: step5
# --> Data interprets: whole_timeline |   t0    |       t1      |     t2      |     t3      |    t4     |   t5  |
# --> Data interprets: whole_timeline |pub_start|doNumComp_start|doNumComp_end|doIntra_start|doIntra_end|sub_end|
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
        whole_timeline[lst_br1_doInter[i,0],4] = lst_br1_doInter[i,2]   # doIntra_end or doInter_end
    if (lst_br2_doIntra[i,2]>0):
        whole_timeline[lst_br2_doIntra[i,0],4] = lst_br2_doIntra[i,2]
    whole_timeline[i,5] = lst_pub_sub_info[i,2]                         # sub_end



# NOTE: drawing TRACE data
t0_t1 = whole_timeline[:,1]-whole_timeline[:,0]
t1_t2 = whole_timeline[:,2]-whole_timeline[:,1]
t2_t3 = whole_timeline[:,3]-whole_timeline[:,2]
t3_t4 = whole_timeline[:,4]-whole_timeline[:,3]
t4_t5 = whole_timeline[:,5]-whole_timeline[:,4]


fig, axs = plt.subplots(5, 1)
axs[0].plot(lst_pub_sub_info[:,0], t0_t1, color='blue')
axs[0].set_title('step0: publish->doNumCmp')
axs[1].plot(lst_pub_sub_info[:,0], t1_t2, color='blue')
axs[1].set_title('step1: doNumCmp_start->doNumCmp_end')
axs[2].plot(lst_pub_sub_info[:,0], t2_t3, color='blue')
axs[2].set_title('step2: doNumCmp_end->doIntra_start')
axs[3].plot(lst_pub_sub_info[:,0], t3_t4, color='blue')
axs[3].set_title('step3: doIntra_start->doIntra_end')
axs[4].plot(lst_pub_sub_info[:,0], t4_t5, color='blue')
axs[4].set_title('step4: doIntra_end->sub_start')

fig, axDiff = plt.subplots()
axDiff.plot(lst_pub_sub_info[:,0], latency_pub_sub, label='latency_pub2sub1', color='blue')
axDiff.plot(lst_pub_sub_info[:,0], latency_pub_sub5, label='latency_pub2sub5', color='red')
axDiff.set_title('total: latency_pub2sub')
plt.legend()

# time-cost percentage of all phases
fig, ax_perct = plt.subplots()
ax_perct.plot(lst_pub_sub_info[:,0], t0_t1/latency_pub_sub, label='step0: publish->doNumCmp')
ax_perct.plot(lst_pub_sub_info[:,0], t1_t2/latency_pub_sub, label='step1: doNumCmp_start->doNumCmp_end')
ax_perct.plot(lst_pub_sub_info[:,0], t2_t3/latency_pub_sub, label='step2: doNumCmp_end->doIntra_start')
ax_perct.plot(lst_pub_sub_info[:,0], t3_t4/latency_pub_sub, label='step3: doIntra_start->doIntra_end')
ax_perct.plot(lst_pub_sub_info[:,0], t4_t5/latency_pub_sub, label='step4: doIntra_end->sub_start')
plt.legend()

print("------------------------------------")
print("t0->t1: average", int(np.mean(t0_t1)))
print("t0->t1: normal", int(np.mean(t0_t1[0:126])), int(np.mean(t0_t1[186:228])), int(np.mean(t0_t1[282:359])), int(np.mean(t0_t1[449:])))
print("t0->t1: interference", int(np.mean(t0_t1[127:185])), int(np.mean(t0_t1[229:281])), int(np.mean(t0_t1[360:448])))
print("------------------------------------")
print("doNumComp| t1->t2: average: ", int(np.mean(t1_t2)))
print("doNumComp| t1->t2: normal", int(np.mean(t1_t2[0:126])), int(np.mean(t1_t2[186:228])), int(np.mean(t1_t2[282:359])))
print("doNumComp| t1->t2: interference", int(np.mean(t1_t2[127:185])), int(np.mean(t1_t2[229:281])), int(np.mean(t1_t2[360:448])))
print("------------------------------------")
print("t2->t3: average", int(np.mean(t2_t3)))
print("t2->t3: normal", int(np.mean(t2_t3[0:126])), int(np.mean(t2_t3[186:228])), int(np.mean(t2_t3[282:359])), int(np.mean(t2_t3[449:])))
print("t2->t3: interference", int(np.mean(t2_t3[127:185])), int(np.mean(t2_t3[229:281])), int(np.mean(t2_t3[360:448])))
print("------------------------------------")
print("doIntra| t3->t4: average", int(np.mean(t3_t4)))
print("doIntra| t3->t4: normal", int(np.mean(t3_t4[0:126])), int(np.mean(t3_t4[186:228])), int(np.mean(t3_t4[282:359])), int(np.mean(t3_t4[449:])))
print("doIntra| t3->t4: interference", int(np.mean(t3_t4[127:185])), int(np.mean(t3_t4[229:281])), int(np.mean(t3_t4[360:448])))
print("------------------------------------")
print("doSub| t4->t5: average", int(np.mean(t4_t5)))
print("doSub| t4->t5: normal", int(np.mean(t4_t5[0:126])), int(np.mean(t4_t5[186:228])), int(np.mean(t4_t5[282:359])), int(np.mean(t4_t5[449:])))
print("doSub| t4->t5: interference", int(np.mean(t4_t5[127:185])), int(np.mean(t4_t5[229:281])), int(np.mean(t4_t5[360:448])))
print("------------------------------------")
print("total_sub1| average: ", int(np.mean(latency_pub_sub)))
print("total_sub1| normal: ", int(np.mean(latency_pub_sub[0:126])), int(np.mean(latency_pub_sub[186:228])), int(np.mean(latency_pub_sub[282:359])), int(np.mean(latency_pub_sub[449:])))
print("total_sub1| interference: ", int(np.mean(latency_pub_sub[127:185])), int(np.mean(latency_pub_sub[229:281])), int(np.mean(latency_pub_sub[360:448])))
print("------------------------------------")
print("total_sub5| average: ", int(np.mean(latency_pub_sub5)))
print("total_sub5| normal: ", int(np.mean(latency_pub_sub5[0:126])), int(np.mean(latency_pub_sub5[186:228])), int(np.mean(latency_pub_sub5[282:359])), int(np.mean(latency_pub_sub5[449:])))
print("total_sub5| interference: ", int(np.mean(latency_pub_sub5[127:185])), int(np.mean(latency_pub_sub5[229:281])), int(np.mean(latency_pub_sub5[360:448])))
print("------------------------------------")

plt.legend()
plt.show()




'''
#MARK:->results
#--->a.trace:
NOTE: results here!
------------------------------------
t0->t1: average 50725
t0->t1: normal 53470 49964 46556 49004
t0->t1: interference 50630 51807 51842
------------------------------------
doNumComp| t1->t2: average:  32031
doNumComp| t1->t2: normal 33515 29674 29672
doNumComp| t1->t2: interference 32381 35242 32267
------------------------------------
t2->t3: average 9134
t2->t3: normal 10641 7658 8018 10214
t2->t3: interference 9005 9483 8210
------------------------------------
doIntra| t3->t4: average 157765
doIntra| t3->t4: normal 56978 50515 49996 52853
doIntra| t3->t4: interference 305148 312501 307998
------------------------------------
doSub| t4->t5: average 688915
doSub| t4->t5: normal 575414 489829 506131 518601
doSub| t4->t5: interference 866965 962309 917293
------------------------------------
total_sub1| average:  938572
total_sub1| normal:  730020 627641 640375 661096
total_sub1| interference:  1264130 1371345 1317612
------------------------------------
total_sub5| average:  1029780
total_sub5| normal:  826977 715474 727085 754367
total_sub5| interference:  1345507 1470276 1408320
------------------------------------
'''