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

# ---------------------
# -------读取数据-------
# ---------------------
plt.close("all")
with open('./data/case1/a0.log', 'r', encoding='utf-8') as f:
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
idx_pub_start   = [x for x in range(len(content)) if ('pb1') in content[x].lower()]
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


fig, ax0 = plt.subplots()
ax0.plot(lst_pub_sub_info[:,0], lst_pub_sub_info[:,2]-lst_pub_sub_info[:,1])
print("intra-sub latency: ", np.mean(lst_pub_sub_info[41:69,2]-lst_pub_sub_info[41:69,1]), np.mean(lst_pub_sub_info[163:194,2]-lst_pub_sub_info[163:194,1]), np.mean(lst_pub_sub_info[222:243,2]-lst_pub_sub_info[222:243,1]))


plt.show()
