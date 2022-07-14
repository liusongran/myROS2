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
idx_pub1_start  = [x for x in range(len(content)) if ('pb1') in content[x].lower()]
idx_pub2_start  = [x for x in range(len(content)) if ('pb2') in content[x].lower()]
idx_pub3_start  = [x for x in range(len(content)) if ('pb3') in content[x].lower()]
idx_pub4_start  = [x for x in range(len(content)) if ('pb4') in content[x].lower()]
idx_pub5_start  = [x for x in range(len(content)) if ('pb5') in content[x].lower()]
idx_pub6_start  = [x for x in range(len(content)) if ('pb6') in content[x].lower()]
idx_sub1_start  = [x for x in range(len(content)) if ('sb1') in content[x].lower()]

lst_pub_sub_info= np.zeros([len(idx_pub1_start), 7+1], dtype = int)

clctPrfData (idx_pub1_start, 1)
clctPrfData (idx_pub2_start, 2)
clctPrfData (idx_pub3_start, 3)
clctPrfData (idx_pub4_start, 4)
clctPrfData (idx_pub5_start, 5)
clctPrfData (idx_pub6_start, 6)
clctPrfData (idx_sub1_start, 7)

fig, ax1 = plt.subplots()
ax1.plot(lst_pub_sub_info[:,0], lst_pub_sub_info[:,2]-lst_pub_sub_info[:,1], label='1')
ax1.plot(lst_pub_sub_info[:,0], lst_pub_sub_info[:,3]-lst_pub_sub_info[:,1], label='2')
ax1.plot(lst_pub_sub_info[:,0], lst_pub_sub_info[:,4]-lst_pub_sub_info[:,1], label='3')
ax1.plot(lst_pub_sub_info[:,0], lst_pub_sub_info[:,5]-lst_pub_sub_info[:,1], label='4')
ax1.plot(lst_pub_sub_info[:,0], lst_pub_sub_info[:,6]-lst_pub_sub_info[:,1], label='5')
plt.legend()

print(np.mean(lst_pub_sub_info[:,2]-lst_pub_sub_info[:,1]))
print(np.mean(lst_pub_sub_info[:,3]-lst_pub_sub_info[:,1])/2)
print(np.mean(lst_pub_sub_info[:,4]-lst_pub_sub_info[:,1])/3)
print(np.mean(lst_pub_sub_info[:,5]-lst_pub_sub_info[:,1])/4)
print(np.mean(lst_pub_sub_info[:,6]-lst_pub_sub_info[:,1])/5)

fig, ax2 = plt.subplots()
ax2.plot(lst_pub_sub_info[:,0], lst_pub_sub_info[:,7]-lst_pub_sub_info[:,6], label='sub-pub6')

plt.legend()
plt.show()

