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
with open('./ubuntu_data/case1/b.log', 'r', encoding='utf-8') as f:
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
idx_pub_start   = [x for x in range(len(content)) if ('pb1') in content[x].lower()]
idx_sub1_start  = [x for x in range(len(content)) if ('sb1') in content[x].lower()]


# ->Prepare: step2
# --> Data interprets: PROFILE |msg_value|pub_start|sub1_start|sub2_start|
lst_pub_sub_info= np.zeros([len(idx_pub_start), 2+1], dtype = int)


clctPrfData (idx_pub_start, 1)
clctPrfData (idx_sub1_start, 2)


fig, ax = plt.subplots()
tempData = lst_pub_sub_info[:,2]-lst_pub_sub_info[:,1]
ax.plot(lst_pub_sub_info[:,0], tempData, color='blue')


ax.set_title('(b)_mul+1intra+4b.log',fontsize=12,color='r')
print("Under normal: (in ns)", int(np.mean(tempData[0:53])), int(np.mean(tempData[107:157])), int(np.mean(tempData[208:263])), int(np.mean(tempData[321:])))
print("Under interference: (in ns)", int(np.mean(tempData[54:106])), int(np.mean(tempData[158:207])), int(np.mean(tempData[264:320])))


plt.show()




'''
#-->results
#--->a:
NOTE: codes here!
ax.set_title('(a)_sig+1intra+4b.log',fontsize=12,color='r')
print("Under normal: (in ns)", int(np.mean(tempData[0:56])), int(np.mean(tempData[102:155])), int(np.mean(tempData[214:255])), int(np.mean(tempData[322:])))
print("Under interference: (in ns)", int(np.mean(tempData[57:101])), int(np.mean(tempData[156:213])), int(np.mean(tempData[256:322])))

NOTE: results here!
Under normal: (in ns) 518753 527257 541760 547602
Under interference: (in ns) 889360 857377 904563

----------------------------------------------------
#--->b:
NOTE: codes here!
ax.set_title('(b)_mul+1intra+4b.log',fontsize=12,color='r')
print("Under normal: (in ns)", int(np.mean(tempData[0:53])), int(np.mean(tempData[107:157])), int(np.mean(tempData[208:263])), int(np.mean(tempData[321:])))
print("Under interference: (in ns)", int(np.mean(tempData[54:106])), int(np.mean(tempData[158:207])), int(np.mean(tempData[264:320])))

NOTE: results here!
Under normal: (in ns) 640756 613986 598072 561078
Under interference: (in ns) 980246 1025059 1030334

'''