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

# --------------------------------------------
# -------读取数据------- use "c.log" or "d.log"
# --------------------------------------------
with open('./ubuntu_data/case1/d.log', 'r', encoding='utf-8') as f:
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
# | pub: pb1 | intra_sub: sb1 | intra_sub: sb5 |
idx_pub_start   = [x for x in range(len(content)) if ('pb1') in content[x].lower()]
idx_sub1_start  = [x for x in range(len(content)) if ('sb1') in content[x].lower()]
idx_sub5_start  = [x for x in range(len(content)) if ('sb5') in content[x].lower()]

# ->Prepare: step2
# --> Data interprets: PROFILE |msg_value|pub_start|sub1_start|sub2_start|
lst_pub_sub_info= np.zeros([len(idx_pub_start), 3+1], dtype = int)
clctPrfData (idx_pub_start, 1)
clctPrfData (idx_sub1_start, 2)
clctPrfData (idx_sub5_start, 3)

fig, ax = plt.subplots()
sb1_latency = lst_pub_sub_info[:,2]-lst_pub_sub_info[:,1]
sb5_latency = lst_pub_sub_info[:,3]-lst_pub_sub_info[:,1]
ax.plot(lst_pub_sub_info[:,0], sb1_latency, color='blue', label='sb1_latency')
ax.plot(lst_pub_sub_info[:,0], sb5_latency, color='red', label='sb5_latency')


ax.set_title('(d)_mul+2intra+4b.log',fontsize=12,color='black')
print("|sub1|Under normal: (in ns)", int(np.mean(sb1_latency[0:68])), int(np.mean(sb1_latency[133:199])), int(np.mean(sb1_latency[265:328])), int(np.mean(sb1_latency[430:])))
print("|sub1|Under interference: (in ns)", int(np.mean(sb1_latency[69:132])), int(np.mean(sb1_latency[200:264])), int(np.mean(sb1_latency[329:429])))
print("|sub5|Under normal: (in ns)", int(np.mean(sb5_latency[0:68])), int(np.mean(sb5_latency[133:199])), int(np.mean(sb5_latency[265:328])), int(np.mean(sb5_latency[430:])))
print("|sub5|Under interference: (in ns)", int(np.mean(sb5_latency[69:132])), int(np.mean(sb5_latency[200:264])), int(np.mean(sb5_latency[329:429])))

plt.legend()
plt.show()




'''
#-->results
#--->c:
NOTE: codes here!
ax.set_title('(c)_sig+2intra+4b.log',fontsize=12,color='black')
print("|sub1|Under normal: (in ns)", int(np.mean(sb1_latency[0:72])), int(np.mean(sb1_latency[193:263])), int(np.mean(sb1_latency[332:390])), int(np.mean(sb1_latency[443:])))
print("|sub1|Under interference: (in ns)", int(np.mean(sb1_latency[73:192])), int(np.mean(sb1_latency[264:331])), int(np.mean(sb1_latency[391:442])))
print("|sub5|Under normal: (in ns)", int(np.mean(sb5_latency[0:72])), int(np.mean(sb5_latency[193:263])), int(np.mean(sb5_latency[332:390])), int(np.mean(sb5_latency[443:])))
print("|sub5|Under interference: (in ns)", int(np.mean(sb5_latency[73:192])), int(np.mean(sb5_latency[264:331])), int(np.mean(sb5_latency[391:442])))


NOTE: results here!
|sub1|Under normal: (in ns) 688085 654746 583692 625380
|sub1|Under interference: (in ns) 1293893 1316483 1282625
|sub5|Under normal: (in ns) 787881 753447 675371 715236
|sub5|Under interference: (in ns) 1391804 1421113 1372011

----------------------------------------------------
#--->b:
NOTE: codes here!
ax.set_title('(d)_mul+2intra+4b.log',fontsize=12,color='black')
print("|sub1|Under normal: (in ns)", int(np.mean(sb1_latency[0:68])), int(np.mean(sb1_latency[133:199])), int(np.mean(sb1_latency[265:328])), int(np.mean(sb1_latency[430:])))
print("|sub1|Under interference: (in ns)", int(np.mean(sb1_latency[69:132])), int(np.mean(sb1_latency[200:264])), int(np.mean(sb1_latency[329:429])))
print("|sub5|Under normal: (in ns)", int(np.mean(sb5_latency[0:68])), int(np.mean(sb5_latency[133:199])), int(np.mean(sb5_latency[265:328])), int(np.mean(sb5_latency[430:])))
print("|sub5|Under interference: (in ns)", int(np.mean(sb5_latency[69:132])), int(np.mean(sb5_latency[200:264])), int(np.mean(sb5_latency[329:429])))


NOTE: results here!
|sub1|Under normal: (in ns) 686393 669479 704803 632755
|sub1|Under interference: (in ns) 1277664 1273200 1307106
|sub5|Under normal: (in ns) 902651 876131 925601 836547
|sub5|Under interference: (in ns) 1437432 1431206 1486897

'''