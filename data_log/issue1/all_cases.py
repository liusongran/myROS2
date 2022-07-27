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
with open('./data_log/issue1/50sub/1pub+50sub_2000hz_intra.multi', 'r', encoding='utf-8') as f:
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
freq_idx = [x for x in range(len(content)) if ('average rate:') in content[x].lower()]



# ->Prepare: step2
freq = np.zeros([len(freq_idx), 1], dtype = int)
count = 0
for i in freq_idx:
    result  = re.findall(r'\d+', content[i])
    freq[count]  = int(result[-2])
    count = count + 1

print("average: ", np.mean(freq))


#---------------------------------------------------------
x_axis              = [1, 2, 3, 4, 5, 6, 7, 8]
my_x_ticks          = ['0sub', '1sub', '3sub', '4sub', '5sub', '10sub', '20sub', '50sub']
r_500hz_intra_hz    = np.array([499.6, 499.2, 499, 499.5, 451.125, 420, 370, 184])/500
r_500hz_intra_cpu   = [19.15, 38.6, 51.23, 63.7, 110.16, 130, 152.48, 154.06]
r_500hz_inter_hz    = np.array([499.6, 499, 499, 499, 499.7, 499.3, 498.5, 308.16])/500
r_500hz_inter_cpu   = [18.94, 27.6, 41.74, 47.13, 70.65, 103, 126.19, 149.86]
r_500hz_single_hz   = np.array([499.6, 499.5, 499.6, 499.6, 499.6, 499.6, 499.9, 380])/500
r_500hz_single_cpu  = [18.94, 21.47, 43.78, 53.5, 63.96, 111.6, 310.4, 504.3]


r_1000hz_inter_hz   = np.array([999.7, 999.7, 993.7, 996.7, 959.9, 940.1 , 940.1 , 940.1])/1000
r_1000hz_inter_cpu  = [34, 50.4, 71.53, 80.75, 95.22, 131.92, 131.92, 131.92]
r_1000hz_intra_hz   = np.array([999.9, 995.7, 995.6, 994.9, 876, 830, 830, 830])/1000
r_1000hz_intra_cpu  = [33.6, 61.89, 88.93, 101.5, 123.54, 150.4, 150.4, 150.4]
r_1000hz_single_hz  = np.array([999.7, 999.3, 998.9, 998.9, 998.9, 999.9, 999.9, 999.9])/1000
r_1000hz_single_cpu = [33.6, 40.46, 82.61, 99.7, 118.02, 181.29, 181.29, 181.29]

r_2000hz_intra_hz   = np.array([1988.7, 1992.6, 1924.5, 1698.4, 1484, 1377, 662.2, 186.542])/2000
r_2000hz_intra_cpu  = [53.9, 100.6, 143.76, 147.1, 149.73, 161.12, 159.14, 154.44]
r_2000hz_inter_hz   = np.array([1996.5, 1987.9, 1962.4, 1965.6, 1935.3, 1503.45, 1000.347, 311.6])/2000
r_2000hz_inter_cpu  = [50, 85.04, 123.85, 135.17, 141.35, 152.31, 155.52, 151.04]
r_2000hz_single_hz  = np.array([1996.5, 1993.8, 1979, 1958, 1945.19, 1620, 1250, 430])/2000
r_2000hz_single_cpu = [50, 71.1, 150.71, 185.3, 220.12, 387.04, 514.3, 520.35]

fig, axs = plt.subplots(3, 1)
axs[0].plot(x_axis, r_500hz_inter_hz, color='blue', label='multi-inter')
#axs[0].plot(x_axis, r_500hz_intra_hz, color='red', label='multi-intra')
axs[0].plot(x_axis, r_500hz_single_hz, color='green', label='single-inter')
#axs[0].set_ylim((0.85,1.01))
axs[0].set_xticks(x_axis)
axs[0].set_xticklabels(my_x_ticks)
axs[0].set_title("Topic HZ under 500hz-pub")
axs[0].legend()

axs[1].plot(x_axis, r_1000hz_inter_hz, color='blue', label='multi-inter')
#axs[1].plot(x_axis, r_1000hz_intra_hz, color='red', label='multi-intra')
axs[1].plot(x_axis, r_1000hz_single_hz, color='green', label='single-inter')
#axs[1].set_ylim((0.85,1.01))
axs[1].set_xticks(x_axis)
axs[1].set_xticklabels(my_x_ticks)
axs[1].set_title("Topic HZ under 1000hz-pub")
axs[1].legend()

axs[2].plot(x_axis, r_2000hz_inter_hz, color='blue', label='multi-inter')
#axs[2].plot(x_axis, r_2000hz_intra_hz, color='red', label='multi-intra')
axs[2].plot(x_axis, r_2000hz_single_hz, color='green', label='single-inter')
#axs[2].set_ylim((0.70,1.01))
axs[2].set_xticks(x_axis)
axs[2].set_xticklabels(my_x_ticks)
axs[2].set_title("Topic HZ under 2000hz-pub")
axs[2].legend()


#---------------------------------------------------------
fig, axscpu = plt.subplots(3, 1)
axscpu[0].plot(x_axis, r_500hz_inter_cpu, color='blue', label='multi-inter')
#axscpu[0].plot(x_axis, r_500hz_intra_cpu, color='red', label='multi-intra')
axscpu[0].plot(x_axis, r_500hz_single_cpu, color='green', label='single-inter')
#axscpu[0].set_ylim((0.85,1.01))
axscpu[0].set_xticks(x_axis)
axscpu[0].set_xticklabels(my_x_ticks)
axscpu[0].set_title("CPU utilization under 500hz-pub")
axscpu[0].legend()

axscpu[1].plot(x_axis, r_1000hz_inter_cpu, color='blue', label='multi-inter')
#axscpu[1].plot(x_axis, r_1000hz_intra_cpu, color='red', label='multi-intra')
axscpu[1].plot(x_axis, r_1000hz_single_cpu, color='green', label='single-inter')
#axscpu[1].set_ylim((0.85,1.01))
axscpu[1].set_xticks(x_axis)
axscpu[1].set_xticklabels(my_x_ticks)
axscpu[1].set_title("CPU utilization under 1000hz-pub")
axscpu[1].legend()

axscpu[2].plot(x_axis, r_2000hz_inter_cpu, color='blue', label='multi-inter')
#axscpu[2].plot(x_axis, r_2000hz_intra_cpu, color='red', label='multi-intra')
axscpu[2].plot(x_axis, r_2000hz_single_cpu, color='green', label='single-inter')
#axscpu[2].set_ylim((0.70,1.01))
axscpu[2].set_xticks(x_axis)
axscpu[2].set_xticklabels(my_x_ticks)
axscpu[2].set_title("CPU utilization under 2000hz-pub")
axscpu[2].legend()




plt.show()