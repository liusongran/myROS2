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
import warnings
from functools import reduce

warnings.filterwarnings('ignore')

plt.style.use('ggplot')
pd.set_option('display.max_rows', None)
pd.set_option('display.max_columns', None)

colors_func = {
    'wait_lock_upper':              'rgb(181, 181, 181)',
    'get_next_executable':          'rgb(46, 139, 87)',
    'execute_any_executable':       'rgb(238, 220, 130)',
    'wait_lock_lower':              'rgb(200, 200, 200)',
    'get_next_ready_executable':    'rgb(78, 238, 148)',
    'wait_for_work':                'rgb(141, 238, 238)',
    'get_next_ready_executable2':   'rgb(84, 255, 159)',
    'execute_subscription':         'rgb(238, 238, 0)',
    'rcl_trigger_guard_condition':  'rgb(238, 180, 34)',
    'execute_timer':                'rgb(0,255,100)'
    }
# -------读取数据-------
with open('./data_log/doTrace/doTrace_mul_inter_0.5s.log', 'r', encoding='utf-8') as f:
    content = f.readlines()

# ->STEP0: a. find thread ID
threadNum_  = 50
threadIdx   = 0
threadID    = np.zeros([threadNum_], dtype = int)
tempIdx     = [x for x in range(3*threadNum_) if ('|rclcpp|') in content[x]]
for i in tempIdx:
    ret_    = re.findall(r'\d+', content[i])
    id_     = int(ret_[0])
    if(np.where(threadID==id_)[0].size==0):
        threadIdx = threadIdx+1
        threadID[threadIdx] = id_

# ->STEP0: b. find thread-ID-specific content-index, for each thread
threadSpecIdx = [x for x in range(2*threadIdx+1) if ('|rclcpp|') in content[x]]
for i in range(threadIdx):
    threadSpecIdx[i] = [x for x in range(len(content)) if (str(threadID[i+1])) in content[x]]

# ->STEP0: c. generate thread-ID-specific data list, for each thread
dataLists = [[] for i in range(threadIdx)]
for i in range(threadIdx):
    for j in threadSpecIdx[i]:
        res_        = re.findall(r'\d+', content[j])
        func_       = ""
        t_start_    = int(res_[-1])     # start
        t_end_      = 0                 # end
        t_delta_    = 0
        thread_     = threadID[i+1]
        if ('[wait_lock_upper]|PT-1=> start') in content[j]:
            behavior_   = "[wait_lock_upper]-start"
            level_      = 1
        elif ('[wait_lock_upper]|PT-1=> end') in content[j]:
            behavior_   = "[wait_lock_upper]-end"
            level_      = 1
        elif ('[get_next_executable]|PT-2=> start') in content[j]:
            behavior_   = "[get_next_executable]-start"
            level_      = 1
        elif ('[get_next_executable]|PT-2=> end') in content[j]:
            behavior_   = "[get_next_executable]-end"
            level_      = 1
        elif ('[get_next_ready_executable]|PT-5=> start') in content[j]:
            behavior_   = "[get_next_ready_executable]-start"
            level_      = 2
        elif ('[get_next_ready_executable]|PT-5=> end') in content[j]:
            behavior_   = "[get_next_ready_executable]-end"
            level_      = 2
        elif ('[wait_for_work]|PT-6=> start') in content[j]:
            behavior_   = "[wait_for_work]-start"
            level_      = 2
        elif ('[wait_for_work]|PT-6=> end') in content[j]:
            behavior_   = "[wait_for_work]-end"
            level_      = 2
        elif ('[get_next_ready_executable2]|PT-7=> start') in content[j]:
            behavior_   = "[get_next_ready_executable2]-start"
            level_      = 2
        elif ('[get_next_ready_executable2]|PT-7=> end') in content[j]:
            behavior_   = "[get_next_ready_executable2]-end"
            level_      = 2
        elif ('[execute_any_executable]|PT-3=> start') in content[j]:
            behavior_   = "[execute_any_executable]-start"
            level_      = 1
        elif ('[execute_any_executable]|PT-3=> end') in content[j]:
            behavior_   = "[execute_any_executable]-end"
            level_      = 1
        elif ('[execute_subscription]|PT-2=> start') in content[j]:
            behavior_   = "[execute_subscription]-start"
            level_      = 2
        elif ('[execute_subscription]|PT-2=> end') in content[j]:
            behavior_   = "[execute_subscription]-end"
            level_      = 2
        elif ('[rcl_trigger_guard_condition]|PT-4=> start') in content[j]:
            behavior_   = "[rcl_trigger_guard_condition]-start"
            level_      = 2
        elif ('[rcl_trigger_guard_condition]|PT-4=> end') in content[j]:
            behavior_   = "[rcl_trigger_guard_condition]-end"
            level_      = 2
        elif ('[execute_timer]|PT-1=> start') in content[j]:
            behavior_   = "[execute_timer]-start"
            level_      = 2
        elif ('[execute_timer]|PT-1=> end') in content[j]:
            behavior_   = "[execute_timer]-end"
            level_      = 2
        elif ('[wait_lock_lower]|PT-4=> start') in content[j]:
            behavior_   = "[wait_lock_lower]-start"
            level_      = 1
        elif ('[wait_lock_lower]|PT-4=> end') in content[j]:
            behavior_   = "[wait_lock_lower]-end"
            level_      = 1
        elif ('pb1_THREAD') in content[j]:
            behavior_   = "pub1"
            level_      = 3
        elif ('sb1_THREAD') in content[j]:
            behavior_   = "sub1"
            level_      = 3
        else:
            print("ERROR!!!!!!!!!!!!!!")
        sublist_    = [level_, t_start_, t_end_, func_, t_delta_, thread_, behavior_]
        dataLists[i].append(sublist_)

# ->STEP1: a. generate pandas-format data
listColumns = ['level_', 't_start_', 't_end_', 'func_', 't_delta_', 'thread_', 'behavior_']

def gantt_thread(list, tidIdx):
    listFrame_level1 = list.loc[list['level_']==1]
    listFrame_level1 = listFrame_level1.reset_index()
    for i in range(len(listFrame_level1)):
        if(i%2==1):
            tString = listFrame_level1.loc[i, 'behavior_']
            listFrame_level1.loc[i, 'level_']   = tidIdx+0.1
            listFrame_level1.loc[i, 'func_']    = tString[tString.index("[")+1:tString.index("]")]
            listFrame_level1.loc[i, 't_end_']   = listFrame_level1.loc[i, 't_start_']
            listFrame_level1.loc[i, 't_start_'] = listFrame_level1.loc[i-1, 't_start_']
            listFrame_level1.loc[i, 't_delta_'] = listFrame_level1.loc[i, 't_end_']-listFrame_level1.loc[i, 't_start_']
    listL1 = listFrame_level1.loc[1::2, ['level_', "t_start_", "t_end_", "func_"]]
    listL1 = listL1.rename(columns={'level_':'Task', 't_start_':'Start', 't_end_':'Finish', 'func_':'func_'})
    listL1['Start'] = pd.to_datetime(listL1['Start'], unit='ns')
    listL1['Finish']= pd.to_datetime(listL1['Finish'], unit='ns')

    listFrame_level2 = list.loc[list['level_']==2]
    listFrame_level2 = listFrame_level2.reset_index()
    for i in range(len(listFrame_level2)):
        if(i%2==1):
            tString = listFrame_level2.loc[i, 'behavior_']
            listFrame_level2.loc[i, 'level_']   = tidIdx+0.2
            listFrame_level2.loc[i, 'func_']    = tString[tString.index("[")+1:tString.index("]")]
            listFrame_level2.loc[i, 't_end_']   = listFrame_level2.loc[i, 't_start_']
            listFrame_level2.loc[i, 't_start_'] = listFrame_level2.loc[i-1, 't_start_']
            listFrame_level2.loc[i, 't_delta_'] = listFrame_level2.loc[i, 't_end_']-listFrame_level2.loc[i, 't_start_']
    listL2 = listFrame_level2.loc[1::2, ['level_', "t_start_", "t_end_", "func_"]]
    listL2 = listL2.rename(columns={'level_':'Task', 't_start_':'Start', 't_end_':'Finish', 'func_':'func_'})
    listL2['Start'] = pd.to_datetime(listL2['Start'], unit='ns')
    listL2['Finish']= pd.to_datetime(listL2['Finish'], unit='ns')
    listThread = listL1.append(listL2)
    return listThread

listTotal = pd.DataFrame([], columns = listColumns)

for i in range(threadIdx):
    listThread = gantt_thread(pd.DataFrame(dataLists[i], columns = listColumns), i)
    listTotal = listTotal.append(listThread)

fig = ff.create_gantt(
    listTotal,                      # 数据
    colors          = colors_func,  # 颜色设置
    index_col       = 'func_',      # 颜色显示字段
    show_colorbar   = True,         # 显示颜色柱
    group_tasks     = True,          # 分组显示
    height          = 2500
)
fig.show()
