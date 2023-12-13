#!/usr/bin/env python
import numpy as np
import pickle
import pandas as pd
import matplotlib.pyplot as plt

filepath = "/home/nakane/Documents/20231207_225405_r_gripper_vertical.csv"
df = pd.read_csv(filepath)
df = df.iloc[:,:]
df_data = df[[# 'l_prox_0', 'l_prox_1', 'l_prox_2', 'l_prox_3',
              # 'l_prox_4', 'l_prox_5', 'l_prox_6', 'l_prox_7',
              'l_prox_8', 'l_prox_9', 'l_prox_10', 'l_prox_11',
              # 'l_prox_12', 'l_prox_13', 'l_prox_14', 'l_prox_15',
              # 'l_prox_16', 'l_prox_17', 'l_prox_18', 'l_prox_19', 
              # 'l_prox_20', 'l_prox_21', 'l_prox_22', 'l_prox_23',
              # 'r_prox_0', 'r_prox_1', 'r_prox_2', 'r_prox_3',
              # 'r_prox_4', 'r_prox_5', 'r_prox_6', 'r_prox_7',
              'r_prox_8', 'r_prox_9', 'r_prox_10', 'r_prox_11',
              # 'r_prox_12', 'r_prox_13', 'r_prox_14', 'r_prox_15',
              # 'r_prox_16', 'r_prox_17', 'r_prox_18', 'r_prox_19',
              # 'r_prox_20', 'r_prox_21', 'r_prox_22', 'r_prox_23'
]]
df_label = df[['label']]
df_data_r = df_data.replace({10:0.1})

N=df.shape[0]
dt = 1/10
for column_name, item in df_data_r.iteritems():
    y_fft = np.fft.fft(item)
    freq = np.fft.fftfreq(N, d=dt)
    Amp = abs(y_fft/(N/2))
    plt.plot(freq[1:int(N/2)], Amp[1:int(N/2)]) 
plt.show()
