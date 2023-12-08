#!/usr/bin/env python
import numpy as np
import pickle
import pandas as pd
import matplotlib.pyplot as plt
from sklearn.neural_network import MLPClassifier
from sklearn.model_selection import train_test_split

def SG(x, y, N, m, d=0):
    dx = x[1] - x[0]
    X = (np.c_[-N:N+1] * dx) ** np.r_[:m+1]
    C = np.linalg.pinv(X) # (X.T X)^-1 X.T
    x_ = x[N:-N]
    y_ = np.convolve(y[::-1], C[d], 'valid')[::-1]
    return x_, y_

filepath = "/home/nakane/Documents/20231207_225405_r_gripper_vertical.csv"
df = pd.read_csv(filepath)
df = df.iloc[:250,:]
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

dx = 1
x = np.r_[0:249+dx:dx]
print(x.shape)
N = 10
m = 4
d = 2
for column_name, item in df_data_r.iteritems():
    plt.plot(*SG(x, item, N, m, d), label=column_name)
plt.hlines(y=0, xmin=0, xmax=250)
df_label_r = df_label.replace({'release':0, 'touch':0.004}).values
plt.plot(x, df_label_r, lw=5)
plt.legend()
plt.show()
