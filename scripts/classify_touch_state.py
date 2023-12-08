#!/usr/bin/env python
import numpy as np
import pickle
import pandas as pd
from sklearn.neural_network import MLPClassifier
from sklearn.model_selection import train_test_split

filepath = "/home/nakane/Documents/20231207_225405_r_gripper_vertical.csv"
df = pd.read_csv(filepath)
df_data = df[['l_prox_0', 'l_prox_1', 'l_prox_2', 'l_prox_3',
              'l_prox_4', 'l_prox_5', 'l_prox_6', 'l_prox_7', 'l_prox_8', 'l_prox_9', 
              'l_prox_10', 'l_prox_11', 'l_prox_12', 'l_prox_13', 'l_prox_14', 
              'l_prox_15', 'l_prox_16', 'l_prox_17', 'l_prox_18', 'l_prox_19', 
              'l_prox_20', 'l_prox_21', 'l_prox_22', 'l_prox_23','r_prox_0', 'r_prox_1', 
              'r_prox_2', 'r_prox_3', 'r_prox_4', 'r_prox_5', 'r_prox_6', 'r_prox_7', 
              'r_prox_8', 'r_prox_9', 'r_prox_10', 'r_prox_11', 'r_prox_12', 
              'r_prox_13', 'r_prox_14', 'r_prox_15', 'r_prox_16', 'r_prox_17', 
              'r_prox_18', 'r_prox_19', 'r_prox_20', 'r_prox_21', 'r_prox_22', 
              'r_prox_23', 'r_gripper_joint']].values

df_target = np.ravel(df[['label']].values)

data_train, data_test, target_train, target_test = train_test_split(
    df_data, df_target, test_size=0.2, random_state=0)

clf = MLPClassifier(max_iter=10000,
                       hidden_layer_sizes=(100,),
                       activation='tanh', solver='lbfgs',
                       learning_rate_init=0.001)
clf.fit(data_train, target_train)
train_score = clf.score(data_train, target_train)
test_score = clf.score(data_test, target_test)
print(train_score, test_score)
print(clf.predict(data_test))
print(target_test)

filename = '100_tanh_lbfgs_0001_r_gripper.sav'
pickle.dump(clf, open(filename, 'wb'))
