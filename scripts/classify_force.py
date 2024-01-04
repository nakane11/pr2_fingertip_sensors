#!/usr/bin/env python
import numpy as np
import pickle
import pandas as pd
from sklearn.neural_network import MLPClassifier
from sklearn.model_selection import train_test_split

filepath = "/home/nakane/Documents/20240104_180355_force_deriv.csv"
df = pd.read_csv(filepath)
df_data = df[['l_force_{}'.format(i) for i in range(24)]
             + ['r_force_{}'.format(i) for i in range(24)]].values

df_target = np.ravel(df[['label']].values)

data_train, data_test, target_train, target_test = train_test_split(
    df_data, df_target, test_size=0.2, random_state=0)

clf = MLPClassifier(max_iter=10000,
                       hidden_layer_sizes=(1000,),
                       activation='relu', solver='adam',
                       learning_rate_init=0.001)
clf.fit(data_train, target_train)
train_score = clf.score(data_train, target_train)
test_score = clf.score(data_test, target_test)
print(train_score, test_score)
print(clf.predict(data_test))
print(target_test)

# filename = 'cf_20231230_155508_r_gripper_04_1000_relu_adam.sav"'
# pickle.dump(clf, open(filename, 'wb'))
