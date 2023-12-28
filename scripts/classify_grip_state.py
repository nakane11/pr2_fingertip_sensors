#!/usr/bin/env python
import numpy as np
import pickle
import pandas as pd
from sklearn.ensemble import RandomForestClassifier
from sklearn.model_selection import train_test_split
from sklearn.metrics import accuracy_score
from sklearn.preprocessing import LabelEncoder
from sklearn import tree
from dtreeviz.trees import *

filepath = "/home/nakane/Documents/20231228_114431_r_gripper.csv"
df = pd.read_csv(filepath)
# df = df[:3310]
columns = ['l_prox_0', 'l_prox_1', 'l_prox_2', 'l_prox_3',
              'l_prox_4', 'l_prox_5', 'l_prox_6', 'l_prox_7', 'l_prox_8', 'l_prox_9',
              'l_prox_10', 'l_prox_11', 'l_prox_12', 'l_prox_13', 'l_prox_14',
              'l_prox_15', 'l_prox_16', 'l_prox_17', 'l_prox_18', 'l_prox_19',
              'l_prox_20', 'l_prox_21', 'l_prox_22', 'l_prox_23','r_prox_0', 'r_prox_1',
              'r_prox_2', 'r_prox_3', 'r_prox_4', 'r_prox_5', 'r_prox_6', 'r_prox_7',
              'r_prox_8', 'r_prox_9', 'r_prox_10', 'r_prox_11', 'r_prox_12',
              'r_prox_13', 'r_prox_14', 'r_prox_15', 'r_prox_16', 'r_prox_17',
              'r_prox_18', 'r_prox_19', 'r_prox_20', 'r_prox_21', 'r_prox_22',
              'r_prox_23']
df_data = df[columns].values
largest_val = np.partition(np.unique(df_data).flatten(), -2)[-2]
df_data = np.where(df_data == 10.0, largest_val*1.1, df_data)
le = LabelEncoder()
df_target = le.fit_transform(np.ravel(df[['label']].values))
mapping = dict(zip(le.transform(le.classes_), le.classes_))
data_train, data_test, target_train, target_test = train_test_split(
    df_data, df_target, test_size=0.2, random_state=0)

rf = RandomForestClassifier()
rf.fit(data_train, target_train)
test = rf.predict(data_test)
score = accuracy_score(target_test, test)
print(score)
columns = pd.core.indexes.base.Index(columns)
filename = 'rf_20231228_114431_r_gripper2.sav'
pickle.dump(rf, open(filename, 'wb'))

viz = dtreeviz( 
    rf.estimators_[0], 
    data_train,  
    target_train,
    feature_names=columns,    
    # target_name='variety',
    class_names=mapping
)

