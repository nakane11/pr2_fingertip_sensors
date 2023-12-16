#!/usr/bin/env python
import pickle
import pandas as pd
# import yaml
from sklearn.neural_network import MLPClassifier
from sklearn.model_selection import train_test_split

# with open('/home/nakane/ros/pr2_fingertip_ws/src/pr2_fingertip_sensors/data/pfs_params.yaml', 'r') as yml:
#     pfs_params = yaml.safe_load(yml)

filepath = "/home/nakane/Documents/20231216_162637_r_gripper_l_fingertip.csv"
dist = pd.read_csv(filepath)

dist_data = dist.iloc[:, 2:].values
# dist_data = dist.iloc[:, 2:].values/pfs_params['pfs']['r_gripper']['l_fingertip']['proximity_a'][8:12]
dist_target = dist.iloc[:, 1].values
data_train, data_test, target_train, target_test = train_test_split(
    dist_data, dist_target, test_size=0.2, random_state=0)

clf = MLPClassifier(max_iter=10000,
                       hidden_layer_sizes=(100,),
                       activation='logistic', solver='adam',
                       learning_rate_init=0.001)
clf.fit(data_train, target_train)
train_score = clf.score(data_train, target_train)
test_score = clf.score(data_test, target_test)
print(train_score, test_score)
print(clf.predict(data_test))
print(target_test)

filename = '100_logistic_adam_0001.sav'
pickle.dump(clf, open(filename, 'wb'))
