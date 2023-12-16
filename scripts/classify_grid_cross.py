#!/usr/bin/env python

import pandas as pd
import matplotlib.pyplot as plt
from sklearn.neural_network import MLPClassifier
from sklearn.model_selection import train_test_split, GridSearchCV
# import yaml

# with open('/home/nakane/ros/pr2_fingertip_ws/src/pr2_fingertip_sensors/data/pfs_params.yaml', 'r') as yml:
#     pfs_params = yaml.safe_load(yml)

filepath = "/home/nakane/Documents/20231216_162637_r_gripper_l_fingertip.csv"
dist = pd.read_csv(filepath)

dist_data = dist.iloc[:, 2:].values
# dist_data = dist.iloc[:, 2:].values/pfs_params['pfs']['r_gripper']['l_fingertip']['proximity_a'][8:12]
dist_target = dist.iloc[:, 1].values
data_train, data_test, target_train, target_test = train_test_split(
    dist_data, dist_target, test_size=0.2, random_state=0)

param_grid = {
    'hidden_layer_sizes': [10, 100, 1000],
    'activation': ['identity', 'logistic', 'tanh', 'relu'],
    'solver': ['adam', 'lbfgs'],
    'learning_rate_init': [0.1, 0.01, 0.001]
}

grid_search = GridSearchCV(clf, param_grid, cv=4, scoring='accuracy', n_jobs=-1)
grid_search.fit(data_train, target_train)

cv_results = pd.DataFrame(grid_search.cv_results_)
                          
print('clf.best_score_', grid_search.best_score_)  # clf.best_score_ 0.98
print('clf.best_params_', grid_search.best_params_) 

# Save the results to a CSV file
cv_results.to_csv("/home/nakane/Documents/cross_validation_results.csv")
