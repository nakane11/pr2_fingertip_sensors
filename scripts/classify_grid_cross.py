#!/usr/bin/env python
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from sklearn.neural_network import MLPClassifier
from sklearn.model_selection import train_test_split, GridSearchCV

filepath = "/home/nakane/Documents/cf_20231230_155508_r_gripper_04.csv"
df = pd.read_csv(filepath)
df_data = df[['l_force_{}'.format(i) for i in range(24)]
             + ['r_force_{}'.format(i) for i in range(24)]].values

df_target = np.ravel(df[['label']].values)
data_train, data_test, target_train, target_test = train_test_split(
    df_data, df_target, test_size=0.2, random_state=0)

param_grid = {
    'hidden_layer_sizes': [100, 1000, 10000],
    # 'activation': ['identity', 'logistic', 'tanh', 'relu'],
    'activation': ['relu'],    
    # 'solver': ['adam', 'lbfgs'],
    'solver': ['adam'],    
    'learning_rate_init': [0.1, 0.01, 0.001]
}

clf = MLPClassifier(max_iter=10000)
grid_search = GridSearchCV(clf, param_grid, cv=4, scoring='accuracy', n_jobs=-1)
grid_search.fit(data_train, target_train)

cv_results = pd.DataFrame(grid_search.cv_results_)
                          
print('clf.best_score_', grid_search.best_score_)
print('clf.best_params_', grid_search.best_params_) 

# Save the results to a CSV file
cv_results.to_csv("/home/nakane/Documents/cross_validation_results.csv")
