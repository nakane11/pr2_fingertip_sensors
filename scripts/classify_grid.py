#!/usr/bin/env python

import pandas as pd
import matplotlib.pyplot as plt

from sklearn.neural_network import MLPClassifier
from sklearn.model_selection import train_test_split

filepath = "/home/nakane/Documents/20231129_proximity.csv"
dist = pd.read_csv(filepath, header=None)

dist_data = dist.iloc[:, 1:].values
dist_target = dist.iloc[:, 0].values
data_train, data_test, target_train, target_test = train_test_split(
    dist_data, dist_target, test_size=0.2, random_state=0)

results = []
for hidden_layer_sizes in [10, 100, 1000]:
    for solver in ['adam', 'lbfgs']:
        for activation in ['identity', 'logistic', 'tanh', 'relu']:
            for learning_rate_init in [0.1, 0.01, 0.001]:
                print("hidden_layer_sizes: {}\nactivation: {}\nsolver: {}\nlearning_rate_init: {}\n".format(
                    hidden_layer_sizes, activation, solver, learning_rate_init))
                clf = MLPClassifier(max_iter=10000,
                       hidden_layer_sizes=(hidden_layer_sizes,), 
                       activation=activation, solver=solver,
                       learning_rate_init=learning_rate_init)
                clf.fit(data_train, target_train)
                train_score = clf.score(data_train, target_train)
                test_score = clf.score(data_test, target_test)
                results.append([hidden_layer_sizes, activation,
                                solver, learning_rate_init, train_score, test_score])

df = pd.DataFrame([dat for dat in sorted(results, key=lambda f: f[4],
            reverse=True)], columns=["hidden_layer_sizes", 
                                     "activation", "solver", "learning_rate_init", "train_score", "test_score"])
df.to_csv("/home/nakane/Documents/results.csv")
