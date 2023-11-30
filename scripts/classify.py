#!/usr/bin/env python
import pickle
import pandas as pd

from sklearn.neural_network import MLPClassifier
from sklearn.model_selection import train_test_split

filepath = "/home/nakane/Documents/proximity.csv"
dist = pd.read_csv(filepath, header=None)

dist_data = dist.iloc[:, 1:].values
dist_target = dist.iloc[:, 0].values
data_train, data_test, target_train, target_test = train_test_split(
    dist_data, dist_target, test_size=0.2, random_state=0)

clf = MLPClassifier(max_iter=10000,
                       hidden_layer_sizes=(100,),
                       activation='identity', solver='lbfgs',
                       learning_rate_init=0.1)
clf.fit(data_train, target_train)
train_score = clf.score(data_train, target_train)
test_score = clf.score(data_test, target_test)
print(train_score, test_score)
print(clf.predict(data_test))
print(target_test)

filename = '100_identity_lbfgs_01_7cls_model.sav'
pickle.dump(clf, open(filename, 'wb'))
