#!/usr/bin/env python
import numpy as np
import pickle
import pandas as pd
from tqdm import tqdm
from sklearn.ensemble import RandomForestClassifier
from sklearn.model_selection import train_test_split
from sklearn.model_selection import GridSearchCV
from sklearn.preprocessing import LabelEncoder
from sklearn.metrics import f1_score

filepath = "/home/nakane/Documents/20231228_114431_r_gripper.csv"
df = pd.read_csv(filepath)
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
df_data = np.where(df_data == 10.0, round(largest_val*1.1, 3), df_data)
le = LabelEncoder()
df_target = le.fit_transform(np.ravel(df[['label']].values))
mapping = dict(zip(le.transform(le.classes_), le.classes_))
data_train, data_test, target_train, target_test = train_test_split(
    df_data, df_target, test_size=0.2, random_state=0)

max_score = 0
SearchMethod = 0
RFC_grid = {RandomForestClassifier(): {"n_estimators": [i for i in range(24, 40, 2)],
                                       "criterion": ["entropy"],
                                       "max_depth":[i for i in range(10, 16, 1)],
                                       "max_features":['auto', 'sqrt'],
                                       # "random_state": [i for i in range(0, 10)]
                                      }}

#ランダムフォレストの実行
for model, param in tqdm(RFC_grid.items()):
    print(model, param)
    clf = GridSearchCV(model, param)
    clf.fit(data_train, target_train)
    pred_y = clf.predict(data_test)
    score = f1_score(target_test, pred_y, average="micro")

    if max_score < score:
        max_score = score
        best_param = clf.best_params_
        best_model = model.__class__.__name__

print("ベストスコア:{}".format(max_score))
print("モデル:{}".format(best_model))
print("パラメーター:{}".format(best_param))

#ハイパーパラメータを調整しない場合との比較
model = RandomForestClassifier()
model.fit(data_train, target_train)
score = model.score(data_test, target_test)
print("")
print("デフォルトスコア:", score)
