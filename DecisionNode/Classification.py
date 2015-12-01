#!/usr/bin/env python

import pandas as pd
from sklearn.svm import SVC
from sklearn import preprocessing
import numpy as np
import time
from sklearn.metrics import accuracy_score

database = pd.read_csv("/home/jmirandadealme/Documents/SistAut/HumanRobotInteraction/others/TrainingData/todos.txt", skipinitialspace=True ,quotechar ="'")

database["t5"] = 0
database["t4"] = 0
database["t3"] = 0
database["t2"] = 0
database["t1"] = 0



le = preprocessing.LabelEncoder()
le.fit([0.0,'closingforearm','armmovingdown','armmovingup','armstopped','forearmstopped', 'handaboveelbow', 'handunderelbow', 'nowalking','openingforearm','walkingbackward','walkingforward','waving', 'calling', 'handing', 'elbowbehindbody', 'elbowinfrontofbody'])
database["g1"]=le.transform(database["g1"])
database["g2"]=le.transform(database["g2"])
database["g3"]=le.transform(database["g3"])
database["g4"]=le.transform(database["g4"])
database["g5"]=le.transform(database["g5"])


# LABELS
labels = database["label"].values

# Train

database.drop("label",axis=1,inplace=True)

features = database.values
labels = wave_labels
svm_classf = SVC()
start_time = time.time()
svm_classf.fit(features,labels)
print("--- %s seconds ---" % (time.time() - start_time))

# Predict

start_time = time.time()
predict_labels=svm_classf.predict(features)
print("--- %s seconds ---" % (time.time() - start_time))
accuracy_score(labels,predict_labels)

# Cross Validation
start_time = time.time()
scores = cross_validation.cross_val_score(svm_classf, features, labels, cv=10)
print("--- %s seconds ---" % (time.time() - start_time))
print("Accuracy: %0.2f (+/- %0.2f)" % (scores.mean(), scores.std() * 2))
