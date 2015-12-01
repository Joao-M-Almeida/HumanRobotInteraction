import pandas as pd
from sklearn.svm import SVC
from sklearn import preprocessing
import numpy as np
import time
from sklearn.metrics import accuracy_score

database_wave = pd.read_csv("/home/jmirandadealme/Documents/SistAut/HumanRobotInteraction/others/ModelTraining/waving.csv", skipinitialspace=True ,quotechar ="'")
database_wave["t5"] = database_wave["t4"]-database_wave["t5"]
database_wave["t4"] = database_wave["t3"]-database_wave["t4"]
database_wave["t3"] = database_wave["t2"]-database_wave["t3"]
database_wave["t2"] = database_wave["t1"]-database_wave["t2"]
database_wave["t1"] = 0

le = preprocessing.LabelEncoder()
le.fit([0.0,'closing forearm','arm moving down','arm moving up','arm stopped', 'forearm stopped','hand above elbow','hand under elbow', 'no walking', 'opening forearm', 'walking backward','walking forward', 'waving', 'caling', 'handing',])
database_wave["g1"]=le.transform(database_wave["g1"])
database_wave["g2"]=le.transform(database_wave["g2"])
database_wave["g3"]=le.transform(database_wave["g3"])
database_wave["g4"]=le.transform(database_wave["g4"])
database_wave["g5"]=le.transform(database_wave["g5"])


# LABELS
wave_labels = np.append(np.zeros((21,)),np.ones((20,)))

# Train

Fsrc = database_wave.values
Lsrc = wave_labels
svm_classf = SVC()
start_time = time.time()
svm_classf.fit(Fsrc,Lsrc)
print("--- %s seconds ---" % (time.time() - start_time))

# Predict

start_time = time.time()
LtrgSVM=svm_classf.predict(Fsrc)
print("--- %s seconds ---" % (time.time() - start_time))
accuracy_score(Lsrc,LtrgSVM)

# Cross Validation
start_time = time.time()
scores = cross_validation.cross_val_score(svm_classf, Fsrc, Lsrc, cv=10)
print("--- %s seconds ---" % (time.time() - start_time))
print("Accuracy: %0.2f (+/- %0.2f)" % (scores.mean(), scores.std() * 2))
