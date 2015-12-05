#!/usr/bin/env python

import pandas as pd
from sklearn.svm import SVC
from sklearn.decomposition import PCA


database = pd.read_csv( "/home/jmirandadealme/Documents/SistAut/HumanRobotInteraction/others/TrainingData/todos2.txt", quotechar ="'")



labels = database['label'].values
database.drop("label",axis=1,inplace=True)
features = database.values

pca = PCA(n_components =17)
pca.fit(features)
features = pca.transform(features)

classf = SVC()
classf.fit(features,labels)
