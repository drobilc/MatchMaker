import os
import glob
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from sklearn.neighbors import KNeighborsClassifier
from sklearn.tree import DecisionTreeClassifier
from sklearn.ensemble import RandomForestClassifier
from sklearn.naive_bayes import GaussianNB
from sklearn.svm import SVC, LinearSVC

# Disable warnings for some sklearn classifiers
import warnings
warnings.simplefilter(action='ignore', category=FutureWarning)

classifiers = {
    "knn, k = 1": KNeighborsClassifier(1),
    "knn, k = 1, weights = distance": KNeighborsClassifier(1, weights='distance'),
    "knn, k = 3": KNeighborsClassifier(3),
    "knn, k = 3, weights = distance": KNeighborsClassifier(3, weights='distance'),
    "knn, k = 5": KNeighborsClassifier(5),
    "knn, k = 5, weights = distance": KNeighborsClassifier(5, weights='distance'),
    "knn, k = 7": KNeighborsClassifier(7),
    "knn, k = 7, weights = distance": KNeighborsClassifier(7, weights='distance'),
    "decision tree, gini": DecisionTreeClassifier(criterion='gini'),
    "decision tree, entropy": DecisionTreeClassifier(criterion='entropy'),
    "random forest, gini": RandomForestClassifier(n_estimators=100, criterion='gini'),
    "random forest, entropy": RandomForestClassifier(n_estimators=100, criterion='entropy'),
    "naive bayes": GaussianNB(),
    "support vector machine, kernel=linear, c=0.025": SVC(kernel="linear", C=0.025),
    "support vector machine, kernel=linear, c=0.05": SVC(kernel="linear", C=0.05),
    "support vector machine, kernel=linear, c=0.1": SVC(kernel="linear", C=0.1),
    "support vector machine, kernel=linear, c=0.2": SVC(kernel="linear", C=0.2),
}

def cosine_similarity(a, b):
    return np.dot(a, b) / (np.linalg.norm(a)*np.linalg.norm(b))

def evaluate_classifiers(x_train, y_train, x_test, y_test):
    for classifier_name in classifiers:
        classifier = classifiers[classifier_name]
        classifier.fit(x_train, y_train)
        test_score = classifier.score(x_test, y_test)
        print("{}: {}".format(classifier_name, test_score))

# Read data from csv file
dataset = pd.read_csv('../encodings/faces_all_simulation.txt')

# Split the dataset into features and labels
x = dataset.iloc[:, :-1].values
y = dataset.iloc[:, 128].values

# Split the data into train set and test sets
from sklearn.model_selection import train_test_split
x_train, x_test, y_train, y_test = train_test_split(x, y, test_size=0.2)

# Normalize all features
from sklearn.preprocessing import StandardScaler
scaler = StandardScaler()
scaler.fit(x_train)
x_train = scaler.transform(x_train)
x_test = scaler.transform(x_test)

evaluate_classifiers(x_train, y_train, x_test, y_test)


# Now train the classifiers on images from simulation and test them on images from camera
dataset_train = pd.read_csv('../encodings/faces_all_simulation.txt')
dataset_test = pd.read_csv('../encodings/faces_all_camera.txt')

x_train = dataset_train.iloc[:, :-1].values
y_train = dataset_train.iloc[:, 128].values
x_test = dataset_test.iloc[:, :-1].values
y_test = dataset_test.iloc[:, 128].values

# Normalize all features
from sklearn.preprocessing import StandardScaler
scaler = StandardScaler()
scaler.fit(x_train)
x_train = scaler.transform(x_train)
x_test = scaler.transform(x_test)

print("\n\nTesting on camera images:")
evaluate_classifiers(x_train, y_train, x_test, y_test)


# Now let's train and evaluate classifiers on both datasets
dataset_all = dataset_train.append(dataset_test)

x = dataset_all.iloc[:, :-1].values
y = dataset_all.iloc[:, 128].values

# Split the data into train and test set
from sklearn.model_selection import train_test_split
x_train, x_test, y_train, y_test = train_test_split(x, y, test_size=0.2)

# Normalize all features
from sklearn.preprocessing import StandardScaler
scaler = StandardScaler()
scaler.fit(x_train)
x_train = scaler.transform(x_train)
x_test = scaler.transform(x_test)
print("\n\nTraining and testing on both datasets")
evaluate_classifiers(x_train, y_train, x_test, y_test)


# Now let's train and test only on camera images
import pandas as pd
dataset = pd.read_csv('../encodings/faces_all_camera.txt')

x = dataset.iloc[:, :-1].values
y = dataset.iloc[:, 128].values

# Split the data
from sklearn.model_selection import train_test_split
x_train, x_test, y_train, y_test = train_test_split(x, y, test_size=0.2)

# Normalize all features
from sklearn.preprocessing import StandardScaler
scaler = StandardScaler()
scaler.fit(x_train)
x_train = scaler.transform(x_train)
x_test = scaler.transform(x_test)
print("\n\nTraining and testing on both datasets")
evaluate_classifiers(x_train, y_train, x_test, y_test)

# Results are good if we train on images from diferent sources seperately