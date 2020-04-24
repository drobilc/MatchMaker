from sklearn.neighbors import KNeighborsClassifier
from sklearn.tree import DecisionTreeClassifier
from sklearn.ensemble import RandomForestClassifier
from sklearn.naive_bayes import GaussianNB
from sklearn.svm import SVC
import numpy
import csv

from colormath.color_objects import XYZColor, sRGBColor, HSVColor, LabColor
from colormath.color_conversions import convert_color

# Disable warnings for some sklearn classifiers
import warnings
warnings.simplefilter(action='ignore', category=FutureWarning)

def read_data(input_file):
    # This function reads data from input file of format r;g;b;class
    rows = csv.reader(csv_file, delimiter = ';')

    data_x, data_y = [], []
    for row in rows:
        if row[3] == "white":
            continue
        data_x.append([float(row[0]), float(row[1]), float(row[2])])
        data_y.append(row[3])

    data_x = numpy.asarray(data_x)
    data_y = numpy.asarray(data_y)
    return data_x, data_y

# Read train data from train.txt file, then read test data from test.txt
with open('train.csv', 'r', encoding='utf-8') as csv_file:
    train_x, train_y = read_data(csv_file)

with open('test.csv', 'r', encoding='utf-8') as csv_file:
    test_x, test_y = read_data(csv_file)

def rgb_to_lab(color):
    lab = convert_color(sRGBColor(*(color / 255)), LabColor)
    return numpy.asarray([lab.lab_l, lab.lab_a, lab.lab_b])

def rgb_to_hsv(color):
    hsv = convert_color(sRGBColor(*(color / 255)), HSVColor)
    return numpy.asarray([hsv.hsv_h, hsv.hsv_s, hsv.hsv_v])

classifiers = {
    "knn, k = 1": KNeighborsClassifier(1),
    "knn, k = 1, weights = distance": KNeighborsClassifier(1, weights='distance'),
    "knn, k = 3": KNeighborsClassifier(3),
    "knn, k = 3, weights = distance": KNeighborsClassifier(3, weights='distance'),
    "knn, k = 5": KNeighborsClassifier(5),
    "knn, k = 5, weights = distance": KNeighborsClassifier(5, weights='distance'),
    "knn, k = 7": KNeighborsClassifier(7),
    "knn, k = 7, weights = distance": KNeighborsClassifier(7, weights='distance'),
    "decision tree": DecisionTreeClassifier(),
    "naive bayes": GaussianNB(),
    "random forest": RandomForestClassifier(),
    "support vector machine": SVC(kernel="linear", C=0.025),
}

color_spaces = {
    "rgb": lambda x: x,
    "hsv": rgb_to_hsv,
    "lab": rgb_to_lab
}

for classifier_name in classifiers:
    # Get the classifier by classifier name
    classifier = classifiers[classifier_name]
    accuracies = {}
    for color_space in color_spaces:
        # Transform the training and testing data to color space
        train_x_transformed = numpy.apply_along_axis(color_spaces[color_space], 1, train_x)
        test_x_transformed = numpy.apply_along_axis(color_spaces[color_space], 1, test_x)
        classifier.fit(train_x_transformed, train_y)
        test_score = classifier.score(test_x_transformed, test_y)
        accuracies[color_space] = test_score
    print('{} & {:.2f} & {:.2f} & {:.2f} \\\\ \\hline'.format(classifier_name, accuracies['rgb'], accuracies['hsv'], accuracies['lab']))