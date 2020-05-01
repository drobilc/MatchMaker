#!/usr/bin/env python
from sklearn.naive_bayes import GaussianNB

from colormath.color_objects import sRGBColor, LabColor
from colormath.color_conversions import convert_color

import rospy
import numpy
import csv

from color_classification.srv import ColorClassification, ColorClassificationResponse

def rgb_to_lab(color):
    lab = convert_color(sRGBColor(*(color / 255)), LabColor)
    return numpy.asarray([lab.lab_l, lab.lab_a, lab.lab_b])

def read_data(input_file):
    # This function reads data from input file of format r;g;b;class
    rows = csv.reader(input_file, delimiter = ';')

    data_x, data_y = [], []
    for row in rows:
        data_x.append([float(row[0]), float(row[1]), float(row[2])])
        data_y.append(row[3])

    data_x = numpy.asarray(data_x)
    data_y = numpy.asarray(data_y)
    return data_x, data_y

class ColorClassifier(object):

    def __init__(self):
        rospy.init_node('color_classifier_server')

        self.training_data_file = rospy.get_param('~training_data', None)
        if self.training_data_file is None:
            raise Exception('No training data supplied to color classifier')

        # Load training data from file
        with open(self.training_data_file, 'r') as csv_file:
            self.train_x, self.train_y = read_data(csv_file)
        
        # Convert from sRGB color to LAB color space
        self.train_x = numpy.apply_along_axis(rgb_to_lab, 1, self.train_x)
        
        # Train our naive bayes classifier
        self.classifier = GaussianNB()
        self.classifier.fit(self.train_x, self.train_y)

        service = rospy.Service('color_classifier', ColorClassification, self.handle_color_classification_request)
    
    def predict(self, color):
        # Use our training classifier to predict color
        return "white"
    
    def handle_color_classification_request(self, request):
        return self.predict(request.color)

if __name__ == '__main__':
    color_classifier = ColorClassifier()
    rospy.loginfo('Color classification node started')
    rospy.spin()