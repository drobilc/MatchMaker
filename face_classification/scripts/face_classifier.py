#!/usr/bin/env python
import rospy
import numpy as np
import pandas as pd
import face_recognition
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

from sklearn.neighbors import KNeighborsClassifier
from sklearn.naive_bayes import GaussianNB
from sklearn.svm import SVC

from face_classification.srv import FaceClassification, FaceClassificationResponse

def read_data(input_file):
    dataset = pd.read_csv(input_file)
    x = dataset.iloc[:, :-1].values
    y = dataset.iloc[:, 128].values
    return x, y

class FaceClassifier(object):

    def __init__(self):
        rospy.init_node('face_classification_server')

        self.training_data_file = rospy.get_param('~training_data', None)
        if self.training_data_file is None:
            raise Exception('No training data supplied to face recognition server')

        # Read the data
        self.train_x, self.train_y = read_data(self.training_data_file)

        # Normalize the features
        from sklearn.preprocessing import StandardScaler
        self.scaler = StandardScaler()
        self.scaler.fit(self.train_x)
        self.train_x = self.scaler.transform(self.train_x)
        
        # Create a classifier and train it
        self.classifier = KNeighborsClassifier(n_neighbors=5, weights='distance')
        # self.classifier = GaussianNB()
        # self.classifier = SVC(kernel="linear", C=0.1)
        self.classifier.fit(self.train_x, self.train_y)

        self.bridge = CvBridge()

        service = rospy.Service('face_classification', FaceClassification, self.handle_face_classification_request)

        self.image_publisher = rospy.Publisher('/face_images', Image, queue_size=10)
    
    def calculate_encoding(self, face_image_message):
        # Convert the image message into cv2 image
        face_image = self.bridge.imgmsg_to_cv2(face_image_message, '8UC3')
        face_image = cv2.cvtColor(face_image, cv2.COLOR_BGR2RGB)
        face_image = np.asarray(face_image, dtype=np.uint8)

        # Calculate encodings of all faces in the image
        encodings = face_recognition.face_encodings(face_image)

        # Normalize the freaking encoding, don't forget that!!!
        encodings = self.scaler.transform(encodings)

        # Return only the first one
        if len(encodings) > 0:
            return encodings[0]

        return None
    
    def predict(self, face_image_message):
        encoding = self.calculate_encoding(face_image_message)
        
        if encoding is None:
            return 'none'
        
        label = self.classifier.predict([encoding])[0]
        rospy.loginfo(label)

        return label

    def handle_face_classification_request(self, request):
        #self.image_publisher.publish(request.face_image)
        label = self.predict(request.face_image)
        return label

if __name__ == '__main__':
    face_classifier = FaceClassifier()
    rospy.loginfo('Face recognition node started')
    print(face_recognition.__file__)
    rospy.spin()