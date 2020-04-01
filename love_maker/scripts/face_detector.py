#!/usr/bin/env python
from __future__ import print_function

import sys
import math
import time
import rospy
import dlib
import cv2
import numpy as np
import tf2_geometry_msgs
import tf2_ros
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped, Vector3, Pose
from nav_msgs.msg import OccupancyGrid

from cv_bridge import CvBridge, CvBridgeError
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

# Both detectors can be used to find faces in images
from detectors.haar_detector import HaarDetector

class FaceFinder(object):
    
    def __init__(self):
        # Initialize node, anonymous means that we can run same node multiple times
        rospy.init_node('face_detector', anonymous=True)

        # Read parameters from our face_finder.launch file
        self.display_camera_window = rospy.get_param('~display_camera_window', False)
        self.haar_cascade_data_file_path = rospy.get_param('~haar_cascade_data_file_path', '')
        
        # An object we use for converting images between ROS format and OpenCV format
        self.bridge = CvBridge()

        # We can use dlib, haar, or hog detector here
        self.face_detector = HaarDetector(self.haar_cascade_data_file_path)

        # Subscriber for new camera images (the video_stream_opencv publishes to different topic)
        self.image_subscriber = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback, queue_size=1)

        # The publisher where face poses will be published once detected
        self.face_publisher = rospy.Publisher('/face_detections_raw', Pose, queue_size=10)

        # How much we should downscale image before trying to find faces
        # For example - factor 4 means that that the new image width will be width / 4
        self.downscale_factor = rospy.get_param('~downscale_factor', 4)

    def process_face(self, image, face):
        # Get coordinates of the rectangle around the face
        x1 = face.left()
        x2 = face.right()
        y1 = face.top()
        y2 = face.bottom()

        # Extract region containing face
        face_region = image[y1:y2,x1:x2]

        # Mark the face on our image
        cv2.rectangle(image, (x1, y1), (x2, y2), (255, 0, 0), 3, 8, 0)

        # TODO: Compute face position in 3d, construct a pose and send it to
        # /face_detections_raw topic
    
    def preprocess_image(self, image):
        # Get image width and height to calculate new size
        height, width, channels = image.shape
        new_width, new_height = (int(width / self.downscale_factor), int(height / self.downscale_factor))

        # Resize image to be 1/4 the original size, so the face detector is faster
        image = cv2.resize(image, (new_width, new_height), interpolation=cv2.INTER_LINEAR)
        
        # Convert the image to rgb for faster image processing
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        return image
    
    def image_callback(self, rgb_image_message):
        # This function will be called when new camera rgb image is received
        rgb_image = self.bridge.imgmsg_to_cv2(rgb_image_message, "bgr8")
        rgb_image = self.preprocess_image(rgb_image)

        # Process received faces
        face_rectangles = self.face_detector.find_faces(rgb_image)
        for face_rectangle in face_rectangles:
            self.process_face(rgb_image, face_rectangle)
        
        if self.display_camera_window:
            cv2.imshow("Image", rgb_image)
            cv2.waitKey(1)

if __name__ == '__main__':
    face_finder = FaceFinder()
    rospy.loginfo('Face detector started')
    rospy.spin()