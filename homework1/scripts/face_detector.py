#!/usr/bin/env python
from __future__ import print_function

import sys
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
from detectors.dlib_detector import HogDetector

class FaceFinder(object):
    
    def __init__(self):
        # Initialize node, anonymous means that we can run same node multiple times
        rospy.init_node('face_detector', anonymous=True)

        # Read parameters from our face_finder.launch file
        self.display_camera_window = rospy.get_param('~display_camera_window', False)
        self.haar_cascade_data_file_path = rospy.get_param('~haar_cascade_data_file_path', '')
        
        # An object we use for converting images between ROS format and OpenCV format
        self.bridge = CvBridge()

        # We can use dlib or haar detector here, just uncomment correct line
        self.face_detector = HogDetector()
        # self.face_detector = HaarDetector(self.haar_cascade_data_file_path)

        # Subscriber for new camera images (the video_stream_opencv publishes to different topic)
        self.image_subscriber = rospy.Subscriber('/videofile/image_raw', Image, self.image_callback, buff_size=200*1024*1024, queue_size=None, tcp_nodelay=True)

        self.frames = 0

    def process_face(self, image, depth_image, face, depth_time):
        # Get coordinates of the rectangle around the face
        x1 = face.left()
        x2 = face.right()
        y1 = face.top()
        y2 = face.bottom()

        # Extract region containing face
        face_region = image[y1:y2,x1:x2]

        # Mark the face on our image
        cv2.rectangle(image, (x1, y1), (x2, y2), (255, 0, 0), 3, 8, 0)
    
    def image_callback(self, rgb_image_message):
        # This function will be called when new camera rgb image is received
        rospy.loginfo('[{}] New image frame received'.format(self.frames))
        rgb_image = self.bridge.imgmsg_to_cv2(rgb_image_message, "bgr8")

        height, width, channels = rgb_image.shape

        rgb_image = cv2.resize(rgb_image, (width / 4, height / 4))

        self.frames += 1

        """face_rectangles = self.face_detector.find_faces(rgb_image)
        rospy.loginfo('Found {} faces'.format(len(face_rectangles)))
        for face_rectangle in face_rectangles:
            self.process_face(rgb_image, depth_image, face_rectangle, depth_time)"""
        
        if self.display_camera_window:
            cv2.imshow("Image", rgb_image)
            cv2.waitKey(1)

if __name__ == '__main__':
    face_finder = FaceFinder()
    rospy.loginfo('Face detector started')
    rospy.spin()