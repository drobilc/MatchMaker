#!/usr/bin/env python
from __future__ import print_function

import sys
import math
import rospy
import cv2
import numpy as np

from localizer.srv import Localize

from std_msgs.msg import Header
from detection_msgs.msg import Detection
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, Pose, Quaternion

from cv_bridge import CvBridge, CvBridgeError

# Both detectors can be used to find faces in images
from detectors.haar_detector import HaarDetector

class FaceFinder(object):

    def __init__(self):
        # Initialize node, anonymous means that we can run same node multiple times
        rospy.init_node('face_detector', anonymous=True)

        # Read parameters from our face_finder.launch file
        self.display_camera_window = rospy.get_param('~display_camera_window', False)
        self.haar_cascade_data_file_path = rospy.get_param('~haar_cascade_data_file_path', '')
        self.focal_length = rospy.get_param('~focal_length', 554)
        
        # An object we use for converting images between ROS format and OpenCV format
        self.bridge = CvBridge()

        # We can use dlib, haar, or hog detector here
        self.face_detector = HaarDetector(self.haar_cascade_data_file_path)

        # Subscriber for new camera images (the video_stream_opencv publishes to different topic)
        self.image_subscriber = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback, queue_size=1)

        # The publisher where face poses will be published once detected
        self.detections_publisher = rospy.Publisher('detections', Detection, queue_size=10)

        # How much we should downscale image before trying to find faces
        # For example - factor 4 means that that the new image width will be width / 4
        self.downscale_factor = rospy.get_param('~downscale_factor', 4)

        self.current_message_number = 0

    def process_face(self, image, face, timestamp):
        # Mark the face on our image
        cv2.rectangle(image, (face.left(), face.top()), (face.right(), face.bottom()), (255, 0, 0), 3, 8, 0)

        return face
    
    def preprocess_image(self, image):
        # Get image width and height to calculate new size
        height, width, channels = image.shape
        new_width, new_height = (int(width / self.downscale_factor), int(height / self.downscale_factor))

        # Resize image to be 1/4 the original size, so the face detector is faster
        image = cv2.resize(image, (new_width, new_height), interpolation=cv2.INTER_LINEAR)
        
        # Convert the image to rgb for faster image processing
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        return image
    
    def construct_detection_message(self, timestamp, frame_id, rgb_image_original, face_rectangle):
        self.current_message_number += 1
        detection_message = Detection()
        detection_message.header.seq = self.current_message_number
        detection_message.header.stamp = timestamp
        detection_message.header.frame_id = frame_id
        detection_message.x = face_rectangle.left() * self.downscale_factor
        detection_message.y = face_rectangle.top() * self.downscale_factor
        detection_message.width = (face_rectangle.right() - face_rectangle.left()) * self.downscale_factor
        detection_message.height = (face_rectangle.bottom() - face_rectangle.top()) * self.downscale_factor
        detection_message.source = 'opencv'
        detection_message.confidence = 1
        face_image = rgb_image_original[face_rectangle.top() * self.downscale_factor :face_rectangle.bottom() * self.downscale_factor,face_rectangle.left()* self.downscale_factor:face_rectangle.right()* self.downscale_factor]
        detection_message.image = self.bridge.cv2_to_imgmsg(face_image, "bgr8")
        return detection_message
    
    def image_callback(self, rgb_image_message):
        # This function will be called when new camera rgb image is received
        rgb_image_original = self.bridge.imgmsg_to_cv2(rgb_image_message, "bgr8")
        rgb_image = self.preprocess_image(rgb_image_original)

        # Get the timestamp of the depth image message
        timestamp = rgb_image_message.header.stamp

        # Find faces in the image
        face_rectangles = self.face_detector.find_faces(rgb_image)

        if len(face_rectangles) <= 0:
            return

        # Iterate over faces and publish all of them to the mapper
        for face_rectangle in face_rectangles:
            cv2.rectangle(rgb_image, (face_rectangle.left(), face_rectangle.top()), (face_rectangle.right(), face_rectangle.bottom()), (255, 0, 0), 3, 8, 0)
            detection_message = self.construct_detection_message(timestamp, rgb_image_message.header.frame_id, rgb_image_original, face_rectangle)
            self.detections_publisher.publish(detection_message)
        
        if self.display_camera_window:
            cv2.imshow("Image", rgb_image)
            cv2.waitKey(1)

if __name__ == '__main__':
    face_finder = FaceFinder()
    rospy.loginfo('Face detector started')
    rospy.spin()