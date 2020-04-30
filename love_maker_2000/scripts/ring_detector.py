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

class RingDetector(object):
    def __init__(self):
        rospy.init_node('ring_detector_beta', anonymous=False)
        rospy.loginfo('Ring detector started')

        self.bridge = CvBridge()

        # Subscriber for new depth images
        self.image_subscriber = rospy.Subscriber('/camera/depth/image_raw', Image, self.image_callback, queue_size=1)

        # Image publisher for testing reasons
        self.image_publisher = rospy.Publisher('/camera/test', Image, queue_size=1)

        # Publisher for ring locations, (maybe publish poses?)
        self.detections_publisher = rospy.Publisher('ring_detections_raw', Detection, queue_size=10)
    
    def publish_image_with_marked_rings(self, image, keypoints):
        # Draw the blobs on the image
        blank = np.zeros((1,1))
        blobs = cv2.drawKeypoints(image, keypoints, blank, (0, 0, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        # Calculate the height threshold below which (in this case above, because y coord in images goes from top to bottom)
        # all detected blobs ("rings") will be discarded, these are mainly square empty spaces below the fence
        height_detection_threshold = (len(blobs) * 4) // 9
        # Draw the line on the image for visualization purposes
        cv2.line(blobs, (0, height_detection_threshold), (len(blobs[0]), height_detection_threshold), (0, 255, 0), 2)   

        # Convert the image into right format, 8 bit unsigned char with 3 channels and publish it
        image_ros = self.bridge.cv2_to_imgmsg(blobs, '8UC3')
        self.image_publisher.publish(image_ros)

    def create_message_from_keypoint(self, keypoint):
        

    def detect_circles(self, image):
        image = cv2.GaussianBlur(image, (3, 3), 0)

        # Object for specyfiying detection parameters, it will not be used in this case
        params = cv2.SimpleBlobDetector_Params()
        # Filter by circularity (no hard edges, shape is elliptical)
        params.filterByCircularity = True
        params.minCircularity = 0.8
        # Set convexity of the blobs
        params.filterByConvexity = True
        params.minConvexity = 0.2
        # Set inertia bounds, 1 -> circle, 0 -> line, 0 - 1 -> ellipse
        params.filterByInertia = False
        params.minInertiaRatio = 0.01

        # Detect blobs, if we wanted to specify custom detection parameters
        # we would pass the params object to the below constructor as argument
        detector = cv2.SimpleBlobDetector_create()
        keypoints = detector.detect(image)

        self.publish_image_with_marked_rings(image, keypoints)

        # Filter out false positives, blobs that are too low to be rings
        height_threshold = (len(image) * 4) // 9
        true_ring_keypoints = []
        for keypoint in keypoints:
            if keypoints.pt.y < height_threshold:
                true_ring_keypoints.append(keypoint)
        
        for keypoint in true_ring_keypoints:
            detection = self.create_message_from_keypoint(keypoint)
            self.detections_publisher.publish(detection)

    def image_callback(self, depth_image_message):
        # Convert image so we can process it with the cv2 library
        depth_image = self.bridge.imgmsg_to_cv2(depth_image_message, "32FC1")
        depth_image = np.nan_to_num(depth_image, 0)
        depth_image = depth_image / np.amax(depth_image)
        depth_image = depth_image * 255
        depth_image = np.floor(depth_image)
        depth_image = depth_image.astype(np.uint8)

        self.detect_circles(depth_image)

if __name__ == '__main__':
    ring_detector = RingDetector()
    rospy.spin()

