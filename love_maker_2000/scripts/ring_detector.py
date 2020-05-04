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

        self.current_message_number = 0

        self.bridge = CvBridge()

        # Subscriber for new depth images
        self.image_subscriber = rospy.Subscriber('/camera/depth/image_raw', Image, self.image_callback, queue_size=1)

        # Image publisher for testing reasons
        self.image_publisher = rospy.Publisher('/camera/test', Image, queue_size=1)

        # Publisher for ring locations, (maybe publish poses?)
        self.detections_publisher = rospy.Publisher('/ring_detections', Detection, queue_size=10)
    
    def publish_image_with_marked_rings(self, image, keypoints):
        keypoint = keypoints[0]
        keypoint_x = int(keypoint.pt[0] - keypoint.size // 2)
        keypoint_y = int(keypoint.pt[1] - keypoint.size // 2)
        keypoint_size = int(keypoint.size)
        # Draw the blobs on the image
        blank = np.zeros((1, 1))
        blobs = cv2.drawKeypoints(image, keypoints, blank, (0, 0, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        # Calculate the height threshold below which (in this case above, because y coord in images goes from top to bottom)
        # all detected blobs ("rings") will be discarded, these are mainly square empty spaces below the fence
        height_detection_threshold = (len(blobs) * 4) // 9
        # Draw the line on the image for visualization purposes
        cv2.line(blobs, (0, height_detection_threshold), (len(blobs[0]), height_detection_threshold), (0, 255, 0), 2)
        cv2.circle(blobs, (keypoint_x, keypoint_y), 2, (255, 0, 0), 2)
        cv2.circle(blobs, (keypoint_x + keypoint_size, keypoint_y + keypoint_size), 2, (255, 0, 0), 2)

        # Convert the image into right format, 8 bit unsigned char with 3 channels and publish it
        image_ros = self.bridge.cv2_to_imgmsg(blobs, '8UC3')
        self.image_publisher.publish(image_ros)

    def construct_detection_message(self, keypoint, timestamp, frame_id):
        self.current_message_number += 1
        detection = Detection()
        detection.header.seq = self.current_message_number
        detection.header.stamp = timestamp
        detection.header.frame_id = frame_id
        detection.x = int(keypoint.pt[0] - keypoint.size / 2)
        detection.y = int(keypoint.pt[1] - keypoint.size / 2)
        detection.width = keypoint.size
        detection.width = keypoint.size
        detection.source = 'opencv'
        detection.confidence = 1
        return detection

    def get_ring_color(self, depth_image, rgb_image, keypoint, depth_threshold=10):
        # Compute the ring bounding box from its radius and center
        center_x, center_y = keypoint.pt
        radius = keypoint.size
        left, right = int(center_x - radius), int(center_x + radius)
        top, bottom = int(center_y - radius), int(center_y + radius)
        
        # Crop a small region around the ring
        depth_region = depth_image[top:bottom, left:right]
        region = rgb_image[top:bottom, left:right]

        # Threshold depth region with fixed distance to obtain mask,
        # then mask the rgb image
        ring_mask = depth_region > depth_threshold
        region = region * ring_mask[...,None]

        # Get the dominant color of the region
        # Code taken from https://stackoverflow.com/questions/43111029
        pixels = np.float32(region[region != 0].reshape(-1, 3))
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 200, .1)
        _, labels, palette = cv2.kmeans(pixels, 5, None, criteria, 10, cv2.KMEANS_RANDOM_CENTERS)
        _, counts = np.unique(labels, return_counts=True)
        dominant = palette[np.argmax(counts)]

        # Convert color from BGR to RGB color space
        dominant_color = (dominant[2], dominant[1], dominant[0])

        return dominant_color

    def detect_circles(self, image, timestamp, frame_id):
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

        # Filter out false positives, blobs that are too low to be rings
        height_threshold = (len(image) * 4) // 9
        true_ring_keypoints = []
        for keypoint in keypoints:
            rospy.loginfo(keypoint.size)
            if keypoint.pt[1] < height_threshold:
                true_ring_keypoints.append(keypoint)
        
        self.publish_image_with_marked_rings(image, true_ring_keypoints)
        
        if len(true_ring_keypoints) > 0:
            # Rings were detected, wait until rgb image is received so we can
            # get ring color
            rgb_image_message = rospy.wait_for_message("/camera/rgb/image_raw", Image)
            rgb_image = self.bridge.imgmsg_to_cv2(rgb_image_message, "bgr8")

            for keypoint in true_ring_keypoints:
                ring_color = self.get_ring_color(image, rgb_image, keypoint)
                detection = self.construct_detection_message(keypoint, timestamp, frame_id)
                self.detections_publisher.publish(detection)

    def image_callback(self, depth_image_message):
        timestamp = depth_image_message.header.stamp
        frame_id = depth_image_message.header.frame_id
        # Convert image so we can process it with the cv2 library
        depth_image = self.bridge.imgmsg_to_cv2(depth_image_message, "32FC1")
        depth_image = np.nan_to_num(depth_image)
        depth_image = depth_image / np.amax(depth_image)
        depth_image = depth_image * 255
        depth_image = np.floor(depth_image)
        depth_image = depth_image.astype(np.uint8)

        self.detect_circles(depth_image, timestamp, frame_id)

if __name__ == '__main__':
    ring_detector = RingDetector()
    rospy.spin()

