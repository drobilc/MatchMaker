#!/usr/bin/env python
from __future__ import print_function

import sys
import rospy
import cv2
import numpy as np
import message_filters

import tf2_geometry_msgs
import tf2_ros

from std_msgs.msg import Header, ColorRGBA
from detection_msgs.msg import Detection
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, Pose, Quaternion, Point
from object_detection_msgs.msg import ObjectDetection

from cv_bridge import CvBridge, CvBridgeError
from image_geometry import PinholeCameraModel

from color_classification.srv import ColorClassification, ColorClassificationResponse

class RingDetector(object):
    def __init__(self):
        rospy.init_node('ring_detector_beta', anonymous=False)
        rospy.loginfo('Ring detector started')

        self.bridge = CvBridge()

        # Create a new time synchronizer to synchronize depth and rgb image callbacks.
        # Also subscribe to camera info so we can get camera calibration matrix.
        self.depth_image_subscriber = message_filters.Subscriber('/camera/depth/image_raw', Image)
        self.image_subscriber = message_filters.Subscriber('/camera/rgb/image_raw', Image)
        self.camera_info_subscriber = message_filters.Subscriber('/camera/rgb/camera_info', CameraInfo)
        self.image_synchronizer = message_filters.TimeSynchronizer([self.depth_image_subscriber, self.image_subscriber, self.camera_info_subscriber], 10)
        self.image_synchronizer.registerCallback(self.on_data_received)

        # Color classification service
        rospy.wait_for_service('color_classifier')
        self.classify_color = rospy.ServiceProxy('color_classifier', ColorClassification)

        # Publisher for ring ObjectDetections
        self.detections_publisher = rospy.Publisher('/ring_detections_raw', ObjectDetection, queue_size=10)

        # Transformation buffer nd listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
    
    def on_data_received(self, depth_image_message, image_message, camera_info):
        """Callback for when depth image, rgb image and camera information is received"""
        # Because of the TimeSynchronizer, depth image, rgb image and camera
        # information have the same timestamps.
        timestamp = depth_image_message.header.stamp

        # Convert images so we can process it with the cv2 library
        depth_image = self.bridge.imgmsg_to_cv2(depth_image_message, "32FC1")
        rgb_image = self.bridge.imgmsg_to_cv2(image_message, "bgr8")

        # Construct a new PinholeCameraModel from received camera information.
        # This can be used to create rays from camera center to points in 3d
        # world.
        camera_model = PinholeCameraModel()
        camera_model.fromCameraInfo(camera_info)

        self.detect_circles(depth_image, rgb_image, camera_model, timestamp)
    
    def detect_circles(self, depth_image, rgb_image, camera_model, timestamp):
        """Detect circles in depth image and send detections to ring robustifier"""
        # Disparity is computed in meters from the camera center
        disparity = np.copy(depth_image)

        # This part is needed to detect circles in image with better precision
        depth_image = np.nan_to_num(depth_image)
        depth_image = depth_image / np.amax(depth_image)
        depth_image = depth_image * 255
        depth_image = np.floor(depth_image)
        depth_image = depth_image.astype(np.uint8)

        depth_image = cv2.GaussianBlur(depth_image, (3, 3), 0)

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
        keypoints = detector.detect(depth_image)

        # Filter out false positives, blobs that are too low to be rings
        height_threshold = (len(depth_image) * 4) // 9
        true_ring_keypoints = []
        for keypoint in keypoints:
            if keypoint.pt[1] < height_threshold:
                true_ring_keypoints.append(keypoint)
        
        for keypoint in true_ring_keypoints:
            # Compute the ring bounding box from its radius and center
            center_x, center_y = keypoint.pt
            radius = keypoint.size / 2.0
            left, right = int(center_x - radius), int(center_x + radius)
            top, bottom = int(center_y - radius), int(center_y + radius)
            
            # Crop a small region around the ring
            depth_region = np.copy(depth_image[top:bottom, left:right])
            image_region = np.copy(rgb_image[top:bottom, left:right])
            disparity_region = disparity[top:bottom, left:right]

            # Compute the average ring color from image and then use the color
            # classification service to get color as string
            ring_color = self.get_ring_color(depth_region, image_region)
            classified_color = self.classify_color(ring_color).classified_color

            # Compute position of ring in 3d world coordinate system
            ring_position = self.to_world_position(keypoint, disparity_region, camera_model, timestamp)

            # Send the ring position to robustifier, if it is 
            if ring_position is not None:
                detection = self.construct_detection_message(ring_position, ring_color, classified_color, timestamp)
                self.detections_publisher.publish(detection)
    
    def to_world_position(self, keypoint, disparity_region, camera_model, timestamp):
        # Get average distance to the ring
        average_distance = np.nanmean(disparity_region)
        
        # Construct a ray from camera center to pixel on image, then stretch
        # it, so its length is the distance from camera to point in 3d space
        center_x, center_y = keypoint.pt
        ray = camera_model.projectPixelTo3dRay((center_x, center_y))
        point = (ray[0] * average_distance, ray[1] * average_distance, ray[2] * average_distance)
        
        # Convert 3d point from camera frame to world frame and publish it
        ring_camera = PoseStamped()
        ring_camera.pose.position.x = point[0]
        ring_camera.pose.position.y = point[1]
        ring_camera.pose.position.z = point[2]
        ring_camera.header.frame_id = "camera_rgb_optical_frame"
        ring_camera.header.stamp = timestamp

        try:
            ring_position = self.tf_buffer.transform(ring_camera, "map")
            return ring_position
        except Exception as e:
            rospy.logwarn(e)
            return None

    def construct_detection_message(self, ring_pose, ring_color, classified_color, timestamp):
        """Construct ObjectDetection from ring detection"""
        detection = ObjectDetection()
        detection.header.frame_id = 'map'
        detection.header.stamp = timestamp
        detection.object_pose = ring_pose.pose
        detection.approaching_point_pose = ring_pose.pose
        detection.color = ring_color
        detection.classified_color = classified_color
        detection.type = 'ring'
        return detection

    def get_ring_color(self, depth_region, image_region, depth_threshold=10):
        """Get average color of image_region but ignore all black pixels"""
        ring_mask = depth_region > depth_threshold
        region = image_region * ring_mask[...,None]
        
        pixels = region.reshape(-1, 3)
        pixel_mask = np.any(pixels != [0, 0, 0], axis=-1)
        
        non_black_pixels = pixels[pixel_mask]
        average_color = np.mean(non_black_pixels, axis=0)

        return ColorRGBA(int(average_color[2]), int(average_color[1]), int(average_color[0]), 255)

if __name__ == '__main__':
    ring_detector = RingDetector()
    rospy.spin()

