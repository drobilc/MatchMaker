#!/usr/bin/env python
from __future__ import print_function

import sys
import math
import time
import rospy
import cv2
import numpy as np
import tf2_geometry_msgs
import tf2_ros
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped, Vector3, Pose
from nav_msgs.msg import OccupancyGrid

from cv_bridge import CvBridge, CvBridgeError
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
        self.focal_length = rospy.get_param('~focal_length', 554)
        
        # An object we use for converting images between ROS format and OpenCV format
        self.bridge = CvBridge()

        # We can use dlib, haar, or hog detector here
        self.face_detector = HaarDetector(self.haar_cascade_data_file_path)

        # Subscriber for new camera images (the video_stream_opencv publishes to different topic)
        self.image_subscriber = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback, queue_size=10)

        # The publisher where face poses will be published once detected
        self.face_publisher = rospy.Publisher('/face_detections_raw', Pose, queue_size=1)

        # How much we should downscale image before trying to find faces
        # For example - factor 4 means that that the new image width will be width / 4
        self.downscale_factor = rospy.get_param('~downscale_factor', 4)

        # Object we use for transforming between coordinate frames
        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf)
    
    def convert_face_to_pose(self, image, face, distance_to_face, timestamp):
        # Compute the image width and height and face center point
        height, width = image.shape
        face_center = face.center_point()

        face_x = width / 2 - face_center[0]
        face_y = height / 2 - face_center[1]

        angle_to_target = np.arctan2(face_x, self.focal_length)

        # Get the angles in the base_link relative coordinate system
        x = distance_to_face * np.cos(angle_to_target)
        y = distance_to_face * np.sin(angle_to_target)

        # Define a stamped message for transformation - in the "camera rgb frame"
        point_s = PointStamped()
        point_s.point.x = -y
        point_s.point.y = 0
        point_s.point.z = x
        point_s.header.frame_id = "camera_rgb_optical_frame"
        point_s.header.stamp = timestamp

        # Get the point in the "map" coordinate system
        try:
            point_world = self.tf_buf.transform(point_s, "map")

            # Create a Pose object with the same position
            pose = Pose()
            pose.position.x = point_world.point.x
            pose.position.y = point_world.point.y
            pose.position.z = point_world.point.z

        except Exception as e:
            rospy.logwarn('Exception while converting face to pose: {}'.format(e))
            pose = None

        return pose

    def process_face(self, image, depth_image, face, timestamp):
        # Mark the face on our image
        cv2.rectangle(image, (face.left(), face.top()), (face.right(), face.bottom()), (255, 0, 0), 3, 8, 0)

        # Find the distance to the detected face (currently simply a mean of the depth)
        # We are running detection on smaller image, so the face detector also returns
        # coordinates on the smaller image. We have to multiply the coordinates by the
        # scaling factor
        
        detected_face_depth = depth_image[
            face.top() * self.downscale_factor : face.bottom() * self.downscale_factor,
            face.left() * self.downscale_factor : face.right() * self.downscale_factor
        ]
        distance_to_face = float(np.nanmean(detected_face_depth))

        # Calculate face position in 3d using detected face position and distance_to_face
        pose = self.convert_face_to_pose(image, face, distance_to_face, timestamp)

        # If the face coordinates were successfully transformed to 3d world
        # coordinates, publish the pose to the robustifier
        if pose is not None:
            self.face_publisher.publish(pose)
    
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
        # First, we will wait until depth image is received
        depth_image_message = rospy.wait_for_message("/camera/depth/image_raw", Image)
        depth_image = self.bridge.imgmsg_to_cv2(depth_image_message, "32FC1")
        rgb_image = self.bridge.imgmsg_to_cv2(rgb_image_message, "bgr8")
        rgb_image = self.preprocess_image(rgb_image)

        # Get the timestamp of the depth image message
        depth_time = depth_image_message.header.stamp

        # Process received faces
        face_rectangles = self.face_detector.find_faces(rgb_image)
        for face_rectangle in face_rectangles:
            # Process face, send rgb and depth image, face rectangle and time when depth image was received
            self.process_face(rgb_image, depth_image, face_rectangle, depth_time)
        
        if self.display_camera_window:
            cv2.imshow("Image", rgb_image)
            cv2.waitKey(1)

if __name__ == '__main__':
    face_finder = FaceFinder()
    rospy.loginfo('Face detector started')
    rospy.spin()