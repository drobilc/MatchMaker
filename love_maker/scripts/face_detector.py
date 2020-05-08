#!/usr/bin/env python
from __future__ import print_function

import rospy
import cv2
import numpy

from std_msgs.msg import Header, Bool
from detection_msgs.msg import Detection
from geometry_msgs.msg import PoseStamped, Pose, Quaternion
from object_detection_msgs.msg import ObjectDetection
import message_filters

from sensor_msgs.msg import Image, CameraInfo
from image_geometry import PinholeCameraModel

from cv_bridge import CvBridge, CvBridgeError

# Both detectors can be used to find faces in images
from detectors.haar_detector import HaarDetector

import utils

import tf2_ros
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from tf2_geometry_msgs import PointStamped

class FaceFinder(object):

    def __init__(self):
        # Initialize node, anonymous means that we can run same node multiple times
        rospy.init_node('face_detector', anonymous=True)

        # Read parameters from our face_finder.launch file
        self.display_camera_window = rospy.get_param('~display_camera_window', False)
        self.haar_cascade_data_file_path = rospy.get_param('~haar_cascade_data_file_path', '')
        self.focal_length = rospy.get_param('~focal_length', 554)

        # How much we should downscale image before trying to find faces
        # For example - factor 4 means that that the new image width will be width / 4
        self.downscale_factor = rospy.get_param('~downscale_factor', 4)
        
        # An object we use for converting images between ROS format and OpenCV format
        self.bridge = CvBridge()

        # We can use dlib, haar, or hog detector here
        self.face_detector = HaarDetector(self.haar_cascade_data_file_path)

        # Create a new time synchronizer to synchronize depth and rgb image callbacks.
        # Also subscribe to camera info so we can get camera calibration matrix.
        self.depth_image_subscriber = message_filters.Subscriber('/camera/depth/image_raw', Image)
        self.image_subscriber = message_filters.Subscriber('/camera/rgb/image_raw', Image)
        self.camera_info_subscriber = message_filters.Subscriber('/camera/rgb/camera_info', CameraInfo)
        self.image_synchronizer = message_filters.TimeSynchronizer([self.depth_image_subscriber, self.image_subscriber, self.camera_info_subscriber], 10)
        self.image_synchronizer.registerCallback(self.on_data_received)

        # Subscriber to enable or disable face detector
        self.enabled = False
        self.toggle_subscriber = rospy.Subscriber('/face_detector_toggle', Bool, self.toggle, queue_size=10)

        # The publisher where face detections will be published once detected
        self.detections_publisher = rospy.Publisher('/face_detections_raw', ObjectDetection, queue_size=10)

        # Transformation buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Get static map information from map service
        self.map_data, self.map_resolution, self.map_origin = utils.get_map_data()
    
    def toggle(self, enable):
        """Callback to enable or disable face detector"""
        self.enabled = enable.data
        rospy.loginfo('Face detector enabled: {}'.format(self.enabled))
    
    def on_data_received(self, depth_image_message, image_message, camera_info):
        """Callback for when depth image, rgb image and camera information is received"""
        if not self.enabled:
            return

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

        self.detect_faces(rgb_image, depth_image, camera_model, timestamp)
    
    def preprocess_image(self, image):
        """Preprocess image befor running face detection on it"""
        # Get image width and height to calculate new size
        height, width, channels = image.shape
        new_width, new_height = (int(width / self.downscale_factor), int(height / self.downscale_factor))

        # Resize image to be 1/4 the original size, so the face detector is faster
        image = cv2.resize(image, (new_width, new_height), interpolation=cv2.INTER_LINEAR)
        
        # Convert the image to rgb for faster image processing
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        return image
    
    def detect_faces(self, rgb_image, depth_image, camera_model, timestamp):
        """Detect faces in image and send each detection to /face_detections_raw"""
        preprocessed_rgb_image = self.preprocess_image(rgb_image)
        face_rectangles = self.face_detector.find_faces(preprocessed_rgb_image)

        if len(face_rectangles) <= 0:
            return

        # Iterate over faces and publish all of them to the mapper
        for face in face_rectangles:
            face_position = self.to_world_position(face, depth_image, camera_model, timestamp)
            if face_position is not None:
                approaching_point = self.compute_approaching_point(face_position)
                if approaching_point is not None:
                    detection = self.construct_detection_message(face_position, approaching_point, timestamp)
                    self.detections_publisher.publish(detection)
                else:
                    rospy.logwarn('Face detected, but the face approaching point could not be determined')
            else:
                rospy.logwarn('Face detected, but the face position could not be determined')
    
    def to_world_position(self, face, depth_image, camera_model, timestamp, average=1):
        """Compute the 3d map coordinates of detected face"""
        left, right = face.left() * self.downscale_factor, face.right() * self.downscale_factor
        top, bottom = face.top() * self.downscale_factor, face.bottom() * self.downscale_factor
        center_x, center_y = int((left + right) / 2.0), int((top + bottom) / 2.0)
        
        # The average parameter defines the region around center to average when
        # computing the distance
        depth_region = depth_image[center_y-average:center_y+average, center_x-average:center_x+average]
        average_distance = numpy.nanmean(depth_region)

        # Construct a ray from camera plane center to pixel coordinate
        ray = camera_model.projectPixelTo3dRay((center_x, center_y))

        # Compute point in 3d as the ray stretched out using the average
        # distance to the face
        point = (ray[0] * average_distance, ray[1] * average_distance, ray[2] * average_distance)

        # Convert 3d point from camera frame to world frame and publish it
        face_camera = PoseStamped()
        face_camera.pose.position.x = point[0]
        face_camera.pose.position.y = point[1]
        face_camera.pose.position.z = point[2]
        face_camera.header.frame_id = "camera_rgb_optical_frame"
        face_camera.header.stamp = timestamp

        try:
            face_position = self.tf_buffer.transform(face_camera, 'map')
            return face_position
        except Exception as e:
            rospy.logwarn(e)
            return None
    
    def compute_approaching_point(self, face_pose):
        # To compute approaching point we should do the following: First, we
        # should get the detected face position and robot position in map
        # coordinates. Then, the face and robot map coordinates should be
        # converted to map pixels. A small region around the face pixel should
        # be extracted where hough line transform should be performed. After
        # lines have been found, they should be sorted by their scalar products
        # with the face - robot vector, so that lines that.

        # Get the robot position in map coordinate system
        robot_point = PointStamped()
        robot_point.header.frame_id = "camera_depth_optical_frame"
        robot_point.header.stamp = face_pose.header.stamp
        robot_point = self.tf_buffer.transform(robot_point, "map")

        # Convert map coordinates to map pixel coordinates
        face_pixel = utils.pose_to_map_pixel(face_pose.pose, self.map_origin, self.map_resolution)
        robot_pixel = utils.to_map_pixel(robot_point, self.map_origin, self.map_resolution)

        # Find the closest wall pixel and line that passes closest to that wall
        # pixel (preferably line that goes through the wall pixel)
        closest_wall = utils.closest_wall_pixel(self.map_data, face_pixel)
        if closest_wall is None:
            rospy.logwarn('No wall found near pixel.')
            return None
        
        line, distance = utils.closest_line(self.map_data, face_pixel, closest_wall)
        if line is None:
            rospy.logwarn('No line was found around pixel.')
            return None

        # Compute which side of the line robot currently is
        line_direction = line[1] - line[0]
        normal = numpy.asarray([line_direction[1], -line_direction[0]])
        orientation = normal
        if numpy.dot(robot_pixel - closest_wall, normal) < 0:
            orientation = -orientation
        
        # Now that we have a wall vector and face position, we can calculate the
        # approaching point
        face_orientation = orientation / numpy.linalg.norm(orientation)
        approaching_point = numpy.asarray([self.map_origin.x, self.map_origin.y]) + (face_pixel * self.map_resolution) + face_orientation * 0.5

        orientation = -orientation
        orientation = numpy.arctan2(orientation[1], orientation[0])
        orientation_quaternion = quaternion_from_euler(0, 0, orientation)
        orientation_quaternion = Quaternion(*orientation_quaternion)

        approaching_pose = PoseStamped()
        approaching_pose.header = face_pose.header
        approaching_pose.header.frame_id = 'map'
        approaching_pose.pose.position.x = approaching_point[0]
        approaching_pose.pose.position.y = approaching_point[1]
        approaching_pose.pose.position.z = 0
        approaching_pose.pose.orientation = orientation_quaternion

        return approaching_pose
    
    def construct_detection_message(self, face_pose, approaching_point_position, timestamp):
        """Constructs ObjectDetection message for detected face data"""
        detection = ObjectDetection()
        detection.header.frame_id = 'map'
        detection.header.stamp = timestamp
        detection.object_pose = face_pose.pose
        detection.approaching_point_pose = approaching_point_position.pose
        detection.type = 'face'
        return detection

if __name__ == '__main__':
    face_finder = FaceFinder()
    rospy.loginfo('Face detector started')
    rospy.spin()