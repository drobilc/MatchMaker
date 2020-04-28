#!/usr/bin/env python
import roslib
roslib.load_manifest('localizer')
import rospy
import sys, select, termios, tty
from std_msgs.msg import String, Bool, ColorRGBA
import sensor_msgs.msg
import message_filters
import collections
from geometry_msgs.msg import PoseStamped
from detection_msgs.msg import Detection
from localizer.srv import Localize
from sensor_msgs.msg import CameraInfo
from visualization_msgs.msg import Marker, MarkerArray
from image_geometry import PinholeCameraModel
from geometry_msgs.msg import Point, Vector3

import math
import numpy as np

from geometry_msgs.msg import Pose, PoseStamped, Vector3, Point, Quaternion
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA

from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf2_ros
from tf2_geometry_msgs import PointStamped

import utils

class DetectionMapper():

    def __init__(self):
        rospy.init_node('mapper', anonymous=False)

        self.region_scope = rospy.get_param('~region', 1)
        self.buffer_size = rospy.get_param('~camera_buffer_size', 50)
        rospy.wait_for_service('localizer/localize')

        self.camera_infos = collections.deque(maxlen = self.buffer_size)
        self.detections_sub = message_filters.Subscriber('detections', Detection)
        self.detections_sub.registerCallback(self.detections_callback)

        self.camera_sub = message_filters.Subscriber('/camera/rgb/camera_info', CameraInfo)
        self.camera_sub.registerCallback(self.camera_callback)

        self.localize = rospy.ServiceProxy('localizer/localize', Localize)

        self.approaching_point_publisher = rospy.Publisher('/face_detections_raw', PoseStamped, queue_size=100)

        self.markers = MarkerArray()
        self.markers_publisher = rospy.Publisher('/faces', MarkerArray, queue_size=1000)

        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf)
    
    def compute_approaching_point(self, face_point):
        # Construct face point
        global_face_point = PointStamped()
        global_face_point.header.frame_id = face_point.header.frame_id
        global_face_point.header.stamp = face_point.header.stamp
        global_face_point.point.x = face_point.pose.position.x
        global_face_point.point.y = face_point.pose.position.y
        global_face_point.point.z = face_point.pose.position.z

        # Construct robot point
        robot_point = PointStamped()
        robot_point.header.frame_id = "camera_depth_optical_frame"
        robot_point.header.stamp = face_point.header.stamp
        robot_point.point.x = 0
        robot_point.point.y = 0
        robot_point.point.z = 0

        # Transform face's and robot's position to global coordinates
        global_face_point = self.tf_buf.transform(global_face_point, "map")
        global_robot_point = self.tf_buf.transform(robot_point, "map")

        # Calculate orientation vector
        global_position_face = global_face_point.point
        global_position_robot = global_robot_point.point
        orientation_parameters = []
        orientation_parameters.append(global_position_robot.x - global_position_face.x)
        orientation_parameters.append(global_position_robot.y - global_position_face.y)
        orientation_parameters.append(0.002472)

        # Calculate orientation
        orientation = math.atan2(orientation_parameters[1], orientation_parameters[0])
        orientation_quaternion = quaternion_from_euler(0, 0, orientation + math.pi)
        orientation_quaternion = Quaternion(orientation_quaternion[0], orientation_quaternion[1], orientation_quaternion[2], orientation_quaternion[3])

        # Normalize orientation vector
        normalized_orientation = orientation_parameters / np.linalg.norm(orientation_parameters)
        
        # Calculate approaching points
        approaching_point = PointStamped()
        approaching_point.header = global_face_point.header
        approaching_point.point.x = global_position_face.x + 0.5 * normalized_orientation[0]
        approaching_point.point.y = global_position_face.y + 0.5 * normalized_orientation[1]
        approaching_point.point.z = global_position_face.z + 0.5 * normalized_orientation[2]

        approaching_point.point.y = max(approaching_point.point.y, 1.2)

        # Construct approaching point pose
        approaching_pose = PoseStamped()
        approaching_pose.header = global_face_point.header
        approaching_pose.pose.position.x = approaching_point.point.x
        approaching_pose.pose.position.y = approaching_point.point.y
        approaching_pose.pose.position.z = approaching_point.point.z
        approaching_pose.pose.orientation = orientation_quaternion

        return approaching_pose

    def camera_callback(self, camera_info):
        self.camera_infos.append(camera_info)

    def detections_callback(self, detection):
        u = detection.x + detection.width / 2
        v = detection.y + detection.height / 2

        camera_info = None
        best_time = 100
        for ci in self.camera_infos:
            time = abs(ci.header.stamp.to_sec() - detection.header.stamp.to_sec())
            if time < best_time:
                camera_info = ci
                best_time = time

        if not camera_info or best_time > 1:
            return

        camera_model = PinholeCameraModel()
        camera_model.fromCameraInfo(camera_info)

        point = Point(((u - camera_model.cx()) - camera_model.Tx()) / camera_model.fx(),
             ((v - camera_model.cy()) - camera_model.Ty()) / camera_model.fy(), 1)

        localization = self.localize(detection.header, point, self.region_scope)

        if not localization:
            return
        
        # Construct a PoseStamped at position of the face and publish it
        pose = PoseStamped()
        pose.header.stamp = detection.header.stamp
        pose.header.frame_id = detection.header.frame_id
        pose.pose = localization.pose

        face_marker = utils.stamped_pose_to_marker(pose, index=len(self.markers.markers), color=ColorRGBA(1, 0, 1, 1))
        self.markers.markers.append(face_marker)
        self.markers_publisher.publish(self.markers)

        # Compute where the approaching point for this face is. Then send this
        # data to robustifier.
        approaching_point = self.compute_approaching_point(pose)
        self.approaching_point_publisher.publish(approaching_point)
   
if __name__ == '__main__':
    mapper = DetectionMapper()
    rospy.spin()
