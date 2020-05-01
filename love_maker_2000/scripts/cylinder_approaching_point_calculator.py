#!/usr/bin/env python

import rospy
import math
import numpy as np

from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf2_ros
from tf2_geometry_msgs import PointStamped

from geometry_msgs.msg import Point, Pose, Quaternion
from love_maker_2000.srv import ApproachingPointCalculator, ApproachingPointCalculatorResponse

class CylinderApproachingPointCalculator():
    def __init__(self):
        rospy.init_node('cylinder_approaching_point_calculator')

        # Object we use for transforming between coordinate frames
        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf)

        service = rospy.Service('cylinder_approaching_point_calculator', CylinderApproachingPointCalculator, self.handle_approaching_point_calculation_request)

    def calculate_approaching_point(self, detection):
        # Detection is Pose() in map coordinates

        # Construct detection point
        global_detection = PointStamped()
        global_detection.header.frame_id = "map"
        global_detection.header.stamp = rospy.Time.now()
        global_detection.point.x = detection.pose.position.x
        global_detection.point.y = detection.pose.position.y
        global_detection.point.z = detection.pose.position.z

        # Construct robot point
        robot_point = PointStamped()
        robot_point.header.frame_id = "camera_depth_optical_frame"
        robot_point.header.stamp = rospy.Time.now()
        robot_point.point.x = 0
        robot_point.point.y = 0
        robot_point.point.z = 0

        # Transform detection's and robot's position to global coordinates
        global_robot_point = self.tf_buf.transform(robot_point, "map")

        # Calculate orientation vector
        global_position_detection = global_detection.point
        global_position_robot = global_robot_point.point
        orientation_parameters = []
        orientation_parameters.append(global_position_robot.x - global_position_detection.x)
        orientation_parameters.append(global_position_robot.y - global_position_detection.y)
        orientation_parameters.append(0.002472)

        # Calculate orientation
        orientation = math.atan2(orientation_parameters[1], orientation_parameters[0])
        orientation_quaternion = quaternion_from_euler(0, 0, orientation + math.pi)
        orientation_quaternion = Quaternion(orientation_quaternion[0], orientation_quaternion[1], orientation_quaternion[2], orientation_quaternion[3])

        # Normalize orientation vector
        normalized_orientation = orientation_parameters / np.linalg.norm(orientation_parameters)
        
        # Set approaching point 0.5m away from detected object
        approaching_point = Point()
        approaching_point.x = global_position_detection.x + 0.5 * normalized_orientation[0]
        approaching_point.y = global_position_detection.y + 0.5 * normalized_orientation[1]
        approaching_point.z = global_position_detection.z + 0.5 * normalized_orientation[2]

        # Construct approaching point pose
        approaching_pose = Pose()
        approaching_pose.position.x = approaching_point.x
        approaching_pose.position.y = approaching_point.y
        approaching_pose.position.z = approaching_point.z
        approaching_pose.orientation = orientation_quaternion

        return approaching_pose

    def handle_approaching_point_calculation_request(self, detection):
        return self.calculate_approaching_point(detection)

if __name__ == '__main__':
    color_classifier = CylinderApproachingPointCalculator()
    rospy.loginfo('Cylinder approaching point calculator node started')
    rospy.spin()