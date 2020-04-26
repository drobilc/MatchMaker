#!/usr/bin/env python
from __future__ import print_function

import rospy
import math
from geometry_msgs.msg import Pose, PoseStamped, Vector3, Point, Quaternion
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA

from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf2_ros
from tf2_geometry_msgs import PointStamped

import numpy as np

class Detection(object):
    
    def __init__(self, pose, number_of_detections = 0):
        self.pose = pose.pose
        self.stamped_pose = pose
        self.number_of_detections = number_of_detections
        self.already_sent = False

        # TODO: set parameter based on the distance and angle to face at the time of detection
        # The parameter that is used to blend two different positions together.
        # How much the received position changes the current position
        self.blending = 0.5
    
    def update(self, other_pose):
        other_pose = other_pose.pose
        # Calculate new position on line between current position and received position.
        # Use the blending parameter to blend current and next position (simple linear interpolation)
        # position = (1 - blending) * current + blending * next
        alpha = 1 - self.blending
        beta = self.blending
        self.pose.position.x = self.pose.position.x * alpha + other_pose.position.x * beta
        self.pose.position.y = self.pose.position.y * alpha + other_pose.position.y * beta
        self.pose.position.z = self.pose.position.z * alpha + other_pose.position.z * beta
    
    def distance_to(self, other_pose):
        other_pose = other_pose.pose
        # Return simple Euclidean distance between this and other pose
        dx = self.pose.position.x - other_pose.position.x
        dy = self.pose.position.y - other_pose.position.y
        dz = self.pose.position.z - other_pose.position.z
        return math.sqrt(dx*dx + dy*dy + dz*dz)

class CylinderRobustifier(object):

    def __init__(self):
        # Initialize node, don't allow running multiple nodes of this type
        rospy.init_node('cylinder_robustifier', anonymous=False)

        # Maximum distance between two cylinders, for them to be considered the same cylinder
        self.maximum_distance = 0.5

        # Minimum number of detections to consider cylinder a true positive
        self.minimum_detections = 7

        # Read positions of detected cylinders from this topic
        self.raw_cylinder_subscriber = rospy.Subscriber('/cylinder_detections_raw', PoseStamped, self.on_cylinder_detected, 10)
        # After determining cylinder position, send a pose to movement_controller node
        self.cylinder_publisher = rospy.Publisher('/cylinder_detections', PoseStamped, queue_size=10)

        self.cylinder_detections = []

        self.markers_publisher = rospy.Publisher('cylinder_markers_rawwww', MarkerArray, queue_size=1000)
        self.marker_array = MarkerArray()
        self.marker_number = 1

    def construct_marker(self, detection, color, scale=0.1):
        self.marker_number += 1
        marker = Marker()
        marker.header.stamp = detection.header.stamp
        marker.header.frame_id = 'map' # camera_depth_optical_frame
        marker.pose = detection.pose
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.frame_locked = False
        marker.lifetime = rospy.Duration.from_sec(10)
        marker.id = self.marker_number
        marker.scale = Vector3(scale, scale, scale)
        marker.color = color
        
        return marker

    def publish_marker(self, pose, color=ColorRGBA(0, 0.5, 0.5, 1), scale=0.1):
        # Send a list of all markers to visualize data
        marker = self.construct_marker(pose, color, scale)
        self.marker_array.markers.append(marker)
        self.markers_publisher.publish(self.marker_array)

    def already_detected(self, cylinder_pose):
        for detection in self.cylinder_detections:
            distance = detection.distance_to(cylinder_pose)
            if distance <= self.maximum_distance:
                detection.stamped_pose.header.stamp = cylinder_pose.header.stamp
                return detection

    def on_cylinder_detected(self, cylinder_pose, smth):
        rospy.loginfo(smth)
        # Cylinder PoseStamped is already in the map coordinate frame, no need to convert it

        saved_pose = self.already_detected(cylinder_pose)
        if saved_pose is not None:
            rospy.loginfo('Cylinder was already detected, it has been detected {} times'.format(saved_pose.number_of_detections))
            saved_pose.number_of_detections += 1

            # If cylinder is detected more than self.minimum_detections, it is now considered
            # true positive, send it to movement_controller
            if saved_pose.number_of_detections >= self.minimum_detections:
                if not saved_pose.already_sent:
                    # TODO: send approaching point to movement controller

                    self.publish_marker(saved_pose.stamped_pose, ColorRGBA(0, 1, 0, 1), 0.3)
            else:
                rospy.loginfo('Detection has not yet surpassed the minimum number of detections needed')
                # The detected cylinder pose and previously saved pose are very similar,
                # calculate mean position of both poses and save data to saved_pose
                saved_pose.update(cylinder_pose)
        else:
            rospy.loginfo('Cylinder is probably new.')
            # Construct new detection object, add it to detections
            saved_pose = Detection(cylinder_pose)
            self.cylinder_detections.append(saved_pose)


if __name__ == '__main__':
    robustifier = CylinderRobustifier()
    rospy.loginfo('Cylinder robustifier started')
    rospy.spin()