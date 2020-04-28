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

import utils

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

class Robustifier(object):

    RAW_MARKER_STYLE = {
        'marker_type': Marker.SPHERE,
        'color': ColorRGBA(1, 0, 0, 1)
    }
    ROBUSTIFIED_MARKER_STYLE = {
        'marker_type': Marker.SPHERE,
        'color': ColorRGBA(0, 1, 0, 1),
        'scale': Vector3(0.2, 0.2, 0.2)
    }

    def __init__(self):
        # Initialize node, don't allow running multiple nodes of this type
        rospy.init_node('robustifier', anonymous=True)

        # This parameter tells us how far two face detections have to be to be
        # considered as different positions
        self.maximum_distance = rospy.get_param('~maximum_distance', 0.5)

        # This parameter tells us how many detections is needed to proclaim a
        # face detection true positive
        self.minimum_detections = rospy.get_param('~minimum_detections', 7)

        # Raw sensor data is read from raw_detection_topic topic, then
        # robustified and the detections are then sent to detection_topic topic
        self.raw_detection_topic = rospy.get_param('~raw_detection_topic', '/face_detections_raw')
        self.detection_topic = rospy.get_param('~detection_topic', '/face_detections')
        self.marker_topic = rospy.get_param('~marker_topic', '/face_markers')

        # Subscriber and publisher for object detections
        self.raw_object_subscriber = rospy.Subscriber(self.raw_detection_topic, PoseStamped, self.on_object_detection, queue_size=10)
        self.object_publisher = rospy.Publisher(self.detection_topic, PoseStamped, queue_size=10)

        # Publisher for publishing raw face detections
        self.markers_publisher = rospy.Publisher(self.marker_topic, MarkerArray, queue_size=1000)
        self.markers = MarkerArray()

        # TODO: Instead of using a list use a quadtree or a grid
        # A list of Detection objects used for finding the closest pose
        self.object_detections = []

        # Object we use for transforming between coordinate frames
        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf)

    def publish_marker(self, pose, marker_style={}):
        # Construct a marker from detected object pose, add it to markers array
        # and publish it to marker_topic
        new_marker = utils.stamped_pose_to_marker(pose, index=len(self.markers.markers), **marker_style)
        self.markers.markers.append(new_marker)
        self.markers_publisher.publish(self.markers)

    def already_detected(self, pose_stamped):
        for detection in self.object_detections:
            distance = detection.distance_to(pose_stamped)
            if distance <= self.maximum_distance:
                # one solution to "location too far in history"
                detection.stamped_pose.header.stamp = pose_stamped.header.stamp
                return detection
    
    def on_object_detection(self, object_pose):
        # A new object has been detected, robustify it

        # First convert the object position to map coordinate frame, so we can
        # check if the distance to the nearest object is less than half a metre If
        # this is not done, the robustifier tries to compute distances between
        # points in camera_depth_optical_frame frame, which has a different
        # scale than map
        object_pose = self.tf_buf.transform(object_pose, "map")

        self.publish_marker(object_pose, Robustifier.RAW_MARKER_STYLE)

        # Check if detected object is already in object_detections. This cannot be
        # done with simple indexof function because the coordinates will not
        # exactly match
        saved_pose = self.already_detected(object_pose)
        if saved_pose is not None:
            rospy.loginfo('Object was already detected, it has been detected {} times'.format(saved_pose.number_of_detections))
            saved_pose.number_of_detections += 1

            # If object was detected more than self.minimum_detections, it is now
            # considered true positive, send it to movement controller
            if saved_pose.number_of_detections >= self.minimum_detections:
                if not saved_pose.already_sent:
                    rospy.loginfo('Sending object location to movement controller')

                    self.publish_marker(saved_pose.stamped_pose, Robustifier.ROBUSTIFIED_MARKER_STYLE)
                    self.object_publisher.publish(saved_pose.stamped_pose)

                    saved_pose.already_sent = True
            else:
                rospy.loginfo('Detection has not yet surpassed the minimum number of detections needed')
                # The detected object pose and previously saved pose are very similar,
                # calculate mean position of both poses and save data to saved_pose
                saved_pose.update(object_pose)
        else:
            rospy.loginfo('Object has not yet been deteted')
            # Construct a new detection object, add it to detections
            saved_pose = Detection(object_pose)
            self.object_detections.append(saved_pose)

if __name__ == '__main__':
    robustifier = Robustifier()
    rospy.loginfo('Robustifier started')
    rospy.spin()