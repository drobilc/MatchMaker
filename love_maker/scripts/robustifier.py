#!/usr/bin/env python
from __future__ import print_function

import rospy
import math
from geometry_msgs.msg import Pose, PoseStamped, Vector3, Point, Quaternion
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA
from object_detection_msgs.msg import ObjectDetection

from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf2_ros
from tf2_geometry_msgs import PointStamped

import numpy as np

import utils

class Detection(object):

    COLOR_MAP = {
        "red": ColorRGBA(1, 0, 0, 1),
        "green": ColorRGBA(0, 1, 0, 1),
        "blue": ColorRGBA(0, 0, 1, 1),
        "yellow": ColorRGBA(1, 1, 0, 1),
        "white": ColorRGBA(1, 1, 1, 1),
        "black": ColorRGBA(0, 0, 0, 1)
    }
    
    def __init__(self, detection, number_of_detections = 0):
        self.detection = detection
        self.number_of_detections = number_of_detections
        self.already_sent = False
        self.blending = 0.5

        self.color_classifications = []
        if detection.classified_color:
            self.color_classifications.append(detection.classified_color)
    
    def get_object_pose(self):
        pose = PoseStamped()
        pose.header.frame_id = self.detection.header.frame_id
        pose.header.stamp = self.detection.header.stamp
        pose.pose = self.detection.object_pose
        return pose
    
    def get_approaching_point_pose(self):
        pose = PoseStamped()
        pose.header.frame_id = self.detection.header.frame_id
        pose.header.stamp = self.detection.header.stamp
        pose.pose = self.detection.approaching_point_pose
        return pose
    
    def get_real_color(self):
        color = ColorRGBA()
        color.r = self.detection.color.r / 255.0
        color.g = self.detection.color.g / 255.0
        color.b = self.detection.color.b / 255.0
        color.a = 1.0
        return color
    
    def get_color(self):
        if len(self.color_classifications) <= 0:
            return self.get_real_color()
        
        most_frequent = max(set(self.color_classifications), key = self.color_classifications.count) 
        return self.COLOR_MAP[most_frequent]
        
    
    def update_object_position(self, other_detection, blending=0.5):
        self_pose = self.detection.object_pose
        other_pose = Detection(other_detection).get_object_pose().pose

        # Calculate new position on line between current position and received position.
        # Use the blending parameter to blend current and next position (simple linear interpolation)
        # position = (1 - blending) * current + blending * next
        alpha, beta = 1 - blending, blending
        self_pose.position.x = self_pose.position.x * alpha + other_pose.position.x * beta
        self_pose.position.y = self_pose.position.y * alpha + other_pose.position.y * beta
        self_pose.position.z = self_pose.position.z * alpha + other_pose.position.z * beta
    
    def update_approaching_point_position(self, other_detection, blending=0.5):
        self_pose = self.detection.approaching_point_pose
        other_pose = Detection(other_detection).get_approaching_point_pose().pose
        alpha, beta = 1 - blending, blending
        self_pose.position.x = self_pose.position.x * alpha + other_pose.position.x * beta
        self_pose.position.y = self_pose.position.y * alpha + other_pose.position.y * beta
        self_pose.position.z = self_pose.position.z * alpha + other_pose.position.z * beta

    def update(self, other_detection):
        # Update detected object position, then update approacing point
        # position.
        self.update_object_position(other_detection, self.blending)
        self.update_approaching_point_position(other_detection, self.blending)

        # Add color classification to list of color classifications
        if other_detection.classified_color:
            self.color_classifications.append(other_detection.classified_color)
    
    def distance_to(self, other_detection):
        self_pose = self.get_object_pose().pose
        other_pose = Detection(other_detection).get_object_pose().pose
        # Return simple Euclidean distance between this and other pose
        dx = self_pose.position.x - other_pose.position.x
        dy = self_pose.position.y - other_pose.position.y
        dz = self_pose.position.z - other_pose.position.z
        return math.sqrt(dx*dx + dy*dy + dz*dz)

class Robustifier(object):

    RAW_MARKER_STYLE = {
        'face': {
            'marker_type': Marker.SPHERE,
            'color': ColorRGBA(1, 0, 0, 1)
        },
        'ring': {
            'marker_type': Marker.SPHERE
        },
        'cylinder': {
            'marker_type': Marker.CYLINDER
        }
    }

    ROBUSTIFIED_MARKER_STYLE = {
        'face': {
            'marker_type': Marker.SPHERE,
            'scale': Vector3(0.2, 0.2, 0.2),
            'color': ColorRGBA(0, 1, 0, 1)
        },
        'ring': {
            'marker_type': Marker.SPHERE,
            'scale': Vector3(0.2, 0.2, 0.2)
        },
        'cylinder': {
            'marker_type': Marker.CYLINDER,
            'scale': Vector3(0.2, 0.2, 0.2)
        }
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
        self.raw_object_subscriber = rospy.Subscriber(self.raw_detection_topic, ObjectDetection, self.on_object_detection, queue_size=10)
        self.object_publisher = rospy.Publisher(self.detection_topic, ObjectDetection, queue_size=10)

        # Publisher for publishing raw object detections
        self.markers_publisher = rospy.Publisher(self.marker_topic, MarkerArray, queue_size=1000)
        self.markers = MarkerArray()

        # TODO: Instead of using a list use a quadtree or a grid
        # A list of Detection objects used for finding the closest pose
        self.object_detections = []

        # Object we use for transforming between coordinate frames
        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf)

    def publish_marker(self, detection, marker_style={}):
        # Construct a marker from detected object, add it to markers array
        # and publish it to marker_topic
        pose = detection.get_object_pose()

        marker_style_copy = marker_style.copy()

        # If color is defined in marker style, it overrides the received color
        color = detection.get_color()
        if 'color' in marker_style:
            color = marker_style['color']
            marker_style_copy.pop('color')
        
        marker_style_copy['color'] = color

        new_marker = utils.stamped_pose_to_marker(pose,
            index=len(self.markers.markers),
            **marker_style_copy
        )
        self.markers.markers.append(new_marker)
        self.markers_publisher.publish(self.markers)

    def already_detected(self, new_detection):
        for detection in self.object_detections:
            distance = detection.distance_to(new_detection)
            if distance <= self.maximum_distance:
                # one solution to "location too far in history"
                detection.detection.header.stamp = new_detection.header.stamp
                return detection
    
    def on_object_detection(self, detection):
        # Here we assume that detection.object_pose and
        # detection.approaching_point_pose are in map coordinate frame.
        if detection.header.frame_id != 'map':
            return
        
        self.publish_marker(Detection(detection), Robustifier.RAW_MARKER_STYLE[detection.type])

        # Check if detected object is already in object_detections. This cannot be
        # done with simple indexof function because the coordinates will not
        # exactly match
        saved_pose = self.already_detected(detection)

        if saved_pose is not None:
            rospy.loginfo('Object was already detected, it has been detected {} times'.format(saved_pose.number_of_detections))
            saved_pose.number_of_detections += 1

            # If object was detected more than self.minimum_detections, it is now
            # considered true positive, send it to movement controller
            if saved_pose.number_of_detections >= self.minimum_detections:
                if not saved_pose.already_sent:
                    rospy.loginfo('Sending object location to movement controller')
                    self.publish_marker(saved_pose, Robustifier.ROBUSTIFIED_MARKER_STYLE[saved_pose.detection.type])
                    self.object_publisher.publish(saved_pose.detection)
                    saved_pose.already_sent = True
            else:
                rospy.loginfo('Detection has not yet surpassed the minimum number of detections needed')
                # The detected object pose and previously saved pose are very similar,
                # calculate mean position of both poses and save data to saved_pose
                saved_pose.update(detection)
        else:
            rospy.loginfo('Object has not yet been detected')
            # Construct a new detection object, add it to detections
            saved_pose = Detection(detection)
            self.object_detections.append(saved_pose)

if __name__ == '__main__':
    robustifier = Robustifier()
    rospy.loginfo('Robustifier started')
    rospy.spin()