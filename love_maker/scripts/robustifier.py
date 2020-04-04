#!/usr/bin/env python
from __future__ import print_function

import rospy
import math
from geometry_msgs.msg import Pose, PoseStamped, Vector3
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA

from tf.transformations import euler_from_quaternion

import numpy as np

class Detection(object):
    
    def __init__(self, pose, number_of_detections = 0):
        self.pose = pose.pose
        self.stamped_pose = pose
        self.number_of_detections = number_of_detections
        self.already_sent = False

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

    def __init__(self):
        # Initialize node, don't allow running multiple nodes of this type
        rospy.init_node('robustifier', anonymous=False)

        # This parameter tells us how far two face detections have to be to be
        # considered as different positions
        self.maximum_distance = 0.5

        # This parameter tells us how many detections is needed to proclaim a
        # face detection true positive
        self.minimum_detections = 12

        # The face detector publishes faces to /face_detections_raw, so create a
        # subscriber that we will use to read positions
        self.raw_face_subscriber = rospy.Subscriber('/face_detections_raw', PoseStamped, self.on_face_detection, queue_size=10)
        
        # After the robistufier determines face position, it should send a pose
        # to movement_controller node
        self.face_publisher = rospy.Publisher('/face_detections', PoseStamped, queue_size=10)

        # TODO: Instead of using a list use a quadtree or a grid
        # A list of Detection objects used for finding the closest pose
        self.face_detections = []

        # TODO: Define appropriate structures for storing the received poses and
        self.markers_publisher = rospy.Publisher('face_markers_raw', MarkerArray, queue_size=1000)
        self.marker_array = MarkerArray()
        self.marker_number = 1
    
    def construct_marker(self, detection, color, scale=0.1):
        self.marker_number += 1
        marker = Marker()
        marker.header.stamp = detection.header.stamp
        marker.header.frame_id = 'map'
        marker.pose = detection.pose
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.frame_locked = False
        marker.lifetime = rospy.Duration.from_sec(10)
        marker.id = self.marker_number
        marker.scale = Vector3(scale, scale, scale)
        marker.color = color
        
        return marker

    def already_detected(self, face_pose):
        for detection in self.face_detections:
            distance = detection.distance_to(face_pose)
            if distance <= self.maximum_distance:
                return detection
    
    def publish_marker(self, pose, color=ColorRGBA(1, 0, 0, 1), scale=0.1):
        # Send a list of all markers to visualize data
        marker = self.construct_marker(pose, color, scale)
        self.marker_array.markers.append(marker)
        self.markers_publisher.publish(self.marker_array)

    def convert_to_approaching_point(self, pose):
        # constructs an approaching point which is the point where our robot
        # should greet the face from. The orientation is set as current robot's rotation
        # since we want it to face the face :D
        coordinates = (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)
        orientation_parameters = (pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w) # robot's orientation
        orientation = map(float, euler_from_quaternion(orientation_parameters))

        orientation[0] = 0.5 * orientation[0]
        orientation[1] = 0.5 * orientation[1]
        orientation[2] = 0.5 * orientation[2]

        approaching_point_coordinates = []
        approaching_point_coordinates.append(coordinates[0] + orientation[0])
        approaching_point_coordinates.append(coordinates[1] + orientation[1])
        approaching_point_coordinates.append(coordinates[2] + orientation[2])

        # set coordinates
        approaching_point = PoseStamped()
        approaching_point.pose.position.x = approaching_point_coordinates[0]
        approaching_point.pose.position.y = approaching_point_coordinates[1]
        approaching_point.pose.position.z = approaching_point_coordinates[2]
        approaching_point.pose.orientation.x = orientation_parameters[0]
        approaching_point.pose.orientation.y = orientation_parameters[1]
        approaching_point.pose.orientation.z = orientation_parameters[2]
        approaching_point.pose.orientation.w = orientation_parameters[3]
        approaching_point.header = pose.header
        rospy.logwarn("Converting to approaching point: {}".format(approaching_point))
        return approaching_point
    
    def on_face_detection(self, face_pose):
        # A new face has been detected, robustify it
        # rospy.loginfo('A new face pose received: {}'.format(face_pose))
        self.publish_marker(face_pose)

        # Check if detected face is already in face_detections. This cannot be
        # done with simple indexof function because the coordinates will not
        # exactly match
        saved_pose = self.already_detected(face_pose)
        if saved_pose is not None:
            rospy.loginfo('Face was already detected, it has been detected {} times'.format(saved_pose.number_of_detections))
            saved_pose.number_of_detections += 1

            # If face was detected more than self.minimum_detections, it is now
            # considered true positive, send it to movement controller
            if saved_pose.number_of_detections >= self.minimum_detections:
                if not saved_pose.already_sent:
                    # Move the point 0.5m infront of the face so the robot doesn't bump into it when approaching
                    face_to_approach = saved_pose.stamped_pose
                    approaching_point = self.convert_to_approaching_point(face_to_approach)
                    rospy.loginfo('Sending face location to movement controller')
                    self.face_publisher.publish(approaching_point)
                    self.publish_marker(saved_pose.stamped_pose, ColorRGBA(0, 1, 0, 1), 0.3)
                    saved_pose.already_sent = True
            else:
                rospy.loginfo('Detection has not yet surpassed the minimum number of detections needed')
                # The detected face pose and previously saved pose are very similar,
                # calculate mean position of both poses and save data to saved_pose
                saved_pose.update(face_pose)
        else:
            rospy.loginfo('Face is probably new.')
            # Construct a new detection object, add it to detections
            saved_pose = Detection(face_pose)
            self.face_detections.append(saved_pose)

if __name__ == '__main__':
    robustifier = Robustifier()
    rospy.loginfo('Robustifier started')
    rospy.spin()