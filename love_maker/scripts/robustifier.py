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

class Robustifier(object):

    def __init__(self):
        # Initialize node, don't allow running multiple nodes of this type
        rospy.init_node('robustifier', anonymous=False)

        # This parameter tells us how far two face detections have to be to be
        # considered as different positions
        self.maximum_distance = 0.5

        # This parameter tells us how many detections is needed to proclaim a
        # face detection true positive
        self.minimum_detections = 7

        # The face detector publishes faces to /face_detections_raw, so create a
        # subscriber that we will use to read positions
        self.raw_face_subscriber = rospy.Subscriber('/face_detections_raw', PoseStamped, self.on_face_detection, queue_size=10)
        
        # After the robistufier determines face position, it should send a pose
        # to movement_controller node
        self.face_publisher = rospy.Publisher('/face_detections', PoseStamped, queue_size=10)

        # TODO: Instead of using a list use a quadtree or a grid
        # A list of Detection objects used for finding the closest pose
        self.face_detections = []
        self.true_positives = []

        # TODO: Define appropriate structures for storing the received poses and
        self.markers_publisher = rospy.Publisher('face_markers_raw', MarkerArray, queue_size=1000)
        self.marker_array = MarkerArray()
        self.marker_number = 1

        # Object we use for transforming between coordinate frames
        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf)
    
    def construct_marker(self, detection, color, scale=0.1):
        self.marker_number += 1
        marker = Marker()
        marker.header.stamp = detection.header.stamp
        marker.header.frame_id = detection.header.frame_id # camera_depth_optical_frame
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
                detection.stamped_pose.header.stamp = face_pose.header.stamp # one solution to "locatioin too far in history"
                return detection

    def marker_already_set(self, marker):
        for true_positive in self.true_positives:
            distance = true_positive.distance_to(marker)
            if distance <= self.maximum_distance:
                return true_positive
    
    def publish_marker(self, pose, color=ColorRGBA(1, 0, 0, 1), scale=0.1):
        # Send a list of all markers to visualize data
        marker = self.construct_marker(pose, color, scale)
        self.marker_array.markers.append(marker)
        self.markers_publisher.publish(self.marker_array)

    def convert_to_approaching_point(self, pose_in):
        # Construct face point
        face_point = PointStamped()
        face_point.header.frame_id = pose_in.header.frame_id
        face_point.header.stamp = pose_in.header.stamp
        face_point.point.x = pose_in.pose.position.x
        face_point.point.y = pose_in.pose.position.y
        face_point.point.z = pose_in.pose.position.z

        # Construct robot point
        robot_point = PointStamped()
        robot_point.header.frame_id = pose_in.header.frame_id
        robot_point.header.stamp = pose_in.header.stamp
        robot_point.point.x = 0
        robot_point.point.y = 0
        robot_point.point.z = 0

        # Transform face's and robot's position to global coordinates
        global_face_point = self.tf_buf.transform(face_point, "map")
        global_robot_point = self.tf_buf.transform(robot_point, "map")

        # Calculate orientation vector
        global_position_face = global_face_point.point
        global_position_robot = global_robot_point.point
        orientation_parameters = []
        orientation_parameters.append(global_position_robot.x - global_position_face.x)
        orientation_parameters.append(global_position_robot.y - global_position_face.y)
        orientation_parameters.append(0.002472)

        # Calculate orientation
        orientation = math.atan2(orientation_parameters[0], orientation_parameters[1])
        orientation_quaternion = quaternion_from_euler(0, 0, orientation)
        orientation_quaternion = Quaternion(orientation_quaternion[0], orientation_quaternion[1], orientation_quaternion[2], orientation_quaternion[3])

        # Normalize orientation vector
        normalized_orientation = orientation_parameters / np.linalg.norm(orientation_parameters)
        
        # Calculate approaching points
        approaching_point = PointStamped()
        approaching_point.header = global_face_point.header
        approaching_point.point.x = global_position_face.x + 0.5 * normalized_orientation[0]
        approaching_point.point.y = global_position_face.y + 0.5 * normalized_orientation[1]
        approaching_point.point.z = global_position_face.z + 0.5 * normalized_orientation[2]

        # Transform back to frame_id = camera_depth_optical_frames
        approaching_marker_point = self.tf_buf.transform(approaching_point, "camera_depth_optical_frame")

        # Construct approaching point pose
        approaching_pose = PoseStamped()
        approaching_pose.header = global_face_point.header
        approaching_pose.pose.position.x = approaching_point.point.x
        approaching_pose.pose.position.y = approaching_point.point.y
        approaching_pose.pose.position.z = approaching_point.point.z
        approaching_pose.pose.orientation = orientation_quaternion

        # Construct approaching point marker pose
        approaching_marker = PoseStamped()
        approaching_marker.header = approaching_marker_point.header
        approaching_marker.pose.position.x = approaching_marker_point.point.x
        approaching_marker.pose.position.y = approaching_marker_point.point.y
        approaching_marker.pose.position.z = approaching_marker_point.point.z

        return approaching_pose, approaching_marker
    
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

            # Another solution to "locatioin too far in history":
            # set face_to_approach = face_pose
            # in self.publish_marker(saved_pose...) swap saved_pose.stamped_pose for face_pose

            # If face was detected more than self.minimum_detections, it is now
            # considered true positive, send it to movement controller

            # TODO: Memorize the green markers (true positives) as the second layer of preventing false
            # positives. Often we get more than one green marker per photo. Before publishing, check wheteher
            # a green marker has already been published nearby (0.5m)
            if saved_pose.number_of_detections >= self.minimum_detections:
                if not saved_pose.already_sent:
                    # marker_already_sent = self.marker_already_set(saved_pose)
                    # if marker_already_sent is not None:
                    # Move the point 0.5m infront of the face so the robot doesn't bump into it when approaching
                    face_to_approach = saved_pose.stamped_pose
                    approaching_point, approaching_marker = self.convert_to_approaching_point(face_to_approach)
                    rospy.loginfo('Sending face location to movement controller')
                    self.face_publisher.publish(approaching_point)
                    # publish marker for confirmed true positive
                    self.publish_marker(saved_pose.stamped_pose, ColorRGBA(0, 1, 0, 1), 0.3)
                    
                    # publish marker for approaching point
                    self.publish_marker(approaching_marker, ColorRGBA(0, 0, 1, 1), 0.1)
                    saved_pose.already_sent = True
                    # rospy.logwarn("Green marker: {}".format(saved_pose.stamped_pose))
                    self.true_positives.append(saved_pose) # add to list
            else:
                rospy.loginfo('Detection has not yet surpassed the minimum number of detections needed')
                # The detected face pose and previously saved pose are very similar,
                # calculate mean position of both poses and save data to saved_pose
                # saved_pose.update(face_pose)
        else:
            rospy.loginfo('Face is probably new.')
            # Construct a new detection object, add it to detections
            saved_pose = Detection(face_pose)
            self.face_detections.append(saved_pose)

if __name__ == '__main__':
    robustifier = Robustifier()
    rospy.loginfo('Robustifier started')
    rospy.spin()