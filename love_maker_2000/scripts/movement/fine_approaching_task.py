#!/usr/bin/env python

from __future__ import print_function
import rospy
import numpy
import math

from task import MovementTask

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Quaternion, Vector3
from visualization_msgs.msg import Marker, MarkerArray

import tf2_ros
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from tf2_geometry_msgs import PointStamped

import utils

class FineApproachingTask(MovementTask):

    def __init__(self, movement_controller, callback, object_detection, goal):
        super(FineApproachingTask, self).__init__(movement_controller, callback)
        self.object_detection = object_detection
        self.goal = goal

        self.almost_there = False

        # Transformation buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def on_odometry_received(self, odometry, angle_resolution=8):
        """Callback for when the odometry data is received. Move the robot closer to goal."""
        if self.is_finished or self.is_cancelled:
            return
        
        # First, convert the received odometry pose to map coordinate frame,
        # otherwise the robot position is not accurate
        pose_stamped = PoseStamped()
        pose_stamped.header = odometry.header
        pose_stamped.pose = odometry.pose.pose
        try:
            map_position = self.tf_buffer.transform(pose_stamped, 'map')
        except Exception:
            rospy.logwarn('Can not transform received odometry to map coordinate frame')
            return

        # Visit the current goal using twist messages
        self.move_towards_goal(map_position, self.goal)
    
    def move_towards_goal(self, map_position, current_goal, rotation_threshold=0.08, distance_threshold=0.1):  # 0.8 -> 15 degrees
        """Move the robot to current_goal using twist messages"""
        # First, get the positions of the robot in map coordinate frame
        robot_position = numpy.asarray([map_position.pose.position.x, map_position.pose.position.y])
        goal_position = numpy.asarray([current_goal.position.x, current_goal.position.y])

        # Then compute current robot orientation and rotation from robot
        # position to the goal position
        current_orientation = (utils.orientation_to_angle(map_position.pose) + (2 * numpy.pi)) % (2 * numpy.pi)
        orientation = goal_position - robot_position
        orientation_to_goal = (numpy.arctan2(orientation[1], orientation[0]) + (2 * numpy.pi)) % (2 * numpy.pi)

        # Compute the distance between robot position and goal position
        distance_to_goal = numpy.linalg.norm(orientation)

        twist = Twist()
        # RABIMO MANJSI ANGULAR Z, DA SE POCASNEJE OBRACA
        if distance_to_goal <= distance_threshold:
            # Rotate until the robot is rotated in the detected object orientation    
            goal_orientation = (utils.orientation_to_angle(current_goal) + (2 * numpy.pi)) % (2 * numpy.pi)
            if abs(current_orientation - goal_orientation) <= rotation_threshold:
                # close enough and in the right direction
                rospy.logwarn("YAY, WE MADE IT!")
                self.velocity_publisher.publish(twist)
                self.almost_there = False
                self.finish()
                return
            else:
                # close enough but not rotated correctly
                if not self.almost_there:
                    rospy.logwarn("ALMOST THERE, JUST LET ME SPIN A BIT MORE")
                    self.almost_there = True
                twist.linear.x = 0
                twist.linear.y = 0
                twist.linear.z = 0
                twist.angular.z = (goal_orientation - current_orientation) * 0.5
                self.velocity_publisher.publish(twist)
        else:
            # not close enough, proceed forward
            rotation = orientation_to_goal - current_orientation
            rotation = rotation * 0.5 # times constant to make it better (presumably)
            if abs(rotation) > rotation_threshold:
                twist.angular.z = rotation * 0.5 * 2
                self.velocity_publisher.publish(twist)
            elif distance_to_goal > distance_threshold:
                twist.linear.x = 0.2
                self.velocity_publisher.publish(twist)
    
    def run(self):
        """Callback for when the fine approaching task should start"""
        self.velocity_publisher = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size = 10)
        self.odometry_subscriber = rospy.Subscriber('/odom', Odometry, self.on_odometry_received, queue_size=10)
    
    def finish(self):
        """Callback for when the fine approaching task should finish"""
        self.odometry_subscriber.unregister()

        self.is_finished = True
        rospy.logwarn("GOAL REACHED")

        # Call the callback function
        if self.callback is not None:
            self.callback(self.object_detection, 3, None)

        self.movement_controller.on_finish(self)
    
    def cancel(self):
        """Callback for when the fine approaching task should cancel"""
        self.odometry_subscriber.unregister()
        super(ApproachingTask, self).cancel()
    
    def __str__(self):
        return '<FineApproachingTask, color={}, type={}>'.format(self.object_detection.color, self.object_detection.type)