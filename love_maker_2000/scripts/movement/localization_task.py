#!/usr/bin/env python

from __future__ import print_function
import rospy
import time, math

from task import MovementTask

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from tf.transformations import euler_from_quaternion

class LocalizationTask(MovementTask):
    
    def __init__(self, movement_controller, callback):
        super(LocalizationTask, self).__init__(movement_controller, callback)
    
    def run(self):
        self.localization_publisher = rospy.Publisher('/navigation_velocity_smoother/raw_cmd_vel', Twist, queue_size = 1000)
        self.odometry_subscriber = rospy.Subscriber('/odom', Odometry, self.on_odometry_received, queue_size=10)
        self.is_cancelled = False
    def on_odometry_received(self, odometry):
        if self.is_finished or self.is_cancelled:
            return
        
        # If the object has no last_message_sent attribute, then this is the
        # first odometry message received. Also save starting angle so we know
        # when we have done a full circle rotation. 
        if not hasattr(self, "last_message_sent"):
            self.last_message_sent = time.time()
            quaternion = odometry.pose.pose.orientation
            quaternion_as_list = (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
            self.starting_rotation = euler_from_quaternion(quaternion_as_list)
            self.previous_rotation = self.starting_rotation
        
        # Send a twist message to robot every 0.5 seconds
        current_time = time.time()
        if current_time - self.last_message_sent >= 0.5:
            quaternion = odometry.pose.pose.orientation
            current_rotation = euler_from_quaternion((quaternion.x, quaternion.y, quaternion.z, quaternion.w))

            if hasattr(self, 'previous_rotation') and self.previous_rotation[2] < self.starting_rotation[2] and current_rotation[2] > self.starting_rotation[2]:
                # Stop subscribing to odometry data so this function is not called again
                self.odometry_subscriber.unregister()
                # Call the callback function that tells us that robot has ended localization
                self.localization_finished()
                return

            # Otherwise rotate the robot for 60 degrees
            twist = Twist()
            twist.angular.z = 60 * (3.14 / 180.0) # 60 deg
            self.localization_publisher.publish(twist)
            
            self.last_message_sent = current_time
            # The previous rotation should only be updated if we have rotated for 30 degrees
            if abs(current_rotation[2] - self.previous_rotation[2]) > math.pi / 6:
                self.previous_rotation = current_rotation
    
    def localization_finished(self):
        self.finish()
    
    def finish(self):
        super(LocalizationTask, self).finish()
    
    def cancel(self):
        self.odometry_subscriber.unregister()
        super(LocalizationTask, self).cancel()
    
    def __str__(self):
        return '<LocalizationTask>'