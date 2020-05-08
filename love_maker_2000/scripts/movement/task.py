#!/usr/bin/env python
from __future__ import print_function

import rospy
import math, random, time, numpy

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, PoseStamped, Twist, Quaternion
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# This is the object that is used to compute which points on map are worth
# visiting. The real life analogy is a guidebook, but instead of good places to
# find wine, this MapMaker returns points that cover the most of the map.
# If the tour guide did the same the trip would not be short, much less fun.
from map_maker import MapMaker

import utils

class MovementTask(object):

    def __init__(self, movement_controller, callback):
        self.movement_controller = movement_controller
        self.callback = callback
        self.is_finished = False
        self.has_started = False
        self.is_cancelled = False
    
    def run(self):
        self.has_started = True
        self.is_cancelled = False
        self.finish()
    
    def finish(self):
        self.is_finished = True

        # Call the callback function
        if self.callback is not None:
            self.callback()

        self.movement_controller.on_finish(self)
    
    def cancel(self):
        self.is_cancelled = True
        self.movement_controller.cancel(self)

class ApproachingTask(MovementTask):

    def __init__(self, movement_controller, callback, action_client, object_detection):
        super(ApproachingTask, self).__init__(movement_controller, callback)
        self.action_client = action_client
        self.object_detection = object_detection
    
    def feedback(self, data):
        pass
    
    def active(self):
        pass
    
    def done(self, status, result):
        # This is called when the robot has reached our current goal
        # or something has gone wrong
        self.finish(status, result)
    
    def run(self):
        # Create a new MoveBaseGoal object and set its position and rotation
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.object_detection.header.frame_id
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = self.object_detection.approaching_point_pose

        # Send the MoveBaseGoal to our client and wait for server response
        # Also setup three different callbacks:
        #   * self.done is called when current goal has been reached
        #   * self.active is called when server is processing our current goal
        #   * self.feedback is called to notify us about our robot position
        self.action_client.send_goal(goal, done_cb=self.done, active_cb=self.active, feedback_cb=self.feedback)
    
    def finish(self, goal_status, goal_result):
        self.is_finished = True

        # Call the callback function
        if self.callback is not None:
            self.callback(self.object_detection, goal_status, goal_result)

        self.movement_controller.on_finish(self)

    def cancel(self):
        self.action_client.stop_tracking_goal()
        self.action_client.cancel_all_goals()
        super(ApproachingTask, self).cancel()

class WanderingTask(MovementTask):
    
    def __init__(self, movement_controller, callback, action_client):
        super(WanderingTask, self).__init__(movement_controller, callback)
        self.action_client = action_client

        self.goals = []

        # Use MapMaker to generate exploration points
        map_maker = MapMaker()
        exploration_points = map_maker.generate_points()
        for point in exploration_points:
            self.goals.append(utils.detection_from_point_on_map([point[0], point[1], 0], -2.578))
    
    def feedback(self, data):
        pass
    
    def active(self):
        pass
    
    def done(self, status, result):
        # This is called when the robot has reached our current goal
        # or something has gone wrong
        self.run()
    
    def run(self):
        if len(self.goals) <= 0:
            self.finish()
            return
        
        self.current_goal = self.goals.pop(0)

        # rospy.loginfo('Moving to goal [type = {}] {}'.format(detection.type, detection.approaching_point_pose))
        # Create a new MoveBaseGoal object and set its position and rotation
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.current_goal.header.frame_id
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = self.current_goal.approaching_point_pose

        # Send the MoveBaseGoal to our client and wait for server response
        # Also setup three different callbacks:
        #   * self.done is called when current goal has been reached
        #   * self.active is called when server is processing our current goal
        #   * self.feedback is called to notify us about our robot position
        self.action_client.send_goal(goal, done_cb=self.done, active_cb=self.active, feedback_cb=self.feedback)
    
    def cancel(self):
        self.goals.insert(0, self.current_goal)
        self.action_client.stop_tracking_goal()
        self.action_client.cancel_all_goals()
        super(WanderingTask, self).cancel()

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
        rospy.loginfo('Localization protocol finished')
        self.finish()
    
    def cancel(self):
        self.odometry_subscriber.unregister()
        super(LocalizationTask, self).cancel()