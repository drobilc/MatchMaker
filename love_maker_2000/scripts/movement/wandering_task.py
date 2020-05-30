#!/usr/bin/env python

from __future__ import print_function
import rospy

from task import MovementTask

from move_base_msgs.msg import MoveBaseGoal

import tf2_ros
# from tf2_geometry_msgs import PointStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

# This is the object that is used to compute which points on map are worth
# visiting. The real life analogy is a guidebook, but instead of good places to
# find wine, this MapMaker returns points that cover the most of the map.
# If the tour guide did the same the trip would not be short, much less fun.
from map_maker import MapMaker
import utils

class WanderingTask(MovementTask):
    
    def __init__(self, movement_controller, callback, action_client):
        super(WanderingTask, self).__init__(movement_controller, callback)
        self.action_client = action_client

        self.goals = []

        # Transformation buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.odometry_subscriber = rospy.Subscriber('/odom', Odometry, self.on_odometry_received, queue_size=10)
        self.current_robot_pose = None

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

    def on_odometry_received(self, odometry):
        # First, convert the received odometry pose to map coordinate frame,
        # otherwise the robot position is not accurate
        pose_stamped = PoseStamped()
        pose_stamped.header = odometry.header
        pose_stamped.pose = odometry.pose.pose
        try:
            self.current_robot_pose = self.tf_buffer.transform(pose_stamped, 'map')
        except Exception:
            self.current_robot_pose = None
    
    def get_closest_goal(self):
        # goals are ObjectDetection objects
        # their position can be obtained as detection.object_pose.position

        # Calculate Euclidian distance from robot_point to each goal
        min_distance = 100000
        closest_goal = None

        rospy.logwarn(self.current_robot_pose)

        if self.current_robot_pose == None:
            closest_goal = None
        
        for goal in self.goals:
            distance = (self.current_robot_pose.pose.position.x - goal.object_pose.position.x)**2 + (self.current_robot_pose.pose.position.y - goal.object_pose.position.y)**2
            if distance < min_distance:
                min_distance = distance
                closest_goal = goal
        
        return closest_goal
    
    def run(self):
        if len(self.goals) <= 0:
            self.finish()
            return

        closest_goal = self.get_closest_goal()

        if closest_goal == None:
            idx = 0
        else:
            idx = self.goals.index(closest_goal)

        self.current_goal = self.goals.pop(idx)

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
        if hasattr(self, 'current_goal'):
            if self.current_goal not in self.goals:
                if self.current_goal is not None:
                    self.goals.insert(0, self.current_goal)
            
            self.current_goal = None
        super(WanderingTask, self).cancel()
    
    def finish(self):
        if hasattr(self, 'current_goal'):
            if self.current_goal is not None:
                self.goals.insert(0, self.current_goal)
                self.current_goal = None
        self.action_client.stop_tracking_goal()
        self.action_client.cancel_all_goals()
        super(WanderingTask, self).finish()
    
    def __str__(self):
        return '<WanderingTask, goals={}>'.format(len(self.goals))