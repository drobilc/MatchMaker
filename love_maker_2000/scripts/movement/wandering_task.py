#!/usr/bin/env python

from __future__ import print_function
import rospy

from task import MovementTask

from move_base_msgs.msg import MoveBaseGoal

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
        if hasattr(self, 'current_goal'):
            self.goals.insert(0, self.current_goal)
            self.current_goal = None
        self.action_client.stop_tracking_goal()
        self.action_client.cancel_all_goals()
        super(WanderingTask, self).cancel()
    
    def finish(self):
        super(WanderingTask, self).finish()
    
    def __str__(self):
        return '<WanderingTask, goals={}>'.format(len(self.goals))