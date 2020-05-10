#!/usr/bin/env python

from __future__ import print_function
import rospy

from task import MovementTask

from move_base_msgs.msg import MoveBaseGoal

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