#!/usr/bin/env python
from __future__ import print_function

import rospy
import actionlib
from geometry_msgs.msg import Pose
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# Require python module for ordered heap
import heapq

class MovementController(object):

    def __init__(self, goals):
        # Initialize node, don't allow running multiple nodes of this type
        rospy.init_node('movement_controller', anonymous=False)

        # Create a new simple action client that will connect to move_base topic
        # The action server will listen on /mmove_base/goal and will notify
        # us about status and provide feedback at /move_base/status
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        rospy.loginfo('Connected to movement server')

        # The robustifier node publishes faces to /face_detections
        self.face_subscriber = rospy.Subscriber('/face_detections', Pose, self.on_face_detection, queue_size=10)

        # A list of faces to visit (actually an ordered heap of tuples (priority, pose))
        # https://docs.python.org/2/library/heapq.html
        # We will probably want to construct goals heap from list with this in the future
        # self.goals = heapq.heapify(goals)
        self.goals = []
        # A list of already visited faces (after we pop a face from visited faces, put it here)
        self.visited_goals = []
        # The has_goals variable should be True if the bot is currently moving
        # and its heap is not empty
        self.has_goals = False

        # When this node finishes intializing itself, it should first try to
        # localize itself, so it knows where it is
        self.localize()

        # After turtlebot has localized itself, start moving to goals in the self.faces heap
        self.start()
    
    def start(self):
        # When the start function is called, the robot should stop
        # whatever it was doing and start to visit all the goals in
        # the self.goals list again
        # When self.move_to_goal function is called, the previous
        # goal is simply forgotten
        if len(self.goals) <= 0:
            return
        
        self.has_goals = True
        first_goal = heapq.heappop(self.goals)
        rospy.loginfo('Robot has started to move to its first goal: {}'.format(first_goal))
        self.move_to_goal(first_goal)
    
    def done(self, status, result):
        # This is called when the robot has reached our current goal
        # or something has gone wrong
        rospy.loginfo('Goal has finished with status: {}'.format(status))

        # This bit was added, because the callbacks were only called on the first
        # requested goal, then the robot moves to the second position, the server
        # does notify our client about it, but the callback function is never called
        # Tell the client to stop tracking current goal and cancel all previous goals
        self.client.stop_tracking_goal()
        self.client.cancel_all_goals()

        # To get more information about goal status see
        # http://docs.ros.org/fuerte/api/actionlib_msgs/html/msg/GoalStatus.html
        # Status with number 3 means that robot has reached the goal
        if status == 3:
            rospy.loginfo('Goal has been reached successfully')
        else:
            # TODO: Goal unreachable, add it back to queue, but with higher priority
            rospy.logerr('The robot could not move to the goal')
        
        # If there are no more goals do nothing, otherwise move to next goal
        if len(self.goals) <= 0:
            self.has_goals = False
            return

        # Pop the item with the smallest priority from goals and move there
        next_goal = heapq.heappop(self.goals)
        self.move_to_goal(next_goal)
    
    def feedback(self, data):
        # This is called repeatedly and tells us the robot position,
        # we won't have this data in our real robot, so pretend it
        # doesn't exist
        pass
    
    def active(self):
        # This is called when our goal is being processed on the server
        pass
    
    def move_to_goal(self, element):
        # Get the priority and element from heap
        priority, pose = element
        rospy.loginfo('Moving to goal [priority = {}] {}'.format(priority, pose))
        # Create a new MoveBaseGoal object and set its position and rotation
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()

        # Set the goal pose as the face pose
        goal.target_pose.pose = pose

        self.has_goals = True

        # Send the MoveBaseGoal to our client and wait for server response
        # Also setup three different callbacks:
        #   * self.done is called when current goal has been reached
        #   * self.active is called when server is processing our current goal
        #   * self.feedback is called to notify us about our robot position
        self.client.send_goal(goal, done_cb=self.done, active_cb=self.active, feedback_cb=self.feedback)
    
    def localize(self):
        rospy.loginfo('Started localization protocol')
    
    def on_face_detection(self, face_pose):
        rospy.loginfo('A new robustified face location found: {}'.format(face_pose))
        # Add received pose to the heap with priority 1
        priority = 1
        heapq.heappush(self.goals, (priority, face_pose))

        if not self.has_goals:
            self.start()
        

if __name__ == '__main__':
    controller = MovementController([])
    rospy.loginfo('Movement controller started')
    rospy.spin()