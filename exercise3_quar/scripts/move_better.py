#!/usr/bin/env python

import rospy
import actionlib
from geometry_msgs.msg import Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler

class Mind(object):
	
	def __init__(self, goals):
		# The list of goals that we want to visit
		self.goals = goals
		self.current_goal_index = 0

		# First, initialize connection to move_base server
		self.initialize_client()
	
	def initialize_client(self):
		# Create a new simple action client that will connect to move_base topic
		# The action server will listen on /mmove_base/goal and will notify
		# us about status and provide feedback at /move_base/status
		self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
		self.client.wait_for_server()
		rospy.loginfo('Connected to server')
	
	def start(self):
		# When the start function is called, the robot should stop
		# whatever it was doing and start to visit all the goals in
		# the self.goals list again
		# When self.move_to_goal function is called, the previous
		# goal is simply forgotten
		first_goal = self.goals[0]
		rospy.loginfo('Robot has started to move to its first goal: {}'.format(first_goal))
		self.move_to_goal(first_goal)
	
	def done(self, status, result):
		# This is called when the robot has reached our current goal
		# or something has gone wrong
		rospy.loginfo('Goal {} has finished with status: {}'.format(self.current_goal_index, status))

		# This bit was added, because the callbacks were only called on the first
		# requested goal, then the robot moves to the second position, the server
		# does notify our client about it, but the callback function is never called
		# TODO: Ask assistant what is wrong with this code
		# Tell the client to stop tracking current goal and cancel all previous goals
		self.client.stop_tracking_goal()
		self.client.cancel_all_goals()

		# To get more information about goal status see
		# http://docs.ros.org/fuerte/api/actionlib_msgs/html/msg/GoalStatus.html
		# Status with number 3 means that robot has reached the goal
		if status == 3:
			rospy.loginfo('Goal {} has been reached successfully'.format(self.current_goal_index))
		else:
			rospy.logerr('The robot could not move to the goal {}'.format(self.current_goal_index))
		
		# If there is another goal, try to visit it
		self.current_goal_index += 1
		if self.current_goal_index >= len(self.goals):
			rospy.signal_shutdown('All goals have been reached')
			return
		
		self.move_to_goal(self.goals[self.current_goal_index])
	
	def feedback(self, data):
		# This is called repeatedly and tells us the robot position,
		# we won't have this data in our real robot, so pretend it
		# doesn't exist
		pass
	
	def active(self):
		# This is called when our goal is being processed on the server
		rospy.loginfo('Goal {} is being processed by the server'.format(self.current_goal_index))
		pass

	def move_to_goal(self, goal_data):
		rospy.loginfo('Moving to goal {}'.format(self.current_goal_index))
		# Create a new MoveBaseGoal object and set its position and rotation
		goal = MoveBaseGoal()
		goal.target_pose.header.frame_id = 'map'
		goal.target_pose.header.stamp = rospy.Time.now()

		# Set the goal position x and y. (z is not needed because the ground is flat)
		goal.target_pose.pose.position.x = goal_data['position']['x']
		goal.target_pose.pose.position.y = goal_data['position']['y']

		# Convert goal rotation to quaternion
		rotation = quaternion_from_euler(0, 0, goal_data['rotation']['z'])
		goal.target_pose.pose.orientation = Quaternion(*rotation)

		# Send the MoveBaseGoal to our client and wait for server response
		# Also setup three different callbacks:
		#   * self.done is called when current goal has been reached
		#   * self.active is called when server is processing our current goal
		#   * self.feedback is called to notify us about our robot position
		self.client.send_goal(goal, done_cb=self.done, active_cb=self.active, feedback_cb=self.feedback)

if __name__ == '__main__':
	# Initialize our node, so rospy server know something is up
	rospy.init_node('move_better')

	# Create a new instance of Mind object that is used to move the robot
	# to new goals
	mind = Mind([
		{'position': {'x': -1.54771530628, 'y': 0.67709082365}, 'rotation': {'z': 2.3439462}},
		{'position': {'x': 0.145544111729, 'y': 0.538124322891}, 'rotation': {'z': 2.3439462}},
		{'position': {'x': 0.973760843277, 'y': 1.14928495884}, 'rotation': {'z': 2.3439462}},
		{'position': {'x': 1.57348775864, 'y': -0.612354278564}, 'rotation': {'z': -0.8456257}},
		{'position': {'x': 2.43584132195, 'y': 0.131776213646}, 'rotation': {'z': -0.8456257}},
	])

	# Start moving to our goals
	mind.start()

	rospy.spin()