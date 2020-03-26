#!/usr/bin/env python

from __future__ import print_function
import rospy

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from move_base_msgs.msg import MoveBaseActionFeedback, MoveBaseActionGoal, MoveBaseGoal
from actionlib_msgs.msg import GoalID, GoalStatusArray

# Global goal status variable that holds current status
current_goal_status = None

def position_to_pose(location):
  goal = PoseStamped()
  goal.header.frame_id = 'map'
  goal.header.stamp = rospy.Time.now()
  goal.pose.position.x = location['x']
  goal.pose.position.y = location['y']
  goal.pose.orientation.w = 1.0
  return goal

def position_to_action_goal(position, goal_id):
  action_goal = MoveBaseActionGoal()

  action_goal.header = Header()
  action_goal.header.frame_id = 'map'
  action_goal.header.stamp = rospy.Time.now()

  action_goal.goal_id = GoalID()
  action_goal.goal_id.stamp = rospy.Time.now()
  action_goal.goal_id.id = str(goal_id)

  action_goal.goal = MoveBaseGoal()
  action_goal.goal.target_pose = position_to_pose(position)

  return action_goal

def goal_status(data):
  # More information about goal status can be found on the next url
  # http://docs.ros.org/fuerte/api/move_base_msgs/html/msg/MoveBaseActionResult.html
  global current_goal_status
  if len(data.status_list) <= 0:
    return
  current_goal_status = data.status_list[-1]

def initialize_node(goals):
  global current_goal_status
  # The node must first be initialized
  rospy.init_node('move')

  # Create a new rate with frequency of 2Hz
  rate = rospy.Rate(0.5)
  wait_after_publish_rate = rospy.Rate(0.5)

  # Create a new publisher, that will be used to publish new goals
  # Note that current_goal will be remapped using the launch file
  # Also note that message type should be geometry_msgs/PoseStamped
  # (we can use rostopic type /move_base_simple/goal to figure that out)
  publisher = rospy.Publisher('current_goal', MoveBaseActionGoal, queue_size = 10)

  # Create a new subscriber that will listen for goal changes
  subscriber = rospy.Subscriber('/move_base/status', GoalStatusArray, goal_status)

  current_goal_index = -1

  while not rospy.is_shutdown():
    if current_goal_status == None:
      rospy.loginfo('None')
      # The server has not yet provided any information, wait
      rate.sleep()
      continue
    
    # Here, the current_goal_status variable is available
    # Status number 3 means that the previous goal was reached
    if current_goal_status.status == 3:
      rospy.loginfo('Previous goal has been reached')
      # Send a new goal to server
      current_goal_index += 1
      if current_goal_index >= len(goals):
        return
      next_goal = goals[current_goal_index]
      action_goal = position_to_action_goal(next_goal, current_goal_index + 1)
      publisher.publish(action_goal)
      wait_after_publish_rate.sleep()
      current_goal_status = None

    rate.sleep()

if __name__ == '__main__':
  # Create a queue of hardcoded locations on map
  goals = [
    {"x": -1.39999980927, "y": 0.450000572205},
    {"x": 1.350000190734864, "y": 1.1500003814697272},
  ]

  initialize_node(goals)
