#!/usr/bin/env python
from __future__ import print_function

import rospy
import actionlib
import sound_play
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient
from geometry_msgs.msg import Pose, PoseStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from std_msgs.msg import String

# Require python module for ordered heap
import heapq

from random import randint
from tf.transformations import euler_from_quaternion
import math
import time

class MovementController(object):

    def __init__(self):
        # Initialize node, don't allow running multiple nodes of this type
        rospy.init_node('movement_controller', anonymous=False)

        # Create a Greeter object that controls speech synthetisation
        self.greeter = Greeter()
        self.greeter.say("I am very stupid!")

        # Create a new simple action client that will connect to move_base topic
        # The action server will listen on /move_base/goal and will notify
        # us about status and provide feedback at /move_base/status
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        rospy.loginfo('Connected to movement server')        

        # The robustifier node publishes faces to /face_detections
        self.face_subscriber = rospy.Subscriber('/face_detections', PoseStamped, self.on_face_detection, queue_size=10)
        
        # TODO: If we want the heap to be sorted on euclidian distance to the goal,
        # the priority parameter should be changed to distance. heapq always sorts by the first element in touple.
        # A list of faces to visit (actually an ordered heap of tuples (priority, pose))
        # https://docs.python.org/2/library/heapq.html

        # Initial goals for space exploration
        goals = [
            # my_map
            # (100, pose_from_point_on_map([0.204939, -1.357251, 0.002472])),
            # (101, pose_from_point_on_map([0.921944, -0.779827, 0.002472])),
            # (102, pose_from_point_on_map([1.579607, -0.200512, 0.002472])),
            # (103, pose_from_point_on_map([1.829951, 0.612022, 0.002472])),
            # (104, pose_from_point_on_map([1.162717, 1.044056, 0.002472])),
            # (105, pose_from_point_on_map([0.462207, 0.501959, 0.002472])),
            # (106, pose_from_point_on_map([-0.063535, 0.076191, 0.002472])),
            # (107, pose_from_point_on_map([-0.757831, -0.17621, 0.002472])),
            # (108, pose_from_point_on_map([-1.337893, 0.387238, 0.002472])),
            # (109, pose_from_point_on_map([-0.17028, -0.889452, 0.002472]))
            # my_map1
            (100, pose_from_point_on_map([2, 2, 0.002472]), False),
            (101, pose_from_point_on_map([3, 1.4, 0.002472]), False),
            (102, pose_from_point_on_map([4, 1.4, 0.002472]), False),
            (103, pose_from_point_on_map([4.5, 2, 0.002472]), False),
            (104, pose_from_point_on_map([4.2, 1.5, 0.002472]), False),
            (105, pose_from_point_on_map([4, 2.4, 0.002472]), False),
            (106, pose_from_point_on_map([3.5, 2.7, 0.002472]), False),
            (107, pose_from_point_on_map([3, 2.5, 0.002472]), False),
            (108, pose_from_point_on_map([2.1, 2.5, 0.002472]), False),
            (109, pose_from_point_on_map([2, 4, 0.002472]), False),
            (110, pose_from_point_on_map([2, 2.5, 0.002472]), False),
            (111, pose_from_point_on_map([2, 2.6, 0.002472]), False)

            # (100, pose_from_point_on_map([2, 3.5, 0.002472]), False),
            # (101, pose_from_point_on_map([4.5, 2, 0.002472]), False),
            # (102, pose_from_point_on_map([2, 1, 0.002472]), False)
        ]
        self.initial_goals = goals
        self.goals = goals
        heapq.heapify(goals)

        # The has_goals variable should be True if the bot is currently moving
        # and its heap is not empty
        self.has_goals = False

        # Counter of faces. If we run out of goals but haven't found all of the faces yet,
        # the robot hould explore some more
        self.number_of_detected_faces = 0

        # Number of faces that have to be detected
        self.number_of_faces_in_the_world = 6

        # The goals will be added to priority heap with decreasing priority. The
        # hardcoded goals should have a high priority, so that after new goal is
        # added, we first visit the goal and then the hardcoded location
        self.current_goal_priority = 0

        # When this node finishes intializing itself, it should first try to
        # localize itself, so it knows where it is
        self.is_localized = False
        self.localize()
    
    def start(self):
        # When the start function is called, the robot should stop
        # whatever it was doing and start to visit all the goals in
        # the self.goals list again
        # When self.move_to_goal function is called, the previous
        # goal is simply forgotten
        if len(self.goals) <= 0:
            return
        
        self.greeter.say("Hello, my name is Dora the explorer and I am ready to begin my mission!")
        self.has_goals = True
        first_goal = heapq.heappop(self.goals)
        rospy.loginfo('Robot has started to move to its first goal: {}'.format(first_goal))
        self.move_to_goal(first_goal)
    
    def done(self, status, result):
        if self.current_goal is not None:
            priority, goal, is_face = self.current_goal

            # If the approaching point was reached succesfully, greet.
            if status == 3 and is_face:
                self.greeter.greet()
                self.number_of_detected_faces += 1
                rospy.sleep(1)
                #time.sleep(1)

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
            # Goal unreachable, add it back to queue, but with higher priority
            rospy.logerr('The robot could not move to the goal')
            if is_face:
                heapq.heappush(self.goals, (self.current_goal_priority + 111, self.current_goal[1], True))
            else:
                heapq.heappush(self.goals, (self.current_goal_priority + 161, self.current_goal[1], False))


        # reset current goal
        self.current_goal = None
        
        # If there are no more goals do nothing, otherwise move to next goal
        if self.number_of_detected_faces >= self.number_of_faces_in_the_world:
            self.has_goals = False
            self.greeter.say("My work here is done. I have found all the people.")
            return

        if len(self.goals) <= 0:
            if self.number_of_detected_faces < self.number_of_faces_in_the_world:
                # not all faces have been approached yet, add new goals to the map
                for goal in self.initial_goals:
                    heapq.heappush(self.goals, goal)
            else:
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
        priority, pose, is_face = element
        rospy.loginfo('Moving to goal [priority = {}] {}'.format(priority, pose))
        # Create a new MoveBaseGoal object and set its position and rotation
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()

        # Set the goal pose as the face pose
        goal.target_pose.pose = pose.pose

        # Set the goal as current goal
        self.current_goal = element

        self.has_goals = True

        # Send the MoveBaseGoal to our client and wait for server response
        # Also setup three different callbacks:
        #   * self.done is called when current goal has been reached
        #   * self.active is called when server is processing our current goal
        #   * self.feedback is called to notify us about our robot position
        self.client.send_goal(goal, done_cb=self.done, active_cb=self.active, feedback_cb=self.feedback)
    
    def on_odometry_received(self, odometry):
        # This function will be called while the robot is localizing itself and
        # is receiving odometry information about its position and rotation
        # rospy.loginfo(odometry)
        if self.is_localized:
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

    def localize(self):
        rospy.loginfo('Started localization protocol')
        self.localization_publisher = rospy.Publisher('/navigation_velocity_smoother/raw_cmd_vel', Twist, queue_size = 1000)
        self.odometry_subscriber = rospy.Subscriber('/odom', Odometry, self.on_odometry_received, queue_size=10)
        
    def localization_finished(self):
        self.is_localized = True
        rospy.loginfo('Localization protocol finished')
        # After turtlebot has localized itself, start moving to goals in the self.faces heap
        self.start()
    
    def on_face_detection(self, face_pose):
        # rospy.loginfo('A new robustified face location found: {}'.format(face_pose))
        # Add received pose to the heap with priority 1
        self.current_goal_priority += 1
        heapq.heappush(self.goals, (self.current_goal_priority, face_pose, True))
        rospy.loginfo('New face received, there are currently {} faces in heap'.format(len(self.goals)))

        if self.is_localized and not self.has_goals:
            self.start()
        
def pose_from_point_on_map(point):
    pose = PoseStamped()
    pose.pose = Pose()
    pose.pose.position.x = point[0]
    pose.pose.position.y = point[1]
    pose.pose.position.z = point[2]
    pose.pose.orientation.w = 1
    return pose

# Object that abstracts voice commands
class Greeter(object):

    greetings = [
        "Hello there!", "How are you doing?", "Oh, hi", "Good day", "Hello"
    ]

    def __init__(self):
        self.voice = 'voice_kal_diphone'
        self.volume = 1.0
        self.client = SoundClient()
        rospy.loginfo("Greeter created!")
    
    def say(self, data):
        # Send data to the client, client outputs the data with given voice and volume
        rospy.loginfo("Send to sound_client: " + data)
        self.client.say(data, self.voice, self.volume)
    
    def greet(self, greeting = None):
        if greeting is None:
            random_greeting = self.greetings[randint(0, len(self.greetings) - 1)]
            self.say(random_greeting)
        else:
            self.say(self.greetings[greeting])


if __name__ == '__main__':
    controller = MovementController()
    rospy.loginfo('Movement controller started')
    rospy.spin()