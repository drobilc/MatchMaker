#!/usr/bin/env python
from __future__ import print_function

import rospy
import actionlib
from geometry_msgs.msg import Pose, PoseStamped, Twist, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from std_msgs.msg import String, ColorRGBA
from object_detection_msgs.msg import ObjectDetection

from visualization_msgs.msg import MarkerArray

# This is the object that is used to compute which points on map are worth
# visiting. The real life analogy is a guidebook, but instead of good places to
# find wine, this MapMaker returns points that cover the most of the map.
# If the tour guide did the same the trip would not be short, much less fun.
from map_maker import MapMaker

# To generate map markers, we can use util functions
import utils

# Greeter says whatever we tell it to say. Yes, even cursewords or sentences like "I'm stupid". He's basically a parrot.
import greeter

import tf2_ros
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf2_geometry_msgs
import math, random, time, numpy

class MovementController(object):

    GOAL_ORDERING = {
        'face': 0,
        'cylinder': 1000,
        'torus': 2000,
        'map_point': 3000
    }

    GOAL_EPSILON_DISTANCE = 0.3

    def __init__(self, goals):
        # Create a Greeter object that controls speech synthetisation
        self.greeter = greeter.Greeter()
        self.greetings = []
        self.face_greetings = [
            "Hello there!", "How are you doing?", "Oh, hi", "Good day", "Hello"
        ]
        self.color = ""
        self.object_type = ""
        self.cylinder_and_ring_greetings = [
            "What a beautiful {} {}!".format(self.color, self.object_type), 
            "Well, this is a nice {} {}.".format(self.color, self.object_type),
            "I see a {} {}.".format(self.color, self.object_type),
            "My senses tell me this is a {} {}.".format(self.color, self.object_type),
            "I declare this a {} {}.".format(self.color, self.object_type),
            "{} {}, not suprising.".format(self.color, self.object_type)
        ]

        # Create a new simple action client that will connect to move_base topic
        # The action server will listen on /move_base/goal and will notify
        # us about status and provide feedback at /move_base/status
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        rospy.loginfo('Connected to movement server')

        # To visualize goals, we create a new goal publisher that publishes markers
        self.goals_publisher = rospy.Publisher('goals', MarkerArray, queue_size=1000)

        # Save initial goals that we have received from map_maker
        self.initial_goals = [goal for goal in goals]

        # The goals list contains ObjectDetection detections
        self.goals = goals

        # The has_goals variable should be True if the bot is currently moving
        # and its heap is not empty
        self.has_goals = False

        self.send_marker_goals() 

        # Movement controller should accept faces, cylinders and toruses.
        # Robustifiers will send the same type of object (ObjectDetection), so
        # we can simply use on_object_detection function to get approaching
        # points.
        self.face_subscriber = rospy.Subscriber('/face_detections', ObjectDetection, self.on_object_detection, queue_size=10)
        self.cylinder_subscriber = rospy.Subscriber('/cylinder_detections', ObjectDetection, self.on_object_detection, queue_size=10)
        self.torus_subscriber = rospy.Subscriber('/torus_detections', ObjectDetection, self.on_object_detection, queue_size=10)    

        # When this node finishes intializing itself, it should first try to
        # localize itself, so it knows where it is
        self.is_localized = False
        self.localize()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
    
    def start(self):
        # When the start function is called, the robot should stop
        # whatever it was doing and start to visit all the goals in
        # the self.goals list again
        # When self.move_to_goal function is called, the previous
        # goal is simply forgotten
        if len(self.goals) <= 0:
            return
        
        self.send_marker_goals()        
        
        self.sort_goals()
        self.has_goals = True
        first_goal = self.goals.pop(0)
        rospy.loginfo('Robot has started to move to its first goal: {}'.format(first_goal))
        self.move_to_goal(first_goal)
    
    def current_robot_position(self):
        robot_position = tf2_geometry_msgs.PoseStamped()
        robot_position.header.frame_id = "camera_depth_optical_frame"
        robot_position.header.stamp = rospy.Time()
        robot_position = self.tf_buffer.transform(robot_position, "map")
        return robot_position

    def sort_goals(self):
        # Because we will probably have a small number of goals (very rarely
        # more than 20), we can use simple sorting function with time complexity
        # O(n logn). This function will also not be called very frequently, so
        # this approach is probably ok.
        robot_pose = self.current_robot_position()
        orientation = robot_pose.pose.orientation
        orientation = (orientation.x, orientation.y, orientation.z, orientation.w)
        robot_direction = euler_from_quaternion(orientation)

        def euclidean_distance(goal):
            dx = (goal.approaching_point_pose.position.x - robot_pose.pose.position.x)
            dy = (goal.approaching_point_pose.position.y - robot_pose.pose.position.y)
            return math.sqrt(dx * dx + dy * dy)

        distances = map(euclidean_distance, self.goals)
        max_distance = max(distances)
        unsorted_data = zip(distances, self.goals)

        def metric(goal):
            # The metric computes distance between point and current robot
            # position and adds a penalty that is defined for each goal type in
            # the static variable GOAL_ORDERING.

            # TODO: The metric does not check for any obstacles in the map,
            # which results in some not optimal space exploration. Check if
            # there is a wall between robot position and point (using
            # raycasting) and add a pentalty.
            
            # TODO: The robot has to rotate if the point that is has to visit is
            # behind it. Try to pick points that are in direction where robot is
            # currently looking.
            penalty = 0
            if goal[1].type in MovementController.GOAL_ORDERING:
                penalty = MovementController.GOAL_ORDERING[goal[1].type]
            
            distance_pentalty = (goal[0] / max_distance) * 500

            return penalty + distance_pentalty
        
        sorted_data = sorted(unsorted_data, key=metric)
        self.goals = [goal[1] for goal in sorted_data]
    
    def send_marker_goals(self):
        poses = []
        for goal in self.goals:
            pose_stamped = PoseStamped()
            pose_stamped.header = goal.header
            pose_stamped.pose = goal.approaching_point_pose
            poses.append(pose_stamped)
        
        goal_markers = utils.stamped_poses_to_marker_array(poses, color=ColorRGBA(1, 1, 0, 1))
        self.goals_publisher.publish(goal_markers)

    def done(self, status, result):
        rospy.sleep(1) # just to be sure it detects cylinders and toruses

        self.send_marker_goals()

        if self.current_goal is not None:
            # If the approaching point was reached succesfully, greet.
            if status == 3 and (self.current_goal.type in ["face", "cylinder", "ring"]):
                self.greet()
                rospy.sleep(1)

        # This is called when the robot has reached our current goal
        # or something has gone wrong
        rospy.loginfo('Goal has finished with status: {}'.format(status))

        # To get more information about goal status see
        # http://docs.ros.org/fuerte/api/actionlib_msgs/html/msg/GoalStatus.html
        # Status with number 3 means that robot has reached the goal
        if status == 3:
            rospy.loginfo('Goal has been reached successfully')
        else:
            # Goal unreachable, add it back to queue, but with higher priority
            rospy.logerr('The robot could not move to the goal')

        # reset current goal
        self.current_goal = None

        if len(self.goals) <= 0:
            self.has_goals = False
            return
        
        # Sort goals based on the euclidean distance from current robot
        # position, then choose the first goal and visit it.
        self.sort_goals()
        next_goal = self.goals.pop(0)
        self.move_to_goal(next_goal)

    def greet(self, greeting = None):
        # Based on object type, choose a greeting
        self.color = self.current_goal.classified_color
        self.object_type = self.current_goal.type
        if self.object_type == "face":
            self.greetings = self.face_greetings
        else:
            self.greetings = self.cylinder_and_ring_greetings

        if greeting is None:
            random_greeting = random.choice(self.greetings)
            self.greeter.say(random_greeting)
        else:
            self.greeter.say(self.greetings[greeting])
    
    def feedback(self, data):
        # This is called repeatedly and tells us the robot position,
        # we won't have this data in our real robot, so pretend it
        # doesn't exist
        pass
    
    def active(self):
        # This is called when our goal is being processed on the server
        pass
    
    def move_to_goal(self, detection):
        rospy.loginfo('Moving to goal [type = {}] {}'.format(detection.type, detection.approaching_point_pose))

        # Set the goal as current goal
        self.current_goal = detection

        # Create a new MoveBaseGoal object and set its position and rotation
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()

        # Set the goal pose as the detected object approaching point
        goal.target_pose.pose = detection.approaching_point_pose

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
        self.greeter.say("Hello, my name is Dora the explorer and I am ready to begin my mission!")
        # After turtlebot has localized itself, start moving to goals in the self.faces heap
        self.start()
    
    def on_object_detection(self, object_detection):
        # A new face / cylinder / torus has been detected. If current goal is
        # not a map point, then cancel current goal and visit this goal.
        # Otherwise add it to goals for later.
        self.goals.append(object_detection)
        rospy.loginfo('New object [type = {}] received, there are currently {} goals'.format(object_detection.type, len(self.goals)))

        if self.current_goal.type == 'map_point':
            rospy.loginfo('Received goal is more promising that the one I am currently visiting. Cancelling.')
            self.greeter.say("More promising goal found!")
            # We are canceling current goal, so add current_goal to list of
            # goals to visit later
            
            robot_point = self.current_robot_position().pose.position
            approaching_point = self.current_goal.approaching_point_pose.position
            dx = (robot_point.x - approaching_point.x)
            dy = (robot_point.y - approaching_point.y)
            distance = dx * dx + dy * dy
            if distance > MovementController.GOAL_EPSILON_DISTANCE:
                self.goals.append(self.current_goal)
            
            # When calling self.start, the goals are reordered based on their
            # euclidean distance and goal type. It also cancels all previous
            # goals and starts again.
            self.start()

        elif self.is_localized and not self.has_goals:
            self.start()
        
def detection_from_point_on_map(point, angle_z_axis = None):
    pose = Pose()
    pose.position.x = point[0]
    pose.position.y = point[1]
    pose.position.z = point[2]

    # set correct orientation of the point
    if angle_z_axis is not None:
        rotation = quaternion_from_euler(0, 0, angle_z_axis)
        pose.orientation = Quaternion(*rotation)
    else:
        pose.orientation.w = 1

    detection = ObjectDetection()
    detection.header.frame_id = 'map'
    detection.header.stamp = rospy.Time.now()
    detection.object_pose = pose
    detection.approaching_point_pose = pose
    detection.type = 'map_point'

    return detection


if __name__ == '__main__':
    # Initialize node, don't allow running multiple nodes of this type
    rospy.init_node('movement_controller', anonymous=False)

    goals = []
    map_maker = MapMaker()
    points_to_visit = map_maker.generate_points()
    for index, point in enumerate(points_to_visit):
        goals.append(detection_from_point_on_map([point[0], point[1], 0], -2.578))
    
    controller = MovementController(goals)
    rospy.loginfo('Movement controller started')
    rospy.spin()