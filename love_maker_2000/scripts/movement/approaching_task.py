#!/usr/bin/env python

from __future__ import print_function
import rospy
import numpy
import cv2

from task import MovementTask
import utils

from move_base_msgs.msg import MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from nav_msgs.srv import GetMap
from geometry_msgs.msg import Pose, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

class ApproachingTask(MovementTask):

    def __init__(self, movement_controller, callback, action_client, object_detection, max_tries=3):
        super(ApproachingTask, self).__init__(movement_controller, callback)
        self.action_client = action_client
        self.object_detection = object_detection
        self.max_tries = max_tries
        self.tries = 0
        self.get_map()

        # To visualize goals, we create a new goal publisher that publishes markers
        self.goals_publisher = rospy.Publisher('goals', MarkerArray, queue_size=1000)

    def get_map(self):
        # Get map data from map server
        rospy.wait_for_service('static_map')
        get_map = rospy.ServiceProxy('static_map', GetMap)
        occupancy_grid = get_map().map
        width, height = occupancy_grid.info.width, occupancy_grid.info.height
        self.map_resolution = occupancy_grid.info.resolution
        self.map_origin = occupancy_grid.info.origin.position
        
        # 255 = space where robot cannot go
        map_array = numpy.asarray(occupancy_grid.data)
        map_array = numpy.reshape(map_array, (height, width))
        self.map_data = numpy.transpose(map_array)
        self.map_data[self.map_data == 100] = 255
        self.map_data[self.map_data == -1] = 255
        self.map_data = self.map_data.astype('uint8')

        # Create costmap from map by dilating it with a square kernel. This
        # costmap is then used to choose points that are further from the wall
        # when finding the shortest path
        kernel = numpy.ones((6, 6), numpy.uint8) 
        self.costmap = cv2.dilate(self.map_data, kernel, iterations=1)

    def feedback(self, data):
        pass
    
    def active(self):
        pass
    
    def done(self, status, result):
        # This is called when the robot has reached our current goal
        # or something has gone wrong.
        if status == GoalStatus.SUCCEEDED:
            # The robot has successfully reached the goal
            self.finish(status, result)
        else:
            # Try to visit goal at least self.max_tries, then give up if no path
            # could be found
            self.tries += 1
            if self.tries <= self.max_tries:
                rospy.logerr('Goal not reached, status: {}. Retrying: {} / {}'.format(status, self.tries, self.max_tries))
                self.run()
            else:
                # No path could be found, stop trying to reach this goal
                rospy.logerr('Goal not reached, status: {}. Cancelling.'.format(status))
                self.finish(status, result)

    def move_approaching_pose_away_from_the_wall(self, detection, minimum_distance=10, maximum_iterations=4):
        # To compute approaching point do the following. Calculate where the
        # closest wall from the cylinder approaching point. If there is no wall,
        # the approaching point is ok. Otherwise, move in the opposite direction
        # of the wall. If the approaching point pixel is INSIDE the wall, this
        # function will crash perfoming division of zero...
        map_position = utils.pose_to_map_pixel(detection.approaching_point_pose, self.map_origin, self.map_resolution)
        rospy.loginfo('MAP POSITION: {}'.format(map_position))

        closest_wall = utils.closest_wall_pixel(self.map_data, map_position, max_distance=minimum_distance)
        rospy.loginfo('CLOSEST WALL: {}'.format(closest_wall))

        if closest_wall is None:
            rospy.logwarn("Approaching point already OK")
            return detection.approaching_point_pose
        
        # If there was a wall detected, move in the opposite direction from it
        move_direction = map_position - closest_wall
        distance_to_wall = numpy.linalg.norm(move_direction)
        move_direction = move_direction / distance_to_wall

        new_map_position = map_position + move_direction * (minimum_distance - distance_to_wall + 1)
        rospy.loginfo('NEW PIXEL POSITION: {}'.format(new_map_position))
        
        approaching_pose = Pose()
        approaching_pose.position.x = new_map_position[0] * self.map_resolution + self.map_origin.x
        approaching_pose.position.y = new_map_position[1] * self.map_resolution + self.map_origin.y
        approaching_pose.orientation = detection.approaching_point_pose.orientation

        return approaching_pose
    
    def run(self):
        # Create a new MoveBaseGoal object and set its position and rotation
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.object_detection.header.frame_id
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = self.object_detection.approaching_point_pose

        # Check if approaching point is too close for the wall for cylinders
        # and rings. If so, move it away from the wall so the robot can approach it
        if self.object_detection.type == 'cylinder' or self.object_detection.type == 'ring':
            try:
                goal.target_pose.pose = self.move_approaching_pose_away_from_the_wall(self.object_detection)
            except Exception as e:
                rospy.logwarn('The approaching point could not be moved away from the wall.')
        
        # self.send_marker_goals(self.object_detection)
        goal_markers = utils.stamped_poses_to_marker_array([goal.target_pose], color=ColorRGBA(1, 0.5, 0.5, 0.5))
        self.goals_publisher.publish(goal_markers)

        # Send the MoveBaseGoal to our client and wait for server response
        # Also setup three different callbacks:
        #   * self.done is called when current goal has been reached
        #   * self.active is called when server is processing our current goal
        #   * self.feedback is called to notify us about our robot position
        self.action_client.send_goal(goal, done_cb=self.done, active_cb=self.active, feedback_cb=self.feedback)
    
    def finish(self, goal_status, goal_result):
        self.is_finished = True
        self.action_client.stop_tracking_goal()
        self.action_client.cancel_all_goals()

        # Call the callback function
        if self.callback is not None:
            self.callback(self.object_detection, goal_status, goal_result)

        self.movement_controller.on_finish(self)

    def cancel(self):
        self.action_client.stop_tracking_goal()
        self.action_client.cancel_all_goals()
        super(ApproachingTask, self).cancel()
    
    def __str__(self):
        return '<ApproachingTask, color={}, type={}>'.format(self.object_detection.classified_color, self.object_detection.type)

    def send_marker_goals(self, detection):
        pose_stamped = PoseStamped()
        pose_stamped.header = detection.header
        pose_stamped.pose = detection.approaching_point_pose
        
        goal_markers = utils.stamped_poses_to_marker_array([pose_stamped], color=ColorRGBA(1, 0.5, 0, 0.5))
        self.goals_publisher.publish(goal_markers)