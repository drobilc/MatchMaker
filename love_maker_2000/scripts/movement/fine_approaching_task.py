#!/usr/bin/env python

from __future__ import print_function
import rospy
import numpy

from task import MovementTask

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from move_base_msgs.msg import MoveBaseGoal
from nav_msgs.srv import GetMap

from tf.transformations import euler_from_quaternion, quaternion_from_euler
import utils

from geometry_msgs.msg import PoseStamped, Quaternion, Vector3
from visualization_msgs.msg import Marker, MarkerArray

import heapq

import cv2

import tf2_ros
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from tf2_geometry_msgs import PointStamped

class FineApproachingTask(MovementTask):

    def __init__(self, movement_controller, callback, action_client, object_detection):
        super(FineApproachingTask, self).__init__(movement_controller, callback)
        self.action_client = action_client
        self.object_detection = object_detection

        # Transformation buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Get map data from map server
        rospy.wait_for_service('static_map')
        get_map = rospy.ServiceProxy('static_map', GetMap)
        occupancy_grid = get_map().map
        width, height = occupancy_grid.info.width, occupancy_grid.info.height
        self.map_resolution = occupancy_grid.info.resolution
        self.map_origin = occupancy_grid.info.origin.position
        
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

        # EACH PIXEL NOW HAS RESOLUTION OF 2 * self.map_resolution
        # self.map_resolution = self.map_resolution * 2
        # self.map_data = cv2.resize(self.map_data, (width / 2, height / 2), cv2.INTER_NEAREST)

        self.path_publisher = rospy.Publisher('/fine_path', Path, queue_size = 10)

    def path_steps_to_poses(self, path, angle_resolution):
        """Converts steps (x, y, angle) to poses on map we have to visit in order to get to goal"""
        poses = []
        for point in path:
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = 'map'
            pose_stamped.header.stamp = rospy.Time.now()
            point_on_map = utils.pixel_to_map(point, self.map_origin, self.map_resolution)
            orientation = point[2] * (2 * numpy.pi / angle_resolution)
            pose_stamped.pose.position.x = point_on_map[0]
            pose_stamped.pose.position.y = point_on_map[1]
            pose_stamped.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, orientation))
            poses.append(pose_stamped)
        return poses

    def compute_path(self, start_pose, angle_resolution=8):
        # First, calculate the start and end robot position. The position is a
        # vector (x, y, angle) that expresses robot position in 2d space and its
        # orientation (along the z axis)
        end_pose = self.object_detection.approaching_point_pose
        start_pose = start_pose.pose

        # Compute the start and end pixel
        start_pixel = utils.pose_to_map_pixel(start_pose, self.map_origin, self.map_resolution)
        end_pixel = utils.pose_to_map_pixel(end_pose, self.map_origin, self.map_resolution)

        # Get the orientation along the z axis of start and goal pose
        start_orientation = utils.orientation_to_angle(start_pose)
        end_orientation = utils.orientation_to_angle(self.object_detection.approaching_point_pose)

        def angle_to_grid(angle):
            """Compute the bucket that the angle should fall into"""
            # The angles between 0 and 360 / angle_resolution fall into the
            # bucket with index 0, angles between 360 / angle_resolution and
            # 2 * (360 / angle_resolution), etc...
            angle = (angle + (2 * numpy.pi)) % (2 * numpy.pi)
            return int(angle / (2 * numpy.pi / angle_resolution))

        # Construct the start and end point as defined above (x, y, angle)
        start = (start_pixel[0], start_pixel[1], angle_to_grid(start_orientation))
        end = (end_pixel[0], end_pixel[1], angle_to_grid(end_orientation))

        # The grid consists of angle_resolution layers, where each layer
        # represents an angle between 0 and 360 degrees (there are
        # angle_resolution layers, so each layer represents angles from i * (360
        # / angle_resolution) to (i + 1) * (360 / angle_resolution))
        search_grid = numpy.stack([self.map_data for i in range(angle_resolution)], axis=2)

        def neighbors(search_grid, point):
            """Return neighbors of a point in search_grid"""

            width, height, resolution = search_grid.shape
            current_angle = (point[2] * (2 * numpy.pi / resolution))

            # For each point three neighbors are defined:
            #   * rotation (to the left and to the right by 360 / angle_resolution degrees)
            #   * movement in the direction we are currently facing
            possible_neighbors = [
                (int(numpy.cos(current_angle) * 1.5), int(numpy.sin(current_angle) * 1.5), 0),
                (0, 0, 1), (0, 0, -1)
            ]

            # For each possible neighbor check if it lies inside the grid
            neighbors = []
            for neighbor in possible_neighbors:
                # Check 1: Check if point is out of bounds
                neighbor_position = [point[0] + neighbor[0], point[1] + neighbor[1], point[2] + neighbor[2]]
                if neighbor_position[0] < 0 or neighbor_position[1] < 0 or neighbor_position[0] >= width or neighbor_position[1] >= height:
                    continue
                # Check 2: The angles should wrap around (-1 = angle_resolution - 1)
                neighbor_position[2] = (neighbor_position[2] + resolution) % resolution
                neighbor_position = (int(neighbor_position[0]), int(neighbor_position[1]), int(neighbor_position[2]))
                # Check 3: Check if the neighbor is a wall
                if search_grid[neighbor_position[0], neighbor_position[1], neighbor_position[2]] != 0:
                    continue
                neighbors.append(neighbor_position)

            return neighbors

        # The path in our search space will be searched using the A* path
        # finding algorithm. It requires two additional functions to be defined:
        #   * heuristic - the heuristic function (in our case a simple distance
        #     between two points in 3d space). The A* algorithm will choose a
        #     point that has the best heuristics first.
        #   * cost - the cost function of the edge between two nodes. In our
        #     grid case, this will simply be a 1 if the point lies at least 6
        #     pixels away from the walls and 10 if it lies closer.
        def heuristic(goal, next):
            return abs(goal[0] - next[0]) + abs(goal[1] - next[1]) + abs(goal[2] - next[2])
        
        def cost(current, next):
            return int(self.costmap[next[0], next[1]] / 255.0 * 10) + 1

        # The actual A* algorithm implementation using heap. The frontier is an
        # ordered heap, the came_from dictionary contains the previous pixel in
        # map for each pixel. This is used to reconstruct the path once the
        # algorithm is finished.
        frontier = []
        came_from = {}
        cost_so_far = {}

        heapq.heappush(frontier, (0, start))
        came_from[start] = None
        cost_so_far[start] = 0

        while frontier:
            current_cost, current = heapq.heappop(frontier)

            if current == end:
                break
   
            for next in neighbors(search_grid, current):
                new_cost = cost_so_far[current] + cost(current, next)
                if next not in cost_so_far or new_cost < cost_so_far[next]:
                    cost_so_far[next] = new_cost
                    priority = new_cost + heuristic(end, next)
                    heapq.heappush(frontier, (priority, next))
                    came_from[next] = current
        
        # The algorithm finished, there are two options - the path has been
        # found, we have reached the end point so came_from should contain its
        # previous cell. If it does not, the path was not found.
        if end not in came_from:
            return None
        
        # Reconstruct a path from the end pixel to the start pixel. Then reverse
        # it so we can perform goal merging.
        path = [end]
        current_point = end
        while current_point != start:
            new_point = came_from[current_point]
            path.append(new_point)
            current_point = new_point
        path.append(start)
        path = list(reversed(path))

        # The last part is reducing the number of poses needed to visit the
        # goal. Instead of all the points on path, we only want the important
        # ones. The important points are the last rotation in a series of
        # rotations (rotations are steps where only the angle component in (x,
        # y, angle) changes, x and y stay the same). The points that lie on the
        # same line (the rotation stays the same, only the coordinates change)
        # can be replaced with the first and the last point.
        better_path = []
        last_pose = path[0]
        for i in range(1, len(path)):
            if last_pose[0] == path[i][0] and last_pose[1] == path[i][1]:
                last_pose = path[i]
            elif path[i][2] == last_pose[2]:
                pass
            else:
                better_path.append(last_pose)
                last_pose = path[i]
        better_path.append(last_pose)

        # After points of interest have been calculated, convert them to map
        # poses and return it.
        return self.path_steps_to_poses(better_path, angle_resolution)

    def on_odometry_received(self, odometry, angle_resolution=8):
        """Callback for when the odometry data is received. Move the robot closer to goal."""
        if self.is_finished or self.is_cancelled:
            return
        
        # First, convert the received odometry pose to map coordinate frame,
        # otherwise the robot position is not accurate
        pose_stamped = PoseStamped()
        pose_stamped.header = odometry.header
        pose_stamped.pose = odometry.pose.pose
        try:
            map_position = self.tf_buffer.transform(pose_stamped, 'map')
        except Exception:
            rospy.logwarn('Can not transform received odometry to map coordinate frame')
            return

        # If there is no path between robot position and goal, call the
        # self.compute_path, which returns a list of stamped poses, which will
        # lead to the goal
        if not hasattr(self, 'path'):
            self.path = self.compute_path(map_position, angle_resolution=angle_resolution)
            
            # The path can sometimes not be calculated, notify user and return.
            # Try again when next odometry is received
            if self.path is None:
                rospy.loginfo('No path found')
                return
        
        # If there is no current_goal that means that the path has just been
        # computed, get the first element of the path and visit it
        if not hasattr(self, 'current_goal'):
            self.current_goal = self.path.pop(0)

        # Send a path to the path_publisher so we can visualize it in rviz
        path = Path()
        path.header = map_position.header
        path.poses = [self.current_goal] + self.path
        self.path_publisher.publish(path)

        # Visit the current goal using twist messages
        self.move_to_pose(map_position, self.current_goal)
    
    def move_to_pose(self, map_position, current_goal, rotation_threshold=0.08, distance_threshold=0.2):
        """Move the robot to current_goal using twist messages"""
        # First, get the positions of the robot in map coordinate frame
        robot_position = numpy.asarray([map_position.pose.position.x, map_position.pose.position.y])
        goal_position = numpy.asarray([current_goal.pose.position.x, current_goal.pose.position.y])

        # Then compute current robot orientation and rotation from robot
        # position to the goal position
        current_orientation = (utils.orientation_to_angle(map_position.pose) + (2 * numpy.pi)) % (2 * numpy.pi)
        orientation = goal_position - robot_position
        orientation_to_goal = (numpy.arctan2(orientation[1], orientation[0]) + (2 * numpy.pi)) % (2 * numpy.pi)

        # Compute the distance between robot position and goal position
        distance_to_goal = numpy.linalg.norm(orientation)

        rotation = orientation_to_goal - current_orientation
        
        # To get to the goal, first rotate until we are headed in the direction
        # of the current goal. The rotation_threshold specifies how close to the
        # actual rotation we have to be for the robot to start rotating and
        # start moving. It should be noted that the if the robot orientation
        # deviates from the goal orientation at any point in time, it will first
        # rotate until it is oriented correctly. After the robot is oriented in
        # the direction of the goal, start moving towards it until we are close
        # enough. The distance_threshold is the "close enough" parameter. After
        # current goal has been reached, get the next goal from our path and
        # wait until the odometry is received again to repeat. If there is no
        # more goals, we have successfully reached our destination.
        if abs(rotation) > rotation_threshold:
            twist = Twist()
            twist.angular.z = rotation * 0.5
            self.localization_publisher.publish(twist)
        elif distance_to_goal > distance_threshold:
            twist = Twist()
            twist.linear.x = 0.1
            self.localization_publisher.publish(twist)
        else:
            twist = Twist()
            self.localization_publisher.publish(twist)

            if len(self.path) > 0:
                self.current_goal = self.path.pop(0)
            else:
                self.finish()
    
    def run(self):
        """Callback for when the fine approaching task should start"""
        self.localization_publisher = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size = 10)
        self.odometry_subscriber = rospy.Subscriber('/odom', Odometry, self.on_odometry_received, queue_size=10)
    
    def finish(self):
        """Callback for when the fine approaching task should finish"""
        self.odometry_subscriber.unregister()

        self.is_finished = True

        # Call the callback function
        if self.callback is not None:
            self.callback(self.object_detection, 3, None)

        self.movement_controller.on_finish(self)
    
    def cancel(self):
        """Callback for when the fine approaching task should cancel"""
        self.odometry_subscriber.unregister()
        super(ApproachingTask, self).cancel()
    
    def __str__(self):
        return '<FineApproachingTask, color={}, type={}>'.format(self.object_detection.classified_color, self.object_detection.type)