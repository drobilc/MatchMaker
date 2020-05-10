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

        kernel = numpy.ones((6, 6), numpy.uint8) 
        self.costmap = cv2.dilate(self.map_data, kernel, iterations=1)

        # EACH PIXEL NOW HAS RESOLUTION OF 2 * self.map_resolution
        # self.map_resolution = self.map_resolution * 2
        # self.map_data = cv2.resize(self.map_data, (width / 2, height / 2), cv2.INTER_NEAREST)

    def compute_path(self, start_pose, angle_resolution=8):
        # First, calculate the start and end robot position. The position is a
        # vector (x, y, angle) that expresses robot position in 2d space and its
        # orientation (along the z axis)
        end_pose = self.object_detection.approaching_point_pose
        start_pose = start_pose.pose

        def orientation_to_angle(pose):
            # Get the end orientation (only the z orientation)
            quaternion = pose.orientation
            quaternion_as_list = (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
            euler_rotation = euler_from_quaternion(quaternion_as_list)
            return euler_rotation[2]

        start_orientation = orientation_to_angle(start_pose)
        end_orientation = orientation_to_angle(self.object_detection.approaching_point_pose)

        # Compute the start and end pixel
        start_pixel = utils.pose_to_map_pixel(start_pose, self.map_origin, self.map_resolution)
        end_pixel = utils.pose_to_map_pixel(end_pose, self.map_origin, self.map_resolution)

        def angle_to_grid(angle):
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
            width, height, resolution = search_grid.shape
            current_angle = (point[2] * (2 * numpy.pi / resolution))
            #angle1 = ((point[2] + 1) * (2 * numpy.pi / resolution))
            #angle2 = ((point[2] - 1) * (2 * numpy.pi / resolution))
            possible_neighbors = [
                (int(numpy.cos(current_angle) * 1.5), int(numpy.sin(current_angle) * 1.5), 0),
                #(int(numpy.cos(angle1) * 1.5), int(numpy.sin(angle1) * 1.5), 1),
                #(int(numpy.cos(angle2) * 1.5), int(numpy.sin(angle2) * 1.5), -1),
                (0, 0, 1),
                (0, 0, -1)
            ]
            neighbors = []
            for neighbor in possible_neighbors:
                neighbor_position = [point[0] + neighbor[0], point[1] + neighbor[1], point[2] + neighbor[2]]
                if neighbor_position[0] < 0 or neighbor_position[1] < 0 or neighbor_position[0] >= width or neighbor_position[1] >= height:
                    continue
                neighbor_position[2] = (neighbor_position[2] + resolution) % resolution
                neighbor_position = (int(neighbor_position[0]), int(neighbor_position[1]), int(neighbor_position[2]))
                if search_grid[neighbor_position[0], neighbor_position[1], neighbor_position[2]] != 0:
                    continue
                neighbors.append(neighbor_position)
            return neighbors
        
        def heuristic(goal, next):
            return abs(goal[0] - next[0]) + abs(goal[1] - next[1]) + abs(goal[2] - next[2])
        
        def cost(current, next):
            return int(self.costmap[next[0], next[1]] / 255.0 * 10) + 1

        frontier = []
        heapq.heappush(frontier, (0, start))
        came_from = {}
        cost_so_far = {}
        came_from[start] = None
        cost_so_far[start] = 0

        while frontier:
            current_cost, current = heapq.heappop(frontier)

            if abs(current[0] - end[0]) <= 0 and abs(current[1] - end[1]) <= 0 and abs(current[2] - end[2]) <= 0:
                break
   
            for next in neighbors(search_grid, current):
                new_cost = cost_so_far[current] + cost(current, next)
                if next not in cost_so_far or new_cost < cost_so_far[next]:
                    cost_so_far[next] = new_cost
                    priority = new_cost + heuristic(end, next)
                    heapq.heappush(frontier, (priority, next))
                    came_from[next] = current
        
        if end not in came_from:
            return None
        
        path = [end]

        current_point = end
        while current_point != start:
            new_point = came_from[current_point]
            path.append(new_point)
            current_point = new_point
        path.append(start)
        path = list(reversed(path))

        better_path = []
        last_pose = path[0]
        for i in range(1, len(path)):
            if last_pose[0] == path[i][0] and last_pose[1] == path[i][1]:
                # This step is just rotation
                last_pose = path[i]
            elif path[i][2] == last_pose[2]:
                pass
            else:
                better_path.append(last_pose)
                last_pose = path[i]
        better_path.append(last_pose)

        return better_path

    def on_odometry_received(self, odometry, angle_resolution=8):
        if self.is_finished or self.is_cancelled:
            return

        pose_stamped = PoseStamped()
        pose_stamped.header = odometry.header
        pose_stamped.pose = odometry.pose.pose

        try:
            map_position = self.tf_buffer.transform(pose_stamped, 'map')
        except Exception:
            return
        
        def pixel_to_map(pixel):
            return numpy.asarray([self.map_origin.x, self.map_origin.y]) + (numpy.asarray([pixel[0], pixel[1]]) * self.map_resolution)

        if not hasattr(self, 'path'):
            # Compute path as a list of pixels and rotations (x, y, angle) that
            # our robot has to visit to actually get to the goal.
            self.path = self.compute_path(map_position, angle_resolution=angle_resolution)
            
            if self.path is None:
                rospy.logwarn('No path found')
                return
            
            self.checkpoints = []
            for point in self.path:
                pose_stamped = PoseStamped()
                pose_stamped.header.frame_id = 'map'
                pose_stamped.header.stamp = rospy.Time.now()
                test = pixel_to_map(point)
                orientation = point[2] * (2 * numpy.pi / angle_resolution)
                pose_stamped.pose.position.x = test[0]
                pose_stamped.pose.position.y = test[1]
                pose_stamped.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, orientation))
                self.checkpoints.append(pose_stamped)
        
        if not hasattr(self, 'current_goal'):
            self.current_goal = self.checkpoints.pop(0)
        
        def orientation_to_angle(pose):
            # Get the end orientation (only the z orientation)
            quaternion = pose.orientation
            quaternion_as_list = (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
            euler_rotation = euler_from_quaternion(quaternion_as_list)
            return euler_rotation[2]
        
        # Compute the current robot orientation and orientation from robot to current goal
        current_orientation = (orientation_to_angle(map_position.pose) + (2 * numpy.pi)) % (2 * numpy.pi)
        
        robot_position = numpy.asarray([map_position.pose.position.x, map_position.pose.position.y])
        goal_position = numpy.asarray([self.current_goal.pose.position.x, self.current_goal.pose.position.y])
        orientation = goal_position - robot_position
        orientation_to_goal = (numpy.arctan2(orientation[1], orientation[0]) + (2 * numpy.pi)) % (2 * numpy.pi)

        # TODO: Make robot rotate in the direction of the goal pose
        # goal_orientation = numpy.pi - (self.current_goal[2] * (2 * numpy.pi / angle_resolution))
        distance_to_goal = numpy.linalg.norm(orientation)

        rotation = orientation_to_goal - current_orientation
        if abs(rotation) > 0.08:
            twist = Twist()
            twist.angular.z = rotation * 0.5
            self.localization_publisher.publish(twist)
        elif distance_to_goal > 0.2:
            twist = Twist()
            twist.linear.x = 0.1
            self.localization_publisher.publish(twist)
        else:
            twist = Twist()
            self.localization_publisher.publish(twist)

            if len(self.checkpoints) > 0:
                self.current_goal = self.checkpoints.pop(0)
            else:
                self.finish()
    
    def run(self):
        # When the run function is called, subscribe to odometry data, so we can
        # compute where the robot currently is and where it wants to get
        self.localization_publisher = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size = 10)
        self.odometry_subscriber = rospy.Subscriber('/odom', Odometry, self.on_odometry_received, queue_size=10)
    
    def finish(self):
        self.odometry_subscriber.unregister()

        self.is_finished = True

        # Call the callback function
        if self.callback is not None:
            self.callback(self.object_detection, 3, None)

        self.movement_controller.on_finish(self)