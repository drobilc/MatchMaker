#!/usr/bin/env python

import rospy
import numpy

import Queue

from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Pose, PoseStamped, Twist, Quaternion, Vector3
from std_msgs.msg import String, ColorRGBA

def stamped_poses_to_marker_array(poses, marker_type=Marker.CUBE, lifetime=120, scale=Vector3(0.1, 0.1, 0.1), color=ColorRGBA(1, 0, 0, 1)):
    markers = MarkerArray()
    for index, pose in enumerate(poses):
        marker = stamped_pose_to_marker(pose, index=index, marker_type=marker_type, lifetime=lifetime, scale=scale, color=color)
        markers.markers.append(marker)
    return markers

def stamped_pose_to_marker(pose_stamped, marker_type=Marker.CUBE, lifetime=120, scale=Vector3(0.1, 0.1, 0.1), color=ColorRGBA(1, 0, 0, 1), index=1):
    marker = Marker()
    marker.action = Marker.ADD
    marker.frame_locked = False
    marker.header = pose_stamped.header
    marker.pose = pose_stamped.pose
    marker.type = marker_type
    marker.lifetime = rospy.Duration.from_sec(lifetime)
    marker.id = index
    marker.scale = scale
    marker.color = color
    return marker

def to_map_pixel(position, map_origin, map_resolution):
    # Convert position from map coordinates in m to pixel position (u, v) 
    pixel_x = int((position.point.x - map_origin.x) / map_resolution)
    pixel_y = int((position.point.y - map_origin.y) / map_resolution)
    return numpy.asarray([pixel_x, pixel_y])

def closest_wall_pixel(pixel, map_data, max_distance=5):
    # Use flood fill algorithm with breadth first search to find the
    # closest wall pixel. Use neighboring pixels as defined below
    neighborhood = [(-1, 0), (0, 1), (1, 0), (0, -1)]

    # First, extract a small region around the pixel (a square region of
    # size 2 * max_distance + 1)
    map_region = map_data[pixel[1]-max_distance-1:pixel[1]+max_distance, pixel[0]-max_distance-1:pixel[0]+max_distance]
            
    visited = numpy.zeros_like(map_region, dtype=bool)
    frontier = [(max_distance, max_distance)]
    while len(frontier) > 0:
        current_pixel = frontier.pop(0)

        # If current pixel has a non-zero probability, we have found the closest wall point
        if map_region[current_pixel[1], current_pixel[0]] == 255:
            return (current_pixel[0] + pixel[0] - max_distance, current_pixel[1] + pixel[1] - max_distance)

        # Mark pixel as visited
        visited[current_pixel[1], current_pixel[0]] = True
                
        # Add four neighbours to frontier
        for neighbor in neighborhood:
            new_x, new_y = current_pixel[0] + neighbor[0], current_pixel[1] + neighbor[1]
            if new_x < 0 or new_y < 0 or new_x >= max_distance * 2 or new_y >= max_distance * 2 or visited[new_y, new_x]:
                continue
            frontier.append((new_x, new_y))
    
    return None

def neighbors(pixel, map_data):
    neighbors = []
    if map_data[pixel[1] - 1, pixel[0]] == 255:
        neighbors.append(
            (pixel[0], pixel[1] - 1)
        )
    if map_data[pixel[1] + 1, pixel[0]] == 255:
        neighbors.append(
            (pixel[0], pixel[1] + 1)
        )
    if map_data[pixel[1], pixel[0] - 1] == 255:
        neighbors.append(
            (pixel[0] - 1, pixel[1])
        )
    if map_data[pixel[1], pixel[0] + 1] == 255:
        neighbors.append(
            (pixel[0] + 1, pixel[1])
        )
    return neighbors

def bfs(start, goal, map_data):
    rospy.loginfo("~~~~~Started calculating cylinder approaching point")
    frontier = Queue.Queue()
    frontier.put(start)
    visited = numpy.zeros_like(map_data, dtype=bool)
    rospy.loginfo("Visited: ")
    rospy.loginfo(visited)
    visited[start[1], start[0]] = True

    while not frontier.empty():
        current = frontier.get()
        rospy.loginfo("Current: " + str(current))
        if closest_wall_pixel(current, map_data) is None:
            rospy.loginfo("Found no closest wall pixel")
            return current
        
        for next in neighbors(current, map_data):
            rospy.loginfo(neighbors)
            if not visited[next[1], next[0]]:
                rospy.loginfo("Adding " + str(next) + " to frontier")
                frontier.put(next)
                visited[next[1], next[0]] = True
    
    return None
