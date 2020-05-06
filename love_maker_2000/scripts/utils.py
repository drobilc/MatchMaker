#!/usr/bin/env python

import rospy
import numpy
import cv2

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

def pose_to_map_pixel(pose, map_origin, map_resolution):
    # Convert position from map coordinates in m to pixel position (u, v) 
    pixel_x = int((pose.position.x - map_origin.x) / map_resolution)
    pixel_y = int((pose.position.y - map_origin.y) / map_resolution)
    return numpy.asarray([pixel_x, pixel_y])

def to_map_pixel(position, map_origin, map_resolution):
    # Convert position from map coordinates in m to pixel position (u, v) 
    pixel_x = int((position.point.x - map_origin.x) / map_resolution)
    pixel_y = int((position.point.y - map_origin.y) / map_resolution)
    return numpy.asarray([pixel_x, pixel_y])

def neighbors_all(pixel):
    neighbors = []
    neighbors.append(
        (pixel[0], pixel[1] + 1)
    )
    neighbors.append(
        (pixel[0], pixel[1] - 1)
    )
    neighbors.append(
        (pixel[0] - 1, pixel[1])
    )
    neighbors.append(
        (pixel[0] + 1, pixel[1])
    )
    return neighbors

def nearest_free_pixel(pixel, map_data):
    frontier = Queue.Queue()
    frontier.put(pixel)
    visited = numpy.zeros_like(map_data, dtype=bool)
    visited[pixel[1], pixel[0]] = True

    while not frontier.empty():
        current = frontier.get()
        if map_data[current[1], current[0]] == 255:
            rospy.logerr("Found nearest non-wall pixel")
            return current
        for next in neighbors_all(current):
            if not visited[next[1], next[0]]:
                frontier.put(next)
                visited[next[1], next[0]] = True
    
    rospy.logerr("Couldn't find closest non-wall pixel")
    return None

def move_away_from_the_wall(map_data, point, too_close):
    # Create neighbourhoods
    neighbours_base = [0]
    for i in range(1, too_close + 1):
        neighbours_base.append(i)

    neighbours_left = []
    neighbours_right = []
    neighbours_up = []
    neighbours_down = []
    extended_neighbours_left = []
    extended_neighbours_right = []
    extended_neighbours_up = []
    extended_neighbours_down = []

    for neighbour in neighbours_base:
        neighbours_right.append([point[0] + neighbour, point[1]])
        neighbours_left.append([point[0] + (-1) * neighbour, point[1]])
        neighbours_down.append([point[0], point[1] + neighbour])
        neighbours_up.append([point[0], point[1] + (-1) * neighbour])

        extended_neighbours_right.append([point[0] + 3 * neighbour, point[1]])
        extended_neighbours_left.append([point[0] + (-1) * 3 * neighbour, point[1]])
        extended_neighbours_down.append([point[0], point[1] + 3 * neighbour])
        extended_neighbours_up.append([point[0], point[1] + (-1) * 3* neighbour])
        extended_neighbours_right.append([point[0] + 3 * neighbour + 1, point[1]])
        extended_neighbours_left.append([point[0] + (-1) * 3 * neighbour + 1, point[1]])
        extended_neighbours_down.append([point[0], point[1] + 3 * neighbour + 1])
        extended_neighbours_up.append([point[0], point[1] + (-1) * 3 * neighbour + 1])
        extended_neighbours_right.append([point[0] + 3 * neighbour + 2, point[1]])
        extended_neighbours_left.append([point[0] + (-1) * 3 * neighbour + 2, point[1]])
        extended_neighbours_down.append([point[0], point[1] + 3 * neighbour + 2])
        extended_neighbours_up.append([point[0], point[1] + (-1) * 3 * neighbour + 2])
        
    # Check for walls in each direction
    wall_to_the_right = False
    wall_to_the_left = False
    wall_to_the_bottom = False
    wall_to_the_top = False
    free_to_move_right = True
    free_to_move_left = True
    free_to_move_down = True
    free_to_move_up = True
    for i in range(1, too_close + 1):
        if map_data[neighbours_right[i][1]][neighbours_right[i][0]] < 250:
            wall_to_the_right = True
        if map_data[neighbours_left[i][1]][neighbours_left[i][0]] < 250:
            wall_to_the_left = True
        if map_data[neighbours_up[i][1]][neighbours_up[i][0]] < 250:
            wall_to_the_top = True
        if map_data[neighbours_down[i][1]][neighbours_down[i][0]] < 250:
            wall_to_the_bottom = True

        if map_data[extended_neighbours_right[3 * i -2][1]][extended_neighbours_right[3 * i -2][0]] < 250:
            wall_to_the_right = False
        if map_data[extended_neighbours_right[3 * i -1][1]][extended_neighbours_right[3 * i -1][0]] < 250:
            wall_to_the_right = False
        if map_data[extended_neighbours_right[3 * i][1]][extended_neighbours_right[3 * i][0]] < 250:
            wall_to_the_right = False
        if map_data[extended_neighbours_right[3 * i -2][1]][extended_neighbours_right[3 * i -2][0]] < 250:
            wall_to_the_right = False
        if map_data[extended_neighbours_right[3 * i -1][1]][extended_neighbours_right[3 * i -1][0]] < 250:
            wall_to_the_right = False
        if map_data[extended_neighbours_right[3 * i][1]][extended_neighbours_right[3 * i][0]] < 250:
            wall_to_the_right = False
        if map_data[extended_neighbours_right[3 * i -2][1]][extended_neighbours_right[3 * i -2][0]] < 250:
            wall_to_the_right = False
        if map_data[extended_neighbours_right[3 * i -1][1]][extended_neighbours_right[3 * i -1][0]] < 250:
            wall_to_the_right = False
        if map_data[extended_neighbours_right[3 * i][1]][extended_neighbours_right[3 * i][0]] < 250:
            wall_to_the_right = False
        if map_data[extended_neighbours_right[3 * i -2][1]][extended_neighbours_right[3 * i -2][0]] < 250:
            wall_to_the_right = False
        if map_data[extended_neighbours_right[3 * i -1][1]][extended_neighbours_right[3 * i -1][0]] < 250:
            wall_to_the_right = False
        if map_data[extended_neighbours_right[3 * i][1]][extended_neighbours_right[3 * i][0]] < 250:
            wall_to_the_right = False

    # For each wall to close, move the point too_close pixels in the opposite direction
    if wall_to_the_right and free_to_move_right:
        point[0] -= 2 * too_close
    if wall_to_the_left and free_to_move_left:
        point[0] += 2 * too_close
    if wall_to_the_bottom and free_to_move_up:
        point[1] -= 2 * too_close
    if wall_to_the_top and free_to_move_down:
        point[1] += 2 * too_close

    return point

def closest_wall_pixel(map_data, pixel, max_distance=5):
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