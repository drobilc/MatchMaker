#!/usr/bin/env python

import rospy
import numpy
import cv2

import Queue
from nav_msgs.srv import GetMap

from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Pose, PoseStamped, Twist, Quaternion, Vector3
from std_msgs.msg import String, ColorRGBA

def stamped_poses_to_marker_array(poses, marker_type=Marker.CUBE, lifetime=900, scale=Vector3(0.1, 0.1, 0.1), color=ColorRGBA(1, 0, 0, 1)):
    """Constructs a MarkerArray of Marker objects from poses in poses list"""
    markers = MarkerArray()
    for index, pose in enumerate(poses):
        marker = stamped_pose_to_marker(pose, index=index, marker_type=marker_type, lifetime=lifetime, scale=scale, color=color)
        markers.markers.append(marker)
    return markers

def stamped_pose_to_marker(pose_stamped, marker_type=Marker.CUBE, lifetime=900, scale=Vector3(0.1, 0.1, 0.1), color=ColorRGBA(1, 0, 0, 1), index=1):
    """Constructs a marker from PoseStamped object"""
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
    """Convert Pose from map coordinates in m to pixel position (u, v)"""
    pixel_x = int((pose.position.x - map_origin.x) / map_resolution)
    pixel_y = int((pose.position.y - map_origin.y) / map_resolution)
    return numpy.asarray([pixel_x, pixel_y])

def to_map_pixel(position, map_origin, map_resolution):
    """Convert Point from map coordinates in m to pixel position (u, v) """
    pixel_x = int((position.point.x - map_origin.x) / map_resolution)
    pixel_y = int((position.point.y - map_origin.y) / map_resolution)
    return numpy.asarray([pixel_x, pixel_y])

def get_map_data():
    """Return static map walls as white and everything else black pixels"""
    get_map = rospy.ServiceProxy('static_map', GetMap)
    rospy.wait_for_service('static_map')

    occupancy_grid = get_map().map

    width, height = occupancy_grid.info.width, occupancy_grid.info.height
    map_resolution = occupancy_grid.info.resolution
    map_origin = occupancy_grid.info.origin.position

    map_data = numpy.asarray(occupancy_grid.data)
    map_data = numpy.reshape(map_data, (height, width))
    map_data[map_data == 100] = 255
    map_data[map_data == -1] = 0
    map_data = map_data.astype('uint8')

    return map_data, map_resolution, map_origin

def closest_wall_pixel(map_data, pixel, max_distance=5):
    """Computes the closest pixel on map_data near pixel"""
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

def closest_line(map_data, pixel, wall_pixel, max_distance=5):
    """Computes the start and end point of the closest line that passes near wall_pixel."""
    map_region = map_data[pixel[1]-max_distance-1:pixel[1]+max_distance, pixel[0]-max_distance-1:pixel[0]+max_distance]
    x0, y0 = wall_pixel - pixel + max_distance

    lines = cv2.HoughLinesP(map_region, rho=1, theta=numpy.pi / 180.0, threshold=8, minLineLength=8, maxLineGap=3)
    if lines is None or len(lines) <= 0:
        return None, None
    
    best_line = None
    best_distance = 100000
    for line in lines:
        start = numpy.asarray([line[0][0], line[0][1]])
        end = numpy.asarray([line[0][2], line[0][3]])
        distance = abs(((end[1]-start[1])*x0 - (end[0]-start[0])*y0 + end[0]*start[1] - end[1]*start[0]) / numpy.linalg.norm(end - start))
        if distance < best_distance:
            best_distance = distance
            best_line = (start, end)
    
    return best_line, best_distance