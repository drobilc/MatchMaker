#!/usr/bin/env python

import rospy
import numpy
import cv2
import urllib2

import Queue

from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Pose, PoseStamped, Twist, Quaternion, Vector3
from std_msgs.msg import String, ColorRGBA
from object_detection_msgs.msg import ObjectDetection

import tf2_ros
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf2_geometry_msgs

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

def pixel_to_map(pixel, map_origin, map_resolution):
    """Convert pixel coordinates (u, v) to map position (x, y)"""
    return numpy.asarray([map_origin.x, map_origin.y]) + (numpy.asarray([pixel[0], pixel[1]]) * map_resolution)

def orientation_to_angle(pose):
    """Converts the pose orientation to euler rotation and return rotation around the z axis"""
    quaternion = pose.orientation
    quaternion_as_list = (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
    euler_rotation = euler_from_quaternion(quaternion_as_list)
    return euler_rotation[2]

def neighbors_all(pixel):
    """Finds pixel's direct neighbours to the left, right, top, bottom"""
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
    """Finds closest pixel to given pixel that does not contain wall"""
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
    """Checks if the point has wall in neighborhood of range too_close around given 
    pixel in 4 basic directions. Checks if we would have hit the wall trying to move 
    the point 3*too_close in any of 4 basic directions. Then move point 2*too_close 
    in the opposite direction of that in which the wall is too_close if that means 
    we won't hit the wall."""
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
        # Neighbors in too_close range
        neighbours_right.append([point[0] + neighbour, point[1]])
        neighbours_left.append([point[0] + (-1) * neighbour, point[1]])
        neighbours_down.append([point[0], point[1] + neighbour])
        neighbours_up.append([point[0], point[1] + (-1) * neighbour])

        # Neighbors in 3 * too_close range
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
        # Check if there is wall inside too_close range neighborhood
        if map_data[neighbours_right[i][1]][neighbours_right[i][0]] < 250:
            wall_to_the_right = True
        if map_data[neighbours_left[i][1]][neighbours_left[i][0]] < 250:
            wall_to_the_left = True
        if map_data[neighbours_up[i][1]][neighbours_up[i][0]] < 250:
            wall_to_the_top = True
        if map_data[neighbours_down[i][1]][neighbours_down[i][0]] < 250:
            wall_to_the_bottom = True

        # Check if there is wall in extended neighbourhood that would prevent us from moving 
        # the point in that direction
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
    """Finds closest pixel to given pixel that contains wall"""
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

def detection_from_point_on_map(point, angle_z_axis = None):
    """Create new ObjectDetection from point and z orientation"""
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

class FaceDetails(object):
    def __init__(self, hair_length, hair_color):
        self.hair_length = hair_length
        self.hair_color = hair_color
    def __str__():
        return "{}, {}".format(self.hair_length, self.hair_color)

FACE_DETAILS = {
    'face02': FaceDetails('long'    , 'bright'  ),
    'face03': FaceDetails('long'    , 'dark'    ),
    'face04': FaceDetails('short'   , 'dark'    ),
    'face05': FaceDetails('short'   , 'bright'  ),  # TODO: check with prof to be sure
    'face06': FaceDetails('long'    , 'dark'    ),
    'face07': FaceDetails('long'    , 'dark'    ),  # TODO: -||-
    'face08': FaceDetails('long'    , 'dark'    ),
    'face09': FaceDetails('short'   , 'dark'    ),
    'face10': FaceDetails('long'    , 'bright'  ),
    'face11': FaceDetails('short'   , 'bright'  ),  # TODO: -||-
    'face12': FaceDetails('short'   , 'dark'    ),
    'face13': FaceDetails('long'    , 'bright'  ),
    'face14': FaceDetails('long'    , 'bright'  ),
    'face15': FaceDetails('long'    , 'bright'  ),
    'face16': FaceDetails('short'   , 'dark'    ),
    'face17': FaceDetails('long'    , 'dark'    ),
    'face18': FaceDetails('short'   , 'dark'    ),
    'face19': FaceDetails('short'   , 'bright'  ),
    'face20': FaceDetails('short'   , 'dark'    ),
    'face21': FaceDetails('short'   , 'bright'  ),
}

def get_request(url):
    """Performs a web request to the specified url"""
    response = urllib2.urlopen(url)
    text = response.read()

    data = {}
    for line in text.split('\n'):
        key, value = line.strip().split(':')
        data[key] = value
    
    return data
