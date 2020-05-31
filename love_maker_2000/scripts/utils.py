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
    i = 0

    while not frontier.empty():
        current = frontier.get()
        if map_data[current[1], current[0]] != 255 and i > 0:
            return current
        for next in neighbors_all(current):
            if not visited[next[1], next[0]]:
                frontier.put(next)
                visited[next[1], next[0]] = True
        i += 1
    
    rospy.logwarn("Couldn't find closest non-wall pixel")
    return None

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
            if new_x < 0 or new_y < 0 or new_x >= max_distance * 2 or new_y >= max_distance * 2 or visited[new_y, new_x] or (new_x, new_y) in frontier:
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
    def __str__(self):
        return "{}, {}".format(self.hair_length, self.hair_color)
    def __eq__(self, other):
        return self.hair_color == other.hair_color and self.hair_length == other.hair_length

FACE_DETAILS = {
    'face01': FaceDetails('short'   , 'dark'    ),
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
