#!/usr/bin/env python
import roslib
roslib.load_manifest('localizer')
import rospy
import sys, select, termios, tty
from std_msgs.msg import String, Bool, ColorRGBA
import sensor_msgs.msg
import message_filters
import collections
from geometry_msgs.msg import PoseStamped
from detection_msgs.msg import Detection
from localizer.srv import Localize
from sensor_msgs.msg import CameraInfo
from visualization_msgs.msg import Marker, MarkerArray
from image_geometry import PinholeCameraModel
from geometry_msgs.msg import Point, Vector3

import math
import numpy as np
import numpy

from geometry_msgs.msg import Pose, PoseStamped, Vector3, Point, Quaternion
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA

from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf2_ros
from tf2_geometry_msgs import PointStamped

import utils

from nav_msgs.srv import GetMap
from nav_msgs.msg import OccupancyGrid
import cv2

class DetectionMapper():

    def __init__(self):
        rospy.init_node('mapper', anonymous=False)

        self.region_scope = rospy.get_param('~region', 1)
        self.buffer_size = rospy.get_param('~camera_buffer_size', 50)
        rospy.wait_for_service('localizer/localize')

        self.get_map = rospy.ServiceProxy('static_map', GetMap)
        rospy.wait_for_service('static_map')

        self.occupancy_grid = self.get_map().map
        width, height = self.occupancy_grid.info.width, self.occupancy_grid.info.height
        self.map_resolution = self.occupancy_grid.info.resolution
        self.map_origin = self.occupancy_grid.info.origin.position
        self.map_data = numpy.asarray(self.occupancy_grid.data)
        self.map_data = numpy.reshape(self.map_data, (height, width))
        self.map_data[self.map_data == 100] = 255
        self.map_data[self.map_data == -1] = 0
        self.map_data = self.map_data.astype('uint8')

        self.camera_infos = collections.deque(maxlen = self.buffer_size)
        self.detections_sub = message_filters.Subscriber('detections', Detection)
        self.detections_sub.registerCallback(self.detections_callback)

        self.camera_sub = message_filters.Subscriber('/camera/rgb/camera_info', CameraInfo)
        self.camera_sub.registerCallback(self.camera_callback)

        self.localize = rospy.ServiceProxy('localizer/localize', Localize)

        self.approaching_point_publisher = rospy.Publisher('/face_detections_raw', PoseStamped, queue_size=100)

        self.markers = MarkerArray()
        self.markers_publisher = rospy.Publisher('/faces', MarkerArray, queue_size=1000)

        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf)
    
    def compute_approaching_point(self, detected_face_position):
        # To compute approaching point we should do the following: First, we
        # should get the detected face position and robot position in map
        # coordinates. Then, the face and robot map coordinates should be
        # converted to map pixels. A small region around the face pixel should
        # be extracted where hough line transform should be performed. After
        # lines have been found, they should be sorted by their scalar products
        # with the face - robot vector, so that lines that.
        
        # Get face position in map coordinate system
        face_point = PointStamped()
        face_point.header.frame_id = detected_face_position.header.frame_id
        face_point.header.stamp = detected_face_position.header.stamp
        face_point.point.x = detected_face_position.pose.position.x
        face_point.point.y = detected_face_position.pose.position.y
        face_point.point.z = detected_face_position.pose.position.z
        face_point = self.tf_buf.transform(face_point, "map")

        # Get the robot position in map coordinate system
        robot_point = PointStamped()
        robot_point.header.frame_id = "camera_depth_optical_frame"
        robot_point.header.stamp = detected_face_position.header.stamp
        robot_point = self.tf_buf.transform(robot_point, "map")
        
        def to_map_pixel(position):
            # Convert position from map coordinates in m to pixel position (u, v) 
            pixel_x = int((position.point.x - self.map_origin.x) / self.map_resolution)
            pixel_y = int((position.point.y - self.map_origin.y) / self.map_resolution)
            return numpy.asarray([pixel_x, pixel_y])
        
        def closest_wall_pixel(pixel, max_distance=5):
            # Use flood fill algorithm with breadth first search to find the
            # closest wall pixel. Use neighboring pixels as defined below
            neighborhood = [(-1, 0), (0, 1), (1, 0), (0, -1)]

            # First, extract a small region around the pixel (a square region of
            # size 2 * max_distance + 1)
            map_region = self.map_data[pixel[1]-max_distance-1:pixel[1]+max_distance, pixel[0]-max_distance-1:pixel[0]+max_distance]
            
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

        def closest_line(face_pixel, wall_pixel, max_distance=5):
            map_region = self.map_data[face_pixel[1]-max_distance-1:face_pixel[1]+max_distance, face_pixel[0]-max_distance-1:face_pixel[0]+max_distance]
            x0, y0 = wall_pixel - face_pixel + max_distance
            lines = cv2.HoughLinesP(map_region, 1, 1, 5, 5, 2)
            best_line = None
            best_distance = 100000
            for line in lines:
                start = numpy.asarray([line[0][0], line[0][1]])
                end = numpy.asarray([line[0][2], line[0][3]])
                distance = abs(((end[1]-start[1])*x0 - (end[0]-start[0])*y0 + end[0]*start[1] - end[1]*start[0]) / np.linalg.norm(end - start))
                if distance < best_distance:
                    best_distance = distance
                    best_line = (start, end)
            return best_line, best_distance

        # Convert map coordinates to map pixel coordinates
        face_pixel = to_map_pixel(face_point)
        robot_pixel = to_map_pixel(robot_point)        

        # Find the closest wall pixel and line that passes closest to that wall
        # pixel (preferably line that goes through the wall pixel)
        closest_wall = closest_wall_pixel(face_pixel)
        line, distance = closest_line(face_pixel, closest_wall)

        # Compute which side of the line robot currently is
        line_direction = line[1] - line[0]
        normal = numpy.asarray([line_direction[1], -line_direction[0]])
        orientation = normal
        if numpy.dot(robot_pixel - closest_wall, normal) < 0:
            orientation = -orientation
        
        # Now that we have a wall vector and face position, we can calculate the
        # approaching point
        face_orientation = orientation / numpy.linalg.norm(orientation)
        approaching_point = numpy.asarray([self.map_origin.x, self.map_origin.y]) + (face_pixel * self.map_resolution) + face_orientation * 0.5

        orientation = -orientation
        orientation = math.atan2(orientation[1], orientation[0])
        orientation_quaternion = quaternion_from_euler(0, 0, orientation)
        orientation_quaternion = Quaternion(*orientation_quaternion)

        approaching_pose = PoseStamped()
        approaching_pose.header = face_point.header
        approaching_pose.header.frame_id = 'map'
        approaching_pose.pose.position.x = approaching_point[0]
        approaching_pose.pose.position.y = approaching_point[1]
        approaching_pose.pose.position.z = 0
        approaching_pose.pose.orientation = orientation_quaternion

        return approaching_pose

    def camera_callback(self, camera_info):
        self.camera_infos.append(camera_info)

    def detections_callback(self, detection):
        u = detection.x + detection.width / 2
        v = detection.y + detection.height / 2

        camera_info = None
        best_time = 100
        for ci in self.camera_infos:
            time = abs(ci.header.stamp.to_sec() - detection.header.stamp.to_sec())
            if time < best_time:
                camera_info = ci
                best_time = time

        if not camera_info or best_time > 1:
            return

        camera_model = PinholeCameraModel()
        camera_model.fromCameraInfo(camera_info)

        point = Point(((u - camera_model.cx()) - camera_model.Tx()) / camera_model.fx(),
             ((v - camera_model.cy()) - camera_model.Ty()) / camera_model.fy(), 1)

        localization = self.localize(detection.header, point, self.region_scope)

        if not localization:
            return
        
        # Construct a PoseStamped at position of the face and publish it
        pose = PoseStamped()
        pose.header.stamp = detection.header.stamp
        pose.header.frame_id = detection.header.frame_id
        pose.pose = localization.pose

        face_marker = utils.stamped_pose_to_marker(pose, index=len(self.markers.markers), color=ColorRGBA(1, 0, 1, 1))
        self.markers.markers.append(face_marker)
        self.markers_publisher.publish(self.markers)

        # Compute where the approaching point for this face is. Then send this
        # data to robustifier.
        try:
            approaching_point = self.compute_approaching_point(pose)
            self.approaching_point_publisher.publish(approaching_point)
        except Exception as e:
            rospy.logerr(e)
   
if __name__ == '__main__':
    mapper = DetectionMapper()
    rospy.spin()
