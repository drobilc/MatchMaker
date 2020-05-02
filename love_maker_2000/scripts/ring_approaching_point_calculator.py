#!/usr/bin/env python

import rospy
import math
import numpy

from tf.transformations import quaternion_from_euler

from geometry_msgs.msg import Pose, Quaternion
from nav_msgs.srv import GetMap
from love_maker_2000.srv import ApproachingPointCalculator, ApproachingPointCalculatorResponse

class RingApproachingPointCalculator():
    def __init__(self):
        rospy.init_node('ring_approaching_point_calculator')

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

        service = rospy.Service('ring_approaching_point_calculator', RingApproachingPointCalculator, self.handle_approaching_point_calculation_request)

    def to_pixel(self, position):
        # Convert position from map coordinates in m to pixel position (u, v) 
        pixel_x = int((position.point.x - self.map_origin.x) / self.map_resolution)
        pixel_y = int((position.point.y - self.map_origin.y) / self.map_resolution)
        return numpy.asarray([pixel_x, pixel_y])

    def from_pixel(self, pixel, original_position):
        point_xs = []
        point_ys = []
        errors = [-10, -9, -8, -7, -6, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
        for error in errors:
            point_xs.append((pixel[0] + error) * self.map_resolution + self.map_origin.x)
            point_ys.append((pixel[1] + error) * self.map_resolution + self.map_origin.y)
        
        min_x_distance = 100
        min_y_distance = 100
        best_x = 0
        best_y = 0

        for point_x, point_y in zip(point_xs, point_ys):
            x_distance = numpy.linalg.norm(original_position.position.x - point_x)
            y_distance = numpy.linalg.norm(original_position.position.y - point_y)
            if x_distance < min_x_distance:
                min_x_distance = x_distance
                best_x = point_x
            if y_distance < min_y_distance:
                min_y_distance = y_distance
                best_y = point_y

        best_pixel_point = Pose()
        best_pixel_point.position.x = best_x
        best_pixel_point.position.y = best_y
        best_pixel_point.position.z = 0

    
    def closest_wall_pixel(self, pixel, max_distance=5):
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
    
    def calculate_approaching_point(self, detection):

        # Find nearest wall point (by distance from the detection)
        detection.position.z = 0 # project detection on the floor
        detection_pixel = self.to_pixel(detection)
        nearest_wall_pixel = self.closest_wall_pixel(detection_pixel, max_distance=3)
        nearest_wall_point = self.from_pixel(nearest_wall_pixel, detection)

        # Calculate orientation vector from wall to ring
        orientation_parameters = []
        orientation_parameters.append(detection.position.x - nearest_wall_point.position.x)
        orientation_parameters.append(detection.position.y - nearest_wall_point.position.y)
        orientation_parameters.append(0)

        # get orientation quaternion from orientation vector
        orientation = math.atan2(orientation_parameters[1], orientation_parameters[0])
        orientation_quaternion = quaternion_from_euler(0, 0, orientation + math.pi)
        orientation_quaternion = Quaternion(orientation_quaternion[0], orientation_quaternion[1], orientation_quaternion[2], orientation_quaternion[3])

        # Normalize orientation vector
        normalized_orientation = orientation_parameters / numpy.linalg.norm(orientation_parameters)
        
        # Put approaching point ~ 20cm from wall (9cm from detection) in calculated direction
        # Set approaching point 0.09m away from detected object
        approaching_point = Pose()
        approaching_point.position.x = detection.x + 0.09 * normalized_orientation[0]
        approaching_point.position.y = detection.y + 0.09 * normalized_orientation[1]
        approaching_point.position.z = detection.z + 0.09 * normalized_orientation[2]

        # Calculate direction perpendicular to this one
        rotation = quaternion_from_euler(0, 0, -90)
        approaching_point.orientation = rotation * orientation_quaternion

        # Set this as approaching_point orientation
        return approaching_point

    def handle_approaching_point_calculation_request(self, detection):
        return self.calculate_approaching_point(detection)

if __name__ == '__main__':
    color_classifier = RingApproachingPointCalculator()
    rospy.loginfo('Ring approaching point calculator node started')
    rospy.spin()