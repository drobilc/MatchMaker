#!/usr/bin/env python

import rospy
from nav_msgs.srv import GetMap
from nav_msgs.msg import OccupancyGrid

import numpy
import cv2
from scipy.spatial import Delaunay
from scipy.spatial.distance import pdist
from scipy.cluster.hierarchy import fcluster, single, centroid

class MapMaker(object):

    def __init__(self):
        self.get_map = rospy.ServiceProxy('static_map', GetMap)
        rospy.wait_for_service('static_map')
    
    def non_maxima_suppression(self, image, region_size=5):
        height, width = image.shape
        new_image = numpy.zeros_like(image)
        for y in range(len(image)):
            for x in range(len(image[y])):
                left_x = max(0, x - region_size)
                right_x = min(width, x + region_size)
                top_y = max(0, y - region_size)
                bottom_y = min(height, y + region_size)
                region = image[top_y:bottom_y,left_x:right_x]
                region_maximum = numpy.max(region)
                if region_maximum != 0 and image[y,x] == region_maximum:
                    new_image[y,x] = 255
        return new_image

    def too_close_to_the_wall(self, map_data, point, neighbourhood_size):
        is_too_close_to_the_wall = False

        #create neighbourhood
        neighbourhood_base = [0]
        for i in range(1, neighbourhood_size + 1):
            neighbourhood_base.append(i)
            neighbourhood_base.append(i*(-1))

        neighbourhood = []
        for neighbour_x in neighbourhood_base:
            for neighbour_y in neighbourhood_base:
                neighbourhood.append([point[0] + neighbour_x, point[1] + neighbour_y])
        
        # Check if any of the neighbours lays outside of free space
        for member in neighbourhood:
            suspect = map_data[member[1]][member[0]]
            if suspect < 250:
                is_too_close_to_the_wall = True
                return is_too_close_to_the_wall
        return is_too_close_to_the_wall

    def move_away_from_the_wall(self, map_data, point, too_close):
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

            extended_neighbours_right.append([point[0] + 2 * neighbour, point[1]])
            extended_neighbours_left.append([point[0] + (-1) * 2* neighbour, point[1]])
            extended_neighbours_down.append([point[0], point[1] + 2 *neighbour])
            extended_neighbours_up.append([point[0], point[1] + (-1) * 2* neighbour])
            extended_neighbours_right.append([point[0] + 2 * neighbour + 1, point[1]])
            extended_neighbours_left.append([point[0] + (-1) * 2 * neighbour + 1, point[1]])
            extended_neighbours_down.append([point[0], point[1] + 2 * neighbour + 1])
            extended_neighbours_up.append([point[0], point[1] + (-1) * 2 * neighbour + 1])
        
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

            if map_data[extended_neighbours_right[2 * i -1][1]][extended_neighbours_right[2 * i -1][0]] < 250:
                wall_to_the_right = False
            if map_data[extended_neighbours_right[2 * i][1]][extended_neighbours_right[2 * i][0]] < 250:
                wall_to_the_right = False
            if map_data[extended_neighbours_right[2 * i -1][1]][extended_neighbours_right[2 * i -1][0]] < 250:
                wall_to_the_right = False
            if map_data[extended_neighbours_right[2 * i][1]][extended_neighbours_right[2 * i][0]] < 250:
                wall_to_the_right = False
            if map_data[extended_neighbours_right[2 * i -1][1]][extended_neighbours_right[2 * i -1][0]] < 250:
                wall_to_the_right = False
            if map_data[extended_neighbours_right[2 * i][1]][extended_neighbours_right[2 * i][0]] < 250:
                wall_to_the_right = False
            if map_data[extended_neighbours_right[2 * i -1][1]][extended_neighbours_right[2 * i -1][0]] < 250:
                wall_to_the_right = False
            if map_data[extended_neighbours_right[2 * i][1]][extended_neighbours_right[2 * i][0]] < 250:
                wall_to_the_right = False

        # For each wall to close, move the point too_close pixels in the opposite direction
        if wall_to_the_right and free_to_move_right:
            point[0] -= too_close
        if wall_to_the_left and free_to_move_left:
            point[0] += too_close
        if wall_to_the_bottom and free_to_move_down:
            point[1] -= too_close
        if wall_to_the_top and free_to_move_up:
            point[1] += too_close

        return point
    
    def generate_points(self):
        occupancy_grid = self.get_map().map

        width, height = occupancy_grid.info.width, occupancy_grid.info.height

        map_data = numpy.asarray(occupancy_grid.data)
        map_data = numpy.reshape(map_data, (height, width))

        # Get only the non-occupied parts of the map (the parts that have value
        # of 0). Then, find bounding box of this region on our map.
        non_occupied = map_data == 0
        non_occupied = non_occupied.astype('uint8')
        non_occupied = non_occupied * 255

        walls = map_data == 100
        walls = walls.astype('uint8')
        walls = walls * 255

        # Erode the image using a circular kernel, because our robot is circular.
        # This will remove a part of the free space that can't be visited by robot
        # because it is too close to the wall. The robot's radius is 18 cm, if the
        # map resolution is 0.05 meters / pixels that means, that we must erode 20
        # cm / 5 cm = 4 pixels. We can also include 10 cm of padding.
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (6, 6))

        non_occupied = cv2.dilate(non_occupied, kernel, iterations=1)
        non_occupied = cv2.erode(non_occupied, kernel, iterations=1)
        # non_occupied = cv2.dilate(non_occupied, kernel, iterations=1)

        # Find our map bounding box. Do that by finding indices of columns and
        # rows that contain element that is not False, then find the minimum and
        # maximum row and column.
        rows = numpy.any(non_occupied, axis=1)
        columns = numpy.any(non_occupied, axis=0)
        top, bottom = numpy.where(rows)[0][[0, -1]]
        left, right = numpy.where(columns)[0][[0, -1]]
        top -= 10
        bottom += 10
        left -= 10
        right += 10

        free_space = non_occupied[top:bottom, left:right]
        only_walls = walls[top:bottom, left:right]

        # Find corners using either Harris corner detector
        corners = cv2.cornerHarris(only_walls, blockSize=3, ksize=5, k=0.04)
        # Apply non-maxima-suppression, so we only get the most probable corners
        corners = self.non_maxima_suppression(corners, region_size=3)

        # Get corner map coordinates (y, x)
        corner_positions = numpy.argwhere(corners)

        # Perform triangulation on corner coordinates (using Delaunay triangulation)
        triangulation = Delaunay(corner_positions)

        # Function that computes triangle area, used for filtering too small triangles
        def triangle_area(triangle_points):
            a, b, c = triangle_points
            return (a[0] * (b[1] - c[1]) + b[0] * (c[1] - a[1]) + c[0] * (a[1] - b[1])) / 2

        # Function that returns center of triangle
        def triangle_centroid(triangle_points):
            a, b, c = triangle_points
            y = (a[0] + b[0] + c[0]) / 3
            x = (a[1] + b[1] + c[1]) / 3
            return numpy.asarray([int(y), int(x)])

        # Filter triangles that are too small and their center doesn't lie inside
        # free space
        min_area = 0
        min_distance=18
        points = []
        # filtered_triangles = []

        for i, triangle in enumerate(triangulation.simplices):
            triangle_points = corner_positions[triangle]

            # Compute center point of the triangle
            center_y, center_x = triangle_centroid(triangle_points)
            
            if triangle_area(triangle_points) < min_area:
                continue
            
            if free_space[center_y, center_x] != 255:
                continue

            if self.too_close_to_the_wall(free_space, [center_x, center_y], 1):
                continue
            
            # The center point of the triangle is the point that our robot has to
            # visit. The free_space image has been cropped, so transform the
            # coordinate back to original map coordinate.
            points.append([left + center_x, top + center_y])

        # If the map is big enough (has enough corners), perform clustering
        if len(points) > 10:
            # Perform hierarhical clustering with stooping criteria being the min_distance between points (play with this parameter)
            distances = pdist(points)
            clustering_data = centroid(distances)
            clustered = fcluster(clustering_data, min_distance, criterion="distance")

            # Figure out how to join the points within the same cluster
            new_points = []
            max_cluster_id = max(clustered)
            for cluster_id in range(1, max_cluster_id + 1):
                # For each possible cluster find all point that are assigned to it
                in_cluster = []
                for point_index in enumerate(points):
                    if clustered[point_index[0]] == cluster_id:
                        in_cluster.append(points[point_index[0]])
                # Convert these points to one point -> averaging (for now)
                sum_x = 0
                sum_y = 0
                for cluster_member in in_cluster:
                    sum_x += cluster_member[0]
                    sum_y += cluster_member[1]
                avg_x = sum_x / len(in_cluster)
                avg_y = sum_y / len(in_cluster)
                new_points.append([avg_x, avg_y])
        else:
            new_points = points

        # If any of the points is too close to the wall, move it away from it
        # First check if the point is too close to the wall.
        for point in new_points:
            if self.too_close_to_the_wall(non_occupied, point, 3):
                # if so, find out in which of the four basic directions do we have a wall 
                # within 3px and move it in the opposite direction for 3px
                point = self.move_away_from_the_wall(non_occupied, point, 4)

        # for point in new_points:
        #     point[0] += center_x
        #     point[1] += center_y


        # Convert point from pixels to map coordinates
        for point in new_points:
            point[0] = occupancy_grid.info.origin.position.x + point[0] * occupancy_grid.info.resolution
            point[1] = occupancy_grid.info.origin.position.y + point[1] * occupancy_grid.info.resolution

        return new_points

if __name__ == '__main__':
    map_maker = MapMaker()
    print(map_maker.generate_points())
