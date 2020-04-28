#!/usr/bin/env python

import rospy
from nav_msgs.srv import GetMap
from nav_msgs.msg import OccupancyGrid

import numpy, math
import cv2
import random

def get_field(point, map_data):
    robot_view_radius = 20 # 25 * 0.05 m = 1.25 m
    number_of_rays = 18

    intersections = []
    for ray_index in range(number_of_rays):
        ray_direction = ray_index * (2 * numpy.pi / number_of_rays)
        
        current_distance = 0
        intersection = None
        while current_distance < robot_view_radius:
            x = int(math.cos(ray_direction) * current_distance) + point[0]
            y = int(math.sin(ray_direction) * current_distance) + point[1]
            if map_data[y, x] != 0:
                intersection = (x, y)
                break
            current_distance += 0.5
        
        if intersection is not None:
            intersections.append(intersection)
        else:
            x = int(math.cos(ray_direction) * current_distance) + point[0]
            y = int(math.sin(ray_direction) * current_distance) + point[1]
            intersections.append((x, y))
    
    return intersections

def poisson_disc_samples(width, height, r, k=5):
    cellsize = r / math.sqrt(2)

    grid_width = int(math.ceil(width / cellsize))
    grid_height = int(math.ceil(height / cellsize))
    grid = [None] * (grid_width * grid_height)

    def grid_coordinates(point):
        return int(math.floor(point[0] / cellsize)), int(math.floor(point[1] / cellsize))

    def fits(p, gx, gy):
        yrange = list(range(max(gy - 2, 0), min(gy + 3, grid_height)))
        for x in range(max(gx - 2, 0), min(gx + 3, grid_width)):
            for y in yrange:
                g = grid[x + y * grid_width]
                if g is None:
                    continue
                if (p[1] - g[1]) * (p[1] - g[1]) + (p[0] - g[0]) * (p[0] - g[0])  <= r * r:
                    return False
        return True

    initial_point = width * random.random(), height * random.random()
    queue = [initial_point]
    grid_x, grid_y = grid_coordinates(initial_point)
    grid[grid_x + grid_y * grid_width] = initial_point
    p = initial_point

    while queue:
        qi = int(random.random() * len(queue))
        qx, qy = queue[qi]
        queue[qi] = queue[-1]
        queue.pop()
        for _ in range(k):
            alpha = 2 * math.pi * random.random()
            d = r * math.sqrt(3 * random.random() + 1)
            px = qx + d * math.cos(alpha)
            py = qy + d * math.sin(alpha)
            if not (0 <= px < width and 0 <= py < height):
                continue
            p = (px, py)
            grid_x, grid_y = grid_coordinates(p)
            if not fits(p, grid_x, grid_y):
                continue
            queue.append(p)
            grid[grid_x + grid_y * grid_width] = p
    return [p for p in grid if p is not None]

class MapMaker(object):

    def __init__(self):
        self.get_map = rospy.ServiceProxy('static_map', GetMap)
        rospy.wait_for_service('static_map')
    
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
        non_occupied_pixels = numpy.count_nonzero(non_occupied)

        non_occupied = cv2.morphologyEx(non_occupied, cv2.MORPH_ERODE, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (10, 10)))
        # cv2.imshow('test', non_occupied)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()

        # Find our map bounding box. Do that by finding indices of columns and
        # rows that contain element that is not False, then find the minimum and
        # maximum row and column.
        rows = numpy.any(non_occupied, axis=1)
        columns = numpy.any(non_occupied, axis=0)
        top, bottom = numpy.where(rows)[0][[0, -1]]
        left, right = numpy.where(columns)[0][[0, -1]]

        # Each map cell (or pixel) contains a probability in the inverval [0,
        # 100] of the cell being occupied or -1 if the value is not known.
        map_data[map_data == 100] = 255
        map_data[map_data == -1] = 0
        map_data = map_data.astype('uint8')

        map_image = numpy.empty_like(map_data)
        map_image[:] = map_data

        # Now that we have found our bounding rectangle, we can sample random
        # points inside of it. If the point doesn't lie inside non_occupied
        # area, we can simply ignore it.
        points = map(lambda point: (int(point[0] + left), int(point[1] + top)), poisson_disc_samples(width=right - left, height=bottom - top, r=5))
        points = filter(lambda point: non_occupied[point[1],point[0]], points)
        
        selected_points = []
        while numpy.any(non_occupied) and len(points) > 0:

            best_point = points[0]
            best_field = None
            best_number_of_pixels = 0

            for point in points:
                field = numpy.array(get_field(point, map_data))
                mask = numpy.zeros_like(non_occupied).astype('uint8')
                cv2.drawContours(mask, [field], 0, (255,255,255), -1)

                # test = cv2.bitwise_and(non_occupied, non_occupied, mask = mask)
                test = cv2.bitwise_and(map_data, map_data, mask = mask)

                number_of_pixels = numpy.count_nonzero(test)
                if number_of_pixels > best_number_of_pixels:
                    best_number_of_pixels = number_of_pixels
                    best_point = point
                    best_field = field

            point = best_point
            points.remove(point)

            # Convert point from pixels to map coordinates
            converted_point_x = occupancy_grid.info.origin.position.x + point[0] * occupancy_grid.info.resolution
            converted_point_y = occupancy_grid.info.origin.position.y + point[1] * occupancy_grid.info.resolution
            selected_points.append((converted_point_x, converted_point_y))

            field = best_field
            cv2.drawContours(non_occupied, [field], 0, (0,0,0), -1)
            cv2.drawContours(map_data, [field], 0, (255,255,255), -1)
            
            if numpy.count_nonzero(non_occupied) <= 0.01 * non_occupied_pixels:
                break
        
        return selected_points

if __name__ == '__main__':
    map_maker = MapMaker()
    print(map_maker.generate_points())
