#!/usr/bin/env python
from __future__ import print_function

import sys
import rospy
import cv2
import numpy, math
import message_filters

import utils

import tf2_ros
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from tf2_geometry_msgs import PointStamped

from std_msgs.msg import Header, ColorRGBA, Bool
from detection_msgs.msg import Detection
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, Pose, Quaternion, Point, PointStamped
from object_detection_msgs.msg import ObjectDetection

from cv_bridge import CvBridge, CvBridgeError
from image_geometry import PinholeCameraModel

from color_classification.srv import ColorClassification, ColorClassificationResponse
from nav_msgs.srv import GetMap

class RingDetector(object):
    def __init__(self):
        rospy.init_node('ring_detector_beta', anonymous=False)
        rospy.loginfo('Ring detector started')

        self.bridge = CvBridge()

        # Subscriber to enable or disable ring detector
        self.enabled = True
        self.toggle_subscriber = rospy.Subscriber('/ring_detector_toggle', Bool, self.toggle, queue_size=10)

        # Create a new time synchronizer to synchronize depth and rgb image callbacks.
        # Also subscribe to camera info so we can get camera calibration matrix.
        self.depth_image_subscriber = message_filters.Subscriber('/camera/depth/image_raw', Image)
        self.image_subscriber = message_filters.Subscriber('/camera/rgb/image_raw', Image)
        self.camera_info_subscriber = message_filters.Subscriber('/camera/rgb/camera_info', CameraInfo)
        self.image_synchronizer = message_filters.TimeSynchronizer([self.depth_image_subscriber, self.image_subscriber, self.camera_info_subscriber], 10)
        self.image_synchronizer.registerCallback(self.on_data_received)

        # Color classification service
        rospy.wait_for_service('color_classifier')
        self.classify_color = rospy.ServiceProxy('color_classifier', ColorClassification)

        # Publisher for ring ObjectDetections
        self.detections_publisher = rospy.Publisher('/ring_detections_raw', ObjectDetection, queue_size=10)

        # Transformation buffer nd listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Get map data from map server
        rospy.wait_for_service('static_map')
        get_map = rospy.ServiceProxy('static_map', GetMap)
        self.occupancy_grid = get_map().map
        width, height = self.occupancy_grid.info.width, self.occupancy_grid.info.height
        self.map_resolution = self.occupancy_grid.info.resolution
        self.map_origin = self.occupancy_grid.info.origin.position
        self.map_data = numpy.asarray(self.occupancy_grid.data)
        self.map_data = numpy.reshape(self.map_data, (height, width))
        self.map_data[self.map_data == 100] = 255  # walls
        self.map_data[self.map_data == -1] = 0  # free space
        self.map_data = self.map_data.astype('uint8')
    
    def toggle(self, enable):
        """Callback to enable or disable ring detector"""
        self.enabled = enable.data
        rospy.loginfo('Ring detector enabled: {}'.format(self.enabled))
    
    def on_data_received(self, depth_image_message, image_message, camera_info):
        """Callback for when depth image, rgb image and camera information is received"""
        if not hasattr(self, "enabled") or not self.enabled:
            return
        
        # Because of the TimeSynchronizer, depth image, rgb image and camera
        # information have the same timestamps.
        timestamp = depth_image_message.header.stamp

        # Convert images so we can process it with the cv2 library
        depth_image = self.bridge.imgmsg_to_cv2(depth_image_message, "32FC1")
        rgb_image = self.bridge.imgmsg_to_cv2(image_message, "bgr8")

        # Construct a new PinholeCameraModel from received camera information.
        # This can be used to create rays from camera center to points in 3d
        # world.
        camera_model = PinholeCameraModel()
        camera_model.fromCameraInfo(camera_info)

        self.detect_circles(depth_image, rgb_image, camera_model, timestamp)
    
    def detect_circles(self, depth_image, rgb_image, camera_model, timestamp, circle_padding=5, maximum_distance=2.5):
        """Detect circles in depth image and send detections to ring robustifier"""
        # Disparity is computed in meters from the camera center
        disparity = numpy.copy(depth_image)

        # This part is needed to detect circles in image with better precision
        depth_image = numpy.nan_to_num(depth_image)
        depth_image[depth_image > 3] = 0
        minimum = numpy.min(depth_image)
        maximum = numpy.max(depth_image)
        depth_image = (depth_image - minimum) / (maximum - minimum)
        depth_image = depth_image * 255
        depth_image = numpy.floor(depth_image)
        depth_image = depth_image.astype(numpy.uint8)

        # depth_image = cv2.GaussianBlur(depth_image, (3, 3), 0)

        # Object for specyfiying detection parameters, it will not be used in this case
        params = cv2.SimpleBlobDetector_Params()
        # Filter by circularity (no hard edges, shape is elliptical)
        params.filterByCircularity = True
        params.minCircularity = 0.8
        # Set convexity of the blobs
        params.filterByConvexity = True
        params.minConvexity = 0.2
        # Set inertia bounds, 1 -> circle, 0 -> line, 0 - 1 -> ellipse
        params.filterByInertia = False
        params.minInertiaRatio = 0.01

        # Detect blobs, if we wanted to specify custom detection parameters
        # we would pass the params object to the below constructor as argument
        detector = cv2.SimpleBlobDetector_create()
        keypoints = detector.detect(depth_image)

        # Filter out false positives, blobs that are too low to be rings
        height_threshold = (len(depth_image) * 4) // 9
        true_ring_keypoints = []
        for keypoint in keypoints:
            if keypoint.pt[1] < height_threshold:
                true_ring_keypoints.append(keypoint)
        
        for keypoint in true_ring_keypoints:
            # Compute the ring bounding box from its radius and center
            center_x, center_y = keypoint.pt
            radius = keypoint.size / 2.0 + circle_padding
            left, right = int(center_x - radius), int(center_x + radius)
            top, bottom = int(center_y - radius), int(center_y + radius)
            
            # Crop a small region around the ring
            depth_region = numpy.copy(depth_image[top:bottom, left:right])
            image_region = numpy.copy(rgb_image[top:bottom, left:right])
            disparity_region = disparity[top:bottom, left:right]

            # Try to filter depth_region to only include circle (basically, get
            # the highest bin in histogram and filter out everything that is not
            # in this bin)
            histogram, bins = numpy.histogram(depth_region[depth_region != 0], bins=24, range=(0, 255))
            best_bin = numpy.argmax(histogram)
            depth_region[numpy.abs(depth_region - bins[best_bin]) > 255/24] = 0
            
            depth_mask = depth_region > 0
            disparity_region = disparity_region * depth_mask
            image_region = image_region * depth_mask[...,None]

            # Compute the average ring color from image and then use the color
            # classification service to get color as string
            try:
                ring_color = self.get_ring_color(image_region)
                classified_color = self.classify_color(ring_color).classified_color
            except Exception:
                rospy.logwarn('Ring detected, but the ring color could not be determined')
                continue

            # Compute position of ring in 3d world coordinate system
            ring_position = self.to_world_position(keypoint, disparity_region, camera_model, timestamp, maximum_distance)

            # Send the ring position to robustifier, if it is 
            if ring_position is not None:
                _, approaching_point = self.compute_approaching_point(ring_position, timestamp)
                if approaching_point is not None:
                    detection = self.construct_detection_message(ring_position, approaching_point, classified_color, timestamp)
                    self.detections_publisher.publish(detection)
                else:
                    rospy.logwarn('Ring detected, but the ring approaching point could not be determined')
            else:
                rospy.logwarn('Ring detected, but the ring position could not be determined')
    
    def to_world_position(self, keypoint, disparity_region, camera_model, timestamp, maximum_distance=2.5):
        # Get average distance to the ring
        average_distance = numpy.nanmean(disparity_region)

        if average_distance > maximum_distance:
            return None
        
        # Construct a ray from camera center to pixel on image, then stretch
        # it, so its length is the distance from camera to point in 3d space
        center_x, center_y = keypoint.pt
        ray = camera_model.projectPixelTo3dRay((center_x, center_y))
        point = (ray[0] * average_distance, ray[1] * average_distance, ray[2] * average_distance)
        
        # Convert 3d point from camera frame to world frame and publish it
        ring_camera = PoseStamped()
        ring_camera.pose.position.x = point[0]
        ring_camera.pose.position.y = point[1]
        ring_camera.pose.position.z = point[2]
        ring_camera.header.frame_id = "camera_rgb_optical_frame"
        ring_camera.header.stamp = timestamp

        try:
            ring_position = self.tf_buffer.transform(ring_camera, "map")
            return ring_position
        except Exception as e:
            rospy.logwarn(e)
            return None

    def construct_detection_message(self, ring_pose, approaching_point, classified_color, timestamp):
        """Construct ObjectDetection from ring detection"""
        detection = ObjectDetection()
        detection.header.frame_id = 'map'
        detection.header.stamp = timestamp
        detection.object_pose = ring_pose.pose
        detection.approaching_point_pose = approaching_point.pose
        detection.color = classified_color
        detection.type = 'ring'
        return detection

    def get_ring_color(self, image_region):
        """Get average color of image_region but ignore all black pixels"""        
        pixels = image_region.reshape(-1, 3)
        pixel_mask = numpy.any(pixels != [0, 0, 0], axis=-1)
        
        non_black_pixels = pixels[pixel_mask]
        average_color = numpy.mean(non_black_pixels, axis=0)

        return ColorRGBA(int(average_color[2]), int(average_color[1]), int(average_color[0]), 255)
    
    def compute_approaching_point(self, ring_position, timestamp):
        # To compute approaching point we should do the following: First, we
        # should get the detected face position and robot position in map
        # coordinates. Then, the face and robot map coordinates should be
        # converted to map pixels. A small region around the face pixel should
        # be extracted where hough line transform should be performed. After
        # lines have been found, they should be sorted by their scalar products
        # with the face - robot vector, so that lines that.

        ring_point = PointStamped()
        ring_point.header.frame_id = ring_position.header.frame_id
        ring_point.header.stamp = timestamp
        ring_point.point.x = ring_position.pose.position.x
        ring_point.point.y = ring_position.pose.position.y
        ring_point.point.z = ring_position.pose.position.z

        # Get the robot position in map coordinate system
        robot_point = PointStamped()
        robot_point.header.frame_id = "camera_depth_optical_frame"
        robot_point.header.stamp = timestamp
        robot_point = self.tf_buffer.transform(robot_point, "map")
        
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

        def closest_line(ring_pixel, wall_pixel, max_distance=8):
            map_region = self.map_data[ring_pixel[1]-max_distance-1:ring_pixel[1]+max_distance, ring_pixel[0]-max_distance-1:ring_pixel[0]+max_distance]
            x0, y0 = wall_pixel - ring_pixel + max_distance
            lines = cv2.HoughLinesP(map_region, rho=1, theta=numpy.pi / 180.0, threshold=8, minLineLength=8, maxLineGap=3)
            if lines is None:
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

        # TRENUTNO STANJE
        # Se vedno se zgodi situacija, ko sta temp in closest free pixel enaka => crash.
        # Zadeva bi naj delovala tako, da ce je ring na prosti tocki, 
        # poiscemo najblizji zid in se delamo, da je na zidu. Ce je ze na zidu, ne storimo nicesar. 
        # Nato poiscemo najblizji pixel v okolici ringa, ki je brez zidu. 
        # Razlika nam da smer. Pazi, kako se obnasa linalg.norm, ce je nea stevilka 0.


        # Calculate approaching point position
        # if ring pixel is not in the wall, first find the closest wall
        temp_pixel = to_map_pixel(ring_point)
        temp_pixel_1 = to_map_pixel(ring_point)
        rospy.logwarn(temp_pixel)
        rospy.logwarn(self.map_data[temp_pixel_1[1]][temp_pixel_1[0]])
        if self.map_data[temp_pixel_1[1]][temp_pixel_1[0]] != 255:
            temp_pixel = closest_wall_pixel(temp_pixel_1, max_distance=20)
        
        temp_pixel = [temp_pixel[0], temp_pixel[1]]
        rospy.logwarn(temp_pixel)

        # Find closest free pixel 
        closest_free_pixel = utils.nearest_free_pixel(temp_pixel, self.map_data)
        direction = [closest_free_pixel[0] - temp_pixel[0], closest_free_pixel[1] - temp_pixel[1]]
        # closest_free_pixel_map = utils.pixel_to_map(direction, self.map_origin, self.map_resolution)
        #closest_free_pixel_map = direction / numpy.linalg.norm(direction)

        rospy.logwarn("closest_free_pixel and ring_pixel")
        rospy.logwarn(closest_free_pixel)
        rospy.logwarn(temp_pixel)
        rospy.logwarn("orientation vector")
        rospy.logwarn(direction)

        # Get vector direction from ring to closest free pixel - mogoce zamenjaj 0 in 1
        orientation_parameters = direction #[closest_free_pixel_map[0], closest_free_pixel_map[1], 0]

        # get orientation quaternion from orientation vector
        orientation = math.atan2(orientation_parameters[1], orientation_parameters[0]) # maybe switch the order
        orientation = quaternion_from_euler(0, 0, orientation + math.pi) # mogoce brez + pi
        orientation_quaternion = Quaternion(*orientation)

        # Normalize orientation vector
        normalized_orientation = orientation_parameters / numpy.linalg.norm(orientation_parameters)
        rospy.logwarn("normalized_orientation")
        rospy.logwarn(normalized_orientation)
        
        # Move in this direction for 0.2m
        approaching_point = Pose()
        approaching_point.position.x = ring_point.point.x + 0.2 * normalized_orientation[0]
        approaching_point.position.y = ring_point.point.y + 0.2 * normalized_orientation[1]
        approaching_point.position.z = 0
       
        # Calculate approaching point orientation
        # Convert map coordinates to map pixel coordinates
        #ring_pixel = to_map_pixel(ring_point)
        ring_pixel = numpy.asarray(closest_free_pixel)
        robot_pixel = to_map_pixel(robot_point)
        rospy.logwarn("approaching point:")
        rospy.logwarn(utils.pose_to_map_pixel(approaching_point, self.map_origin, self.map_resolution))

        # Find the closest wall pixel and line that passes closest to that wall
        # pixel (preferably line that goes through the wall pixel)
        closest_wall = closest_wall_pixel(ring_pixel)
        rospy.logwarn("closest wall pixel")
        rospy.logwarn(closest_wall)
        if closest_wall is None:
            return ring_position, None
        line, distance = closest_line(ring_pixel, closest_wall)
        if line is None:
            return ring_position, None

        # Use dot product to find direction of the wall. If dot product is < 0,
        # then the wall orientation is pointing in opposite direction as robot.
        # Flip it.
        line_direction = line[1] - line[0]
        orientation = numpy.asarray([line_direction[0], line_direction[1]])
        if numpy.dot(robot_pixel - closest_wall, orientation) < 0:
            orientation = -orientation
        
        # face_orientation = orientation / numpy.linalg.norm(orientation)

        orientation = -orientation
        orientation = math.atan2(orientation[1], orientation[0])
        orientation_quaternion = quaternion_from_euler(0, 0, orientation)
        orientation_quaternion = Quaternion(*orientation_quaternion)

        approaching_pose = PoseStamped()
        approaching_pose.header = ring_position.header
        approaching_pose.header.frame_id = 'map'
        approaching_pose.pose.position.x = approaching_point.position.x
        approaching_pose.pose.position.y = approaching_point.position.y
        approaching_pose.pose.position.z = approaching_point.position.z
        approaching_pose.pose.orientation = orientation_quaternion

        return ring_position, approaching_pose

if __name__ == '__main__':
    ring_detector = RingDetector()
    rospy.spin()

