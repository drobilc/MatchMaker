#!/usr/bin/env python
from __future__ import print_function

import sys
import rospy
import cv2
import numpy, math
import message_filters
from copy import deepcopy

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

def average(first_pose, second_pose):
    new_pose = PoseStamped()
    new_pose.header = second_pose.header
    new_pose.pose.position.x = first_pose.pose.position.x * 0.5 + second_pose.pose.position.x * 0.5
    new_pose.pose.position.y = first_pose.pose.position.y * 0.5 + second_pose.pose.position.y * 0.5
    new_pose.pose.position.z = first_pose.pose.position.z * 0.5 + second_pose.pose.position.z * 0.5
    return new_pose

def distance_between(first_pose, second_pose):
    # Return simple Euclidean distance between this and other pose
    dx = first_pose.position.x - second_pose.position.x
    dy = first_pose.position.y - second_pose.position.y
    dz = first_pose.position.z - second_pose.position.z
    return numpy.sqrt(dx*dx + dy*dy + dz*dz)

def get_closest(rings, detection, max_distance=0.5):
    closest_ring = None
    distance_closest = 999999999
    for ring in rings:
        distance = distance_between(ring[0].pose, detection.pose)
        if distance < distance_closest:
            closest_ring = ring
            distance_closest = distance
    
    if distance_closest > max_distance:
        return None
    return closest_ring

class RingDetector(object):
    def __init__(self):
        rospy.init_node('ring_detector_beta', anonymous=False)
        rospy.loginfo('Ring detector started')

        self.bridge = CvBridge()

        # Subscriber to enable or disable ring detector
        self.enabled = True
        self.toggle_subscriber = rospy.Subscriber('/ring_detector_toggle', Bool, self.toggle, queue_size=10)

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
        self.map_data[self.map_data == -1] = 0  # other
        self.map_data = self.map_data.astype('uint8')
        # my_dict = {}
        # for i in range(0, len(self.map_data)):
        #     for j in range(0, len(self.map_data[i])):
        #         if self.map_data[i][j] not in my_dict:
        #             my_dict[self.map_data[i][j]] = 1
        # rospy.logwarn(my_dict)

        # Get map data from map server
        self.occupancy_grid_for_boxes = deepcopy(self.occupancy_grid)
        width, height = self.occupancy_grid_for_boxes.info.width, self.occupancy_grid_for_boxes.info.height
        self.map_resolution_for_boxes = self.occupancy_grid_for_boxes.info.resolution
        self.map_origin_for_boxes = self.occupancy_grid_for_boxes.info.origin.position
        self.map_data_for_boxes = numpy.asarray(self.occupancy_grid_for_boxes.data)
        self.map_data_for_boxes = numpy.reshape(self.map_data_for_boxes, (height, width))
        self.map_data_for_boxes[self.map_data_for_boxes == 100] = 255  # walls
        self.map_data_for_boxes[self.map_data_for_boxes == -1] = 100  # unknown territory
        self.map_data_for_boxes = self.map_data_for_boxes.astype('uint8')

        # my_dict = {}
        # for i in range(0, len(self.map_data)):
        #     for j in range(0, len(self.map_data[i])):
        #         if self.map_data_for_boxes[i][j] not in my_dict:
        #             my_dict[self.map_data_for_boxes[i][j]] = 1
        # rospy.logwarn(my_dict)

        self.last_detections = []

        # Create a new time synchronizer to synchronize depth and rgb image callbacks.
        # Also subscribe to camera info so we can get camera calibration matrix.
        self.depth_image_subscriber = message_filters.Subscriber('/camera/depth/image_raw', Image)
        self.image_subscriber = message_filters.Subscriber('/camera/rgb/image_raw', Image)
        self.camera_info_subscriber = message_filters.Subscriber('/camera/rgb/camera_info', CameraInfo)
        self.image_synchronizer = message_filters.TimeSynchronizer([self.depth_image_subscriber, self.image_subscriber, self.camera_info_subscriber], 10)
        self.image_synchronizer.registerCallback(self.on_data_received)
    
    def toggle(self, enable):
        """Callback to enable or disable ring detector"""
        self.enabled = enable.data
        rospy.loginfo('Ring detector enabled: {}'.format(self.enabled))
    
    def on_data_received(self, depth_image_message, image_message, camera_info):
        """Callback for when depth image, rgb image and camera information is received"""
        if not self.enabled:
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
    
    def detect_circles(self, depth_image, rgb_image, camera_model, timestamp, circle_padding=5, maximum_distance=2.5, number_of_bins=16, detections_needed=10):
        """Detect circles in depth image and send detections to ring robustifier"""
        # Disparity is computed in meters from the camera center
        disparity = numpy.copy(depth_image)

        # This part is needed to detect circles in image with better precision
        depth_image = numpy.nan_to_num(depth_image)
        depth_image[depth_image > maximum_distance] = 0
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

            # First filtering step: Remove too bright areas from the colour
            # image (this will make our robot not be able to detect white rings)
            # First, get all depths that are non-zero from the coloured image
            non_zero_regions = depth_region > 0
            filtered_image_region = image_region * non_zero_regions[...,None]

            # Then convert the coloured image to grayscale and remove all parts
            # that are too white (threshold is set to 200 at the moment, maybe
            # change this in the future)
            gray_filtered_image_region = cv2.cvtColor(filtered_image_region, cv2.COLOR_BGR2GRAY)
            too_bright_regions = gray_filtered_image_region < 200

            colour_mask = non_zero_regions * too_bright_regions

            depth_region = depth_region * colour_mask
            disparity_region = disparity_region * colour_mask
            image_region = image_region * colour_mask[...,None]

            # Try to filter depth_region to only include circle (basically, get
            # the highest bin in histogram and filter out everything that is not
            # in this bin)
            histogram, bins = numpy.histogram(depth_region[depth_region != 0], bins=number_of_bins, range=(0, 255))
            best_bin = numpy.argmax(histogram)
            depth_region[numpy.abs(depth_region - bins[best_bin]) > 255/number_of_bins] = 0
            
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
                closest_detection = get_closest(self.last_detections, ring_position, max_distance=0.5)
                if closest_detection is not None:
                    closest_ring, number_of_detections = closest_detection
                    self.last_detections.remove(closest_detection)
                    averaged_ring = average(closest_ring, ring_position)
                    if number_of_detections + 1 < detections_needed:
                        self.last_detections.append((averaged_ring, number_of_detections + 1))
                    else:
                        _, approaching_point = self.compute_approaching_point(closest_ring, timestamp)
                        if approaching_point is not None:
                            detection = self.construct_detection_message(closest_ring, approaching_point, classified_color, timestamp)
                            self.detections_publisher.publish(detection)
                        else:
                            rospy.logwarn('Ring detected, but the ring approaching point could not be determined')
                            self.last_detections.append((averaged_ring, number_of_detections + 1))
                else:
                    self.last_detections.append((ring_position, 1))
            else:
                rospy.logwarn('Ring detected, but the ring position could not be determined')
    
    def to_world_position(self, keypoint, disparity_region, camera_model, timestamp, maximum_distance=2.5):
        # Get average distance to the ring, if the slice that we are trying to
        # get the mean of is empty this will result in error.
        try:
            average_distance = numpy.nanmean(disparity_region[numpy.nonzero(disparity_region)])
        except Exception as e:
            rospy.logerr('Average distance could not be computed')
            return None

        if average_distance > maximum_distance:
            return None
        
        # Construct a ray from camera center to pixel on image, then stretch
        # it, so its length is the distance from camera to point in 3d space
        center_x, center_y = keypoint.pt
        rectified_center = camera_model.rectifyPoint((center_x, center_y))
        ray = camera_model.projectPixelTo3dRay(rectified_center)
        point = (ray[0] * average_distance, ray[1] * average_distance, ray[2] * average_distance)
        
        # Convert 3d point from camera frame to world frame and publish it
        ring_camera = PoseStamped()
        ring_camera.pose.position.x = point[0]
        ring_camera.pose.position.y = point[1]
        ring_camera.pose.position.z = point[2]
        ring_camera.header.frame_id = "camera_depth_optical_frame"
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
        # be extracted where hough line transform should be performed. 

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

            # img = map_region.astype('uint8')
            # img = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
            # cv2.line(img, (int(start[0]), int(start[1])), (int(end[0]), int(end[1])), (0, 255, 0), 2)
            # img = cv2.resize(img, (200, 200))
            # cv2.imshow('image', img)
            # cv2.waitKey(0)
            # cv2.destroyAllWindows()
            return best_line, best_distance

        # Convert map coordinates to map pixel coordinates
        ring_pixel = utils.to_map_pixel(ring_point, self.map_origin, self.map_resolution)

        # Find the closest wall pixel and line that passes closest to that wall
        # pixel (preferably line that goes through the wall pixel)
        closest_wall = utils.closest_wall_pixel(self.map_data, ring_pixel)
        if closest_wall is None:
            return ring_position, None
        line, distance = closest_line(ring_pixel, closest_wall)
        if line is None:
            return ring_position, None

        line_direction = line[1] - line[0]
        orientation = numpy.asarray([line_direction[0], line_direction[1]])
        
        # move in the direction of the normal and check if we land on free space or inside the box
        # If we land inside the box, multiply orianetation with -1
        
        normalized_orientation = orientation / numpy.linalg.norm(orientation)
        normalized_orientation = normalized_orientation.astype(int)

        # rotate 90 degrees to the right
        point = [normalized_orientation[0], normalized_orientation[1]]
        rotated = []
        rotated.append(int(-math.sin(-math.radians(90)) * point[1])) # the cosinus part is == 0
        rotated.append(int(math.sin(-math.radians(90)) * point[0])) # the cosinus part is == 0

        # if the neighbouing pixel on the normal to the wall is grey, we are looking in the wrong direction
        middle_point_on_line = []
        middle_point_on_line.append(int((line[0][0] + line[1][0]) * 0.5))
        middle_point_on_line.append(int((line[0][1] + line[1][1]) * 0.5))
        test_pixel = middle_point_on_line + rotated
        if self.map_data_for_boxes[test_pixel[1], test_pixel[0]] != 100:
            rotated[0] = - rotated[0]
            rotated[1] = - rotated[1]

        approaching_point = Pose()
        approaching_point.position.x = ring_point.point.x + 0.4 * rotated[0]
        approaching_point.position.y = ring_point.point.y + 0.4 * rotated[1]

        approaching_point_orientation = [0, 0]
        approaching_point_orientation[0] = -rotated[0]
        approaching_point_orientation[1] = -rotated[1]

        orientation = math.atan2(approaching_point_orientation[1], approaching_point_orientation[0])
        orientation_quaternion = quaternion_from_euler(0, 0, orientation)
        orientation_quaternion = Quaternion(*orientation_quaternion)

        approaching_pose = PoseStamped()
        approaching_pose.header = ring_position.header
        approaching_pose.header.frame_id = 'map'
        approaching_pose.pose.position.x = approaching_point.position.x  # ring_position.pose.position.x
        approaching_pose.pose.position.y = approaching_point.position.y  # ring_position.pose.position.y
        approaching_pose.pose.position.z = 0
        approaching_pose.pose.orientation = orientation_quaternion

        return ring_position, approaching_pose

if __name__ == '__main__':
    ring_detector = RingDetector()
    rospy.spin()

