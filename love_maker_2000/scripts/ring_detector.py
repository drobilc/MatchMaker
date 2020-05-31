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

from visualization_msgs.msg import MarkerArray, Marker

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
        self.enabled = False
        self.toggle_subscriber = rospy.Subscriber('/ring_detector_toggle', Bool, self.toggle, queue_size=10)

        # Color classification service
        rospy.wait_for_service('color_classifier')
        self.classify_color = rospy.ServiceProxy('color_classifier', ColorClassification)

        # Publisher for ring ObjectDetections
        self.detections_publisher = rospy.Publisher('/ring_detections_raw', ObjectDetection, queue_size=10)

        # Publisher for publishing raw object detections
        self.markers = MarkerArray()
        self.markers_publisher = rospy.Publisher('/ring_markers', MarkerArray, queue_size=1000)
        self.COLOR_MAP = {
            "red": ColorRGBA(1, 0, 0, 1),
            "green": ColorRGBA(0, 1, 0, 1),
            "blue": ColorRGBA(0, 0, 1, 1),
            "yellow": ColorRGBA(1, 1, 0, 1),
            "white": ColorRGBA(1, 1, 1, 1),
            "black": ColorRGBA(0, 0, 0, 1)
        }

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
    
    def detect_circles(self, depth_image, rgb_image, camera_model, timestamp, circle_padding=5, maximum_distance=2.5, number_of_bins=10):
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

            # First filtering step: Remove too bright areas from the colour
            # image (this will make our robot not be able to detect white rings)
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

            # publish the ring_position
            if ring_position != None:
                style = {}
                style['color'] = self.COLOR_MAP[classified_color]
                style['marker_type'] = Marker.SPHERE
                rospy.logwarn(style)
                rospy.logwarn(ring_position.pose.position)
                new_marker = utils.stamped_pose_to_marker(ring_position, index=len(self.markers.markers), **style)
                self.markers.markers.append(new_marker)
                self.markers_publisher.publish(self.markers)

            # Send the ring position to robustifier, if it is 
            if ring_position is not None:
                _, approaching_point = self.compute_approaching_point(ring_position, timestamp)
                if approaching_point is not None:
                   detection = self.construct_detection_message(ring_position, approaching_point, classified_color, timestamp)
                   self.detections_publisher.publish(detection)
                else:
                   rospy.logwarn('Ring detected, but the ring approaching point could not be determined')
                detection = self.construct_detection_message(ring_position, ring_position, classified_color, timestamp)
                self.detections_publisher.publish(detection)
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

        # Calculate approaching point position
        # if ring pixel is not in the wall, first find the closest wall
        temp_pixel = utils.to_map_pixel(ring_point, self.map_origin, self.map_resolution)
        temp_pixel_1 = utils.to_map_pixel(ring_point, self.map_origin, self.map_resolution)

        if self.map_data[temp_pixel_1[1]][temp_pixel_1[0]] != 255:
            temp_pixel_2 = utils.closest_wall_pixel(self.map_data, temp_pixel_1, max_distance=10)
            if temp_pixel_2 == None:
                return ring_position, None
            else:
                temp_pixel = temp_pixel_2

        temp_pixel = [temp_pixel[0], temp_pixel[1]]

        # Find closest free pixel 
        closest_free_pixel = utils.nearest_free_pixel(temp_pixel, self.map_data)
        direction = [closest_free_pixel[0] - temp_pixel[0], closest_free_pixel[1] - temp_pixel[1]]

        # Get vector direction from ring to closest free pixel
        orientation_parameters = direction

        # Normalize orientation vector
        normalized_orientation = orientation_parameters / numpy.linalg.norm(orientation_parameters)
        
        # Move in this direction for 0.2m => pickup_point
        pickup_point = Pose()
        pickup_point.position.x = ring_point.point.x + 0.2 * normalized_orientation[0]
        pickup_point.position.y = ring_point.point.y + 0.2 * normalized_orientation[1]
        pickup_point.position.z = ring_point.point.z

        # Move in this direction for 0.5m => get_close_point
        get_close_point = Pose()
        get_close_point.position.x = ring_point.point.x + 0.5 * normalized_orientation[0]
        get_close_point.position.y = ring_point.point.y + 0.5 * normalized_orientation[1]
        get_close_point.position.z = 0

        # Orientation for get_close_point should be towards the wall
        get_close_orientation_parameters = normalized_orientation * (-1)
        get_close_orientation = math.atan2(get_close_orientation_parameters[1], get_close_orientation_parameters[0])
        get_close_quaternion = quaternion_from_euler(0, 0, get_close_orientation)
        get_close_quaternion = Quaternion(*get_close_quaternion)


        # Orientation of the ring is the direction, rotated 90 degrees to the right
        # This will only work correctly, if ring detections are accurate and close to the wall

        # rotate 90 degrees to the right
        point = [normalized_orientation[0], normalized_orientation[1]]
        rotated = []
        rotated.append(-math.sin(-math.radians(90)) * point[1]) # the cosinus part is == 0
        rotated.append(math.sin(-math.radians(90)) * point[0]) # the cosinus part is == 0

        # convert to quaternion
        orientation = math.atan2(rotated[1], rotated[0])
        orientation_quaternion = quaternion_from_euler(0, 0, orientation)
        orientation_quaternion = Quaternion(*orientation_quaternion)

        # set get_close_point
        approaching_pose = PoseStamped()
        approaching_pose.header = ring_position.header
        approaching_pose.header.frame_id = 'map'
        approaching_pose.pose.position.x = get_close_point.position.x
        approaching_pose.pose.position.y = get_close_point.position.y
        approaching_pose.pose.position.z = 0
        approaching_pose.pose.orientation = get_close_quaternion

        # set pickup_point
        ring_position.pose.position.x = pickup_point.position.x
        ring_position.pose.position.y = pickup_point.position.y
        #ring_position.pose.position.z = 0  # pickup_point.position.z
        ring_position.pose.orientation = orientation_quaternion

        return ring_position, approaching_pose

if __name__ == '__main__':
    ring_detector = RingDetector()
    rospy.spin()

