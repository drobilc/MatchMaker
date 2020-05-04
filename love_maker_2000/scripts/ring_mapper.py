#!/usr/bin/env python
import roslib
import rospy
roslib.load_manifest('localizer')

import numpy, math
import collections
import utils
import cv2

import message_filters

from std_msgs.msg import Header, ColorRGBA
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import Pose, PoseStamped, Vector3, Point, Quaternion
from nav_msgs.msg import OccupancyGrid

# Custom message types
from detection_msgs.msg import Detection
from object_detection_msgs.msg import ObjectDetection

# Services
from localizer.srv import Localize
from nav_msgs.srv import GetMap
from color_classification.srv import ColorClassification, ColorClassificationResponse

from image_geometry import PinholeCameraModel

import tf2_ros
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from tf2_geometry_msgs import PointStamped

class DetectionMapper():

    def __init__(self):
        rospy.init_node('mapper', anonymous=False)

        self.region_scope = rospy.get_param('~region', 1)
        self.buffer_size = rospy.get_param('~camera_buffer_size', 50)
        rospy.wait_for_service('localizer/localize')

        self.get_map = rospy.ServiceProxy('static_map', GetMap)
        rospy.wait_for_service('static_map')

        rospy.wait_for_service('color_classifier')
        self.classify_color = rospy.ServiceProxy('color_classifier', ColorClassification)

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
        self.detections_sub = message_filters.Subscriber('/detection_ring', Detection)
        self.detections_sub.registerCallback(self.detections_callback)

        self.camera_sub = message_filters.Subscriber('/camera/rgb/camera_info', CameraInfo)
        self.camera_sub.registerCallback(self.camera_callback)

        self.localize = rospy.ServiceProxy('localizer/localize', Localize)

        self.object_detection_publisher = rospy.Publisher('/ring_detections_raw', ObjectDetection, queue_size=100)

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
        face_pose = PoseStamped()
        face_pose.header.frame_id = detected_face_position.header.frame_id
        face_pose.header.stamp = detected_face_position.header.stamp
        face_pose.pose.position.x = detected_face_position.pose.position.x
        face_pose.pose.position.y = detected_face_position.pose.position.y
        face_pose.pose.position.z = detected_face_position.pose.position.z
        face_pose = self.tf_buf.transform(face_pose, "map")

        return face_pose, face_pose

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

        # Compute where the approaching point for this face is. Then send this
        # data to robustifier.
        try:
            detection_point, approaching_point = self.compute_approaching_point(pose)
                
            message_header = Header()
            message_header.stamp = detection_point.header.stamp
            message_header.frame_id = detection_point.header.frame_id

            object_pose = detection_point.pose
            approaching_point_pose = approaching_point.pose
            color = detection.color
            object_type = 'ring'

            color_classification_response = self.classify_color(color)

            object_detection_message = ObjectDetection()
            object_detection_message.header = message_header
            object_detection_message.object_pose = object_pose
            object_detection_message.approaching_point_pose = approaching_point_pose
            object_detection_message.color = color
            object_detection_message.type = object_type
            object_detection_message.classified_color = color_classification_response.classified_color
            
            self.object_detection_publisher.publish(object_detection_message)
        except Exception as e:
            rospy.logwarn(e)
   
if __name__ == '__main__':
    mapper = DetectionMapper()
    rospy.spin()
