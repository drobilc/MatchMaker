#!/usr/bin/env python
from __future__ import print_function

import rospy
import math
from geometry_msgs.msg import Pose, PoseStamped, Vector3, Point
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA
from object_detection_msgs.msg import ObjectDetection
from face_classification.srv import FaceClassification

import utils

class Detection(object):

    COLOR_MAP = {
        "red": ColorRGBA(1, 0, 0, 1),
        "green": ColorRGBA(0, 1, 0, 1),
        "blue": ColorRGBA(0, 0, 1, 1),
        "yellow": ColorRGBA(1, 1, 0, 1),
        "white": ColorRGBA(1, 1, 1, 1),
        "black": ColorRGBA(0, 0, 0, 1)
    }
    
    def __init__(self, detection, number_of_detections = 1, dummy=False):
        self.detection = detection

        self.number_of_detections = number_of_detections
        self.already_sent = False
        self.blending = 0.5

        self.color_classifications = []
        if detection.classified_color:
            self.color_classifications.append(detection.classified_color)
 
        self.face_labels = []
        
        # Create object as a dummy object when it is temporary and only used
        # to make certain computations/tasks easier. In this case don't execute the code below.
        if dummy:
            return

        # If object is a face, try to recognize it, computationaly expensive
        self.append_new_face_label(self.detection.image)
    
    # Try to recognize the face and append the label to the face_labels list
    def append_new_face_label(self, image_message):
        if self.detection.type != 'face':
            return

        response = self.get_new_face_label(image_message)
        if response is not None:
            self.face_labels.append(response.face_label)
    
    # Get face label for a new face
    def get_new_face_label(self, image_message):
        rospy.wait_for_service('face_classification')
        try:
            recognize_face = rospy.ServiceProxy('face_classification', FaceClassification)
            label = recognize_face(image_message)
            return label
        except rospy.ServiceException as e:
            rospy.loginfo("Face classification service call failed: {}".format(e))
            return None
    
    def get_most_frequent_label(self):
        max_count = 0
        el = None
        for i in self.face_labels:
            count = self.face_labels.count(i)
            if count >= max_count:
                max_count = count
                el = i
        return el
    
    def get_object_pose(self):
        """Get object PoseStamped with detection header and robustified pose"""
        pose = PoseStamped()
        pose.header.frame_id = self.detection.header.frame_id
        pose.header.stamp = self.detection.header.stamp
        pose.pose = self.detection.object_pose
        return pose
    
    def get_approaching_point_pose(self, on_floor=True):
        """Get approaching point PoseStamped with detection header and robustified pose"""
        pose = PoseStamped()
        pose.header.frame_id = self.detection.header.frame_id
        pose.header.stamp = self.detection.header.stamp
        pose.pose = self.detection.approaching_point_pose
        if on_floor:
            pose.pose.position.z = 0
        return pose
    
    def get_real_color(self):
        """Get ColorRGBA from detection color, where each component is a value in range [0, 1]"""
        color = ColorRGBA()
        color.r = self.detection.color.r / 255.0
        color.g = self.detection.color.g / 255.0
        color.b = self.detection.color.b / 255.0
        color.a = 1.0
        return color
    
    def get_color(self):
        """Get ColorRGBA using the color classification color (or real color if it is not set)"""
        # If color has not been classified, return the object real color (the
        # received color from face / cylinder / ring detector)
        if len(self.color_classifications) <= 0:
            return self.get_real_color()
        
        # Otherwise use possible color_classifications to determine the most
        # frequent classification color. White should only be returned if the
        # color classifier only returned white in all detections. Otherwise,
        # white color should be ignored when finding the most frequent color.
        ignore = set(['white'])
        colors = set(self.color_classifications).difference(ignore)
        if len(colors) <= 0:
            return self.COLOR_MAP['white']
        
        most_frequent = max(colors, key=self.color_classifications.count)
        return self.COLOR_MAP[most_frequent]
    
    def update_object_position(self, other_detection, blending=0.5):
        """Update object pose by averaging it using the blending parameter"""
        self_pose = self.detection.object_pose
        other_pose = Detection(other_detection, dummy=True).get_object_pose().pose

        # Calculate new position on line between current position and received position.
        # Use the blending parameter to blend current and next position (simple linear interpolation)
        # position = (1 - blending) * current + blending * next
        alpha, beta = 1 - blending, blending
        self_pose.position.x = self_pose.position.x * alpha + other_pose.position.x * beta
        self_pose.position.y = self_pose.position.y * alpha + other_pose.position.y * beta
        self_pose.position.z = self_pose.position.z * alpha + other_pose.position.z * beta
    
    def update_approaching_point_position(self, other_detection, blending=0.5):
        """Update approaching point pose by averaging it using the blending parameter"""
        self_pose = self.detection.approaching_point_pose
        other_pose = Detection(other_detection, dummy=True).get_approaching_point_pose().pose
        alpha, beta = 1 - blending, blending
        self_pose.position.x = self_pose.position.x * alpha + other_pose.position.x * beta
        self_pose.position.y = self_pose.position.y * alpha + other_pose.position.y * beta
        self_pose.position.z = self_pose.position.z * alpha + other_pose.position.z * beta

    def update(self, other_detection):
        """Update object pose, approaching point and detection color"""
        # Update detected object position, then update approacing point
        # position.
        self.update_object_position(other_detection, self.blending)
        self.update_approaching_point_position(other_detection, self.blending)

        if len(self.detection.additional_information) <= 0 and len(other_detection.additional_information) > 0:
            self.detection.additional_information = other_detection.additional_information

        # Add color classification to list of color classifications. The
        # color_classifications list is then used to get the most frequent color
        # classification. For example, ['red', 'blue', 'red'] should return 'red'
        if other_detection.classified_color:
            self.color_classifications.append(other_detection.classified_color)
    
    def distance_to(self, other_detection):
        """Compute Euclidean distance between two detections"""
        self_pose = self.get_object_pose().pose
        other_pose = Detection(other_detection, dummy=True).get_object_pose().pose
        # Return simple Euclidean distance between this and other pose
        dx = self_pose.position.x - other_pose.position.x
        dy = self_pose.position.y - other_pose.position.y
        dz = self_pose.position.z - other_pose.position.z
        return math.sqrt(dx*dx + dy*dy + dz*dz)
    
    def get_detection(self):        
        """Get robustified detection with robustified object pose, approaching point and color"""
        # Compute color from classifications and update detection classification
        # color
        ignore = set(['white'])
        colors = set(self.color_classifications).difference(ignore)
        if len(colors) <= 0:
            self.detection.classified_color = 'white'
        else:
            most_frequent = max(colors, key=self.color_classifications.count)
            self.detection.classified_color = most_frequent
        
        # Select most frequent face label, add it to detection
        label = self.get_most_frequent_label()
        self.detection.face_label = label
        rospy.loginfo("Sending new face detection to brain with label: {}".format(label))
        
        return self.detection

class Robustifier(object):

    APPROACHING_POINT_STYLE = {
        'marker_type': Marker.SPHERE,
        'color': ColorRGBA(1, 1, 0, 1)
    }

    RAW_MARKER_STYLE = {
        'face': {
            'marker_type': Marker.SPHERE,
            'color': ColorRGBA(1, 0, 0, 1)
        },
        'ring': {
            'marker_type': Marker.SPHERE
        },
        'cylinder': {
            'marker_type': Marker.CYLINDER
        }
    }

    ROBUSTIFIED_MARKER_STYLE = {
        'face': {
            'marker_type': Marker.SPHERE,
            'scale': Vector3(0.2, 0.2, 0.2),
            'color': ColorRGBA(1, 0, 0.5, 1)
        },
        'ring': {
            'marker_type': Marker.SPHERE,
            'scale': Vector3(0.2, 0.2, 0.2)
        },
        'cylinder': {
            'marker_type': Marker.CYLINDER,
            'scale': Vector3(0.2, 0.2, 0.2)
        }
    }

    def __init__(self):
        # Initialize node, don't allow running multiple nodes of this type
        rospy.init_node('robustifier', anonymous=True)

        # This parameter tells us how far two face detections have to be to be
        # considered as different positions
        self.maximum_distance = rospy.get_param('~maximum_distance', 0.5)

        # This parameter tells us how many detections is needed to proclaim a
        # face detection true positive
        self.minimum_detections = rospy.get_param('~minimum_detections', 7)

        # Raw sensor data is read from raw_detection_topic topic, then
        # robustified and the detections are then sent to detection_topic topic
        self.raw_detection_topic = rospy.get_param('~raw_detection_topic', '/face_detections_raw')
        self.detection_topic = rospy.get_param('~detection_topic', '/face_detections')
        self.marker_topic = rospy.get_param('~marker_topic', '/face_markers')

        self.publish_raw_markers = rospy.get_param('~publish_raw_markers', True)

        # Subscriber and publisher for object detections
        self.raw_object_subscriber = rospy.Subscriber(self.raw_detection_topic, ObjectDetection, self.on_object_detection, queue_size=10)
        self.object_publisher = rospy.Publisher(self.detection_topic, ObjectDetection, queue_size=10)

        # Publisher for publishing raw object detections
        self.markers_publisher = rospy.Publisher(self.marker_topic, MarkerArray, queue_size=1000)
        self.markers = MarkerArray()

        # TODO: Instead of using a list use a quadtree or a grid
        # A list of Detection objects used for finding the closest pose
        self.object_detections = []

    def publish_marker(self, detection, marker_style={}, approaching_point=False):
        # Construct a marker from detected object, add it to markers array
        # and publish it to marker_topic
        pose = detection.get_object_pose()

        marker_style_copy = marker_style.copy()

        # If color is defined in marker style, it overrides the received color
        color = detection.get_color()
        if 'color' in marker_style:
            color = marker_style['color']
            marker_style_copy.pop('color')
        
        marker_style_copy['color'] = color

        new_marker = utils.stamped_pose_to_marker(pose,
            index=len(self.markers.markers),
            **marker_style_copy
        )
        self.markers.markers.append(new_marker)

        if approaching_point:
            approaching_pose = detection.get_approaching_point_pose()
            approaching_point_marker = utils.stamped_pose_to_marker(approaching_pose,
                index=len(self.markers.markers),
                **Robustifier.APPROACHING_POINT_STYLE
            )
            self.markers.markers.append(approaching_point_marker)

        self.markers_publisher.publish(self.markers)

    def already_detected(self, new_detection):
        for detection in self.object_detections:
            distance = detection.distance_to(new_detection)
            if distance <= self.maximum_distance:
                # one solution to "location too far in history"
                detection.detection.header.stamp = new_detection.header.stamp
                return detection
    
    def on_object_detection(self, detection):
        # Here we assume that detection.object_pose and
        # detection.approaching_point_pose are in map coordinate frame.
        if detection.header.frame_id != 'map':
            return
        
        if self.publish_raw_markers:
            self.publish_marker(Detection(detection, dummy=True), Robustifier.RAW_MARKER_STYLE[detection.type])

        # Check if detected object is already in object_detections. This cannot be
        # done with simple indexof function because the coordinates will not
        # exactly match
        saved_pose = self.already_detected(detection)

        if saved_pose is not None:
            saved_pose.number_of_detections += 1

            if len(saved_pose.face_labels) <= 3:
                saved_pose.append_new_face_label(detection.image)

            # If object was detected more than self.minimum_detections, it is now
            # considered true positive, send it to movement controller
            if saved_pose.number_of_detections >= self.minimum_detections:
                if not saved_pose.already_sent:
                    self.publish_marker(saved_pose, Robustifier.ROBUSTIFIED_MARKER_STYLE[saved_pose.detection.type], approaching_point=True)
                    self.object_publisher.publish(saved_pose.get_detection())
                    saved_pose.already_sent = True
            else:
                # The detected object pose and previously saved pose are very similar,
                # calculate mean position of both poses and save data to saved_pose
                saved_pose.update(detection)
        else:
            # Construct a new detection object, add it to detections
            saved_pose = Detection(detection)
            self.object_detections.append(saved_pose)

if __name__ == '__main__':
    robustifier = Robustifier()
    rospy.loginfo('Robustifier started')
    rospy.spin()