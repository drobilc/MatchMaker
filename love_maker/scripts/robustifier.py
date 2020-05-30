#!/usr/bin/env python
from __future__ import print_function

import rospy
import math
from geometry_msgs.msg import Pose, PoseStamped, Vector3, Point
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA
from object_detection_msgs.msg import ObjectDetection
from face_classification.srv import FaceClassification

from collections import Counter

import utils

# A global ID integer that is used every time a new detection object is created to compute its id
DETECTION_ID = 0

class Detection(object):

    DEFAULT_COLOR = "white"
    DEFAULT_LABEL = "none"

    COLOR_MAP = {
        "red": ColorRGBA(1, 0, 0, 1),
        "green": ColorRGBA(0, 1, 0, 1),
        "blue": ColorRGBA(0, 0, 1, 1),
        "yellow": ColorRGBA(1, 1, 0, 1),
        "white": ColorRGBA(1, 1, 1, 1),
        "black": ColorRGBA(0, 0, 0, 1)
    }

    def __init__(self, detection, dummy=False):
        # Detection object is an object that adds some aditional metadata to the
        # ObjectDetection message.
        self.detection = detection

        # When a new Detection object is constructed, the number of its
        # detections should be one.
        self.number_of_detections = 1

        # The already_sent parameter is set to True when the object is first
        # sent to the Brain node. If the object has drastically changed, the
        # needs_resending parameter will be set to True.
        self.already_sent = False
        self.needs_resending = False

        # The object colours and face labels are chosen to be the most frequent
        # ones. So the updates must be saved in two lists.
        self.colors = {}
        self.labels = {}

        # Add the initial color and labels to frequency dictionaries
        self.update_color(detection)
        self.update_label(detection)

        if dummy:
            return
        
        # Generate detection ID, use the DETECTION_ID as a global counter. This
        # is not really considered a good practice, but Python doesn't really
        # have static variables like Java does
        global DETECTION_ID
        self.detection.id = "{}{}".format(self.detection.type, DETECTION_ID)
        DETECTION_ID += 1
    
    def average_poses(self, first_pose, second_pose, blending=0.5):
        """Computes the averaged pose of the two poses."""
        # Use the blending parameter to blend current and next position (simple
        # linear interpolation)
        # position = (1 - blending) * current + blending * next
        alpha, beta = 1 - blending, blending

        new_pose = Pose()
        new_pose.position.x = first_pose.position.x * alpha + second_pose.position.x * beta
        new_pose.position.y = first_pose.position.y * alpha + second_pose.position.y * beta
        new_pose.position.z = first_pose.position.z * alpha + second_pose.position.z * beta
        
        # The orientation of the second pose is chosen as the new pose orientation
        new_pose.orientation = second_pose.orientation

        return new_pose
    
    def update_frequency_dict(self, dictionary, key):
        """Updates colors or face_labels dictionary"""
        # If the key is empty (this is, if the object is cylinder or ring),
        # don't save anything
        if len(key) <= 0:
            return
        
        # Otherwise, save update number of detections in dictionary
        if key not in dictionary:
            dictionary[key] = 0
        dictionary[key] += 1
    
    def get_most_frequent_key(self, dictionary, blacklist=[]):
        """Gets the most frequent key in dictionary, ignoring keys in @blacklist"""
        dictionary_copy = dict(dictionary)
        # Remove all keys from blacklist
        for item in blacklist:
            if item in dictionary_copy:
                del dictionary_copy[item]
        
        # Find the most frequent key in dictionary
        try:
            key, count = Counter(dictionary_copy).most_common(1)[0]
            return key
        except Exception:
            return None
    
    def update_color(self, other_detection):
        self.update_frequency_dict(self.colors, other_detection.color)
        # Get the most frequent color, but ignore "white" label. If no color is
        # found, the object is probably white.
        most_frequent_color = self.get_most_frequent_key(self.colors, ["", "white"])
        if most_frequent_color is None:
            most_frequent_color = Detection.DEFAULT_COLOR        
        self.detection.color = most_frequent_color
    
    def update_label(self, other_detection):
        self.update_frequency_dict(self.labels, other_detection.face_label)
        # Get the most frequent face label but ignore the "none" label. If no
        # label is found, the label is none.
        most_frequent_label = self.get_most_frequent_key(self.labels, ["", "none"])
        if most_frequent_label is None:
            most_frequent_label = Detection.DEFAULT_LABEL
        
        # If the face label has changed, the data must be resent to the brain
        # component.
        if self.detection.face_label != most_frequent_label:
            self.needs_resending = True

        self.detection.face_label = most_frequent_label
    
    def face_recognition_necessary(self):
        return self.detection.type == 'face' and sum(self.labels.values()) <= 3
    
    def update_barcode_data(self, other_detection):
        # If barcode data has been updated, the detection must be sent to the brain again
        if len(self.detection.barcode_data) <= 0 and len(other_detection.barcode_data) > 0:
            self.detection.barcode_data = other_detection.barcode_data
            self.needs_resending = True
    
    # Try to recognize the face and append the label to the face_labels list
    def compute_new_face_label(self, image_message):
        if self.detection.type != 'face':
            return
        return

    def get_color(self):
        return Detection.COLOR_MAP[self.detection.color]

    def update_timestamp(self, timestamp):
        self.detection.header.stamp = timestamp
    
    def update(self, other_detection):
        """Updates current object detection with other received object detection"""
        # The other_detection might have better information about the object
        # than the previous one. The folowing information gets updated:
        #   * object_pose and approaching_pose are averaged
        #   * object color and face label are chosen so that only the most frequent value is used
        #   * additional_information (QR code) is updated if the code has been recognized
        # The needs_resending flag is set to True only if additional_information changes
        
        # Update the object pose and approaching point pose
        new_object_pose = self.average_poses(self.detection.object_pose, other_detection.object_pose)
        new_approaching_point_pose = self.average_poses(self.detection.approaching_point_pose, other_detection.approaching_point_pose)
        self.detection.object_pose = new_object_pose
        self.detection.approaching_point_pose = new_approaching_point_pose

        # Update detection color and label
        self.update_color(other_detection)
        self.update_label(other_detection)

        # Update QR code data
        self.update_barcode_data(other_detection)
    
    def to_pose_stamped(self, pose, on_floor=False):
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = self.detection.header.frame_id
        pose_stamped.header.stamp = self.detection.header.stamp
        pose_stamped.pose = pose
        if on_floor:
            pose_stamped.pose.position.z = 0
        return pose_stamped
    
    def get_object_pose(self):
        """Get object PoseStamped with detection header and robustified pose"""
        return self.to_pose_stamped(self.detection.object_pose)
    
    def get_approaching_point_pose(self, on_floor=True):
        """Get approaching point PoseStamped with detection header and robustified pose"""
        return self.to_pose_stamped(self.detection.approaching_point_pose, on_floor=on_floor)

    def get_detection(self):        
        """Get robustified detection with robustified object pose, approaching point and color"""       
        return self.detection
    
    def distance_to(self, other_detection):
        """Compute Euclidean distance between two detections"""
        self_pose = self.detection.object_pose
        other_pose = other_detection.object_pose
        # Return simple Euclidean distance between this and other pose
        dx = self_pose.position.x - other_pose.position.x
        dy = self_pose.position.y - other_pose.position.y
        dz = self_pose.position.z - other_pose.position.z
        return math.sqrt(dx*dx + dy*dy + dz*dz)

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

        # Face classification service that is used to recognize face
        rospy.wait_for_service('face_classification')
        self.face_recognition = rospy.ServiceProxy('face_classification', FaceClassification)

        # Publisher for publishing raw object detections
        self.markers = MarkerArray()
        self.markers_publisher = rospy.Publisher(self.marker_topic, MarkerArray, queue_size=1000)

        # TODO: Instead of using a list use a quadtree or a grid
        # A list of Detection objects used for finding the closest pose
        self.object_detections = []

        # Subscriber and publisher for object detections
        self.raw_object_subscriber = rospy.Subscriber(self.raw_detection_topic, ObjectDetection, self.on_object_detection, queue_size=10)
        self.object_publisher = rospy.Publisher(self.detection_topic, ObjectDetection, queue_size=10)

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
                detection.update_timestamp(new_detection.header.stamp)
                return detection
    
    def recognize_face(self, detection):
        if detection.type != 'face':
            return None
        
        try:
            label = self.face_recognition(detection.image)
            return label.face_label
        except rospy.ServiceException as e:
            rospy.loginfo("Face classification service call failed: {}".format(e))
            return None
    
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

            if saved_pose.face_recognition_necessary():
                recognized_face = self.recognize_face(detection)
                if recognized_face is not None:
                    detection.face_label = recognized_face

            # If object was detected more than self.minimum_detections, it is now
            # considered true positive, send it to movement controller
            if saved_pose.number_of_detections >= self.minimum_detections:
                if not saved_pose.already_sent or saved_pose.needs_resending:
                    self.publish_marker(saved_pose, Robustifier.ROBUSTIFIED_MARKER_STYLE[saved_pose.detection.type], approaching_point=True)
                    self.object_publisher.publish(saved_pose.get_detection())
                    saved_pose.already_sent = True
                    saved_pose.needs_resending = False
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