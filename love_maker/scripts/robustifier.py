#!/usr/bin/env python
from __future__ import print_function

import rospy
from geometry_msgs.msg import Pose

import numpy as np

class Robustifier(object):

    def __init__(self):
        # Initialize node, don't allow running multiple nodes of this type
        rospy.init_node('robustifier', anonymous=False)

        # The face detector publishes faces to /face_detections_raw, so create a
        # subscriber that we will use to read positions
        self.raw_face_subscriber = rospy.Subscriber('/face_detections_raw', Pose, self.on_face_detection, queue_size=10)
        
        # After the robistufier determines face position, it should send a pose
        # to movement_controller node
        self.face_publisher = rospy.Publisher('/face_detections', Pose, queue_size=10)

        # TODO: Define appropriate structures for storing the received poses and

        # positions of already visited faces.
        # A list of last n received poses (maybe queue?)
        self.previous_detected_face_poses = []

        # A list of pairs of location tuples for "already visited" faces.
        # A touple is a pair of x and y coordinates.
        # Touples in a pair represent an interval that is marked as "visited"
        self.already_visited = []


    def get_x(self, pose):
        return pose.position.x

    def get_y(self, pose):
        return pose.position.y

    def get_z(self, pose):
        return pose.position.z

    def get_orientation_x(self, pose):
        return pose.orientation.x

    def get_orientation_y(self, pose):
        return pose.orientation.y

    def get_orientation_z(self, pose):
        return pose.orientation.z

    def get_orientation_w(self, pose):
        return pose.orientation.w

    def calculate_mean(self):
        self.detected_faces_x = map(self.get_x, self.previous_detected_face_poses)
        self.detected_faces_y = map(self.get_y, self.previous_detected_face_poses)
        self.detected_faces_z = map(self.get_z, self.previous_detected_face_poses)
        # TODO: Check if we need orientation, also, do we even need z position above?
        self.detected_faces_orientation_x = map(self.get_orientation_x, self.previous_detected_face_poses)
        self.detected_faces_orientation_y = map(self.get_orientation_y, self.previous_detected_face_poses)
        self.detected_faces_orientation_z = map(self.get_orientation_z, self.previous_detected_face_poses)
        self.detected_faces_orientation_w = map(self.get_orientation_w, self.previous_detected_face_poses)
        mean = map(float, map(np.nanmean, [self.detected_faces_x, self.detected_faces_y, self.detected_faces_z, self.detected_faces_orientation_x, self.detected_faces_orientation_y, self.detected_faces_orientation_z, self.detected_faces_orientation_w]))
        return mean   

    # TODO: Do we need orientation as well? Is z cooordinate even needed?
    def are_faces_close_to_mean(self, mean, epsilon):
        x_y_distance_from_mean = []
        good_detection = [] # mark faces close to mean as good (True) and others as bad (False)
        
        for face in range (0, len(self.previous_detected_face_poses)):
            x_y_distance_from_mean.append(0)
            x_y_distance_from_mean[face] += abs(self.detected_faces_x[face] - mean[0])
            x_y_distance_from_mean[face] += abs(self.detected_faces_y[face] - mean[1])
            x_y_distance_from_mean[face] += abs(self.detected_faces_z[face] - mean[2])
            
            if x_y_distance_from_mean > epsilon:
                good_detection.append(False)
            else:
                good_detection.append(True)
        
        return good_detection

    # TODO: compute global coordinates based on image rotation
    def calculate_approaching_point(self, correct_position):
        return correct_position
    
    def on_face_detection(self, face_pose):
        # A new face has been detected
        rospy.loginfo('A new face pose received: {}'.format(face_pose))

        # TODO: Only send each face pose once and only send true positive faces

        # Variable to determine whether or not to send the coordinates for approaching to the movement unit.
        # When set to False, the coordinates are not sent.
        true_positive_confirmed = False

        # Add the received pose to the list
        self.previous_detected_face_poses.append(face_pose)

        # When enough detections are collected, calculate their mean position.
        if len(self.previous_detected_face_poses) > 9:
        
            mean = self.calculate_mean()
            epsilon = 0.5

            # Be a Santa, determine whether the faces are good (within epsilon range from mean) or bad
            good_detection = self.are_faces_close_to_mean(mean, epsilon)
            
            # If all faces are in range epsilon around mean, they represent the same position.
            # In that case, set the mean as the correct face position and confirm true positive by setting 
            # true_positive_confirmed to True
            all_good = all(good_detection) # Applies & to all elements
            if all_good:
                true_positive_confirmed = True
                correct_position = mean
                
                # calculate approaching point
                approaching_point = self.calculate_approaching_point(correct_position)

                # Empty the list of detected faces and ignore all incoming faces 
                # until the approaching point is reached. 
                # TODO: set up a new topic between robustifier and movement_controller to communicate this
                # Maybe also stop face detection until approaching point is reached
                self.previous_detected_face_poses = []
                add_new_detections_to_list = False # regulate adding detections from /detected_faces_raw



        # For now, only forward the received face to movement controller
        true_positive_confirmed = True
        if true_positive_confirmed:
            self.face_publisher.publish(face_pose) # change face_pose to approaching_point

if __name__ == '__main__':
    robustifier = Robustifier()
    rospy.loginfo('Robustifier started')
    rospy.spin()