#!/usr/bin/env python
from __future__ import print_function

import rospy
from object_detection_msgs.msg import ObjectDetection

from movement.controller import MovementController

class Brain(object):
    
    def __init__(self, goals):
        rospy.init_node('brain', anonymous=False)
        # Brain component is the component that collects face, cylinder and ring
        # information and then plans how to solve the given task. It can use
        # MovementController to move the robot to goals. It should also be
        # responsible for greeting the objects, voice recognition, etc.
        self.face_subscriber = rospy.Subscriber('/face_detections', ObjectDetection, self.on_object_detection, queue_size=10)
        self.cylinder_subscriber = rospy.Subscriber('/cylinder_detections', ObjectDetection, self.on_object_detection, queue_size=10)
        self.ring_subscriber = rospy.Subscriber('/ring_detections', ObjectDetection, self.on_object_detection, queue_size=10)

        rospy.loginfo('Brain component created')

        # Movement controller is used to control robot movement, when brain
        # component is created, the robot should localize itself
        self.movement_controller = MovementController()

        def on_localization_finished():
            rospy.loginfo('Localization finished!')
            self.set_detectors_enabled(should_be_enabled=True)
        
        # Before the robot is localized, disable all detectors. The
        # on_localization_finished is a callback function that will be called
        # after the localization task will be completed.
        self.set_detectors_enabled(should_be_enabled=False)
        self.movement_controller.localize(callback=on_localization_finished)
        
        # After the localization has been done, add a wandering task to our
        # robot. Wandering task first creates exploration points and then moves
        # to each point.
        self.movement_controller.wander()
    
    def set_detectors_enabled(self, should_be_enabled=True):
        """Enables or disables all object detectors"""
        rospy.loginfo('Detectors enabled: {}'.format(should_be_enabled))

    def on_object_detection(self, object_detection):
        rospy.loginfo('New object detected: {}'.format(object_detection.type))

        # If new object has been detected, approach it
        self.movement_controller.approach(object_detection)

if __name__ == '__main__':
    controller = Brain([])
    rospy.spin()