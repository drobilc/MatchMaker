#!/usr/bin/env python
from __future__ import print_function

import rospy
from std_msgs.msg import Bool
from object_detection_msgs.msg import ObjectDetection

from geometry_msgs.msg import PoseStamped

from movement.controller import MovementController
from greeter import Greeter

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

        # Greeter object can be used to greet objects once we approach them
        self.greeter = Greeter()

        def on_localization_finished():
            self.set_detectors_enabled(True)
        
        # Before the robot is localized, disable all detectors. The
        # on_localization_finished is a callback function that will be called
        # after the localization task will be completed.
        self.set_detectors_enabled(False)
        localization_task = self.movement_controller.localize(callback=on_localization_finished)
        self.movement_controller.add_to_queue(localization_task)
        
        # After the localization has been done, add a wandering task to our
        # robot. Wandering task first creates exploration points and then moves
        # to each point.
        self.wandering_task = self.movement_controller.wander()
        self.movement_controller.add_to_queue(self.wandering_task)

        self.object_detections = []
    
    def set_detectors_enabled(self, should_be_enabled=True):
        """Enables or disables all object detectors"""
        if not hasattr(self, 'face_detector_toggle'):
            self.face_detector_toggle = rospy.Publisher('/face_detector_toggle', Bool, queue_size=10)
            self.ring_detector_toggle = rospy.Publisher('/ring_detector_toggle', Bool, queue_size=10)
            self.cylinder_detector_toggle = rospy.Publisher('/cylinder_detector_toggle', Bool, queue_size=10)
        
        message = Bool()
        message.data = should_be_enabled

        self.face_detector_toggle.publish(message)
        self.ring_detector_toggle.publish(message)
        self.cylinder_detector_toggle.publish(message)

    def on_object_detection(self, object_detection):
        rospy.loginfo('New object detected: {}'.format(object_detection.type))

        def object_greet(detection, goal_status, goal_result):
            rospy.loginfo('Goal finished with status: {}'.format(goal_status))
            self.greeter.say('Hello {} {}!'.format(object_detection.classified_color, object_detection.type))
            self.object_detections.remove(detection)
            if len(self.object_detections) <= 0:
                self.movement_controller.add_to_queue(self.wandering_task)
        
        self.object_detections.append(object_detection)

        # If new object has been detected, approach it. If its type is ring,
        # approach it using the fine approaching task
        fine = object_detection.type == 'ring'
        task = self.movement_controller.approach(object_detection, callback=object_greet, fine=fine)
        self.movement_controller.run_immediately(task)

if __name__ == '__main__':
    controller = Brain([])
    rospy.spin()