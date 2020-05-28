#!/usr/bin/env python
from __future__ import print_function

import rospy
from transitions import Machine, State, Transition
from std_msgs.msg import Bool
from object_detection_msgs.msg import ObjectDetection

from geometry_msgs.msg import PoseStamped

from movement.controller import MovementController
from greeter import Greeter
from utils import FACE_DETAILS, FaceDetails

from zbar_detector.msg import Marker

from actionlib_msgs.msg import GoalStatus
import utils

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
        self.gargamel = None
        self.women = []
        self.rings = []
        self.cylinders = []
        self.preferences = None

        self.setup_state_machine()
    
    def setup_state_machine(self):
        finding_gargamel = State('finding_gargamel')
        approaching_gargamel = State('approaching_gargamel')
        finding_woman = State('finding_woman')
        approaching_woman = State('approaching_woman')
        states = [finding_gargamel, approaching_gargamel, finding_woman, approaching_woman]

        self.machine = Machine(model=self, states=states, initial=finding_gargamel, ignore_invalid_triggers=True)

        self.machine.add_transition('start_approaching_gargamel', [finding_gargamel, approaching_woman], approaching_gargamel, before=self.on_start_approaching_gargamel)
        self.machine.add_transition('start_finding_woman', approaching_gargamel, finding_woman, before=self.on_start_finding_woman)
        self.machine.add_transition('start_approaching_woman', finding_woman, approaching_woman, before=self.on_start_approaching_woman)

    def on_start_approaching_gargamel(self):
        # Add new task to approach gargamel
        self.wandering_task.cancel()
        task = self.movement_controller.approach(self.gargamel, callback=self.on_gargamel_approached)
        self.movement_controller.add_to_queue(task)
    
    def on_gargamel_approached(self, detection, goal_status, goal_result):
        if self.preferences is None:
            self.preferences = self.get_gargamels_preferences()
            self.start_finding_woman()
    
    def on_start_finding_woman(self):
        for woman in self.women:
            # If this woman has characteristics that gargamel likes, approach her
            if self.in_accordance_with_preferences(woman):
                self.start_approaching_woman(woman)
                return
        
        # If woman that gargamel would like has been detected yet, find her
        self.movement_controller.add_to_queue(self.wandering_task)
    
    def on_start_approaching_woman(self, woman):
        self.wandering_task.cancel()
        task = self.movement_controller.approach(woman, callback=self.on_woman_approached)
        self.movement_controller.add_to_queue(task)

    # PLACEHOLDER
    def on_woman_approached(self, detection, goal_status, goal_result):
        rospy.loginfo("Woman has been successfuly approached")

    # PLACEHOLDER
    def in_accordance_with_preferences(self, woman):
        if self.preferences is None:
            return False
        else:
            return True

    # PLACEHOLDER   
    def get_gargamels_preferences(self):
        return FaceDetails('short', 'dark')

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
    
    def on_object_approach(self, detection, goal_status, goal_result):
        """Callback function that is called when robot approaches or fails to approach object"""
        if goal_status == GoalStatus.SUCCEEDED:
            self.on_successful_object_approach(detection)
        else:
            self.on_unsuccessful_object_approach(detection)
    
    def on_successful_object_approach(self, detection):
        """Callback function that is called when the object was successfully approached"""
        self.greeter.say('Hello {} {}!'.format(detection.color, detection.type))
        if detection.type == 'face':
            if detection.barcode_data is not None and len(detection.barcode_data) > 0:
                face_data = utils.get_request(detection.barcode_data)
                rospy.loginfo("Data received: {}".format(face_data))                

        self.object_detections.remove(detection)
        if len(self.object_detections) <= 0:
            self.movement_controller.add_to_queue(self.wandering_task)
    
    def on_unsuccessful_object_approach(self, detection):
        """Callback function that is called if the robot could not successfully approach the object"""
        # Create a new movement task and add it to the end of the movement
        # controller queue. Basically visit this object at latter time.
        fine = detection.type == 'ring'
        task = self.movement_controller.approach(detection, callback=self.on_object_approach, fine=fine)
        self.movement_controller.add_to_queue(task)

    def on_object_detection(self, object_detection):
        rospy.loginfo('New object detected: {}, id = {}'.format(object_detection.type, object_detection.id))

        if object_detection.type == 'face':

            rospy.loginfo('Found new face with label: {}'.format(object_detection.face_label))
            # We have found Gargamel, let's approach him now, here face19 for testing because it's the 
            # first face we find
            if object_detection.face_label == 'face19':
                self.gargamel = object_detection
                self.start_approaching_gargamel()
            
            # Otherwise it's a woman
            # Add this woman to the list and check if we are she is in accordance with gargamel's preferences
            else:
                self.women.append(object_detection)

                if self.in_accordance_with_preferences(object_detection):
                    self.start_approaching_woman(object_detection)


        # If new object has been detected, approach it. If its type is ring,
        # approach it using the fine approaching task
        # fine = object_detection.type == 'ring'
        # task = self.movement_controller.approach(object_detection, callback=self.on_object_approach, fine=fine)
        # self.movement_controller.add_to_queue(task)

if __name__ == '__main__':
    controller = Brain([])
    rospy.spin()