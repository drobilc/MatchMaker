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
        
        # After the localization has been done, retract the robot arm, so that
        # we don't hit floating objects with it.
        self.movement_controller.add_to_queue(self.movement_controller.retract_arm())

        # After arm has been retracted, add a wandering task to our robot.
        # Wandering task first creates exploration points and then moves to each
        # point.
        self.wandering_task = self.movement_controller.wander()
        self.movement_controller.add_to_queue(self.wandering_task)

        self.gargamel = None
        self.current_woman = None

        self.women = []
        self.rings = []
        self.cylinders = []
        self.preferences = None
        self.favorite_color = None

        self.setup_state_machine()
    
    def setup_state_machine(self):
        finding_gargamel = State('finding_gargamel')
        approaching_gargamel = State('approaching_gargamel')
        finding_woman = State('finding_woman')
        approaching_woman = State('approaching_woman')
        finding_cylinder = State('finding_cylinder')
        approaching_cylinder = State('approaching_cylinder')
        finding_ring = State('finding_ring')
        approaching_ring = State('approaching_ring')
        states = [finding_gargamel, approaching_gargamel, finding_woman, approaching_woman, finding_cylinder, approaching_cylinder, finding_ring, approaching_ring]

        self.machine = Machine(model=self, states=states, initial=finding_gargamel, ignore_invalid_triggers=True)

        self.machine.add_transition('start_approaching_gargamel', [finding_gargamel, approaching_woman], approaching_gargamel, before=self.on_start_approaching_gargamel)
        self.machine.add_transition('start_finding_woman', approaching_gargamel, finding_woman, before=self.on_start_finding_woman)
        self.machine.add_transition('start_approaching_woman', [finding_woman, approaching_ring], approaching_woman, before=self.on_start_approaching_woman)
        self.machine.add_transition('start_finding_cylinder', approaching_gargamel, finding_cylinder, before=self.on_start_finding_cylinder)
        self.machine.add_transition('start_approaching_cylinder', finding_cylinder, approaching_cylinder, before=self.on_start_approaching_cylinder)
        self.machine.add_transition('start_finding_ring', approaching_cylinder, finding_ring, before=self.on_start_finding_ring)
        self.machine.add_transition('start_approaching_ring', finding_ring, approaching_ring, before=self.on_start_approaching_ring)

    # There are two cases where we approach Gargamel
    # 1. We inquire about his preferences
    # 2. We inquire if he likes found woman (param need_confirmation is used for that)
    def on_start_approaching_gargamel(self, need_confirmation=False):
        # Approach him and inquire about preferences
        if not need_confirmation:
            self.wandering_task.cancel()
            task = self.movement_controller.approach(self.gargamel, callback=self.on_gargamel_approached)
            self.movement_controller.add_to_queue(task)
        
        # Approach him and inquire if he likes the woman
        else:
            task = self.movement_controller.approach(self.gargamel, callback=self.on_gargamel_approached)
            self.movement_controller.add_to_queue(task)
    
    # There are two cases where we approach Gargamel
    def on_gargamel_approached(self, detection, goal_status, goal_result):
        # If preferences are not set, we have come to ask what he likes on women
        if self.preferences is None:
            self.preferences = self.get_gargamels_preferences()
            rospy.loginfo("Gargamel likes women with following qualities: {}".format(self.preferences))
            self.start_finding_woman()

        # If preferences are set, we have come see if he likes a particular woman
        else:
            likes = self.get_gargamels_affirmation(self.current_woman)

            # If he likes the woman, first need to approach a cylinder and then a ring
            if likes:
                rospy.loginfo("Gargamel likes this woman and wants YOU to find him a ring.")
                self.start_finding_cylinder()
            else:
                rospy.loginfo("Gargamel doesn't like this woman's taste in color, picky mother*ucker")
                self.women.remove(self.current_woman)
                self.current_woman = None
                self.favorite_color = None
                self.start_finding_woman()

    
    # PLACEHOLDER
    def get_gargamels_affirmation(self, woman):
        import random
        # return random.uniform(0, 1) < 0.5
        return True
    
    def on_start_finding_woman(self):
        for woman in self.women:
            # If this woman has characteristics that gargamel likes, approach her
            if self.in_accordance_with_preferences(woman):
                self.start_approaching_woman(woman)
                self.current_woman = woman
                return
        
        # If woman that gargamel would like has not been detected yet, find her
        rospy.loginfo("Adding a wandering task to find a new woman")
        self.movement_controller.add_to_queue(self.wandering_task)
    
    def on_start_approaching_woman(self, woman):
        self.wandering_task.cancel()
        task = self.movement_controller.approach(woman, callback=self.on_woman_approached)
        self.movement_controller.add_to_queue(task)

    def on_woman_approached(self, detection, goal_status, goal_result):
        # This is the first time we are visiting this woman
        # ask her favorite color and go back to Gargamel
        if self.favorite_color is None:
            self.favorite_color = self.get_womans_favorite_color()
            self.start_approaching_gargamel()
            rospy.loginfo("This woman's favorite color is {}".format(self.favorite_color))
        
        # This is the second time we are here, propose
        else:
            rospy.loginfo("Hey will you marry Gargamel?")
            accepts_proposal = self.get_womans_affirmation()

            if accepts_proposal:
                rospy.loginfo("They lived happily ever after")
            
            else:
                rospy.loginfo("She said no.. ugh, now I have to find a new one")
                self.women.remove(self.current_woman)
                self.favorite_color = None
                self.start_finding_woman()
    
    # PLACEHOLDER
    def get_womans_favorite_color(self):
        import random
        return random.choice(['red', 'green', 'blue', 'yellow', 'white', 'black'])

    def in_accordance_with_preferences(self, woman):
        if self.preferences is None:
            return False

        else:
            woman_face_details = FACE_DETAILS[woman.face_label]
            return woman_face_details == self.preferences

    # PLACEHOLDER   
    def get_gargamels_preferences(self):
        # TODO: this is the part where we ask him
        return FaceDetails('short', 'dark')
    
    def on_start_finding_cylinder(self):
        for cylinder in self.cylinders:
            if cylinder.color == self.favorite_color:
                self.start_approaching_cylinder(cylinder)
                self.current_cylinder = cylinder
                return
        
        self.movement_controller.add_to_queue(self.wandering_task)
    
    def on_start_approaching_cylinder(self, cylinder):
        self.wandering_task.cancel()
        task = self.movement_controller.approach(cylinder, callback=self.on_cylinder_approached)
        self.movement_controller.add_to_queue(task)
    
    def on_cylinder_approached(self, detection, goal_status, goal_result):
        self.start_finding_ring()

    def on_start_finding_ring(self):
        for ring in self.rings:
            if ring.color == self.favorite_color:
                self.start_approaching_ring(ring)
                return
            
        self.movement_controller.add_to_queue(self.wandering_task)

    def on_start_approaching_ring(self, ring):
        self.wandering_task.cancel()
        task = self.movement_controller.approach(ring, callback=self.on_ring_approached)
        self.movement_controller.add_to_queue(task)
    
    def on_ring_approached(self, detection, goal_status, goal_result):
        # TODO: pick up the ring or something first
        self.start_approaching_woman(self.current_woman)

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
            self.on_face_detection(object_detection)

        elif object_detection.type == 'ring':
            rospy.loginfo('New {} ring detected'.format(object_detection.color))
            self.on_ring_detection(object_detection)
        
        else:
            rospy.loginfo('New {} cylinder detected'.format(object_detection.color))
            self.on_cylinder_detection(object_detection)
    
    def on_face_detection(self, object_detection):
        # We have found Gargamel, let's approach him now, here face19 for testing because it's 
        # usually the first face we find
        if object_detection.face_label == 'face19':
            self.gargamel = object_detection
            self.start_approaching_gargamel()
            
        # Otherwise it's a woman, if we do not know Gargamel's preferences yet
        # just add her to the list
        elif self.preferences is None:
            self.women.append(object_detection)

        # If we already know gargamel's preferences and if this woman is a potential match,
        # add her to the list and approach her if we are not approaching any other woman already
        elif self.preferences is not None:
            if self.in_accordance_with_preferences(object_detection):

                if self.state != 'approaching_woman':
                    self.start_approaching_woman(object_detection)
                    self.current_woman = object_detection

                self.women.append(object_detection)

    def on_cylinder_detection(self, cylinder):
        self.cylinders.append(cylinder)

        # If we are currently looking for a cylinder of this color, approach it
        if self.state == 'finding_cylinder' and cylinder.color == self.favorite_color:
            self.start_approaching_cylinder(cylinder)
    
    def on_ring_detection(self, ring):
        self.rings.append(ring)

        # If we are currently looking for a ring of this color, approach it
        if self.state == 'finding_ring' and ring.color == self.favorite_color:
            self.start_approaching_ring(ring)

if __name__ == '__main__':
    controller = Brain([])
    rospy.spin()