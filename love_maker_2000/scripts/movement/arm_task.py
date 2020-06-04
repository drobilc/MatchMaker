#!/usr/bin/env python

from __future__ import print_function
import rospy

from task import MovementTask

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import String

from greeter import Greeter

class ArmTask(MovementTask):

    EXTENDED_POSITIONS = [0, 0.3, 1, 0]
    RETRACTED_POSITIONS = [0, -1.3, 2.2, 1]
    GRAB_RING_POSITIONS = [0, -1.0, 1.5, 1]

    def __init__(self, movement_controller, callback, arm_joint_positions, duration=2.0):
        super(ArmTask, self).__init__(movement_controller, callback)
        # The joint positions tell the arm how to position its joints and the
        # duration is the duration that the task should wait before finishing
        self.arm_joint_positions = arm_joint_positions
        self.number_of_positions = len(arm_joint_positions)
        self.duration = duration
    
    def generate_join_trajectory_message(self, positions):
        joint_trajectory = JointTrajectory()
        joint_trajectory.joint_names = [
            "arm_shoulder_pan_joint",
            "arm_shoulder_lift_joint",
            "arm_elbow_flex_joint",
            "arm_wrist_flex_joint"
        ]
        joint_trajectory.points = [JointTrajectoryPoint(
            positions=positions,
            time_from_start=rospy.Duration(1)
        )]
        return joint_trajectory

    def next(self):
        if len(self.arm_joint_positions) > 0:
            positions = self.arm_joint_positions.pop(0)
            joint_trajectory = self.generate_join_trajectory_message(positions)
            self.arm_movement_publisher.publish(joint_trajectory)
        
        def on_timer_end(event):
            if len(self.arm_joint_positions) > 0:
                self.next()
            else:
                self.finish()
        
        waiting_time = self.duration / float(self.number_of_positions)
        rospy.Timer(rospy.Duration(waiting_time), on_timer_end, oneshot=True)

    def run(self):
        # Create a new publisher and publish a new JoinTrajectory message. After
        # the message is received, wait self.duration seconds and finish with this task
        self.arm_movement_publisher = rospy.Publisher('/turtlebot_arm/arm_controller/command', JointTrajectory, queue_size=10)
        rospy.sleep(0.5)

        self.next()

class TossACoinTask(ArmTask):
    
    def __init__(self, movement_controller, callback, duration=2.0):
        self.greeter = Greeter()
        self.greeter.say("I wish for Gargamel and his chosen one to fall in love, get married and be happy.")
        super(TossACoinTask, self).__init__(movement_controller, callback, [[0, 0.5, 0.5, 0.8], ArmTask.RETRACTED_POSITIONS], duration=duration)

    def __str__(self):
        return '<TossACoinTask>'

class PickUpRingTask(ArmTask):

    def __init__(self, movement_controller, callback, duration=2.0):
        super(PickUpRingTask, self).__init__(movement_controller, callback, [ArmTask.GRAB_RING_POSITIONS, ArmTask.RETRACTED_POSITIONS], duration=duration)
    
    def __str__(self):
        return '<PickUpRingTask>'

class RetractArmTask(ArmTask):

    def __init__(self, movement_controller, callback, duration=2.0):
        super(RetractArmTask, self).__init__(movement_controller, callback, [ArmTask.RETRACTED_POSITIONS], duration=duration)
    
    def __str__(self):
        return '<RetractArmTask>'