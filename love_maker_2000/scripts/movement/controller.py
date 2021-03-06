#!/usr/bin/env python
from __future__ import print_function

import rospy
import actionlib

from move_base_msgs.msg import MoveBaseAction

from approaching_task import ApproachingTask
from localization_task import LocalizationTask
from wandering_task import WanderingTask
from fine_approaching_task import FineApproachingTask
from arm_task import TossACoinTask, PickUpRingTask, RetractArmTask

class MovementController(object):
    
    def __init__(self):
        # Movement controller is responsible for moving the robot around. It
        # executes a list of MovementTasks (self.tasks) one by one.
        self.tasks = []
        self.is_running = False

        # Create a new simple action client that will connect to move_base topic
        # The action server will listen on /move_base/goal and will notify
        # us about status and provide feedback at /move_base/status
        self.action_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.action_client.wait_for_server()
        rospy.loginfo('Connected to movement server')
    
    def run_immediately(self, task):
        """Cancel all goals and run this task immediately"""
        # rospy.logwarn('MovementController.run_immediately called, task = {}'.format(task))
        other_tasks = self.cancel_all()
        self.tasks.insert(0, task)
        self.tasks.extend(other_tasks)

        self.start()

    def add_to_queue(self, task):
        """Add task to queue to be executed when tasks before are finished"""
        # rospy.logwarn('MovementController.add_to_queue called, task = {}'.format(task))
        self.tasks.append(task)
        if not self.is_running:
            self.start()
    
    def cancel_all(self):
        """Cancel all tasks and return them"""
        # Get current queue of tasks and insert the current running task to the
        # beggining of the list to be run later
        # rospy.logwarn('MovementController.cancel_all called')
        old_tasks = self.tasks
        if hasattr(self, 'current_task') and self.current_task is not None:
            old_tasks.insert(0, self.current_task)
            # Cancel the current running task
            self.current_task.cancel()

        # Clear task queue
        self.tasks = []
        return old_tasks
    
    def cancel(self, task):
        # This function is only meant to be called from tasks, not on its own
        # because it will not cancel the function
        # rospy.logwarn('MovementController.cancel called, task = {}'.format(task))
        # There are two options - the task is currently running or the task is
        # waiting in queue
        if task in self.tasks:
            self.tasks.remove(task)

        if task == self.current_task:
            self.current_task = None
            self.is_running = False
            self.start()
        
        return task
    
    def on_finish(self, task):
        """Callback function that is called after task is finished"""
        # rospy.logwarn('MovementController.on_finish called, task = {}'.format(task))
        self.is_running = False
        self.current_task = None
        # Execute next task in queue
        self.start()
    
    def start(self):
        """Get the first task from queue and run it"""
        # rospy.logwarn('MovementController.start called')
        # rospy.loginfo('Task queue: {}'.format(self.tasks))

        if len(self.tasks) <= 0 or self.is_running:
            return
        
        self.current_task = self.tasks.pop(0)
        self.is_running = True

        rospy.loginfo('Running task: {}'.format(self.current_task))
        self.current_task.run()
    
    def localize(self, callback=None):
        """Create a new localization task and add it to queue"""
        return LocalizationTask(self, callback)
    
    def approach(self, object_detection, callback=None, fine=False):
        """Create a new rough approaching task to approach object"""
        if fine:
            return (
                # First, approach the object's approaching point
                ApproachingTask(self, None, self.action_client, object_detection),
                # Then, use the fine approaching task to get very close to the object
                FineApproachingTask(self, None, object_detection, object_detection.object_pose),
                # Then, move back to the object's approaching point, so we can continue
                FineApproachingTask(self, callback, object_detection, object_detection.approaching_point_pose)
            )
        return ApproachingTask(self, callback, self.action_client, object_detection)
    
    def wander(self, callback=None):
        """Create a new wandering task to explore the space"""
        return WanderingTask(self, callback, self.action_client)
    
    def toss_a_coin(self, callback=None, duration=2.0):
        """Create a new coin tossing task that moves the arm"""
        return TossACoinTask(self, callback, duration=duration)
    
    def pick_up_ring(self, callback=None, duration=2.0):
        """Create a new ring picking task that moves the arm"""
        return PickUpRingTask(self, callback, duration=duration)
    
    def retract_arm(self, callback=None, duration=2.0):
        """Create a new task that retracts the arm"""
        return RetractArmTask(self, callback, duration=duration)