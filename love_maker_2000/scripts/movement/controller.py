#!/usr/bin/env python
from __future__ import print_function

import rospy
import actionlib

from task import *

class MovementController(object):
    
    def __init__(self):
        # Movement controller is responsible for moving the robot around. It
        # executes a list of MovementTasks (self.tasks) one by one.
        self.tasks = []
        self.is_running = False

        # TODO: Save current task to a self.current_task, so we can access it if
        # needed.
        
        # TODO: Rewrite tasks to be cancellable, so we can actually reorder
        # them. When calling task.cancel(), the task should be canceled if it
        # currently running, otherwise we can just remove it from queue. The
        # self.cancel_all() method should return previous queue (THE FIRST
        # ELEMENT BEING self.current_task).

        # TODO: Rewrite tasks so that they can be paused an resumed (this will
        # come in handy when WanderingTask is running)

        # TODO: Add method self.run_now(task), that cancels current running task
        # and runs the received task. The self.current_task should be put to the
        # beginning of the self.tasks queue.

        # TODO: Make self.localize, self.approach and self.wander return
        # MovementTask objects so programmer can add them to queue themself:
        #   * self.add_to_queue(self.localize())
        #   * self.run_now(self.localize())

        # Create a new simple action client that will connect to move_base topic
        # The action server will listen on /move_base/goal and will notify
        # us about status and provide feedback at /move_base/status
        self.action_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.action_client.wait_for_server()
        rospy.loginfo('Connected to movement server')
    
    def cancel_all(self):
        """Cancel all tasks and return them"""
        old_tasks = self.tasks
        self.tasks = []
        return old_tasks
    
    def cancel(self, task):
        """Cancel task and run the next task in queue"""
        rospy.loginfo('Cancelling task {}'.format(task))
        if task in self.tasks:
            self.tasks.remove(task)
        
        self.is_running = False
        
        # Run next task in queue
        self.start()
    
    def on_finish(self, task):
        """Callback function that is called after task is finished"""
        rospy.loginfo('Task {} finished'.format(task))
        
        self.is_running = False

        # Call the task callback if it is set
        if task.callback is not None:
            task.callback()
        
        # Execute next task in queue
        self.start()
    
    def start(self):
        """Get the first task from queue and run it"""
        if len(self.tasks) <= 0:
            return

        current_task = self.tasks.pop(0)
        rospy.loginfo('Running task: {}'.format(current_task))
        self.is_running = True
        current_task.run()
    
    def add_to_queue(self, task):
        """Add task to queue to be executed when tasks before are finished"""
        self.tasks.append(task)
        if not self.is_running:
            self.start()
    
    def localize(self, callback=None):
        """Create a new localization task and add it to queue"""
        localization_task = LocalizationTask(self, callback)
        self.add_to_queue(localization_task)
        return localization_task
    
    def approach(self, object_detection, callback=None):
        """Create a new rough approaching task to approach object"""
        approaching_task = ApproachingTask(self, callback, self.action_client, object_detection)
        self.add_to_queue(approaching_task)
        return approaching_task
    
    def wander(self, callback=None):
        """Create a new wandering task to explore the space"""
        wandering_task = WanderingTask(self, callback, self.action_client)
        self.add_to_queue(wandering_task)
        return wandering_task