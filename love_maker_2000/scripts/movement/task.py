#!/usr/bin/env python

class MovementTask(object):

    def __init__(self, movement_controller, callback):
        self.movement_controller = movement_controller
        self.callback = callback
        self.is_finished = False
        self.has_started = False
        self.is_cancelled = False
    
    def run(self):
        self.has_started = True
        self.is_cancelled = False
    
    def finish(self):
        self.is_finished = True

        # Call the callback function
        if self.callback is not None:
            self.callback()

        self.movement_controller.on_finish(self)
    
    def cancel(self):
        self.is_cancelled = True
        self.movement_controller.cancel(self)
    
    def __str__(self):
        return '<MovementTask>'

    def __repr__(self):
        return self.__str__()