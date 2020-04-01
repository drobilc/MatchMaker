#!/usr/bin/env python
from __future__ import print_function

import rospy

class MovementController(object):

    def __init__(self):
        # Initialize node, don't allow running multiple nodes of this type
        rospy.init_node('movement_controller', anonymous=False)

if __name__ == '__main__':
    controller = MovementController()
    rospy.loginfo('Movement controller started')
    rospy.spin()