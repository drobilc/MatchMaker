#!/usr/bin/env python
from __future__ import print_function

import rospy
from geometry_msgs.msg import Pose

class MovementController(object):

    def __init__(self):
        # Initialize node, don't allow running multiple nodes of this type
        rospy.init_node('movement_controller', anonymous=False)

        # The robustifier node publishes faces to /face_detections
        self.face_subscriber = rospy.Subscriber('/face_detections', Pose, self.on_face_detection, queue_size=10)

        # When this node finishes intializing itself, it should first try to
        # localize itself, so it knows where it is
        self.localize()
    
    def localize(self):
        rospy.loginfo('Started localization protocol')
    
    def on_face_detection(self, face_pose):
        rospy.loginfo('A new robustified face location found: {}'.format(face_pose))

if __name__ == '__main__':
    controller = MovementController()
    rospy.loginfo('Movement controller started')
    rospy.spin()