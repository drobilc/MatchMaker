#!/usr/bin/env python
from __future__ import print_function

import rospy
from geometry_msgs.msg import Pose

class Robustifier(object):

    def __init__(self):
        # Initialize node, don't allow running multiple nodes of this type
        rospy.init_node('robustifier', anonymous=False)

        # The face detector publishes faces to /face_detections_raw, so create a
        # subscriber that we will use to read positions
        self.face_subscriber = rospy.Subscriber('/face_detections_raw', Pose, self.on_face_detection, queue_size=10)
    
    def on_face_detection(self, face_pose):
        # A new face has been detected
        rospy.loginfo('A new face pose received: {}'.format(face_pose))

if __name__ == '__main__':
    robustifier = Robustifier()
    rospy.loginfo('Robustifier started')
    rospy.spin()