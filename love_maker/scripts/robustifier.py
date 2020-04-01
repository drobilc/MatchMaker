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
        self.raw_face_subscriber = rospy.Subscriber('/face_detections_raw', Pose, self.on_face_detection, queue_size=10)
        
        # After the robistufier determines face position, it should send a pose
        # to movement_controller node
        self.face_publisher = rospy.Publisher('/face_detections', Pose, queue_size=10)
    
    def on_face_detection(self, face_pose):
        # A new face has been detected
        rospy.loginfo('A new face pose received: {}'.format(face_pose))

        # TODO: Only send each face pose once and only send true positive faces
        # For now, only forward the received face to movement controller
        self.face_publisher.publish(face_pose)

if __name__ == '__main__':
    robustifier = Robustifier()
    rospy.loginfo('Robustifier started')
    rospy.spin()