#!/usr/bin/env python

import rospy
from hw1.msg import hw1

def callback(data):
    rospy.loginfo(data.message + ': '+ str(data.messageID))

def listener():
    rospy.init_node('hw1_subscriber')
    rospy.Subscriber('hw1_id', hw1, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()