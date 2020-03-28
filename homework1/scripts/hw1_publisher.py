#!/usr/bin/env python

import rospy
from hw1.msg import hw1

def publish_ID():
    pub = rospy.Publisher('hw1_id', hw1, queue_size=10)
    rospy.init_node('hw1_publisher')
    rate = rospy.Rate(1)
    id = 0
    while not rospy.is_shutdown():
        message = "This is your new ID"
        id = id + 1
        pub.publish(message, id)
        rate.sleep()

if __name__ == '__main__':
    publish_ID()