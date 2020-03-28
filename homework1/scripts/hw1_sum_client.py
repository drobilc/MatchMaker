#!/usr/bin/env python

import rospy
from hw1.srv import *
from random import randint

def add_ints_client(array_of_ints):
    rospy.wait_for_service('hw1_sum_server')
    try:
        add_ints = rospy.ServiceProxy('hw1_sum_server', hw1_sum)
        response = add_ints(array_of_ints)
        return response.sum_of_numbers
    except rospy.ServiceException, e:
        print "Service call failed: %s" %e

if __name__ == "__main__":
    array_of_ints = []
    rand_len = randint(0, 10)
    for x in range(rand_len):
        array_of_ints.append(randint(0, 1000))
    print "Requesting addition! Now!"
    suma = add_ints_client(array_of_ints)
    print "The sum of %s is %s" %(str(array_of_ints), suma)