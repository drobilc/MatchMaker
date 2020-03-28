#!/usr/bin/env python

import rospy
from hw1.srv import hw1_sum, hw1_sumResponse
# from package_name.folder import name_of_srv, name_of_srvResponse

def add_ints(req):
    sumands = ""
    suma = 0
    our_array = req.array_of_numbers
    for number in our_array:
        suma += number
        if our_array.index(number) == len(our_array) - 1:
            sumands += str(number) + ' = '
        else:
            sumands += str(number) + ' + '
    print sumands + str(suma)
    return hw1_sumResponse(suma)

def add_ints_server():
    rospy.init_node('hw1_sum_server')
    print "Ready to do some addition."
    server = rospy.Service('hw1_sum_server', hw1_sum, add_ints)
    
    rospy.spin()

if __name__ == "__main__":
    add_ints_server()