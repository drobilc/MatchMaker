#!/usr/bin/env python

import rospy

from love_maker_2000.srv import ApproachingPointCalculator, ApproachingPointCalculatorResponse

class CylinderApproachingPointCalculator():
    def __init__(self):
        rospy.init_node('cylinder_approaching_point_calculator')

        service = rospy.Service('cylinder_approaching_point_calculator', CylinderApproachingPointCalculator, self.handle_approaching_point_calculation_request)

    def calculate_approaching_point(self, detection):
        # TODO: calculate approaching point
        # this is a dummy file, it just returns tha request as response for now
        return detection

    def handle_approaching_point_calculation_request(self, detection):
        return self.calculate_approaching_point(detection)

if __name__ == '__main__':
    color_classifier = CylinderApproachingPointCalculator()
    rospy.loginfo('Cylinder approaching point calculator node started')
    rospy.spin()