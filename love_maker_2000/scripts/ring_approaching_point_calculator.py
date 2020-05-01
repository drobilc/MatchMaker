#!/usr/bin/env python

import rospy

from love_maker_2000.srv import ApproachingPointCalculator, ApproachingPointCalculatorResponse

class RingApproachingPointCalculator():
    def __init__(self):
        rospy.init_node('ring_approaching_point_calculator')

        service = rospy.Service('ring_approaching_point_calculator', RingApproachingPointCalculator, self.handle_approaching_point_calculation_request)

    def calculate_approaching_point(self, detection):
        # TODO: calculate approaching point
        # this is a dummy file, it just returns tha request as response for now
        return detection

    def handle_approaching_point_calculation_request(self, detection):
        return self.calculate_approaching_point(detection)

if __name__ == '__main__':
    color_classifier = RingApproachingPointCalculator()
    rospy.loginfo('Ring approaching point calculator node started')
    rospy.spin()