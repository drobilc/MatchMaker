#!/usr/bin/env python

import rospy

from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Pose, PoseStamped, Twist, Quaternion, Vector3
from std_msgs.msg import String, ColorRGBA

def stamped_poses_to_marker_array(poses, marker_type=Marker.CUBE, lifetime=120, scale=Vector3(0.1, 0.1, 0.1), color=ColorRGBA(1, 0, 0, 1)):
    markers = MarkerArray()
    for index, pose in enumerate(poses):
        marker = stamped_pose_to_marker(pose, index=index, marker_type=marker_type, lifetime=lifetime, scale=scale, color=color)
        markers.markers.append(marker)
    return markers

def stamped_pose_to_marker(pose_stamped, marker_type=Marker.CUBE, lifetime=120, scale=Vector3(0.1, 0.1, 0.1), color=ColorRGBA(1, 0, 0, 1), index=1):
    marker = Marker()
    marker.action = Marker.ADD
    marker.frame_locked = False
    marker.header = pose_stamped.header
    marker.pose = pose_stamped.pose
    marker.type = marker_type
    marker.lifetime = rospy.Duration.from_sec(lifetime)
    marker.id = index
    marker.scale = scale
    marker.color = color
    return marker