"""Utilities for ROS Python"""

import rospy

def wait_for_service(srv):
    rospy.loginfo('Waiting for service %s to become available...', srv)
    rospy.wait_for_service(srv)
    rospy.loginfo('Service %s is now available!', srv)
