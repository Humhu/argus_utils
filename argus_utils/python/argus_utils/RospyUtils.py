"""Utilities for ROS Python"""

import rospy

def wait_for_service(srv):
    if rospy.get_param('/use_sim_time', False):
        rospy.loginfo('Using sim time - skipping wait for service %s', srv)
        return
    rospy.loginfo('Waiting for service %s to become available...', srv)
    rospy.wait_for_service(srv)
    rospy.loginfo('Service %s is now available!', srv)
