#!/usr/bin/env python

import rospy
import broadcast

if __name__ == '__main__':
    rospy.init_node('test_receiver')

    push_rx = broadcast.Receiver(stream_name='test_push_stream')
    pull_rx = broadcast.Receiver(stream_name='test_pull_stream')

    rate = rospy.Rate(2.0)
    while not rospy.is_shutdown():

        now = rospy.Time.now()
        push_time, push_data = push_rx.read_stream(now, 'closest_before')
        pull_time, pull_data = pull_rx.read_stream(now, 'closest_before')

        rospy.loginfo('Push data: %s Pull data: %s', str(push_data), str(pull_data))
        rate.sleep()
    