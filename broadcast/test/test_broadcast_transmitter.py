#!/usr/bin/env python

import rospy
import broadcast

if __name__ == '__main__':
    rospy.init_node('test_transmitter')

    push_tx = broadcast.Transmitter(stream_name='test_push_stream',
                                    feature_size=1,
                                    description='A test push stream',
                                    mode='push',
                                    queue_size=10)
    pull_tx = broadcast.Transmitter(stream_name='test_pull_stream',
                                    feature_size=1,
                                    description='A test pull stream',
                                    mode='pull',
                                    cache_time=1.0)

    counter = 0
    rate = rospy.Rate(1.0)
    while not rospy.is_shutdown():

        now = rospy.Time.now()
        data = [counter]

        rospy.loginfo('Publishing %s at %s', str(data), str(now))

        push_tx.publish(now, data)
        pull_tx.publish(now, data)
        counter += 1
        rate.sleep()
