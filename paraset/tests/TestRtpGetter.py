#!/usr/bin/env python

import rospy
import paraset

class TestRtpGetter(object):
    def __init__(self):
        self.test_param = paraset.RuntimeParamGetter(param_type=float,
                                                     name='test_param',
                                                     init_val=0,
                                                     description='Test realtime parameter')
        self.test_param.add_check(paraset.IntegerValued())
        self.timer = rospy.Timer(rospy.Duration(2.0), self.timer_callback)

    def timer_callback(self, event):
        rospy.loginfo('Parameter: %f', self.test_param.value)


if __name__ == '__main__':
    rospy.init_node('test_rtp_getter')
    rtp = TestRtpGetter()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
