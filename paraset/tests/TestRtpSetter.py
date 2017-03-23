#!/usr/bin/env python

import rospy
import paraset


class TestRtpSetter(object):
    def __init__(self):
        self.test_param = paraset.RuntimeParamSetter(param_type=float,
                                                     name='test_param',
                                                     base_topic='/test_getter/')
        self.timer = rospy.Timer(rospy.Duration(1.0), self.timer_callback)

        rospy.loginfo('Read parameter info: %s', self.test_param.get_info())

    def timer_callback(self, event):
        now = rospy.Time.now().to_sec()
        actual = self.test_param.set_value(now)
        rospy.loginfo('Set parameter to: %f actual: %f', now, actual)


if __name__ == '__main__':
    rospy.init_node('test_rtp_setter')
    rtp = TestRtpSetter()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
