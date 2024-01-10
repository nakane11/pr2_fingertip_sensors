#!/usr/bin/env python

import rospy
from pr2_fingertip_sensors.msg import SensorArray
from std_msgs.msg import String

SENSITIVITY = 7
class ForceDetection(object):
    def __init__(self):
        self.gripper = 'r_gripper'
        self.fingertips = ['l_fingertip', 'r_fingertip']
        self.pub = rospy.Publisher('force_state', String, queue_size=1)
        self.subs = {}
        self.force_sum = {}
        for fingertip in self.fingertips:
            self.force_sum[fingertip] = 0.0
            self.subs[fingertip] = rospy.Subscriber(
                '/pfs/{}/{}/forces_derivative'.format(self.gripper, fingertip),
                SensorArray, self.cb, fingertip, queue_size=1)

    def cb(self, msg, fingertip):
        self.force_sum[fingertip] = sum(msg.data)
        s = sum(self.force_sum.values)
        if s > SENSITIVITY:
            pub_msg = String(data="s")
        elif s < -SENSITIVITY:
            pub_msg = String(data="w")
        else:
            pub_msg = String(data="u")
        self.pub.publish(pub_msg)

if __name__ == '__main__':
    rospy.init_node('force_detection')
    fd = ForceDetection()
    rospy.spin()
