#!/usr/bin/env python

import rospy
from pr2_fingertip_sensors.msg import SensorArray
import numpy as np

WINDOW_SIZE = 4

class ProximityDerivativeMA(object):
    def __init__(self):
        self.gripper = 'r_gripper'
        self.fingertips = ['l_fingertip', 'r_fingertip']
        self.queue = {}
        self.pubs = {}
        self.subs = {}
        for fingertip in self.fingertips:
            self.queue[fingertip] = np.zeros((WINDOW_SIZE, 24))
            self.subs[fingertip] = rospy.Subscriber(
                '/pfs/{}/{}/proximity_derivative'.format(self.gripper, fingertip),
                SensorArray, self.cb, fingertip, queue_size=1)
            self.pubs[fingertip] = rospy.Publisher(
                '/pfs/{}/{}/proximity_derivative_ma'.format(self.gripper, fingertip),
                SensorArray, queue_size=1)

    def cb(self, msg, fingertip):
        self.queue[fingertip] = np.append(self.queue[fingertip], np.array(msg.data).reshape(1, 24), axis=0)
        self.queue[fingertip] = np.delete(self.queue[fingertip], 0, axis=0)
        pub_msg = SensorArray(header=msg.header)
        pub_msg.data = np.average(self.queue[fingertip], axis=0)
        self.pubs[fingertip].publish(pub_msg)

if __name__ == '__main__':
    rospy.init_node('proximity_derivative_ma')
    pd = ProximityDerivativeMA()
    rospy.spin()
