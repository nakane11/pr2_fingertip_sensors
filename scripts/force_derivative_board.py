#!/usr/bin/env python

import rospy
from pr2_fingertip_sensors.msg import SensorArray

board2index = {'pfs_a_front': [0, 8],
               'pfs_b_top': [8, 12],
               'pfs_b_back': [12, 16],
               'pfs_b_left': [16, 20],
               'pfs_b_right': [20, 24]}

class ProximityDerivative(object):
    def __init__(self):
        self.board = rospy.get_param('~board', 'pfs_a_front')
        self.EA = rospy.get_param('~ea', 0.4)
        self.average_value = [0] * (board2index[self.board][1] - board2index[self.board][0])
        self.pub = rospy.Publisher('contact_force_derivative', SensorArray, queue_size=1)
        self.sub = rospy.Subscriber('contact_force', SensorArray,self.cb, queue_size=1)

    def cb(self, msg):
        forces_raw = msg.data
        fa2 = [0] * (board2index[self.board][1] - board2index[self.board][0])
        pub_msg = SensorArray(header=msg.header)
        for i in range(board2index[self.board][1] - board2index[self.board][0]):
            force_raw = forces_raw[i]
            fa2[i] = self.average_value[i] - force_raw
            self.average_value[i] = self.EA * force_raw + (1-self.EA) * self.average_value[i]
        pub_msg.data = fa2
        self.pub.publish(pub_msg)

if __name__ == '__main__':
    rospy.init_node('force_derivative')
    pd = ProximityDerivative()
    rospy.spin()

        
