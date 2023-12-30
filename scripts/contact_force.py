#!/usr/bin/env python

import numpy as np
import rospy
from std_msgs.msg import UInt8MultiArray
from pr2_fingertip_sensors.msg import SensorArray

board2index = {'pfs_a_front': [0, 8],
               'pfs_b_top': [8, 12],
               'pfs_b_back': [12, 16],
               'pfs_b_left': [16, 20],
               'pfs_b_right': [20, 24]}

class ContactForce(object):
    def __init__(self):
        self.fingertip = rospy.get_param('~fingertip', 'r_fingertip')
        self.board = rospy.get_param('~board', 'pfs_a_front')

        self.touch_state = [0] * (board2index[self.board][1] - board2index[self.board][0])
        self.free_force = np.zeros(board2index[self.board][1] - board2index[self.board][0])
        self.base_force = np.zeros(board2index[self.board][1] - board2index[self.board][0])

        # 0:no contact, 1:detect contact, 2:on contact
        self.flg = -1
        self.pub = rospy.Publisher('/pfs/r_gripper/{}/{}/contact_force'.format(self.fingertip, self.board), SensorArray, queue_size=1)
        self.subs = []
        self.subs.append(rospy.Subscriber('sensor_state_lowpass', UInt8MultiArray,
                                          self.cb, queue_size=1))
        self.subs.append(rospy.Subscriber('/pfs/r_gripper/{}/forces'.format(
            self.fingertip), SensorArray, self.force_cb, queue_size=1))
        rospy.sleep(2)

    def cb(self, msg):
        for i in range(len(self.touch_state)):
            if msg.data[i] == 1:
                self.touch_state[i] = 1
            elif msg.data[i] == 2:
                self.touch_state[i] = 0
        if self.flg == 0:
            if sum(self.touch_state) > 1:
                # wait for saving
                self.base_force = np.zeros(board2index[self.board][1] - board2index[self.board][0])
                self.count = 0
                self.flg = 1
        if self.flg != 0:
            if sum(self.touch_state) <= 1:
                self.flg = 0
                # self.touch_state = [0] * (board2index[self.board][1] - board2index[self.board][0])
                rospy.loginfo("{}:stop".format(self.board))
                rospy.loginfo(self.flg)

    def force_cb(self, msg):
        diff = [0] * (board2index[self.board][1] - board2index[self.board][0])
        forces = np.array(msg.data[board2index[self.board][0]:board2index[self.board][1]])
        if self.flg == -1:
            self.free_force = forces
            self.flg = 0
        elif self.flg == 1:
            if self.count < 9:
                self.base_force += forces
                self.count += 1
            else:
                self.base_force += forces
                self.base_force /= 10
                # rospy.loginfo(self.free_force)
                # rospy.loginfo(self.base_force)
                self.sign = np.where(self.base_force - self.free_force < 0, -1, 1)
                # rospy.loginfo(self.sign)
                self.flg = 2
                rospy.loginfo("{}:save".format(self.board))
        elif self.flg == 2:
            #compare
            diff = list((forces - self.base_force) / self.sign)
            pub_msg = SensorArray(data = diff)
            self.pub.publish(pub_msg)

if __name__ == '__main__':
    rospy.init_node('contact_force')
    cf = ContactForce()
    rospy.spin()
        
