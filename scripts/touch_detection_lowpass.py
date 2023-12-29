#!/usr/bin/env python

import numpy as np
import rospy
from pr2_fingertip_sensors.msg import SensorArray
from std_msgs.msg import UInt8MultiArray

class TouchDetectionBoard(object):
    def __init__(self):
        gripper = 'r_gripper'
        fingertip = rospy.get_param('~fingertip', 'l_fingertip')
        self.board = rospy.get_param('~board', 'pfs_a_front')
        if self.board == 'pfs_a_front':
            self.buffer = np.full((2,8), 99, int)
        else:
            self.buffer = np.full((2,4), 99, int)
        self.pub = rospy.Publisher('sensor_state_lowpass'.format(
            gripper, fingertip, self.board), UInt8MultiArray, queue_size=1)
        self.sub = rospy.Subscriber('sensor_state', SensorArray, self.cb, queue_size=1)

    def cb(self, msg):
        self.buffer = np.concatenate(
            [self.buffer, np.array(msg.data).reshape(1,-1)], 0)
        self.buffer = self.buffer[1:]
        pub_msg = UInt8MultiArray()
        pub_msg.data = [0]*self.buffer.shape[1]
        for j in range(self.buffer.shape[1]):
            if np.count_nonzero(self.buffer[:, j] == 2) > 1:
                pub_msg.data[j] = 2
            elif np.count_nonzero(self.buffer[:, j] == 1) > 1:
                pub_msg.data[j] = 1
                
        # for i in range(3):
        #     if np.count_nonzero(self.buffer[:i+1] == 2) > 1:
        #         for j in range(self.buffer.shape[1]):
        #             if np.count_nonzero(self.buffer[:, j] == 2) > 1:
        #                 pub_msg.data = 2
        #                 break
        #     elif np.count_nonzero(self.buffer[:i+1] == 1) > 1:
        #         for j in range(self.buffer.shape[1]):
        #             if np.count_nonzero(self.buffer[:, j] == 1) > 1:
        #                 pub_msg.data = 1
        #                 break

        self.pub.publish(pub_msg)

if __name__ == '__main__':
    rospy.init_node('touch_detection_board', anonymous=True)
    tdb = TouchDetectionBoard()
    rospy.spin()
        
