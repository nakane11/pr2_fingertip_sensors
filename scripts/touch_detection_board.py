#!/usr/bin/env python

import numpy as np
import rospy
from std_msgs.msg import UInt8MultiArray, UInt8

class TouchDetectionBoard(object):
    def __init__(self):
        self.board = rospy.get_param('~board', 'pfs_a_front')
        self.pub = rospy.Publisher('touch_state', UInt8, queue_size=1)
        self.sub = rospy.Subscriber('sensor_state_lowpass', UInt8MultiArray, self.cb, queue_size=1)

    def cb(self, msg):
        pub_msg = UInt8()
        if sum(x==1 for x in msg.data) > 0:
            if sum(x==1 for x in msg.data) > sum(x==2 for x in msg.data):
                pub_msg.data = 1
            else:
                pub_msg.data = 2
        elif sum(x==2 for x in msg.data) > 0:
            pub_msg.data = 2
        else:
            pub_msg.data = 0
        self.pub.publish(pub_msg)

if __name__ == '__main__':
    rospy.init_node('touch_detection_board', anonymous=True)
    tdb = TouchDetectionBoard()
    rospy.spin()
        
