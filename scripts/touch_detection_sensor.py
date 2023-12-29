#!/usr/bin/env python

import rospy
from pr2_fingertip_sensors.msg import PR2FingertipSensor, SensorArray

EA = 0.3
SENSITIVITY = 150

board2index = {'pfs_a_front': [0, 8],
               'pfs_b_top': [8, 12],
               'pfs_b_back': [12, 16],
               'pfs_b_left': [16, 20],
               'pfs_b_right': [20, 24]}

class TouchDetectionSensor(object):
    def __init__(self):
        gripper = 'r_gripper'
        fingertip = rospy.get_param('~fingertip', 'l_fingertip')
        self.board = rospy.get_param('~board', 'pfs_a_front')
        self.sensor_num = board2index[self.board][1] - board2index[self.board][0]
        self.average_value = [0] * self.sensor_num
        self.pub = rospy.Publisher('/pfs/{}/{}/{}/sensor_state'.format(
            gripper, fingertip, self.board), SensorArray, queue_size=1)
        self.sub = rospy.Subscriber('/pfs/{}/{}'.format(gripper, fingertip),
                                    PR2FingertipSensor, self.cb, queue_size=1)

    def cb(self, msg):
        proximities_raw = msg.proximity
        fa2 = [0] * self.sensor_num
        mode = [0] * self.sensor_num
        mode_str = ''
        pub_msg = SensorArray(header=msg.header)
        for i, idx in enumerate(range(board2index[self.board][0], board2index[self.board][1])):
            proximity_raw = proximities_raw[idx]
            fa2[i] = self.average_value[i] - proximity_raw
            if fa2[i] < -SENSITIVITY:
                mode_str = mode_str + '\033[31m'+'T'+'\033[0m' + ' '
                mode[i] = 1
            elif fa2[i] > SENSITIVITY:
                mode_str = mode_str + '\033[32m'+'R'+'\033[0m' + ' '
                mode[i] = 2                
            else:
                mode_str = mode_str + '\033[37m'+'0'+'\033[0m' + ' '
                mode[i] = 0
            self.average_value[i] = EA * proximity_raw + (1-EA) * self.average_value[i]
        pub_msg.data = mode
        self.pub.publish(pub_msg)
        rospy.loginfo(mode_str)
        # print(mode_str)

if __name__ == '__main__':
    rospy.init_node('touch_detection_sensor', anonymous=True)
    tds = TouchDetectionSensor()
    rospy.spin()

        
