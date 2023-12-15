#!/usr/bin/env python

import rospy
from pr2_fingertip_sensors.msg import PR2FingertipSensor, SensorArray

SENSOR_NUM_START = 8
SENSOR_NUM = 4
EA = 0.3
SENSITIVITY = 22500

class ProximityDerivative(object):
    def __init__(self):
        self.gripper = 'r_gripper'
        self.fingertips = ['l_fingertip', 'r_fingertip']
        self.average_value = {}
        self.pfs_params = rospy.get_param('/pfs')
        self.mode = {}
        self.pubs = {}
        self.subs = {}
        for fingertip in self.fingertips:
            self.average_value[fingertip] = [0]*SENSOR_NUM
            self.pubs[fingertip] = rospy.Publisher('/pfs/{}/{}_proximity_derivative'.format(self.gripper, fingertip),
                                       SensorArray, queue_size=1)
            self.subs[fingertip] = rospy.Subscriber(
                '/pfs/{}/{}'.format(self.gripper, fingertip),
                PR2FingertipSensor, self.cb, fingertip, queue_size=1)

    def cb(self, msg, fingertip):
        mode = ''
        proximities_raw = msg.proximity
        fa2 = [0] * (SENSOR_NUM)
        pub_msg = SensorArray(header=msg.header)
        for i in range(SENSOR_NUM):
            proximity_raw = proximities_raw[SENSOR_NUM_START+i]
            a = self.pfs_params[self.gripper][fingertip]['proximity_a'][SENSOR_NUM_START+i]
            fa2[i] = (self.average_value[fingertip][i] - proximity_raw) / a
            if fa2[i] < -SENSITIVITY:
                mode = mode + '\033[31m'+'T'+'\033[0m' + ' '
            elif fa2[i] > SENSITIVITY:
                mode = mode + '\033[32m'+'R'+'\033[0m' + ' '
            else:
                mode = mode + '\033[37m'+'0'+'\033[0m' + ' '
            self.average_value[fingertip][i] = EA * proximity_raw + (1-EA) * self.average_value[fingertip][i]
        pub_msg.data = fa2
        self.pubs[fingertip].publish(pub_msg)
        self.mode[fingertip] = mode
        print(self.mode['l_fingertip'] + ' ' + self.mode['r_fingertip'])

if __name__ == '__main__':
    rospy.init_node('proximity_derivative')
    pd = ProximityDerivative()
    rospy.spin()

        
