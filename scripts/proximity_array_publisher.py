#!/usr/bin/env python

import rospy
from pr2_fingertip_sensors.msg import PR2FingertipSensor
from force_proximity_ros.msg import ProximityArray, Proximity

SENSOR_NUM = 24
EA = 0.3
SENSITIVITY = 50

class ProximityArrayPublisher(object):
    def __init__(self):
        self.average_value = [0]*SENSOR_NUM
        self.fa2 = [0]*SENSOR_NUM
        self.fa2derivative = [0]*SENSOR_NUM
        
        self.pub = rospy.Publisher('/pfs/l_gripper/l_fingertip/proximity_array',
                                    ProximityArray, queue_size=1)
        self.sub = rospy.Subscriber('/pfs/l_gripper/l_fingertip',
                                    PR2FingertipSensor, self.cb, queue_size=1)

    def cb(self, msg):
        proximities_raw = msg.proximity
        proximities = []
        mode = ''
        pub_msg = ProximityArray(header=msg.header)
        for i, proximity_raw in enumerate(proximities_raw):
            self.fa2derivative[i] = self.average_value[i] - proximity_raw - self.fa2[i]
            self.fa2[i] = self.average_value[i] - proximity_raw
            proximity = Proximity()
            proximity.proximity = proximity_raw
            proximity.average = int(self.average_value[i])
            proximity.fa2 = int(self.fa2[i])
            proximity.fa2derivative = int(self.fa2derivative[i])
            if self.fa2[i] < -SENSITIVITY:
                proximity.mode = "T"
                mode = mode + '\033[31m'+proximity.mode+'\033[0m' + ' '
            elif self.fa2[i] > SENSITIVITY:
                proximity.mode = "R"
                mode = mode + '\033[32m'+proximity.mode+'\033[0m' + ' '
            else:
                proximity.mode = "0"
                mode = mode + '\033[37m'+proximity.mode+'\033[0m' + ' '
            proximities.append(proximity)
            self.average_value[i] = EA * proximity_raw + (1-EA) * self.average_value[i]
        pub_msg.proximities = proximities
        self.pub.publish(pub_msg)
        print(mode)

if __name__ == '__main__':
    rospy.init_node('proximity_array_publisher')
    pa = ProximityArrayPublisher()
    rospy.spin()

        
