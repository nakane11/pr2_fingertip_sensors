#!/usr/bin/env python

import csv
import datetime
import numpy as np
import rospy
from pr2_fingertip_sensors.msg import SensorArray

ARRAY_SIZE = 10
SENSOR_NUM = 24

class Counter(object):
    def __init__(self):
        self.n = 0
        self.label = ''
        self.subs = {}
        self.pubs = {}        
        self.data_array = {}
        self.data_num = {}
        self.fingertips = ['l_fingertip', 'r_fingertip']

        savedir = rospy.get_param('~savedir', '/home/nakane/Documents')
        dt = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        self.filepath = '{}/{}_force_deriv.csv'.format(savedir, dt)
        indices = ['time', 'label']\
            + ['l_force_{}'.format(i) for i in range(24)] \
            + ['r_force_{}'.format(i) for i in range(24)]
        with open(self.filepath, 'a') as f:
            writer = csv.writer(f)
            writer.writerow(indices)

        for fingertip in self.fingertips:
            self.data_array[fingertip] = [0]*SENSOR_NUM
            self.subs[fingertip] = rospy.Subscriber(
                '/pfs/r_gripper/{}/force_derivative'.format(fingertip),
                SensorArray, self.cb, fingertip, queue_size=1)

    def cb(self, msg, fingertip):
        self.data_array[fingertip] = list(msg.data)


    def start(self, count, label):
        rospy.loginfo("Start: {}".format(label))
        self.n = count
        self.label = label
        rospy.Timer(rospy.Duration(3.5), self.run, oneshot=True)
        
    def run(self, event):
        rospy.loginfo("{}: {}".format(self.n, self.label))
        rospy.sleep(0.7)
        self.save()
        if self.n > 0:
            rospy.Timer(rospy.Duration(1.5), self.run, oneshot=True)
        else:
            rospy.loginfo("Finish.")

    def get_data(self, label):
        start_time = rospy.Time.now()
        save_data = [start_time, label]
        for fingertip in self.fingertips:
            save_data += self.data_array[fingertip]
        return save_data

    def save(self):
        label = self.label
        save_data = self.get_data(label)
        for i in range(5):
            if label == 's':
                if np.count_nonzero(np.array(save_data[2:]) > 1.0) < 2:
                    if i<4:
                        save_data = self.get_data(label)
                    else:
                        rospy.loginfo("unreliable")
                        return
                else:
                    break
            if label == 'w':
                if np.count_nonzero(np.array(save_data[2:]) < -1.0) < 2:
                    if i<4:
                        save_data = self.get_data(label)
                    else:
                        rospy.loginfo("unreliable")
                        return
                else:
                    break
        with open(self.filepath, 'a') as f:
            writer = csv.writer(f)
            writer.writerow(save_data)
        if self.label in self.data_num.keys():
            self.data_num[self.label] += 1
        else:
            self.data_num[self.label] = 1
        rospy.loginfo("save: {}{}".format(self.label, self.data_num[self.label]))
        self.n = self.n - 1

if __name__ == '__main__':
    rospy.init_node('predict')
    counter = Counter()
    counter.start(200, 'w')
    print('end')
    rospy.spin()
