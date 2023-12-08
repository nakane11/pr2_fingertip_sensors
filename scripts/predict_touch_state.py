#!/usr/bin/env python

import numpy as np
import pickle
import rospy
import message_filters
from pr2_fingertip_sensors.msg import SensorArray
from sensor_msgs.msg import JointState

class Predict(object):
    def __init__(self):
        filename = 'data/100_tanh_lbfgs_0001_r_gripper.sav'
        self.model = pickle.load(open(filename, 'rb'))
        self.gripper = 'r_gripper'
        self.fingertips = ['l_fingertip', 'r_fingertip']
        self.subs = []
        for fingertip in self.fingertips:
            sensor_sub = message_filters.Subscriber(
                '/pfs/{}/{}/proximity_distances'.format(self.gripper, fingertip),
                SensorArray)
            self.subs.append(sensor_sub)
        joint_sub = message_filters.Subscriber(
            '/joint_states', JointState)
        self.subs.append(joint_sub)
        sync = message_filters.ApproximateTimeSynchronizer(
                self.subs, queue_size=10, slop=0.1)
        sync.registerCallback(self.cb)

    def cb(self, l_msg, r_msg, joint_msg):
        if not 'r_gripper_joint' in joint_msg.name:
            return
        idx = joint_msg.name.index('r_gripper_joint')
        l_data = np.array(l_msg.data, ndmin=2)
        r_data = np.array(r_msg.data, ndmin=2)
        input_array = np.concatenate([l_data, r_data], 1)
        input_array = np.append(input_array, joint_msg.position[idx]).reshape(1, -1)
        label = self.model.predict(input_array)
        if label == 'release':
            rospy.loginfo(label)
        else:
            rospy.logwarn(label)
if __name__ == '__main__':
    rospy.init_node('predict')
    pr = Predict()
    rospy.spin()
