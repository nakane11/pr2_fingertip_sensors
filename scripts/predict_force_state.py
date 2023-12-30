#!/usr/bin/env python

import os
import numpy as np
import pickle
import rospy
import rospkg
from pr2_fingertip_sensors.msg import SensorArray
from std_msgs.msg import String
from sensor_msgs.msg import JointState

rospack = rospkg.RosPack()
package_path = rospack.get_path('pr2_fingertip_sensors')
model_path = {'open':'data/cf_20231230_155508_r_gripper_04_1000_relu_adam_0001.sav',
              'close':'data/cf_20231230_155508_r_gripper_04_1000_relu_adam_0001.sav'}

board2index = {'pfs_a_front': [0, 8],
               'pfs_b_top': [8, 12],
               'pfs_b_back': [12, 16],
               'pfs_b_left': [16, 20],
               'pfs_b_right': [20, 24]}

class Predict(object):
    def __init__(self):
        self.models = {}
        for model_type in model_path.keys():
            filename = os.path.join(package_path, model_path[model_type])
            self.models[model_type] = pickle.load(open(filename, 'rb'))
        self.gripper = 'r_gripper'
        self.fingertips = ['l_fingertip', 'r_fingertip']
        self.pub = rospy.Publisher('/pfs/{}/force_state'.format(self.gripper), String, queue_size=1)
        self.gripper_status = None
        self.sub = {}
        self.data_array = {}
        for fingertip in self.fingertips:
            self.sub[fingertip] = {}
            self.data_array[fingertip] = [0] * 24
            for board in board2index.keys():
                self.sub[fingertip][board] = rospy.Subscriber(
                    '/pfs/{}/{}/{}/contact_force_derivative'.format(self.gripper, fingertip, board),
                    SensorArray, self.sensor_cb, (fingertip, board), queue_size=1)
        self.sub['joint'] = rospy.Subscriber(
            '/joint_states', JointState, self.joint_cb, queue_size=1)
        rospy.Timer(rospy.Duration(0.1), self.timer_cb)

    def sensor_cb(self, msg, args):
        fingertip = args[0]
        board = args[1]
        self.data_array[fingertip][board2index[board][0]:board2index[board][1]] = list(msg.data)

    def joint_cb(self, msg):
        if not 'r_gripper_joint' in msg.name:
            return
        idx = msg.name.index('r_gripper_joint')
        gripper_pos = msg.position[idx]
        print(gripper_pos)
        if gripper_pos > 0.023:
            self.gripper_status = 'open'
        else:
            self.gripper_status = 'close'

    def timer_cb(self, event):
        input_array = []
        now = rospy.Time.now()
        if self.gripper_status is None:
            return
        for fingertip in self.fingertips:
            if self.data_array[fingertip] is None:
                return
            last_time = self.data[fingertip].header.stamp
            if (now - last_time).to_sec() > 1:
                return
            input_array += self.data_array[fingertip]
        input_array = np.array(input_array).reshape(1,-1)
        label = self.models[self.gripper_status].predict(input_array)[0]
        pub_msg = String(data=label)
        self.pub.publish(pub_msg)

if __name__ == '__main__':
    rospy.init_node('predict')
    pr = Predict()
    rospy.spin()

