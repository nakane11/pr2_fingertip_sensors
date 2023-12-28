#!/usr/bin/env python

import os
import numpy as np
import pickle
import yaml
import rospy
import rospkg
from pr2_fingertip_sensors.msg import SensorArray
from std_msgs.msg import String
from sensor_msgs.msg import JointState

rospack = rospkg.RosPack()
package_path = rospack.get_path('pr2_fingertip_sensors')
model_path = {'open':'data/rf_20231228_114431_r_gripper.sav',
              'close':'data/rf_20231228_114431_r_gripper.sav'}
with open('/home/nakane/ros/pr2_fingertip_ws/src/pr2_fingertip_sensors/data/mapping.yaml', 'r') as yml:
    mapping = yaml.load(yml,Loader=yaml.Loader)
    print(mapping)
    
class Predict(object):
    def __init__(self):
        self.models = {}
        for model_type in model_path.keys():
            filename = os.path.join(package_path, model_path[model_type])
            self.models[model_type] = pickle.load(open(filename, 'rb'))
        self.gripper = 'r_gripper'
        self.pub = rospy.Publisher('/pfs/{}/grip_status'.format(self.gripper), String, queue_size=1)
        self.fingertips = ['l_fingertip', 'r_fingertip']
        self.subs = {}
        self.data = {}
        # self.gripper_status = None
        self.gripper_status = 'close'
        for fingertip in self.fingertips:
            self.data[fingertip] = None
            self.subs[fingertip] = rospy.Subscriber(
                '/pfs/{}/{}/proximity_distances'.format(self.gripper, fingertip),
                SensorArray, self.sensor_cb, fingertip, queue_size=10)
        self.subs['joint'] = rospy.Subscriber(
            '/joint_states', JointState, self.joint_cb, queue_size=1)
        rospy.Timer(rospy.Duration(0.1), self.timer_cb)

    def sensor_cb(self, msg, fingertip):
        self.data[fingertip] = msg

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
            if self.data[fingertip] is None:
                return
            last_time = self.data[fingertip].header.stamp
            if (now - last_time).to_sec() > 1:
                return
            input_array += list(self.data[fingertip].data)
        input_array = np.array(input_array).reshape(1,-1)
        label = self.models[self.gripper_status].predict(input_array)[0]
        pub_msg = String(data=mapping[label])
        self.pub.publish(pub_msg)

if __name__ == '__main__':
    rospy.init_node('predict')
    pr = Predict()
    rospy.spin()

