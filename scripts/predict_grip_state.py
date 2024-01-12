#!/usr/bin/env python

import os
import numpy as np
import pickle
import yaml
import rospy
import rospkg
from pr2_fingertip_sensors.msg import SensorArray
from pr2_fingertip_sensors.srv import SwitchGripperState, SwitchGripperStateResponse
from std_msgs.msg import String

rospack = rospkg.RosPack()
package_path = rospack.get_path('pr2_fingertip_sensors')
model_path = {'state2':'data/rf_20240112_174323_r_gripper_state2.sav',
              'state3':'data/rf_20240112_133423_r_gripper_state3.sav'}

mapping_path = {'state2':'data/mapping_state2.yaml',
                'state3':'data/mapping_state3.yaml'}
    
class Predict(object):
    def __init__(self):
        self.models = {}
        self.mappings = {}
        for state in model_path.keys():
            filename = os.path.join(package_path, model_path[state])
            self.models[state] = pickle.load(open(filename, 'rb'))
            filename = os.path.join(package_path, mapping_path[state])
            self.mappings[state] = yaml.load(open(filename, 'r'),Loader=yaml.Loader)
        self.gripper = 'r_gripper'
        self.pub = rospy.Publisher('/pfs/{}/grip_state'.format(self.gripper), String, queue_size=1)
        self.fingertips = ['l_fingertip', 'r_fingertip']
        self.subs = {}
        self.data = {}
        self.gripper_state = None
        for fingertip in self.fingertips:
            self.data[fingertip] = None
            self.subs[fingertip] = rospy.Subscriber(
                '/pfs/{}/{}/proximity_distances'.format(self.gripper, fingertip),
                SensorArray, self.sensor_cb, fingertip, queue_size=10)
        s = rospy.Service('~switch_gripper', SwitchGripperState, self.gripper_cb)
        rospy.Timer(rospy.Duration(0.1), self.timer_cb)

    def sensor_cb(self, msg, fingertip):
        self.data[fingertip] = msg

    def gripper_cb(self, req):
        self.gripper_state = req.state
        rospy.loginfo('switch to {}'.format(self.gripper_state))
        return SwitchGripperStateResponse()

    def timer_cb(self, event):
        input_array = []
        now = rospy.Time.now()
        if self.gripper_state is None:
            return
        for fingertip in self.fingertips:
            if self.data[fingertip] is None:
                return
            last_time = self.data[fingertip].header.stamp
            if (now - last_time).to_sec() > 1:
                return
            input_array += list(self.data[fingertip].data)
        input_array = np.array(input_array).reshape(1,-1)
        label = self.models[self.gripper_state].predict(input_array)[0]
        pub_msg = String(data=self.mappings[self.gripper_state][label])
        self.pub.publish(pub_msg)

if __name__ == '__main__':
    rospy.init_node('predict')
    pr = Predict()
    rospy.spin()

