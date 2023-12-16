#!/usr/bin/env python

# import yaml
import os
import numpy as np
import pickle
import rospy
import rospkg
from pr2_fingertip_sensors.msg import SensorArray
from std_msgs.msg import Bool

# with open('/home/nakane/ros/pr2_fingertip_ws/src/pr2_fingertip_sensors/data/pfs_params.yaml', 'r') as yml:
#     pfs_params = yaml.safe_load(yml)

rospack = rospkg.RosPack()
package_path = rospack.get_path('pr2_fingertip_sensors')
model_name = 'data/100_logistic_adam_0001.sav'

class Predict(object):
    def __init__(self):
        filename = os.path.join(package_path, model_name)
        self.model = pickle.load(open(filename, 'rb'))
        self.gripper = 'r_gripper'
        self.fingertips = ['l_fingertip']
        self.pub = rospy.Publisher('touch_status', Bool, queue_size=1)
        self.subs = {}
        for fingertip in self.fingertips:
            self.subs[fingertip] = rospy.Subscriber(
                '/pfs/{}/{}/proximity_derivative_ma'.format(self.gripper, fingertip),
                SensorArray, self.cb, fingertip, queue_size=10)

    def cb(self, msg, fingertip):
        input_array = np.array(msg.data).reshape(1, -1)# /pfs_params['pfs']['r_gripper']['l_fingertip']['proximity_a'][8:12]
        label = self.model.predict(input_array)
        if label == '0':
            rospy.loginfo(label)
        elif label == 'T':
            rospy.logerr(label)
        elif label == 'R':
            rospy.logwarn(label)

if __name__ == '__main__':
    rospy.init_node('predict')
    pr = Predict()
    rospy.spin()
