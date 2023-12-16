#!/usr/bin/env python

# import yaml
import os
import numpy as np
import pickle
import rospy
import rospkg
from pr2_fingertip_sensors.msg import SensorArray
from std_msgs.msg import String

# with open('/home/nakane/ros/pr2_fingertip_ws/src/pr2_fingertip_sensors/data/pfs_params.yaml', 'r') as yml:
#     pfs_params = yaml.safe_load(yml)

rospack = rospkg.RosPack()
package_path = rospack.get_path('pr2_fingertip_sensors')
model_name = {'l_fingertip':'data/20231216_162637_r_gripper_l_fingertip_100_logistic_adam_0001.sav',
              'r_fingertip':'data/20231216_162637_r_gripper_r_fingertip_100_logistic_adam_0001.sav'}

class Predict(object):
    def __init__(self):
        self.models = {} 
        self.gripper = 'r_gripper'
        self.fingertips = ['l_fingertip', 'r_fingertip']
        self.pubs = {}
        self.subs = {}
        for fingertip in self.fingertips:
            filename = os.path.join(package_path, model_name[fingertip])
            self.models[fingertip] = pickle.load(open(filename, 'rb'))
            self.pubs[fingertip] = rospy.Publisher('/pfs/{}/{}/touch_status'.format(self.gripper, fingertip), String, queue_size=1)
            self.subs[fingertip] = rospy.Subscriber(
                '/pfs/{}/{}/proximity_derivative_ma'.format(self.gripper, fingertip),
                SensorArray, self.cb, fingertip, queue_size=10)

    def cb(self, msg, fingertip):
        input_array = np.array(msg.data).reshape(1, -1)# /pfs_params['pfs']['r_gripper']['l_fingertip']['proximity_a'][8:12]
        label = self.models[fingertip].predict(input_array)
        pub_msg = String(data=label[0])
        self.pubs[fingertip].publish(pub_msg)
        # if label == '0':
        #     rospy.loginfo(label)
        # elif label == 'T':
        #     rospy.logerr(label)
        # elif label == 'R':
        #     rospy.logwarn(label)

if __name__ == '__main__':
    rospy.init_node('predict')
    pr = Predict()
    rospy.spin()
