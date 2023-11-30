#!/usr/bin/env python

import numpy as np
import pickle
import rospy
from pr2_fingertip_sensors.msg import ProximityDistanceArray

class Predict(object):
    def __init__(self):
        filename = 'data/100_identity_lbfgs_01_7cls.sav'
        self.model = pickle.load(open(filename, 'rb'))
        self.gripper = 'l_gripper'
        self.fingertips = ['l_fingertip']
        self.sub = {}
        for fingertip in self.fingertips:
            self.sub[fingertip] = rospy.Subscriber(
                        '/pfs/{}/{}/proximity_distances'.format(self.gripper, fingertip),
                        ProximityDistanceArray, self.cb, fingertip, queue_size=1)

    def cb(self, msg, arg):
        fingertip = arg
        topic = '/pfs/{}/{}/proximity_distances'.format(self.gripper, fingertip)
        distances = np.array(msg.distances, ndmin=2)
        label = self.model.predict(distances)
        rospy.loginfo(label)
        
if __name__ == '__main__':
    rospy.init_node('predict')
    pr = Predict()
    rospy.spin()
