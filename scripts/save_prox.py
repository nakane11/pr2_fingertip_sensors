#!/usr/bin/env python

import sys
import csv
from pynput.keyboard import Listener, Key, KeyCode
import rospy
from rostopic import ROSTopicHz 
from pr2_fingertip_sensors.msg import ProximityDistanceArray

class SaveProx(object):
    def __init__(self):
        self.gripper = 'l_gripper'
        self.fingertips = ['l_fingertip']
        self.update_time = None
        self.dist_array = []
        self.rt = ROSTopicHz(10) # window size
        self.rate = rospy.Rate(50)
        self.filepath = rospy.get_param('~filepath', '/home/nakane/Documents/proximity.csv')
        self.s = ''
        self.cnt = 0
        self.subscribe()
        
    def subscribe(self):
        self.sub = {}
        for fingertip in self.fingertips:
            self.sub[fingertip] = rospy.Subscriber(
                        '/pfs/{}/{}/proximity_distances'.format(self.gripper, fingertip),
                        ProximityDistanceArray, self.cb, fingertip, queue_size=1)

    def cb(self, msg, arg):
        fingertip = arg
        topic = '/pfs/{}/{}/proximity_distances'.format(self.gripper, fingertip)
        self.rt.callback_hz(msg, topic)
        self.dist_array = list(msg.distances)
        self.update_time = msg.header.stamp

    def save(self, label):
        rospy.loginfo('label: {}'.format(label))
        for fingertip in self.fingertips:
            start_time = rospy.Time.now()
            rate = self.rt.get_hz('/pfs/{}/{}/proximity_distances'.format(
                self.gripper, fingertip))[0]
            while (rospy.Time.now() - start_time).to_sec() < 2/rate:
                if (self.update_time -start_time).to_sec() > 0:
                    self.cnt += 1
                    with open(self.filepath, 'a') as f:
                        writer = csv.writer(f)
                        writer.writerow([label]+self.dist_array)
                    rospy.loginfo('Save proximities: {}'.format(self.cnt))
                    return
                self.rate.sleep()
            rospy.loginfo('/pfs/{}/{}/proximity_distances has not been updated'.format(
                self.gripper, fingertip))

    def on_press(self, key):
        if key == Key.esc:
            sys.exit()
        elif key == Key.enter:
            self.save(self.s)
        elif key == Key.backspace:
            self.s = self.s[:-1]
        elif type(key) == KeyCode:
            self.s = self.s + key.char
        rospy.loginfo('input: {}'.format(self.s))
            


                
if __name__ == '__main__':
    rospy.init_node('save_prox')
    sp = SaveProx()
    with Listener(on_press=sp.on_press) as listener:
        listener.join()
    rospy.spin()
    
