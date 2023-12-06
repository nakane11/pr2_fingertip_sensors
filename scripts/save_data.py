#!/usr/bin/env python

import sys
import csv
import datetime
from pynput.keyboard import Listener, Key, KeyCode
import rospy
from rostopic import ROSTopicHz 
from pr2_fingertip_sensors.msg import SensorArray
from sensor_msgs.msg import JointState

class SaveData(object):
    def __init__(self):
        self.gripper = 'r_gripper'
        self.fingertips = ['l_fingertip', 'r_fingertip']
        self.topics = ['proximity_distances', 'forces']

        self.update_time = {}
        self.data_array = {}
        for fingertip in self.fingertips:
            self.update_time[fingertip] = {}
            for topic in self.topics:
                self.update_time[fingertip][topic] = rospy.Time(0)
            self.data_array[fingertip] = {}
        self.update_time['joint'] = rospy.Time(0)
        self.data_array['joint'] = None

        self.rt = ROSTopicHz(10) # window size
        self.rate = rospy.Rate(50)
        
        savedir = rospy.get_param('~savedir', '/home/nakane/Documents')
        dt = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        self.filepath = '{}/{}_{}.csv'.format(savedir, dt, self.gripper)
        indices = ['label'] + ['l_prox_{}'.format(i) for i in range(24)] \
            + ['l_force_{}'.format(i) for i in range(24)] \
            + ['r_prox_{}'.format(i) for i in range(24)] \
            + ['r_force_{}'.format(i) for i in range(24)] + ['r_gripper_joint']
        with open(self.filepath, 'a') as f:
            writer = csv.writer(f)
            writer.writerow(indices)
        self.s = ''
        self.cnt = 0
        self.subscribe()
        
    def subscribe(self):
        self.sub = {}
        for fingertip in self.fingertips:
            self.sub[fingertip] = {}
            for topic in self.topics:
                self.sub[fingertip][topic] = rospy.Subscriber(
                    '/pfs/{}/{}/{}'.format(self.gripper, fingertip, topic),
                    SensorArray, self.cb, (fingertip, topic), queue_size=1)
        self.sub['joint'] = rospy.Subscriber(
            '/joint_states', JointState, self.joint_cb, queue_size=1)

    def cb(self, msg, args):
        fingertip = args[0]
        topic = args[1]
        topic_name = '/pfs/{}/{}/{}'.format(self.gripper, fingertip, topic)
        self.rt.callback_hz(msg, topic_name)
        self.data_array[fingertip][topic] = list(msg.data)
        self.update_time[fingertip][topic] = msg.header.stamp

    def joint_cb(self, msg):
        self.rt.callback_hz(msg, '/joint_states')
        if not 'r_gripper_joint' in msg.name:
            return
        idx = msg.name.index('r_gripper_joint')
        self.data_array['joint'] = [msg.position[idx]]
        self.update_time['joint'] = msg.header.stamp

    def save(self, label):
        rospy.loginfo('label: {}'.format(label))
        start_time = rospy.Time.now()
        save_data = [label]
        for fingertip in self.fingertips:
            for topic in self.topics:
                rate = self.rt.get_hz('/pfs/{}/{}/{}'.format(
                    self.gripper, fingertip, topic))[0]
                while True:
                    if (self.update_time[fingertip][topic] -start_time).to_sec() > 0:
                        save_data += self.data_array[fingertip][topic]
                        break
                    if (rospy.Time.now() - start_time).to_sec() > 2/rate:
                        rospy.logerr('Timeout: failed to get /pfs/{}/{}/{}').format(
                            self.gripper, fingertip, topic)[0]
                        return                    
                    self.rate.sleep()
                    rospy.loginfo('Waiting for /pfs/{}/{}/{}...'.format(
                        self.gripper, fingertip, topic))
        rate = self.rt.get_hz('/joint_states')[0]
        while True:
            if (self.update_time['joint'] -start_time).to_sec() > 0:
                save_data += self.data_array['joint']
                break
            if (rospy.Time.now() - start_time).to_sec() > 2/rate:
                rospy.logerr('Timeout: failed to get /joint_states')
                return            
            self.rate.sleep()
            rospy.loginfo('Waiting for /joint_states...')
        self.cnt += 1
        with open(self.filepath, 'a') as f:
            writer = csv.writer(f)
            writer.writerow(save_data)
        rospy.loginfo('Save data: {}'.format(self.cnt))
        return

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
    rospy.init_node('save_data')
    sd = SaveData()
    with Listener(on_press=sd.on_press) as listener:
        listener.join()
    rospy.spin()
    
