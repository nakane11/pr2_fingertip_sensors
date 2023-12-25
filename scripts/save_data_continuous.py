#!/usr/bin/env python

import sys
import csv
import datetime
from pynput.keyboard import Listener, Key, KeyCode
import rospy
from rostopic import ROSTopicHz 
from pr2_fingertip_sensors.msg import SensorArray
from sensor_msgs.msg import JointState

class MyException(Exception): pass

class SaveDataContinuous(object):
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
        indices = ['time', 'label'] + ['l_prox_{}'.format(i) for i in range(24)] \
            + ['l_force_{}'.format(i) for i in range(24)] \
            + ['r_prox_{}'.format(i) for i in range(24)] \
            + ['r_force_{}'.format(i) for i in range(24)]
        # indices = ['time', 'label'] + ['prox_{}'.format(i) for i in range(24)] \
        #     + ['force_{}'.format(i) for i in range(24)] + ['r_gripper_joint']
        with open(self.filepath, 'a') as f:
            writer = csv.writer(f)
            writer.writerow(indices)
        self.s = ''
        self.cnt = 0
        self.subscribe()
        self.record = False
        rospy.Timer(rospy.Duration(0.1), self.timer_cb)
        
    def subscribe(self):
        self.sub = {}
        for fingertip in self.fingertips:
            self.sub[fingertip] = {}
            for topic in self.topics:
                self.sub[fingertip][topic] = rospy.Subscriber(
                    '/pfs/{}/{}/{}'.format(self.gripper, fingertip, topic),
                    SensorArray, self.cb, (fingertip, topic), queue_size=1)        
        # self.sub['joint'] = rospy.Subscriber(
        #     '/joint_states', JointState, self.joint_cb, queue_size=1)

    def cb(self, msg, args):
        fingertip = args[0]
        topic = args[1]
        topic_name = '/pfs/{}/{}/{}'.format(self.gripper, fingertip, topic)
        self.rt.callback_hz(msg, topic_name)
        self.data_array[topic] = list(msg.data)
        self.update_time[topic] = msg.header.stamp

    def joint_cb(self, msg):
        self.rt.callback_hz(msg, '/joint_states')
        if not 'r_gripper_joint' in msg.name:
            return
        idx = msg.name.index('r_gripper_joint')
        self.data_array['joint'] = [msg.position[idx]]
        self.update_time['joint'] = msg.header.stamp

    def save(self):
        label = self.s
        start_time = rospy.Time.now()
        save_data = [start_time, label]
        for fingertip in self.fingertips:        
            for topic in self.topics:
                tmp = self.rt.get_hz('/pfs/{}/{}/{}'.format(
                    self.gripper, fingertip, topic))
                if tmp is not None:
                    sensor_rate = tmp[0]
                else:
                    sensor_rate = 12
                while True:
                    if (self.update_time[topic] -start_time).to_sec() > 0:
                        save_data += self.data_array[topic]
                        break
                    if (rospy.Time.now() - start_time).to_sec() > 2/sensor_rate:
                        rospy.logerr('Timeout: failed to get /pfs/{}/{}/{}'.format(
                            self.gripper, fingertip, topic))
                        return
                    self.rate.sleep()
        # tmp = self.rt.get_hz('/joint_states')
        # if tmp is not None:
        #     joint_rate = tmp[0]
        # else:
        #     joint_rate = 100 
        # while True:
        #     if (self.update_time['joint'] -start_time).to_sec() > 0:
        #         save_data += self.data_array['joint']
        #         break
        #     if (rospy.Time.now() - start_time).to_sec() > 2/joint_rate:
        #         rospy.logerr('Timeout: failed to get /joint_states')
        #         return
        #     self.rate.sleep()
        self.cnt += 1
        with open(self.filepath, 'a') as f:
            writer = csv.writer(f)
            writer.writerow(save_data)
        rospy.loginfo('Save data: {}..{}'.format(self.cnt, label))
        return

    def timer_cb(self, event):
        if self.record is True:
            self.save()
        return
    
    def on_press(self, key):
        if key == Key.esc:
            raise MyException()
        elif key == Key.space:
            if self.record is True:
                self.record = False
                rospy.logerr('record stop')
            else:
                self.record = True
                rospy.logerr('record start')
        elif key == Key.backspace:
            self.s = self.s[:-1]                
        elif type(key) == KeyCode:
            self.s = self.s + key.char
        rospy.loginfo('input: {}'.format(self.s))
            
if __name__ == '__main__':
    rospy.init_node('save_data_continuous')
    sd = SaveDataContinuous()
    with Listener(on_press=sd.on_press) as listener:
        try:
            listener.join()
        except MyException as e:
            rospy.loginfo('Exit.')
            sys.exit()

    
