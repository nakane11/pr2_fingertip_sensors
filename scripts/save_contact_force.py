#!/usr/bin/env python

import sys
import csv
import datetime
from pynput.keyboard import Listener, Key, KeyCode
import rospy
from pr2_fingertip_sensors.msg import SensorArray

board2index = {'pfs_a_front': [0, 8],
               'pfs_b_top': [8, 12],
               'pfs_b_back': [12, 16],
               'pfs_b_left': [16, 20],
               'pfs_b_right': [20, 24]}

class MyException(Exception): pass

class SaveTouchState(object):
    def __init__(self):
        self.gripper = 'r_gripper'
        self.fingertips = ['l_fingertip', 'r_fingertip']
        self.update_time = {}
        self.data_array = {}
        for fingertip in self.fingertips:
            self.data_array[fingertip] = [0] * 24
        
        savedir = rospy.get_param('~savedir', '/home/nakane/Documents')
        dt = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        self.filepath = '{}/cf_{}_{}.csv'.format(savedir, dt, self.gripper)
        indices = ['time', 'label'] + ['l_force_{}'.format(i) for i in range(24)] \
            + ['r_force_{}'.format(i) for i in range(24)]
        with open(self.filepath, 'a') as f:
            writer = csv.writer(f)
            writer.writerow(indices)
        self.cnt = {'s':0, 'w':0, 'f':0}
        self.record = False
        self.s = ''
        self.subscribe()
        rospy.Timer(rospy.Duration(0.1), self.timer_cb)
        
    def subscribe(self):
        self.sub = {}
        for fingertip in self.fingertips:
            self.sub[fingertip] = {}
            for board in board2index.keys():
                self.sub[fingertip][board] = rospy.Subscriber(
                    '/pfs/{}/{}/{}/contact_force_derivative'.format(self.gripper, fingertip, board),
                    SensorArray, self.cb, (fingertip, board), queue_size=1)

    def cb(self, msg, args):
        fingertip = args[0]
        board = args[1]
        self.data_array[fingertip][board2index[board][0]:board2index[board][1]] = list(msg.data)

    def save(self, label):
        start_time = rospy.Time.now()
        save_data = [start_time, label]
        for fingertip in self.fingertips:
            save_data += self.data_array[fingertip]
        self.cnt[label] += 1
        with open(self.filepath, 'a') as f:
            writer = csv.writer(f)
            writer.writerow(save_data)
        if label == 's':
            rospy.logwarn('Save s data: {}'.format(self.cnt[label]))
        elif label == 'w':
            rospy.loginfo('Save w data: {}'.format(self.cnt[label]))
        elif label == 'f':
            rospy.loginfo('Save f data: {}'.format(self.cnt[label]))
        return

    def timer_cb(self, event):
        if self.record is True:
            self.save(self.s)
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
    rospy.init_node('save_touch_state')
    sd = SaveTouchState()
    with Listener(on_press=sd.on_press) as listener:
        try:
            listener.join()
        except MyException as e:
            rospy.loginfo('Exit.')
            sys.exit()

    
