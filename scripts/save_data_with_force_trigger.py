#!/usr/bin/env python

import sys
import csv
import datetime
import time
import rospy
import random
from pr2_fingertip_sensors.msg import SensorArray
from std_msgs.msg import Bool

class SaveDatawithForceTrigger(object):
    def __init__(self):
        self.gripper = 'r_gripper'
        self.fingertips = ['l_fingertip', 'r_fingertip']
        self.pfs_params = rospy.get_param('/pfs')

        self.filepath = {}
        savedir = rospy.get_param('~savedir', '/home/nakane/Documents')
        dt = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        indices = ['time', 'label'] + ['prox_{}'.format(i) for i in range(4)]

        self.rate = rospy.Rate(100)
        self.count = {}
        self.prox_buffer = {}
        self.sub = {}
        self.touch_status = {}
        for fingertip in self.fingertips:
            self.filepath[fingertip] = '{}/{}_{}_{}.csv'.format(savedir, dt, self.gripper, fingertip)
            with open(self.filepath[fingertip], 'a') as f:
                writer = csv.writer(f)
                writer.writerow(indices)

            self.count[fingertip] = {'T':0, 'R':0, '0':0}
            self.prox_buffer[fingertip] = []
            self.touch_status[fingertip] = None
            self.sub[fingertip] = {}
            self.sub[fingertip]['boolean'] = rospy.Subscriber(
                '/boolean_node_checking_conditions_{}/output/or'.format(fingertip),
                Bool, self.bool_cb, fingertip, queue_size=1)
            self.sub[fingertip]['proximity'] = rospy.Subscriber(
                '/pfs/{}/{}/proximity_derivative_ma'.format(self.gripper, fingertip),
                SensorArray, self.prox_cb, fingertip, queue_size=10)

    def bool_cb(self, msg, fingertip):
        time = rospy.Time.now()
        if self.touch_status[fingertip] == False and \
           msg.data == True:
            self.get_touch_data(time, fingertip)
            if fingertip == 'l_fingertip':
                rospy.logwarn("{} 'T':{} 'R':{} '0':{}".format(fingertip, self.count[fingertip]['T'], self.count[fingertip]['R'], self.count[fingertip]['0']))
            else:
                rospy.loginfo("{} 'T':{} 'R':{} '0':{}".format(fingertip, self.count[fingertip]['T'], self.count[fingertip]['R'], self.count[fingertip]['0']))
        elif self.touch_status[fingertip] == True and \
           msg.data == False:
            self.get_release_data(time, fingertip)
            if fingertip == 'l_fingertip':
                rospy.logwarn("{} 'T':{} 'R':{} '0':{}".format(fingertip, self.count[fingertip]['T'], self.count[fingertip]['R'], self.count[fingertip]['0']))
            else:
                rospy.loginfo("{} 'T':{} 'R':{} '0':{}".format(fingertip, self.count[fingertip]['T'], self.count[fingertip]['R'], self.count[fingertip]['0']))
        self.touch_status[fingertip] = msg.data
        rospy.sleep(0.5)

    def prox_cb(self, msg, fingertip):
        self.prox_buffer[fingertip].append(msg)

    def get_touch_data(self, detect_time, fingertip):
        rospy.logerr("touch")
        free_data = False
        idx = 0
        for i, p in enumerate(self.prox_buffer[fingertip]):
            self.rate.sleep()
            if (detect_time - p.header.stamp).to_sec() > 2.0:
                idx = i
            elif (detect_time - p.header.stamp).to_sec() > 0.5:
                # discard old data
                continue
            elif (detect_time - p.header.stamp).to_sec() > 0.1:
                if not free_data:
                    # idx~i-1から保存
                    self.get_free_data(self.prox_buffer[fingertip][idx:i], fingertip)
                    free_data = True
                self.save(p, 'T', fingertip)
                continue
            else:
                self.prox_buffer[fingertip] = self.prox_buffer[fingertip][i:]
                return

    def get_release_data(self, detect_time, fingertip):
        rospy.logerr("release")
        now = rospy.Time.now()
        while (now - detect_time).to_sec() < 0.5:
            flg = False
            for i, p in enumerate(self.prox_buffer[fingertip]):
                self.rate.sleep()
                if (p.header.stamp - detect_time).to_sec() < -0.05:
                    # discard old data
                    pass
                elif (p.header.stamp - detect_time).to_sec() < 0.4:
                    self.save(p, 'R', fingertip)
                else:
                    self.prox_buffer[fingertip] = self.prox_buffer[fingertip][i:]
                    flg = True
                    break
            if flg:
                break
            # bufferがなくなった場合
            self.prox_buffer[fingertip] = []
            # wait for new data
            self.rate.sleep()
            now = rospy.Time.now()
        time.sleep(0.5) # stop saving

    def get_free_data(self, prox_data, fingertip):
        # Max10個保存
        random_data = random.sample(prox_data, min(len(prox_data), 10))
        for p in random_data:
            self.save(p, '0', fingertip)

    def save(self, p, label, fingertip):
        self.count[fingertip][label] += 1
        save_data = [p.header.stamp, label] + list(p.data)
        with open(self.filepath[fingertip], 'a') as f:
            writer = csv.writer(f)
            writer.writerow(save_data)

if __name__ == '__main__':
    rospy.init_node('save_data_with_force_trigger')
    sd = SaveDatawithForceTrigger()
    rospy.spin()
    
