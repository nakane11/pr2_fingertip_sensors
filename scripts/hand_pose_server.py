#!/usr/bin/env python

import actionlib
import rospy
from std_msgs.msg import String, UInt8
from pr2_fingertip_sensors.srv import HandPose, HandPoseResponse
from pr2_controllers_msgs.msg import Pr2GripperCommandAction, Pr2GripperCommandGoal
stable_grip = []
unstable_grip = []

class HandPoseServer(object):
    def __init__(self):
        self.grip_state_buf = []
        self.force_state_buf = []
        self.touch_state_buf = {'l_fingertip':[], 'r_fingertip':[]}
        self.r = rospy.Rate(5)
        self.client = actionlib.SimpleActionClient('/r_gripper_controller/gripper_action',
                                                   Pr2GripperCommandAction)
        self.client.wait_for_server()
        self.grip_sub = rospy.Subscriber('/pfs/r_gripper/grip_state', String,
                                    self.grip_state_cb, queue_size=1)
        self.l_touch_sub = rospy.Subscriber('/pfs/r_gripper/l_fingertip/pfs_a_front/touch_state', UInt8,
                                            self.touch_state_cb, "l_fingertip", queue_size=1)
        self.r_touch_sub = rospy.Subscriber('/pfs/r_gripper/r_fingertip/pfs_a_front/touch_state', UInt8,
                                            self.touch_state_cb, "r_fingertip", queue_size=1)        
        self.force_sub = rospy.Subscriber('/pfs/r_gripper/force_state', String,
                                          self.force_state_cb, queue_size=10)
        s = rospy.Service('~start_holding', HandPose, self.start_holding)

    def start_holding(self, req):
        # 握られるのを待つ
        count = 0
        while (not rospy.is_shutdown()) and count < 40: # 40/5=8s
            # 間に指を入れたそうなとき
            if sum(x == statistics.mode(self.grip_state_buf)
                   for x in self.grip_state_buf) > 20 and \
                statistics.mode(self.grip_state_buf) == 'aidaniyubi':
                break
            # 同じ握り方が続くかつ安定した握り方の場合
            if sum(x == statistics.mode(self.grip_state_buf)
                   for x in self.grip_state_buf) > 30 and \
                statistics.mode(self.grip_state_buf) in stable_grip:
                self.pose = statistics.mode(self.grip_state_buf)
                return HandPoseResponse(handpose=self.pose)
            # 強く握られた場合
            if sum(x=='s' for x in self.force_state_buf) > 2:
                self.pose = statistics.mode(self.grip_state_buf)
                return HandPoseResponse(handpose=self.pose)
            count += 1
            self.r.sleep()

        # 指を開く
        goal = Pr2GripperCommandGoal()
        goal.command.position = 0.09
        goal.command.max_effort = 25
        self.client.send_goal(goal)
        self.client.wait_for_result()
        # 握られるのを待つ
        while not rospy.is_shutdown():
            if statistics.mode(self.grip_state_buf) == 'left':
                self.pose = 'left'
                break
            if statistics.mode(self.grip_state_buf) == 'right':
                self.pose = 'right'
                break
            self.r.sleep()
        # 接触するまで握り返す
        i = 0
        while not rospy.is_shutdown():
            goal = Pr2GripperCommandGoal()
            goal.command.position = 0.09-0.005*i
            goal.command.max_effort = 25
            self.client.send_goal(goal)
            self.client.wait_for_result()
            self.r.sleep()
            if self.pose == 'left':
                if 1 in self.touch_state_buf['l_fingertip']:
                    break
            if self.pose == 'right':
                if 1 in self.touch_state_buf['r_fingertip']:
                    break
            i += 1
            if i == 14: # グリッパの最小幅
                break
            self.r.sleep()
        return HandPoseResponse(handpose='hoge')

    def grip_state_cb(self, msg):
        self.grip_state_buf.append(msg.data)
        if len(self.grip_state_buf) > 50:
            self.grip_state_buf = self.grip_state_buf[1:]

    def touch_state_cb(self, msg, fingertip):
        self.touch_state_buf[fingertip].append(msg.data)
        if len(self.force_state_buf) > 5:
            self.touch_state_buf[fingertip] = self.touch_state_buf[fingertip][1:]

    def force_state_cb(self, msg):
        self.force_state_buf.append(msg.data)
        if len(self.force_state_buf) > 5:
            self.force_state_buf = self.force_state_buf[1:]

if __name__ == '__main__':
    rospy.init_node('hand_pose_server')
    hps = HandPoseServer()
    rospy.spin()
