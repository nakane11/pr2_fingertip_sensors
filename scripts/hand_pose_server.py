#!/usr/bin/env python

import actionlib
import rospy
import statistics
from std_msgs.msg import String, UInt8
from pr2_fingertip_sensors.srv import HandPose, HandPoseResponse
from pr2_fingertip_sensors.srv import SwitchGripperState
from pr2_controllers_msgs.msg import Pr2GripperCommandAction, Pr2GripperCommandGoal

RATE = 10
class HandPoseServer(object):
    def __init__(self):
        rospy.loginfo('11111111111')
        self.grip_state_buf = []
        self.force_state_buf = []
        self.touch_state_buf = {'l_fingertip':[], 'r_fingertip':[]}
        self.gripper_state = None
        self.hand_pose = None
        self.r = rospy.Rate(RATE)
        self.timer_running = False
        rospy.loginfo('222222222222')
        self.pr2_gripper_client = actionlib.SimpleActionClient('/r_gripper_controller/gripper_action',
                                                   Pr2GripperCommandAction)
        rospy.loginfo('33333333333333333')
        self.pr2_gripper_client.wait_for_server()
        rospy.loginfo('44444444444444444')
        rospy.loginfo('found gripper_action')
        rospy.wait_for_service('/predict_grip_state/switch_gripper')
        self.gripper_state_client = rospy.ServiceProxy('/predict_grip_state/switch_gripper', SwitchGripperState)
        self.grip_sub = rospy.Subscriber('/pfs/r_gripper/grip_state', String,
                                    self.grip_state_cb, queue_size=1)
        self.l_touch_sub = rospy.Subscriber('/pfs/r_gripper/l_fingertip/pfs_a_front/touch_state', UInt8,
                                            self.touch_state_cb, "l_fingertip", queue_size=1)
        self.r_touch_sub = rospy.Subscriber('/pfs/r_gripper/r_fingertip/pfs_a_front/touch_state', UInt8,
                                            self.touch_state_cb, "r_fingertip", queue_size=1)        
        self.force_sub = rospy.Subscriber('/pfs/r_gripper/force_state', String,
                                          self.force_state_cb, queue_size=10)
        start = rospy.Service('~start_holding', HandPose, self.start_holding)
        stop = rospy.Service('~stop_holding', HandPose, self.stop_holding)
        rospy.Timer(rospy.Duration(0.1), self.timer_cb)
        rospy.loginfo('init end')

    def move_gripper(self, pos, effort=25): # pos(m)
        goal = Pr2GripperCommandGoal()
        goal.command.position = pos
        goal.command.max_effort = effort
        self.pr2_gripper_client.send_goal(goal)
        self.pr2_gripper_client.wait_for_result()

    def start_holding(self, req):
        self.hand_pose = None
        if req.reset is True or self.gripper_state is None:
            self.set_state(0)
        self.timer_running = True
        while not rospy.is_shutdown():
            if self.hand_pose is not None:
                break
            self.r.sleep()
        if not req.continuous:
            self.timer_running = False
        return HandPoseResponse(handpose=self.hand_pose)

    def stop_holding(self, req):
        rospy.loginfo("stop")
        self.hand_pose = None
        self.move_gripper(0.09) #グリッパを開く
        rospy.sleep(2)
        self.move_gripper(0.0)
        return HandPoseResponse()

    def set_state(self, state):
        rospy.loginfo('{}->{}'.format(self.gripper_state, state))
        if state == 0:
            self.move_gripper(0.015) #グリッパを少し開く
            res = self.gripper_state_client('state0')
        elif state == 1:
            self.move_gripper(0.0) #グリッパを閉じる
            res = self.gripper_state_client('state1')
        elif state == 2:
            self.move_gripper(0.09) #グリッパを開く
            res = self.gripper_state_client('state2')
        elif state == 3:
            res = self.gripper_state_client('state3')
        self.gripper_state = state
        self.count = 0

    def timer_cb(self, event):
        rospy.loginfo('timer {} {}'.format(self.timer_running, self.gripper_state))
        if self.timer_running is False:
            return
        if self.gripper_state == 0:
            self.state0()
        elif self.gripper_state == 1:
            self.state1()
        elif self.gripper_state == 2:
            self.state2()
        elif self.gripper_state == 3:
            self.state3()

    def state0(self):
        # 安定して握られているとき
        if sum(x == statistics.mode(self.grip_state_buf) for x in self.grip_state_buf) > 12 and \
           statistics.mode(self.grip_state_buf) == 'both_front':
            self.set_state(1)
        elif sum(x == statistics.mode(self.grip_state_buf) for x in self.grip_state_buf) > 12 and \
             statistics.mode(self.grip_state_buf) == 'both_back':
            self.set_state(1)
        # 間に指を入れたそうなとき
        elif sum(x == statistics.mode(self.grip_state_buf) for x in self.grip_state_buf) > 8 and \
             statistics.mode(self.grip_state_buf) == 'left':
            self.set_state(2)
        elif sum(x == statistics.mode(self.grip_state_buf) for x in self.grip_state_buf) > 8 and \
             statistics.mode(self.grip_state_buf) == 'right':
            self.set_state(2)
        # 握り方が不安定なとき
        elif sum(x == statistics.mode(self.grip_state_buf) for x in self.grip_state_buf) > 12 and \
             statistics.mode(self.grip_state_buf) == 'both_bottom':
            self.set_state(2)
        # 握り方が一貫しないとき
        elif self.count > 8*RATE: # 8s
            rospy.loginfo('state0 timeout')
            self.set_state(2)
        else:
            self.count += 1

    def state1(self):
        # 安定して握れているとき
        if sum(x == statistics.mode(self.grip_state_buf) for x in self.grip_state_buf) > 8 and \
           statistics.mode(self.grip_state_buf) == 'both_front':
            self.count = 0
            self.hand_pose = 'both_front'
        elif sum(x == statistics.mode(self.grip_state_buf) for x in self.grip_state_buf) > 8 and \
           statistics.mode(self.grip_state_buf) == 'both_back':
            self.count = 0
            self.hand_pose = 'both_back'
        # 手が離されたとき
        elif sum(x == statistics.mode(self.grip_state_buf) for x in self.grip_state_buf) > 18 and \
           statistics.mode(self.grip_state_buf) == 'free':
            self.hand_pose = None
            self.set_state(0)
        # 握り方が一貫しないとき
        elif self.count > 8*RATE: # 8s
            rospy.loginfo('state1 timeout')
            self.hand_pose = None
            self.set_state(0)
        else:
            self.count += 1

    def state2(self):
        # 左指を握られたとき
        if sum(x == statistics.mode(self.grip_state_buf) for x in self.grip_state_buf) > 8 and \
           statistics.mode(self.grip_state_buf) == 'left':
            if sum(x=='s' for x in self.force_state_buf) > 2:
                self.grip_back('left')
                self.hand_pose = 'left'
                self.set_state(3)
            else:
                self.count = 0
        # 右指を握られたとき
        if sum(x == statistics.mode(self.grip_state_buf) for x in self.grip_state_buf) > 8 and \
           statistics.mode(self.grip_state_buf) == 'right':
            rospy.loginfo(self.force_state_buf)
            if sum(x=='s' for x in self.force_state_buf) > 2:
                self.grip_back('right')
                self.hand_pose = 'right'
                self.set_state(3)
            else:
                self.count = 0
        # 握り方が一貫しないとき
        elif self.count > 8*RATE: # 8s
            rospy.loginfo('state2 timeout')
            self.hand_pose = None
            self.set_state(0)
        else:
            self.count += 1

    def state3(self):
        # 手を離したそうなとき
        if sum(x == 'left_post' for x in self.grip_state_buf) > 6:
            self.hand_pose = None
            self.set_state(2)
        elif sum(x == 'right_post' for x in self.grip_state_buf) > 6:
            self.hand_pose = None
            self.set_state(2)

    def grip_back(self, finger):
        rospy.loginfo("grip back")
        # 接触するまで握り返す
        i = 0
        while not rospy.is_shutdown():
            self.move_gripper(0.09-0.0025*i)
            self.r.sleep()
            if finger == 'left':
                rospy.loginfo(self.touch_state_buf['l_fingertip'])
                if 1 in self.touch_state_buf['l_fingertip']:
                    break
                i += 1
                rospy.loginfo('{}'.format(i))
                if i == 30: # グリッパの最小幅
                    rospy.loginfo('limit')
                    break
            if finger == 'right':
                rospy.loginfo(self.touch_state_buf['r_fingertip'])
                if 1 in self.touch_state_buf['r_fingertip']:
                    break
                i += 1
                rospy.loginfo('{}'.format(i))
                if i == 26: # グリッパの最小幅
                    rospy.loginfo('limit')
                    break

    def grip_state_cb(self, msg):
        # rospy.loginfo("new grip msg: {}".format(msg.data))
        self.grip_state_buf.append(msg.data)
        if len(self.grip_state_buf) > 20:
            self.grip_state_buf = self.grip_state_buf[1:]

    def touch_state_cb(self, msg, fingertip):
        # rospy.loginfo("new touch msg: {}".format(msg.data))
        self.touch_state_buf[fingertip].append(msg.data)
        if len(self.touch_state_buf[fingertip]) > 5:
            self.touch_state_buf[fingertip] = self.touch_state_buf[fingertip][1:]

    def force_state_cb(self, msg):
        # rospy.loginfo("new force msg: {}".format(msg.data))
        self.force_state_buf.append(msg.data)
        if len(self.force_state_buf) > 5:
            self.force_state_buf = self.force_state_buf[1:]

if __name__ == '__main__':
    rospy.init_node('hand_pose_server')
    hps = HandPoseServer()
    rospy.spin()
