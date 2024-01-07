#!/usr/bin/env python

import rospy
from pr2_fingertip_sensors.msg import Float32Stamped
# import matplotlib.pyplot as plt 
import numpy as np
from datetime import datetime

rospy.init_node('test')
pub = rospy.Publisher('~output', Float32Stamped, queue_size=1)
pub2 = rospy.Publisher('~trigger', Float32Stamped, queue_size=1)
wave = [[0,2],[0.5,2.5],
        [0,2],[0.5,1],[1,2],[0.5,1.5],
        [0,2],[0.5,1.5],
        [0,1.5],[0.5,2],[1,1.0],
        [0,1]]

# 描画領域を取得
# fig, ax = plt.subplots(1, 1)

# y軸方向の描画幅を指定
# ax.set_ylim((-0.1, 1.1))

# x軸:時刻
x_length = 0
y = []
for w in wave:
    x_length += w[1]
    y += [w[0]] * int(w[1]/0.01)
y = np.array(y)
x = np.arange(0, int(x_length), 0.01)

i = 0
tmp = 0
while not rospy.is_shutdown():
    if i >= len(x):
        break
    # start = datetime.now()
    pub_msg = Float32Stamped()
    pub_msg.header.stamp = rospy.Time.now()
    pub_msg.data = y[i]
    pub.publish(pub_msg)
    if y[i] > tmp:
        pub_msg.data = 0
        pub2.publish(pub_msg)
    elif y[i] < tmp:
        pub_msg.data = 1
        pub2.publish(pub_msg)
    tmp = y[i]
    rospy.sleep(0.01)
    # if x[i] < 5:
    #     ax.set_xlim((0, 5))
    #     x_plt = x[:i+1]
    #     y_plt = y[:i+1]
    # else:
    #     ax.set_xlim((x[i]-5, x[i]))
    #     x_plt = x[i-499:i+1]
    #     y_plt = y[i-499:i+1]
    # end = datetime.now()
    # print(end - start)
    # line, = ax.plot(x_plt, y_plt, color='blue')
    # plt.pause(0.01)
    # line.remove()    
    i += 1
