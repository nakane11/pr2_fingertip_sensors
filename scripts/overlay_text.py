#!/usr/bin/env python

from jsk_rviz_plugins.msg import OverlayText
from std_msgs.msg import String
import rospy

def colored_message(msg):
    if msg.data == '0':
        return '<span style="color: white;">%s</span>' % msg.data
    elif msg.data == 'R':
        return '<span style="color: green;">%s</span>' % msg.data
    elif msg.data == 'T':
        return '<span style="color: red;">%s</span>' % msg.data

def callback(msg):
    global lines
    lines = lines + [colored_message(msg)]
    if len(lines) > line_buffer_length:
        lines = lines[-line_buffer_length:]
    text = OverlayText()
    text.left = 20
    text.top = 20
    text.width = 1200
    text.height = 1200
    text.fg_color.a = 1.0
    text.fg_color.r = 0.3
    text.text_size = 30
    text.text = "\n".join(lines)
    pub.publish(text)


if __name__ == "__main__":
    rospy.init_node("overlay_text")
    line_buffer_length = rospy.get_param("~line_buffer_length", 10)
    lines = []
    sub = rospy.Subscriber("~input", String, callback)
    pub = rospy.Publisher("~output", OverlayText, queue_size=1)
    rospy.spin()
