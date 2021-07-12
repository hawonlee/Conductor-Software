#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray

def callback(data):
	rospy.loginfo("first listener received: %s", data.data)

def listener():
	rospy.init_node('listener', anonymous=True)
	rospy.Subscriber("chatter", Float64MultiArray, callback)
	rospy.spin()

if __name__ == '__main__':
	listener()
