#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray

def callback(data):
	rospy.loginfo("last listener received: %s", data.data)

def last_listener():
	rospy.init_node('last_listener', anonymous=True)
	rospy.Subscriber("talk", Float64MultiArray, callback)
	rospy.spin()

if __name__ == '__main__':
	last_listener()
