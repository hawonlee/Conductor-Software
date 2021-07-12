#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray

pub = rospy.Publisher('talk', Float64MultiArray, queue_size=10)

def callback(data):
	pub.publish(data)
	rospy.loginfo("second listener recieved: %s", data.data)

def second_listener():
	rospy.init_node('second_listener', anonymous=True)
	rospy.Subscriber("chatter", Float64MultiArray, callback)
	rospy.spin()
	
if __name__ == '__main__':
	second_listener()
