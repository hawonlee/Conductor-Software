#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray

def hello():
	pub = rospy.Publisher('chatter', Float64MultiArray, queue_size=10)
	rospy.init_node('hello', anonymous=True)
	rate = rospy.Rate(1)
	while not rospy.is_shutdown():
		data_to_send = Float64MultiArray()
		data_to_send.data = [1, 2, 3]
		rospy.loginfo("the message is: %s", data_to_send.data)
		pub.publish(data_to_send)
		rate.sleep()
		
if __name__ == '__main__':
	try:
		hello()
	except rospy.ROSInterruptException:
		pass
