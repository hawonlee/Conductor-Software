#!/usr/bin/env python

import rospy
import numpy as np
from hello_world.msg import kinect_data

def tracking():
	pub = rospy.Publisher('kinect_data', kinect_data_t, queue_size=10)
	rospy.init_node('tracking', anonymous=True)
	rate = rospy.Rate(1)
	while not rospy.is_shutdown():
		msg = kinect_data()
		msg.origin = [1, 2, 3]
		msg.head = [1, 2, 3]
		msg.leftShoulder = [1, 2, 3]
		msg.rightShoulder = [1, 2, 3]
		msg.hipTorso = [1, 2, 3]
		msg.timestamp = str(rospy.get_time())
		rospy.loginfo(msg)
		pub.publish(msg)
		rate.sleep()
		
if __name__ == '__main__':
	try:
		tracking()
	except rospy.ROSInterruptException:
		pass
