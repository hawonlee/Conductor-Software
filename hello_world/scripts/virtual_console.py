#!/usr/bin/env python

import rospy
import numpy as np
from hello_world.msg import source_data

def virtual_console():
	pub = rospy.Publisher('mult_type_chatter', source_data, queue_size=10)
	rospy.init_node('virtual_console', anonymous=True)
	rate = rospy.Rate(1)
	while not rospy.is_shutdown():
		msg = source_data()
		msg.source_position = [1, 2, 3]
		msg.source_direction = [1, 2, 3]
		msg.source_aperture = [1, 2, 3]
		msg.mAs = 10
		msg.shield_Euler_angles = [1, 2, 3]
		msg.shield_translation = [1, 2, 3]
		msg.shield_attCoef = [1, 2, 3]
		msg.energy_spectrum_file = "energy spectrum file"
		msg.timestamp = str(rospy.get_time())
		rospy.loginfo(msg)
		pub.publish(msg)
		rate.sleep()
		
if __name__ == '__main__':
	try:
		virtual_console()
	except rospy.ROSInterruptException:
		pass
