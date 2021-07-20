#!/usr/bin/env python

import rospy
from hello_world.msg import source_data_t

def virtual_console():
	pub = rospy.Publisher('source_data', source_data_t, queue_size=10)
	rospy.init_node('virtual_console', anonymous=True)
	rate = rospy.Rate(1)
	while not rospy.is_shutdown():
		msg = source_data_t()
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
