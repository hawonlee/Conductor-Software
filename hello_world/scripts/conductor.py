#!/usr/bin/env python

import rospy
from hello_world.msg import *
#timestamp = "a"

def callback_virtual_console(data):
	rospy.loginfo("source data\nsource position: %s \nsource direction: %s, \nsource aperture: %s \nmAs: %s \nshield Euler angles: %s \nshield translation: %s \nshield attCoef: %s \nenergy spectrum file: %s \ntimestamp: %s" % (data.source_position, data.source_direction, data.source_aperture, data.mAs, data.shield_Euler_angles, data.shield_translation, data.shield_attCoef, data.energy_spectrum_file, data.timestamp))
#	global timestamp
#	timestamp = data.timestamp
	monte_carlo(data)
	print()

def callback_tracking(data):
	rospy.loginfo("tracking\norigin: %s \nhead: %s \nleft shoulder: %s \nright shoulder: %s \nhip torso: %s \ntimestamp: %s" % (data.origin, data.head, data.leftShoulder, data.rightShoulder, data.hipTorso, data.timestamp))
	print()

def monte_carlo(data):
	print("run monte carlo code")

def conductor():
	rospy.init_node('conductor', anonymous=True)
	
	rospy.Subscriber("SOURCE_CHANNEL", source_data_t, callback_virtual_console)
	rospy.Subscriber("kinect_data", kinect_data_t, callback_tracking)
	rospy.spin()
	
#	last_timestamp = "a"
#	while not rospy.is_shutdown():
#		try:
#			rospy.Subscriber("SOURCE_CHANNEL", source_data_t, callback_virtual_console)
#		except rospy.ROSInterruptException:
#			pass
#		global timestamp
#		if last_timestamp != timestamp
#			last_timestamp = timestamp
#			print("run monte carlo code")
#			run monte carlo

if __name__ == '__main__':
	conductor()
