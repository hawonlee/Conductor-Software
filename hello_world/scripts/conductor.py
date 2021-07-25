#!/usr/bin/env python

import rospy
import time
from hello_world.msg import *
message = False

def callback_virtual_console(data):
#	global message
#	if message != True:
	rospy.loginfo("source data\nsource position: %s \nsource direction: %s, \nsource aperture: %s \nmAs: %s \nshield Euler angles: %s \nshield translation: %s \nshield attCoef: %s \nenergy spectrum file: %s \ntimestamp: %s" % (data.source_position, data.source_direction, data.source_aperture, data.mAs, data.shield_Euler_angles, data.shield_translation, data.shield_attCoef, data.energy_spectrum_file, data.timestamp))
	#create a queue of messages or send messages to Monte Carlo node
	message = True
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
	global message

	while not rospy.core.is_shutdown():
		rospy.rostime.wallsleep(0.5)
		if message:
			print("message received")
			message = False

if __name__ == '__main__':
	conductor()
