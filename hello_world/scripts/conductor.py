#!/usr/bin/env python

import rospy
from hello_world.msg import *

def callback_virtual_console(data):
	rospy.loginfo("source data\nsource position: %s, \nsource direction: %s, \nsource aperture: %s, \nmAs: %s, \nshield Euler angles: %s, \nshield translation: %s, \nshield attCoef: %s, \nenergy spectrum file: %s, \ntimestamp: %s" % (data.source_position, data.source_direction, data.source_aperture, data.mAs, data.shield_Euler_angles, data.shield_translation, data.shield_attCoef, data.energy_spectrum_file, data.timestamp))
	print()

def callback_tracking(data):
	rospy.loginfo("tracking\norigin: %s, \nhead: %s, \nleft shoulder: %s, \nright shoulder: %s, \nhip torso: %s, \ntimestamp: %s" % (data.origin, data.head, data.leftShoulder, data.rightShoulder, data.hipTorso, data.timestamp))
	print()

def mult_listener():
	rospy.init_node('mult_listener', anonymous=True)
	rospy.Subscriber("mult_type_chatter", source_data, callback_virtual_console)
	rospy.Subscriber("mult_type", kinect_data, callback_tracking)
	rospy.spin()

if __name__ == '__main__':
	mult_listener()
