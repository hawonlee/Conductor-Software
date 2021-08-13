#!/usr/bin/env python

import rospy
import time
from virtual_dosimeter.msg import *
from std_msgs.msg import Float32MultiArray

source_pub = rospy.Publisher('SOURCE_CONDUCTOR', source_data_t, queue_size=10)
source_mcgpu_pub = rospy.Publisher('SOURCE_MCGPU_CONDUCTOR', source_data_t, queue_size=10)
kinect_mcgpu_pub = rospy.Publisher('KINECT_MCGPU_CONDUCTOR', kinect_data_t, queue_size=10)
dose_pub = rospy.Publisher('DOSE_CONDUCTOR', Float32MultiArray, queue_size=10)

class KinectData:
	def __init__(self):
		self.kinect = []
		self.kinect_received = False

kinect_data = KinectData()

def callback_source(data):
	rospy.loginfo("Source data received from \"SOURCE_CHANNEL\" at: %s\n", data.timestamp)	
#	rospy.loginfo("source data\nsource position: %s \nsource direction: %s, \nsource aperture: %s \nmAs: %s \nshield Euler angles: %s \nshield translation: %s \nshield attCoef: %s \nenergy spectrum file: %s \ntimestamp: %s" % (data.source_position, data.source_direction, data.source_aperture, data.mAs, data.shield_Euler_angles, data.shield_translation, data.shield_attCoef, data.energy_spectrum_file, data.timestamp))
	
#	while pub.get_num_connections() == 0:
#		rospy.sleep(0.5)
	source_pub.publish(data)
	print ("Source data published to \"SOURCE_CONDUCTOR\"\n")
	send_source_kinect_data(data)
	

def send_source_kinect_data(source_data):
	while not kinect_data.kinect_received:
		rospy.sleep(0.5)
#	source_mcgpu_pub.publish(source_data)
	kinect_mcgpu_pub.publish(kinect_data.kinect)
	kinect_data.kinect_received = False
	print ("Source and kinect data published\n")
	
def callback_tracking(data):
	rospy.loginfo("Kinect data received from \"KINECT_CHANNEL\" at: %s\n", data.timestamp)
	kinect_data.kinect = data
	kinect_data.kinect_received = True
#	rospy.loginfo("tracking\norigin: %s \nhead: %s \nleft shoulder: %s \nright shoulder: %s \nhip torso: %s \ntimestamp: %s" % (data.origin, data.head, data.leftShoulder, data.rightShoulder, data.hipTorso, data.timestamp))
	print()

def callback_MCGPU(data):
	rospy.loginfo("MCGPU data recieved from \"MCGPU_CHANNEL\": %s\n", (data.data))
	dose_pub.publish(data)
	print ("MCGPU data published to \"DOSE_CONDUCTOR\"\n")

def conductor():
	rospy.init_node('conductor', anonymous=True)
	rospy.Subscriber("SOURCE_CHANNEL", source_data_t, callback_source)
	rospy.Subscriber("KINECT_CHANNEL", kinect_data_t, callback_tracking)
	rospy.Subscriber("MCGPU_CHANNEL", Float32MultiArray, callback_MCGPU)

	while not rospy.core.is_shutdown():
		rospy.rostime.wallsleep(0.5)

if __name__ == '__main__':
	conductor()
