#!/usr/bin/env python

#======================================================
#
#  ** CONDUCTOR MODULE (2021-8-20) **
#
#   This program exchanges ROS messages between the modules.
#   All messages that are published by the modules are received here
#   and published to the next module.
#
#======================================================

import rospy
import time

# Import my message types:
from virtual_dosimeter.msg import *
from std_msgs.msg import Float32MultiArray

# Declare node as publisher and set its parameters:
source_pub = rospy.Publisher('SOURCE_CONDUCTOR', source_data_t, queue_size=10)
source_mcgpu_pub = rospy.Publisher('SOURCE_MCGPU_CONDUCTOR', source_data_t, queue_size=10)
kinect_mcgpu_pub = rospy.Publisher('KINECT_MCGPU_CONDUCTOR', kinect_data_t, queue_size=10)
dose_pub = rospy.Publisher('DOSE_CONDUCTOR', Float32MultiArray, queue_size=10)

# Function to create KinectData objects which contain the kinect data
# and a variable that signals when the kinect message has been received
class KinectData:
	def __init__(self):
		self.kinect = []
		self.kinect_received = False

# Create KinectData() object
kinect_data = KinectData()

# Function to receive source data from Source module:
def callback_source(data):
	rospy.loginfo("Source data received from \"SOURCE_CHANNEL\" at: %s\n", data.timestamp)	
	
	# Publish the source data to "SOURCE_CONDUCTOR" - to be received by the 
	# trigger_operator module:
	source_pub.publish(data)
	print ("Source data published to \"SOURCE_CONDUCTOR\"\n")
	
	# Call function to publish source and kinect data with the source data
	send_source_kinect_data(data)
	
# Function to publish source and kinect data once both have been received:
def send_source_kinect_data(source_data):
	
	# Wait until kinect_data has also been received:
	while not kinect_data.kinect_received:
		rospy.sleep(0.5)
	
	# Publish source and kinect data to separate topics:
	source_mcgpu_pub.publish(source_data)
	kinect_mcgpu_pub.publish(kinect_data.kinect)
	
	kinect_data.kinect_received = False
	print ("Source and kinect data published\n")
	
# Function to receive kinect data from Tracking module
def callback_tracking(data):
	rospy.loginfo("Kinect data received from \"KINECT_CHANNEL\" at: %s\n", data.timestamp)
	
	# Set data in the kinect_data object to new data
	kinect_data.kinect = data
	kinect_data.kinect_received = True
	print()

# Function to receive dose data from Monte Carlo module
def callback_MCGPU(data):
	rospy.loginfo("MCGPU data recieved from \"MCGPU_CHANNEL\": %s\n", (data.data))
	dose_pub.publish(data)
	print ("MCGPU data published to \"DOSE_CONDUCTOR\"\n")

# Function to initialize node and declare the node as subscribers:
def conductor():
	print ("conductor running")
	rospy.init_node('conductor', anonymous=True)
	rospy.Subscriber("SOURCE_CHANNEL", source_data_t, callback_source)
	rospy.Subscriber("KINECT_CHANNEL", kinect_data_t, callback_tracking)
	rospy.Subscriber("MCGPU_CHANNEL", Float32MultiArray, callback_MCGPU)

	while not rospy.core.is_shutdown():
		rospy.rostime.wallsleep(0.5)

if __name__ == '__main__':
	conductor()
