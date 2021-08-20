#!/usr/bin/env python

#======================================================
#
#  ** KINECT INTERFACE (2021-8-20) **
#
#   Simplified version of the kinect interface.
#   The program runs an infinte loop mocking the kinect tracking loop and
#   sends a set of positions at the moment a message from the source is received:
#   asynchronous message reception!
#
#======================================================

import rospy
import time

# Import my message data types:
from virtual_dosimeter.msg import *

exposure_counter = 1 # This variable is initializaed to 1 and increases at each call to count the number of messages
status = False # This variable is used to signal when the source data has been received

#
# Function to initialize the kinect data to be sent:
#
def get_kinect_data(my_kinect_data):
	global exposure_counter
  
	# Set sample data in the PATIENT reference system:
	my_kinect_data.origin[0]       =   0.0; my_kinect_data.origin[1]       =  0.0; my_kinect_data.origin[2]       =  0.0; 
	my_kinect_data.head[0]         = -30.0; my_kinect_data.head[1]         = 80.0; my_kinect_data.head[2]         = 30.0;
	my_kinect_data.leftShoulder[0] = -30.0; my_kinect_data.leftShoulder[1] = 60.0; my_kinect_data.leftShoulder[2] = 15.0;
	my_kinect_data.rightShoulder[0]= -30.0; my_kinect_data.rightShoulder[1]= 60.0; my_kinect_data.rightShoulder[2]= 45.0; 
	my_kinect_data.hipTorso[0]     = -30.0; my_kinect_data.hipTorso[1]     = 15.0; my_kinect_data.hipTorso[2]     = 30.0;


	# -- For even number of messages, move 1 meter towards the patient head:
	if (0==(exposure_counter%2)):  # TRUE for even number of messages sent.
		my_kinect_data.head[2]         = 130.0
		my_kinect_data.leftShoulder[2] = 115.0
		my_kinect_data.rightShoulder[2]= 145.0
		my_kinect_data.hipTorso[2]     = 130.0
 
	exposure_counter += 1
	
	# Set the timestamp in ASCII
	my_kinect_data.timestamp = time.ctime()
	
	return my_kinect_data

#
# Function to receive the source data from the Conductor module:
#
def callback_source(data):
	print("\n --- Received message on channel \"source\". Timestamp: %s", data.timestamp)
	print("       X-ray projection source parameters:\n")
	print("          source_position   = (%f, %f, %f)\n" % (data.source_position[0], data.source_position[1], data.source_position[2]))
	print("          source_direction  = (%f, %f, %f)\n" % (data.source_direction[0], data.source_direction[1], data.source_direction[2]))
	print("          source_aperture   = (%f, %f)\n" % (data.source_aperture[0], data.source_aperture[1]))
	print("          conversion factor =  %f\n" % (data.mAs))
	print("          energy_spectrum_file = \"%s\"\n\n", data.energy_spectrum_file)
	print("          shield_Euler_angles = (%f, %f, %f)\n" % (data.shield_Euler_angles[0], data.shield_Euler_angles[1], data.shield_Euler_angles[2]))    #!!August2014!!
	print("          shield_translation  = (%f, %f, %f)\n" % (data.shield_translation[0], data.shield_translation[1], data.shield_translation[2]))
	print("          shield_size         = (%f, %f, %f)\n" % (data.shield_size[0], data.shield_size[1], data.shield_size[2]))
	print("          shield_AttenuationCoeficient = %f\n\n", data.shield_attCoef)
	
	# Signal that the source data was received
	global status
	status = True

#
# Function to send the kinect data once the source data has been received
#
def trigger_operator():
	global status
	
	# Init ROS node and declare node as publisher
	rospy.init_node('trigger_operator', anonymous=True)
	pub = rospy.Publisher('KINECT_CHANNEL', kinect_data_t, queue_size=10)

	print("\n\n\n   ** KINECT INTERFACE **\n\n")
	print(    "      Send a system-wide message using ROS message with the geometric parameters of the operator acquired\n")
	print(    "      with the kinect depth camera. The message emission is triggered by a message received from the x-ray source.\n")
	print(    "      ROS topic:  \"KINECT_CHANNEL\"; message description file: \"mcgpu_kinect_data_t.msg\"\n")
	print(    "      ROS topic channel: \"SOURCE_CHANNEL\"; message description file: \"mcgpu_source_data_t.msg\"\n\n")       
  
	# -- Listen to the messages from the source only:
	rospy.Subscriber("SOURCE_CONDUCTOR", source_data_t, callback_source)

	while not rospy.core.is_shutdown():
		if(not status):
			# No messages
			print("waiting for message from the source...\n")
			# Wait until a message is received
			while not rospy.core.is_shutdown():
				rospy.rostime.wallsleep(0.5)
				if status:
					break

		elif (status):
			# -- Send message to KINECT_CHANNEL:
			my_kinect_data = kinect_data_t()
			my_kinect_data = get_kinect_data(my_kinect_data)     # Init message
			print("\n  +++ Sending a message to the \"KINECT_CHANNEL\" at: %s", my_kinect_data.timestamp)
			print("         [Geometric landmarks in the kinect reference system]\n")                    #!!DeBuG!! Verbose
			print("             origin        = (%f, %f, %f)\n" % (my_kinect_data.origin[0], my_kinect_data.origin[1], my_kinect_data.origin[2]))
			print("             head          = (%f, %f, %f)\n" % (my_kinect_data.head[0], my_kinect_data.head[1], my_kinect_data.head[2]))
			print("             leftShoulder  = (%f, %f, %f)\n" % (my_kinect_data.leftShoulder[0], my_kinect_data.leftShoulder[1], my_kinect_data.leftShoulder[2]))
			print("             rightShoulder = (%f, %f, %f)\n" % (my_kinect_data.rightShoulder[0], my_kinect_data.rightShoulder[1], my_kinect_data.rightShoulder[2]))
			print("             hipTorso      = (%f, %f, %f)\n\n" % (my_kinect_data.hipTorso[0], my_kinect_data.hipTorso[1], my_kinect_data.hipTorso[2]))
			
			# Publish kinect data
			pub.publish(my_kinect_data)
			
			status = False

if __name__ == '__main__':
	try:
		trigger_operator()
	except rospy.ROSInterruptException:
		pass
