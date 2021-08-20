#!/usr/bin/env python

#======================================================
#
#  ** MONTE CARLO MODULE (2021-8-20) **
#
#   This program publishes an array of set values when the source and
#   tracking data are received to mock the Monte Carlo code.
#
#======================================================

import rospy

# Import my message data types:
from virtual_dosimeter.msg import *
from std_msgs.msg import Float32MultiArray

# Declare node as publisher and set its parameters:
pub = rospy.Publisher('MCGPU_CHANNEL', Float32MultiArray, queue_size=10)


# Function to receive source data from Conductor module:
def callback_source(data):
	rospy.loginfo("Source data received from \"SOURCE_MCGPU_CONDUCTOR\" at: %s\n", data.timestamp)
	
	
# Function to receive kinect data from Conductor module:
def callback_kinect(data):
	rospy.loginfo("Kinect data received from \"KINECT_MCGPU_CONDUCTOR\" at: %s\n", data.timestamp)
	
	# To mock the Monte Carlo code, call method to publish dose data after the tracking data has been received
	dose_data()


# Function to publish message that contains an array with set "dose values":
def dose_data():
	# Initialize mock dose data message
	dose = Float32MultiArray()
	dose.data = [1, 1, 1, 1]
	rospy.loginfo(dose.data)
	
	# Publish dose data message
	pub.publish(dose)
	print ("Dose data published")
	
# Function to initialize node and declare the node as subscribers:
def monte_carlo():
	rospy.init_node('monte_carlo', anonymous=True)
	rospy.Subscriber("SOURCE_MCGPU_CONDUCTOR", source_data_t, callback_source)
	rospy.Subscriber("KINECT_MCGPU_CONDUCTOR", kinect_data_t, callback_kinect)

	# Keep node from exiting until it has been shut down
	while not rospy.core.is_shutdown():
		rospy.rostime.wallsleep(0.5)

if __name__ == '__main__':
    try:
        monte_carlo()
    except rospy.ROSInterruptException:
        pass
