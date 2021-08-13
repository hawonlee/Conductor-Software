#!/usr/bin/env python

import rospy
from virtual_dosimeter.msg import *
from std_msgs.msg import Float32MultiArray

pub = rospy.Publisher('MCGPU_CHANNEL', Float32MultiArray, queue_size=10)

def callback_source(data):
	rospy.loginfo("Source data received from \"SOURCE_MCGPU_CONDUCTOR\" at: %s\n", data.timestamp)
	
def callback_kinect(data):
	rospy.loginfo("Kinect data received from \"KINECT_MCGPU_CONDUCTOR\" at: %s\n", data.timestamp)
	dose_data()
	
def dose_data():
	dose = Float32MultiArray()
	dose.data = [1, 1, 1, 1]
	rospy.loginfo(dose.data)
	pub.publish(dose)
	print ("Dose data published")
	
def monte_carlo():
	rospy.init_node('monte_carlo', anonymous=True)
	rospy.Subscriber("SOURCE_MCGPU_CONDUCTOR", source_data_t, callback_source)
	rospy.Subscriber("KINECT_MCGPU_CONDUCTOR", kinect_data_t, callback_kinect)

	while not rospy.core.is_shutdown():
		rospy.rostime.wallsleep(0.5)

if __name__ == '__main__':
    try:
        monte_carlo()
    except rospy.ROSInterruptException:
        pass
