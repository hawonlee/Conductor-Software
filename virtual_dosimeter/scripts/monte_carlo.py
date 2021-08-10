#!/usr/bin/env python

import rospy
from virtual_dosimeter.msg import *
from std_msgs.msg import Float32MultiArray

def monte_carlo():
    pub = rospy.Publisher('dose_display_channel', Float32MultiArray, queue_size=10)
    rospy.init_node('monte_carlo', anonymous=True)
    dose = Float32MultiArray()
    dose.data = [1, 1, 1, 1]
    rospy.loginfo(dose.data)
    pub.publish(dose)

if __name__ == '__main__':
    try:
        monte_carlo()
    except rospy.ROSInterruptException:
        pass
