#!/usr/bin/env python

import rospy
import matplotlib.pyplot as plt
import numpy as np
from std_msgs.msg import Float32MultiArray
from tkinter import *

class DoseData:
	def __init__(self):
		self.doses = np.empty((0, 4), float)
		self.message = False

dose_data = DoseData()

def callback_dose(data):
	rospy.loginfo("%s", (data.data))
	new_doses = np.array([data.data])
	dose_data.doses = np.append(dose_data.doses, new_doses, axis=0)
	dose_data.message = True

def make_display():
	x = ['Skin', 'Testes', 'Bone', 'Red bone marrow']
	y = ['Total dose']
	total_dose = np.array([0, 0, 0, 0])
	fig, ax = plt.subplots(figsize=(9, 7))
	width = 0.5
	exposure = 0
	
	while not rospy.core.is_shutdown():
		data = np.array(dose_data.doses)

		# set up the axes
		if len(data)!=0:
			ax.bar(x, data[-1], width, bottom=total_dose, label='exposure'+str(exposure))
			total_dose = np.add(total_dose, data[-1])
			table_data = np.append(data, [total_dose], axis=0)
			the_table = plt.table(cellText=table_data, rowLabels=y, colLabels=x, loc='bottom', bbox=[0, -0.8, 1, 0.6])			
			the_table.auto_set_font_size(False)
			the_table.set_fontsize(10)
		
		# set up the labels and legend
		ax.set_ylabel('Equivalent dose (μSv/(Gy cm\u00b2))', fontsize=14)
		ax.set_title('Operator Organ Dose')
		ax.legend(fontsize=14)
		plt.tick_params(labelsize=14)
		plt.subplots_adjust(left=0.2, bottom=0.5)
		
		# show the display
		plt.ion()
		plt.show()
		plt.pause(0.001)
		
		# wait until organ doses were recieved from Monte Carlo module
		while not rospy.core.is_shutdown():
			rospy.sleep(0.5)
			if dose_data.message:
				dose_data.message = False
				break
		
		exposure = exposure + 1
		y.insert(len(y)-1,"Exposure " + str(exposure))
		

def display():
	rospy.init_node('display', anonymous=True)
	rospy.Subscriber("DOSE_CONDUCTOR", Float32MultiArray, callback_dose)
	make_display()
#	while not rospy.core.is_shutdown():
#		rospy.rostime.wallsleep(0.5) 
	
if __name__ == '__main__':
	display()
