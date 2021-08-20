#!/usr/bin/env python

#======================================================
#
#  ** DOSE DISPLAY MODULE (2021-8-20) **
#
#   This program displays the computed estimated radiation dose values
#   in a 2D bar graph and a table. The plots update with each message
#   received.
#
#======================================================

import rospy

# Import Matplot library and NumPy to generate 2D plots:
import matplotlib.pyplot as plt
import numpy as np

# Import my message data types
from std_msgs.msg import Float32MultiArray
from tkinter import *

# Function to create DoseData objects which contain a 2D array of dose values
# from each exposure and a variable that signals when the dose value message 
# has been received
class DoseData:
	def __init__(self):
		self.doses = np.empty((0, 4), float)
		self.message = False

# Create DoseData object
dose_data = DoseData()

# Function to receive dose data messages from the Conductor module:
def callback_dose(data):
	rospy.loginfo("%s", (data.data))
	new_doses = np.array([data.data])
	
	# Add new dose values to the 2D array of the DoseData object
	dose_data.doses = np.append(dose_data.doses, new_doses, axis=0)
	
	# Signal that the dose data has been received
	dose_data.message = True

# Function to plot the bar chart and table of dose values
def make_display():

	# Set the labels for the x and y axes
	x = ['Skin', 'Testes', 'Bone', 'Red bone marrow']
	y = ['Total dose']
	
	# For each exposure, the dose values are added to this variable:
	total_dose = np.array([0, 0, 0, 0])

	fig, ax = plt.subplots(figsize=(9, 7))
	width = 0.5
	
	# This variable counts the number of exposures:
	exposure = 0
	
	# This infinite loop maintains the figure window
	while not rospy.core.is_shutdown():
		data = np.array(dose_data.doses)

		# Once a dose data message is received, set up the axes and table rows:
		if len(data)!=0:
			
			# Add the most recent dose data to the bar chart:
			ax.bar(x, data[-1], width, bottom=total_dose, label='Exposure '+str(exposure))
			total_dose = np.add(total_dose, data[-1]) # The dose values are added to the total dose
			
			table_data = np.append(data, [total_dose], axis=0)
			
			# Plot of the dose data and the total dose in the table:
			the_table = plt.table(cellText=table_data, rowLabels=y, colLabels=x, loc='bottom', bbox=[0, -0.8, 1, 0.6])			
			the_table.auto_set_font_size(False)
			the_table.set_fontsize(10)
		
		# Set up the labels and legend:
		ax.set_ylabel('Equivalent dose (μGy)', fontsize=14)
		ax.set_title('Operator Organ Dose')
		ax.legend(fontsize=14)
		plt.tick_params(labelsize=14)
		plt.subplots_adjust(left=0.2, bottom=0.5)
		fig.canvas.set_window_title('Virtual Dosimeter Display')
		
		# **Alarm: when the total dose exceeds a predetermined threshold, turn the display
		# background red to warn operator or potentially dangerous radiation exposure:
		alarm_threshold = 2 # Determine the threhold
		if total_dose[0]>alarm_threshold:
			fig.patch.set_facecolor('red')
		
		# show the display:
		plt.ion() #Turn off interactive mode
		plt.show()
		plt.pause(0.001)
		
		# Wait until organ doses are recieved from Conductor module
		while not rospy.core.is_shutdown():
			rospy.rostime.wallsleep(0.5)
			if dose_data.message:
				dose_data.message = False
				break
		
		exposure = exposure + 1 # Increase exposure count by 1
		y.insert(len(y)-1,"Exposure " + str(exposure)) # Add a label for the new dose data
		
# Function to initialize node and declare the node as a subscriber:
def display():
	rospy.init_node('display', anonymous=True)
	rospy.Subscriber("DOSE_CONDUCTOR", Float32MultiArray, callback_dose)
	make_display() # Call make_display to start the plots
	
if __name__ == '__main__':
	display()
