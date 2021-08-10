#!/usr/bin/env python

#
#  KINECT INTERFACE:
#  Simplified version of the kinect interface.
#  The program runs an infinte loop mocking the kinect tracking loop and
#  sends a set of positions at the moment a message from the source is received:
#  asynchronous message reception!
#
#
#  Compilation:
#    $lcm-gen -c mcgpu_kinect_data_t.lcm mcgpu_source_data_t.lcm -d 
#    $gcc -o sourceListenerAsync_kinectTrigger.x sourceListenerAsync_kinectTrigger.c mcgpu_kinect_data_t.c mcgpu_source_data_t.c -llcm
#

#include <stdio.h>
#include <inttypes.h>
#include <sys/select.h>t

import rospy
import time
from virtual_dosimeter.msg import *
exposure_counter = 1

#
# Function to initialize the kinect data to be sent:
#
def get_kinect_data(my_kinect_data)
{      
  my_kinect_data = kinect_data_t()
  global exposure_counter
  
  # Set sample data in the PATIENT reference system:
  my_kinect_data.origin[0]       =   0.0; my_kinect_data.origin[1]       =  0.0; my_kinect_data.origin[2]       =  0.0; 
  my_kinect_data.head[0]         = -30.0; my_kinect_data.head[1]         = 80.0; my_kinect_data.head[2]         = 30.0;
  my_kinect_data.leftShoulder[0] = -30.0; my_kinect_data.leftShoulder[1] = 60.0; my_kinect_data.leftShoulder[2] = 15.0;
  my_kinect_data.rightShoulder[0]= -30.0; my_kinect_data.rightShoulder[1]= 60.0; my_kinect_data.rightShoulder[2]= 45.0; 
  my_kinect_data.hipTorso[0]     = -30.0; my_kinect_data.hipTorso[1]     = 15.0; my_kinect_data.hipTorso[2]     = 30.0;
  

  # -- For even number of messages, move 1 meter towards the patient head:
# static exposure_counter=1    # This value of this static variable is remembered between function calls (initializaed to 1 and increases at each call).
  if (0==(exposure_counter%2)):  # TRUE for even number of messages sent.
    my_kinect_data.head[2]         = 130.0
    my_kinect_data.leftShoulder[2] = 115.0
    my_kinect_data.rightShoulder[2]= 145.0
    my_kinect_data.hipTorso[2]     = 130.0
 
  exposure_counter += 1

  # Calculate timestamp in ASCII:
  time_t rawtime
  struct tm *timeinfo
  time(&rawtime)
  timeinfo = localtime(&rawtime)
  static char my_static_timestamp[80]     # Store timestamp in a static variable to make sure the pointer will be valid after function returns.
  strcpy(my_static_timestamp, asctime(timeinfo))
  my_kinect_data.timestamp = my_static_timestamp 


///////////////////////////////////////////////////////////////////////////////

def my_handler_source(data):
    print("\n --- Received message on channel \"%s\". Timestamp: %s", % (channel, data.timestamp))
    print("       X-ray projection source parameters:\n")
    print("          source_position   = (%f, %f, %f)\n" % (data.source_position[0], data.source_position[1], data.source_position[2]))
    print("          source_direction  = (%f, %f, %f)\n" % (data.source_direction[0], data.source_direction[1], data.source_direction[2]))
    print("          source_aperture   = (%f, %f)\n" % (data.source_aperture[0], data.source_aperture[1]))
    print("          conversion factor =  %f\n" % (data.mAs))
    print("          energy_spectrum_file = \"%s\"\n\n" % (data.energy_spectrum_file))
    
    print("          shield_Euler_angles = (%f, %f, %f)\n" % (data.shield_Euler_angles[0], data.shield_Euler_angles[1], data.shield_Euler_angles[2]))    #!!August2014!!
    print("          shield_translation  = (%f, %f, %f)\n" % (data.shield_translation[0], data.shield_translation[1], data.shield_translation[2]))
    print("          shield_size         = (%f, %f, %f)\n" % (data.shield_size[0], data.shield_size[1], data.shield_size[2]))
    print("          shield_AttenuationCoeficient = %f\n\n" % (data.shield_attCoef))



def trigger_operator():
	print("\n\n\n   ** KINECT INTERFACE **\n\n")
	print(    "      Send a system-wide message using the LCM middleware with the geometric parameters of the operator acquired\n")
	print(    "      with the kinect depth camera. The message emission is triggered by a message received from the x-ray source.\n")
	print(    "      LCM EMISSION channel:  \"KINECT_CHANNEL\"; message description file: \"mcgpu_kinect_data_t.lcm\"\n")
	print(    "      LCM RECEPTION channel: \"SOURCE_CHANNEL\"; message description file: \"mcgpu_source_data_t.lcm\"\n\n")
  
	pub = rospy.Publisher('KINCT_DATA', kinect_data_t, queue_size=10)
	rospy.init_node('trigger_operator', anonymous=True)
  
  lcm_t *lcm_kinect = lcm_create(NULL),
        *lcm_source = lcm_create(NULL)            
  
	if(!lcm_kinect || !lcm_source):
		return 1
  
  # -- Listen to the messages from the source only:
  rospy.Subscriber("SOURCE_CHANNEL", source_data_t, my_handler_source)
  
  while(1):
      # setup the LCM file descriptor for waiting.
      int lcm_fd = lcm_get_fileno(lcm_source)        # Check SOURCE_CHANNEL
      fd_set fds
      FD_ZERO(&fds)
      FD_SET(lcm_fd, &fds)

      # wait a limited amount of time for an incoming message // !!DeBuG!! I can use time 0: not blocking (?)
      struct timeval timeout = { 
          10,  # seconds         # setting long interval because no kinect connected yet... !!DeBuG!!
          0   # microseconds
      
      status = select(lcm_fd + 1, &fds, 0, 0, &timeout)   # !!DeBuG!! Wait till timeout for changes in file: See info: http://en.wikipedia.org/wiki/Select_%28Unix%29

      if(0 == status):
          # no messages
          print("waiting for message from the source...\n")   #!!DeBuG!! In this part of the loop the kinect will do teh regular tracking things...

      else if(FD_ISSET(lcm_fd, &fds)):
          # LCM has events ready to be processed.
          
          # -- Receive message from SOURCE_CHANNEL:
          lcm_handle(lcm_source)
          
          # -- Send message to KINECT_CHANNEL:
          my_kinect_data = kinect_data_t()
          get_kinect_data(my_kinect_data)     # Init message      
          printf("\n  +++ Sending a message to the \"KINECT_CHANNEL\" at: %s", my_kinect_data.timestamp)
              
              print("         [Geometric landmarks in the kinect reference system]\n")                    //!!DeBuG!! Verbose
              print("             origin        = (%f, %f, %f)\n" % (my_kinect_data.origin[0], my_kinect_data.origin[1], my_kinect_data.origin[2]))
              print("             head          = (%f, %f, %f)\n" % (my_kinect_data.head[0], my_kinect_data.head[1], my_kinect_data.head[2]))
              print("             leftShoulder  = (%f, %f, %f)\n" % (my_kinect_data.leftShoulder[0], my_kinect_data.leftShoulder[1], my_kinect_data.leftShoulder[2]))
              print("             rightShoulder = (%f, %f, %f)\n" % (my_kinect_data.rightShoulder[0], my_kinect_data.rightShoulder[1], my_kinect_data.rightShoulder[2]))
              print("             hipTorso      = (%f, %f, %f)\n\n" % (my_kinect_data.hipTorso[0], my_kinect_data.hipTorso[1], my_kinect_data.hipTorso[2]))
          
          pub.publish(my_kinect_data)

  mcgpu_source_data_t_unsubscribe(lcm_source, sub)
  
  return 0

if __name__ == '__main__':
	try:
		trigger_operator()
	except rospy.ROSInterruptException:
		pass
