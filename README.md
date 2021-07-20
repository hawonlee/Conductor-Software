# Overview of the Virtual Dosimeter for Interventional Radiology project

Interventional fluoroscopy is an imaging modality that provides a real-time x-ray image of a patient’s anatomy. The radiation dose delivered to operators and patients during interventional fluoroscopy procedures can be substantial and must be monitored to minimize adverse health effects. Investigators at CDRH/OSEL/DIDSR are developing an open-source _Virtual Dosimeter_ system to estimate computationally the radiation delivered to operators and patients in real-time. The initial version of the system consisted of three computational modules: (1) the MC-GPU x-ray transport module, (2) the virtual x-ray source module, and (3) the operator position tracker module (based on a Kinect depth-camera). The modules were coordinated by Lightweight Communications and Marshalling (LCM) messages. However, the LCM library is limited and no longer supported. We have updated the Virtual Dosimeter system with a new software application based on the popular open-source Robot Operating System library to overcome the limitations of LCM. Additionally, we developed a new module to display the computed radiation doses to the operator: (4) the dose display module. The coordinating software is a command-line application, written in Python, that is able to run in a small NVIDIA Jetson TX2 embedded computer for easy deployment in a catheterization lab. The modular design of the software allows testing different versions of the four individual components (using different operator tracking technologies, for example) without affecting the execution of the complete system. The performance of the Virtual Dosimeter was evaluated by simulating a typical interventional fluoroscopy procedure.

# Conductor-Software
This software uses the Robot Operating System to coordinate four independent modules that compose the Virtual Dosimeter.

# Dependencies
Install ROS through this site: https://www.ros.org/

Create a catkin workspace: http://wiki.ros.org/catkin/Tutorials/create_a_workspace

Before executing any code, run roscore
```
$ roscore
```
and source your environment
```
$ cd ~/catkin_ws
$ source ./devel/setup.bash
```

# Usage
To run python code, use rosrun hello_world file_name.py
```
$ rosrun hello_world trigger_source_GUI.py
$ rosrun hello_world conductor.py
$ rosrun hello_world virtual_console.py
$ rosrun hello_world tracking.py
```
To run c++ code, use rosrun hello_world file_name
```
$ rosrun hello_world publisher
$ rosrun hello_world subscriber
```

