# Overview of the Virtual Dosimeter for Interventional Radiology project

Interventional fluoroscopy is an imaging modality that provides a real-time x-ray image of a patient’s anatomy. The radiation dose delivered to operators and patients during interventional fluoroscopy procedures can be substantial and must be monitored to minimize adverse health effects. Investigators at CDRH/OSEL/DIDSR are developing an open-source _Virtual Dosimeter_ system to estimate computationally the radiation delivered to operators and patients in real-time. The initial version of the system consisted of three computational modules: (1) the MC-GPU x-ray transport module, (2) the virtual x-ray source module, and (3) the operator position tracker module (based on a Kinect depth-camera). The modules were coordinated by Lightweight Communications and Marshalling (LCM) messages. However, the LCM library is limited and no longer supported. We have updated the Virtual Dosimeter system with a new software application based on the popular open-source Robot Operating System library to overcome the limitations of LCM. Additionally, we developed a new module to display the computed radiation doses to the operator: (4) the dose display module. The coordinating software is a command-line application, written in Python, that is able to run in a small NVIDIA Jetson TX2 embedded computer for easy deployment in a catheterization lab. The modular design of the software allows testing different versions of the four individual components (using different operator tracking technologies, for example) without affecting the execution of the complete system. The performance of the Virtual Dosimeter was evaluated by simulating a typical interventional fluoroscopy procedure.

# Conductor-Software
This software uses the Robot Operating System to coordinate four independent modules that compose the Virtual Dosimeter.

# Code features
This package contains five modules:

1. Conductor
This module coordinates all the modules by exchanging messages between them.

2. Virtual x-ray source
This module recevies x-ray source parameters through user input.

3. Operatoring position tracker
This module mocks the real operator position tracking with set sample reference points.

4. Monte Carlo
This module mocks the real Monte Carlo code with set dose data values.

5. Dose display
This module plots a bar graph and a table of the dose data and updates with new data for every exposure.

# Code output
This code generates 2D plots of estimated dose data that is "computed" by the other modules. Currently, the dose data are set values as the real Monte Carlo and tracking modules are not yet implemented.

# Dependencies
Install ROS through this site: https://www.ros.org/

Create a catkin workspace: http://wiki.ros.org/catkin/Tutorials/create_a_workspace

Before executing any code, source your environment:
```
$ cd ~/catkin_ws
$ source ./devel/setup.bash
```
When running code individually, run roscore before executing any code:
```
$ roscore
```

# Usage
To launch the software, use roslaunch:
```
$ roslaunch virtual_dosimeter virtual_dosimeter.launch
```

To run python code individually, use rosrun virtual_dosimeter file_name.py
```
$ rosrun virtual_dosimeter trigger_source_GUI.py
$ rosrun virtual_dosimeter trigger_operator.py
$ rosrun virtual_dosimeter conductor.py
$ rosrun virtual_dosimeter monte_carlo.py
$ rosrun virtual_dosimeter dose_display.py
```
To run c++ code individually, use rosrun hello_world file_name
```
$ rosrun virtual_dosimeter publisher
$ rosrun virtual_dosimeter subscriber
```
