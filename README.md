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

Install Matplotlib: https://matplotlib.org/stable/users/installing.html

Install NumPy: https://numpy.org/install/

Before executing any code, source your environment:
```
$ cd ~/catkin_ws
$ source ./devel/setup.bash
```

Place the virtual_dosimeter package in the src folder of the catkin workspace use:
```
$ catkin_make
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

# Disclaimer
This software and documentation (the "Software") were developed at the Food and Drug Administration (FDA) by employees of the Federal Government in the course of their official duties. Pursuant to Title 17, Section 105 of the United States Code, this work is not subject to copyright protection and is in the public domain. Permission is hereby granted, free of charge, to any person obtaining a copy of the Software, to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, or sell copies of the Software or derivatives, and to permit persons to whom the Software is furnished to do so. FDA assumes no responsibility whatsoever for use by other parties of the Software, its source code, documentation or compiled executables, and makes no guarantees, expressed or implied, about its quality, reliability, or any other characteristic. Further, use of this code in no way implies endorsement by the FDA or confers any advantage in regulatory decisions. Although this software can be redistributed and/or modified freely, we ask that any derivative works bear some notice that they are derived from it, and any modified versions bear some notice that they have been modified.
