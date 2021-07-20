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

