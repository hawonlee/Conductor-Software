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
To run the code, use rosrun hello_world file_name.py
```
$ rosrun hello_world hello.py
$ rosrun hello_world listener.py
$ rosrun hello_world second_listener.py
$ rosrun hello_world last_listener.py
```
