# renewableAutomation


# Slides

## Extend ROS with your packages - Catkin


``` bash 

sudo apt install build-essential python-catkin-tools python-rosdep


mkdir -p catkin_ws/src
cd catkin_ws
catkin init
catkin config --extend /opt/ros/melodic
``` 

## Create a new package

``` bash
cd catkin_ws/src
#catkin_create_pkg package_name package_dependencies
catkin_create_pkg mmi_package rospy roscpp
``` 

## Build and run Python node

``` bash 
catkin build
rosrun mmi_package mynode.py
``` 

