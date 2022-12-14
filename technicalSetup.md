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
rosdep install mmi_package
catkin build
rosrun mmi_package mynode.py
roslaunch src/mmi_package/launch/mynode.launch
``` 

# Cpp Modbus

Cmake:
```bash
add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})
#add_dependencies(talker beginner_tutorials_generate_messages_cpp)

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
#add_dependencies(listener beginner_tutorials_generate_messages_cpp)
``` 

```bash
# In your catkin workspace
cd ~/catkin_ws
catkin_make  

rosrun modbus-cpp listener
```
# Config
Configure in panel:



SolarEdge:
192.168.1.30
1502

I AC Power: 40083, int16, ac power value
I AC Power SF: 40084, int16, ac power value scaling factor


Registers:
85 -> freq
86 -> freq_sf

206 -> total power
210 -> total power SF

Victron:
192.168.1.125
port: default 502

id: battery 225
    gridmeter 32 (beim eingang)
    pvInverter 33
NOTE: Use unit-id 100 for the com.victronenergy.system data, for more information see FAQ.


ESS BatteryLife SoC limit (read only) -> 2903 This value is maintained by BatteryLife. The Active SOC limit is the lower of this value, and register 2901. Also see https://www.victronenergy.com/media/pg/Energy_Storage_System/en/controlling-depth-of-discharge.html#UUID-af4a7478-4b75-68ac-cf3c-16c381335d1e

808,809,810 -> pvInput phase 1,2,3





