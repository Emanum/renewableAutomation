# Renewable Automation


## Project Idea

The goal of this project is to build a base system that allows private homes to optimally make use of a renewable energy source such as a photovoltaic system.

Often these systems already come with an interface to read the current power production. This could
be used as a sensor information in an ROS automated smart home to turn on/off different energy
consumers such as a heat pump for warm water or charging an electronic vehicle.
Another way to make use of an automated reading of this sensor information to calculate statistics
such as daily/weekly energy production/consumption.

The project will be built for our own house with our specific gear in mind. However, the idea can be
applied to other systems as well.

## Demo


https://user-images.githubusercontent.com/9011769/211167296-1c7a2726-df9c-4389-8beb-735d02985d31.mp4

![demo](https://user-images.githubusercontent.com/9011769/211167321-298f0dd3-57ca-4d75-93ee-166475890e64.png)


## Project Overview

The project is run with ros melodic. We have the following nodes:

### modbus

This is a publisher node that connects to a central modbus server, reads the data and publishes them onto various ros topics.

The following topics are published:
* total_grid_power: wattage positiv:energy is consumed from grid, negativ:energy is delivered to grid
* total_pv_power: wattage always >= 0
* battery_state_of_charge: percentage 0-100
* battery_power: wattage positiv:battery is charging, negativ: battery is discharging 
* battery_life_soc_limit: percentage the battery will not get discharged beyound that level https://www.victronenergy.com/media/pg/Energy_Storage_System/en/controlling-depth-of-discharge.html#UUID-af4a7478-4b75-68ac-cf3c-16c381335d1e 

For total_grid_power and total_pv_power I read all 3 Phases seperatly and combine then to one number each.

**This node is preconfigured to listen to our specific modbus server(ip,port,slaveID) and read the correct addresses.** The addresses can be found in CCGX-Modbus-TCP-register-list-2.90.xlsl

Code: catkin_ws4/src/modbus-cpp/src/modbus.cpp
catkin_ws4/src/modbus-cpp/src/modbus.h - this is an external lib from https://github.com/fz-lyu/modbuspp

### computeNode

This is a listener and publisher nodes. I listen to the topics from the modbus node apply some logic that decides when a relay should be open or closed. This value will get written on the topic relay1_state (1 => open, 0 => closed).

Currently, we have a simple threshold for the total_grid_power which the relay opens or closes. In the future we want to extend that logic to also use the battery state and total_pv_power.

This threshold can be configured in the launch file. currentValue < threshold -> open otherwise close.

Code: catkin_ws4/src/modbus-cpp/src/computeNode.cpp

### Rosserial Arduino Relay Toggle

This is a listener node which runs on an Arduino that is physically wired to a relay that can switch 220V 10A. This relay is connected to a standard power plug, so we can switch any device (that does not exceed the limits) with it.

We listen to the relay1_state topic and either open or close the relay. 

It is connected to the roscore with `rosserial_python serial_node.py` 

Code: catkin_ws4/src/modbus-cpp/src/rosserial_arduino_relay_toggle/rosserial_arduino_relay_toggle.ino

## How to run it

Requirements: 
Ubuntu 18.04
Ros Melodic http://wiki.ros.org/melodic/Installation/Ubuntu

**It only works when the correct modbus server is available. The preconfigured server can be found in catkin_ws4/src/modbus-cpp/src/modbus.cpp**

I put a demo video and screenshots in the folder demo as well as pictures of the used hardware.

Otherwise it is all connected to a central launch file:  catkin_ws4/src/modbus-cpp/launch/modbus.launch

``` bash
cd catkin_ws4
catkin_make
source devel/bash.setup
roslaunch modbus-cpp modbus.launch
```



## Technical Modbus Documentation

### Modbus Connection Victron Cerbo GX

https://www.victronenergy.de/panel-systems-remote-monitoring/cerbo-gx
https://www.victronenergy.de/panel-systems-remote-monitoring/cerbo-gx#technical-information

Victron:
192.168.1.125
port: default 502
NOTE: Use unit-id 100 for the com.victronenergy.system data, for more information see FAQ.

Addresses that we use

* batteryState 843
* battery_power 842
* batteryLife_soc_limit 2903
* pvProduction phase1,2,3 808,809,810
* gridPower phase1,2,3 820,821,823
