/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
// %Tag(FULLTEXT)%
// %Tag(ROS_HEADER)%
#include "ros/ros.h"
// %EndTag(ROS_HEADER)%
// %Tag(MSG_HEADER)%
#include "std_msgs/String.h"
// %EndTag(MSG_HEADER)%

#include <sstream>

#include "modbus.h"
#include <string>


int convertUint(int uint){
  if(uint > 32767){
      uint = (uint - 65536)*-1;
    }
    return uint;
}

std::string scaleValueInt16(int16_t value, int16_t scalingFactor){
  std::string erg{std::to_string(value)};
  if(scalingFactor > 0){
    return erg.insert(erg.length()-scalingFactor,".");
  }else{
    return erg;
  }
}


/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
// %Tag(INIT)%
  ros::init(argc, argv, "talker");
// %EndTag(INIT)%

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
// %Tag(NODEHANDLE)%
  ros::NodeHandle n;
// %EndTag(NODEHANDLE)%

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
// %Tag(PUBLISHER)%
  ros::Publisher total_grid_power_pub = n.advertise<std_msgs::String>("total_grid_power", 1000);
  ros::Publisher total_pv_power_pub = n.advertise<std_msgs::String>("total_pv_power", 1000);
  ros::Publisher battery_state_of_charge_pub = n.advertise<std_msgs::String>("battery_state_of_charge", 1000);
  ros::Publisher battery_power_pub = n.advertise<std_msgs::String>("battery_power", 1000);
  ros::Publisher battery_life_soc_limit_pub = n.advertise<std_msgs::String>("battery_life_soc_limit", 1000);


// %EndTag(PUBLISHER)%

// %Tag(LOOP_RATE)%
  ros::Rate loop_rate(10);
// %EndTag(LOOP_RATE)%

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
// %Tag(ROS_OK)%
  int count = 0;

      // create a modbus object
    modbus mb = modbus("192.168.1.30", 1502);
    // set slave id
    mb.modbus_set_slave_id(1);

    modbus mb_victron_battery = modbus("192.168.1.125", 502);
    mb_victron_battery.modbus_set_slave_id(100);
  while (ros::ok())
  {

// %EndTag(ROS_OK)%
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
// %Tag(FILL_MESSAGE)%
    std_msgs::String msg;
    std_msgs::String msg_pv;
    std_msgs::String msg_battery;
    std_msgs::String msg_battery_power;
    std_msgs::String msg_batteryLife_soc_limit;



    //std::stringstream ss;
    //std::stringstream ss_pv;


    //int total_grid_power = 1;
    //read_modbus(206,mb,&total_grid_power);

    // connect with the server
    //mb.modbus_connect();
    mb_victron_battery.modbus_connect();

    // TOTAL GRID POWER
    /*
    SOLAR EDGE MODBUS
    uint16_t modbus_buff[1];  
    mb.modbus_read_holding_registers(206, 1, modbus_buff);
    int16_t total_grid_power;
    memcpy(&total_grid_power,modbus_buff,sizeof(int16_t));
    total_grid_power *= -1;

    uint16_t modbus_buff2[1];  
    mb.modbus_read_holding_registers(210, 1, modbus_buff2);
    int16_t total_grid_power_sf;
    memcpy(&total_grid_power_sf,modbus_buff2,sizeof(int16_t));
  
    // PV Production
    uint16_t modbus_buff3[1];  
    mb.modbus_read_holding_registers(83, 1, modbus_buff3);
    int16_t ac_power;
    memcpy(&ac_power,modbus_buff3,sizeof(int16_t));
    ac_power *= -1;

    uint16_t modbus_buff4[1];  
    mb.modbus_read_holding_registers(84, 1, modbus_buff4);
    int16_t ac_power_sf;
    memcpy(&ac_power_sf,modbus_buff4,sizeof(int16_t));
    */


    // VICTRON MODBUS

    // Battery State
    uint16_t modbus_buff5[1];  
    mb_victron_battery.modbus_read_holding_registers(843, 1, modbus_buff5);
    uint16_t battery_state;
    memcpy(&battery_state,modbus_buff5,sizeof(uint16_t));
    
    // Battery battery_power
    uint16_t modbus_buff6[1];  
    mb_victron_battery.modbus_read_holding_registers(842, 1, modbus_buff6);
    int16_t battery_power;
    memcpy(&battery_power,modbus_buff6,sizeof(int16_t));

    // Max discharge current
    uint16_t modbus_buff7[1];  
    mb_victron_battery.modbus_read_holding_registers(2903, 1, modbus_buff7);
    uint16_t batteryLife_soc_limit;
    memcpy(&batteryLife_soc_limit,modbus_buff7,sizeof(uint16_t));
    batteryLife_soc_limit = batteryLife_soc_limit/10;//fixed scaling factor

    // PV Production from victron
    uint16_t modbus_buff8[1];  
    mb_victron_battery.modbus_read_holding_registers(808, 1, modbus_buff8);
    uint16_t pv_input_phase1;
    memcpy(&pv_input_phase1,modbus_buff8,sizeof(uint16_t));

    uint16_t modbus_buff9[1];   
    mb_victron_battery.modbus_read_holding_registers(809, 1, modbus_buff9);
    uint16_t pv_input_phase2;
    memcpy(&pv_input_phase2,modbus_buff9,sizeof(uint16_t));

    uint16_t modbus_buff10[1];   
    mb_victron_battery.modbus_read_holding_registers(810, 1, modbus_buff10);
    uint16_t pv_input_phase3;
    memcpy(&pv_input_phase3,modbus_buff10,sizeof(uint16_t));

    uint16_t pv_input_sum = pv_input_phase1+pv_input_phase2+pv_input_phase3;

    uint16_t modbus_buff11[1];   
    mb_victron_battery.modbus_read_holding_registers(820, 1, modbus_buff11);
    int16_t grid_phase1;
    memcpy(&grid_phase1,modbus_buff11,sizeof(int16_t));

    uint16_t modbus_buff12[1];   
    mb_victron_battery.modbus_read_holding_registers(821, 1, modbus_buff12);
    int16_t grid_phase2;
    memcpy(&grid_phase2,modbus_buff12,sizeof(int16_t));

    uint16_t modbus_buff13[1];   
    mb_victron_battery.modbus_read_holding_registers(822, 1, modbus_buff13);
    int16_t grid_phase3;
    memcpy(&grid_phase3,modbus_buff13,sizeof(int16_t));

    int16_t grid_power = grid_phase1+grid_phase2+grid_phase3;

    //mb.modbus_close();
    mb_victron_battery.modbus_close();

    msg.data = std::to_string(grid_power);
    msg_pv.data = std::to_string(pv_input_sum);
    msg_battery.data = std::to_string(battery_state);
    msg_battery_power.data = std::to_string(battery_power);
    msg_batteryLife_soc_limit.data = std::to_string(batteryLife_soc_limit);



    ROS_INFO("total_grid_power %s W \n", msg.data.c_str());
    ROS_INFO("total_pv_power %s W \n", msg_pv.data.c_str());
    ROS_INFO("battery_state_of_charge %s % \n", msg_battery.data.c_str());
    ROS_INFO("battery_power %s W \n", msg_battery_power.data.c_str());
    ROS_INFO("batteryLife_soc_limit %s % \n", msg_batteryLife_soc_limit.data.c_str());


    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    total_grid_power_pub.publish(msg);
    total_pv_power_pub.publish(msg_pv);
    battery_state_of_charge_pub.publish(msg_battery);
    battery_power_pub.publish(msg_battery_power);
    battery_life_soc_limit_pub.publish(msg_batteryLife_soc_limit);


    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
// %EndTag(FULLTEXT)%