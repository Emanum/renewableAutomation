
// %Tag(FULLTEXT)%
#include "ros/ros.h"
#include "std_msgs/String.h"

int total_grid_power = 0;//wattage => positiv:energy is consumed from grid, negativ:energy is delivered to grid
int total_pv_power = 0;//wattage >= 0
int battery_state_of_charge = 0;//percentage
int battery_power = 0;//wattage positiv:battery is charging, negativ: battery is discharging 

int total_grid_power_relay_switch = 2430;

void total_grid_power_callback(const std_msgs::String::ConstPtr& msg)
{
  total_grid_power = std::stoi( msg->data.c_str() );
  ROS_INFO("total_grid_power: [%d]", total_grid_power);
}

void total_pv_power_callback(const std_msgs::String::ConstPtr& msg)
{
  total_pv_power = std::stoi( msg->data.c_str() );
  ROS_INFO("total_pv_power: [%d]", total_pv_power);
}

void battery_state_of_charge_callback(const std_msgs::String::ConstPtr& msg)
{
  battery_state_of_charge  = std::stoi( msg->data.c_str() );
  ROS_INFO("battery_state_of_charge: [%d]", battery_state_of_charge);
}

void battery_power_callback(const std_msgs::String::ConstPtr& msg)
{
  battery_power  = std::stoi( msg->data.c_str() );
  ROS_INFO("battery_power: [%d]", battery_power);
}

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
  ros::init(argc, argv, "listener");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
// %Tag(SUBSCRIBER)%
  ros::Subscriber sub1 = n.subscribe("total_grid_power", 1000, total_grid_power_callback);
  ros::Subscriber sub2 = n.subscribe("total_pv_power", 1000, total_pv_power_callback);
  ros::Subscriber sub3 = n.subscribe("battery_state_of_charge", 1000, battery_state_of_charge_callback);
  ros::Subscriber sub4 = n.subscribe("battery_power", 1000, battery_power_callback); 
// %EndTag(SUBSCRIBER)%


  ros::Publisher relay1_state_pub = n.advertise<std_msgs::String>("relay1_state", 1000);

  ros::Rate r(10); // 10 hz
  while (ros::ok())
  {
    ros::spinOnce();//handels subscribers and publishers

    std_msgs::String msg;
    
    int relay_state = 1; 
    if(total_grid_power > total_grid_power_relay_switch){
      relay_state = 0;
    }
    msg.data = std::to_string(relay_state);
    ROS_INFO("relay1_state %s", msg.data.c_str());
    relay1_state_pub.publish(msg);
    
    r.sleep();
  }

  return 0;
}
// %EndTag(FULLTEXT)%