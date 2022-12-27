/*
 * rosserial Subscriber Example
 * Blinks an LED on callback
 */

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>

ros::NodeHandle nh;
std_msgs::Int32 pushed_msg;
ros::Publisher pub_button("pushed", &pushed_msg);

void switch_relay(const std_msgs::Int32 &msg){
  if(msg.data > 0){
    digitalWrite(12, HIGH);
  }else{
    digitalWrite(12, LOW);
  }
  pushed_msg.data = msg.data;
  pub_button.publish(&pushed_msg);
}

ros::Subscriber<std_msgs::Int32> sub("relay1_state", &switch_relay );

void setup()
{
  pinMode(12, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub_button); 
}

void loop()
{
  nh.spinOnce();

  delay(1);
}
