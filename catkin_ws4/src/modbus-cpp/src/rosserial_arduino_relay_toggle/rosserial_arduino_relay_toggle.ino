/*
 * rosserial Subscriber Example
 * Blinks an LED on callback
 */

#include <ros.h>
#include <std_msgs/String.h>

ros::NodeHandle nh;

void switch_relay( const std_msgs::String& msg){
  if(msg.data == "1"){
    digitalWrite(13, HIGH);
  }else{
    digitalWrite(13, HIGH);
  }
}

ros::Subscriber<std_msgs::String> sub("relay1_state", &switch_relay );

void setup()
{
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{
  nh.spinOnce();
  delay(1);
}
