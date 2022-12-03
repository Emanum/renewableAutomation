#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from pyModbusTCP.client import ModbusClient


def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10)#1hz

    rospy.loginfo("Talker node started ...")


    while not rospy.is_shutdown():
        hello_str = "hello world {time}".format(time = rospy.get_time()) 
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try: 
        talker()
    except rospy.ROSInterruptException:
        pass