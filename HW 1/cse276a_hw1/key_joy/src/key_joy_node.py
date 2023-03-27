#!/usr/bin/env python
import sys
import rospy
from sensor_msgs.msg import Joy
import time
from key_parser import get_key, save_terminal_settings, restore_terminal_settings


def open_loop():
    
    pub_joy = rospy.Publisher("/joy", Joy, queue_size=1)
    rospy.init_node('key_joy')

    
    #flag = True
    joy_msg = Joy()
    joy_msg.axes = [0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0]
    joy_msg.buttons = [0, 0, 0, 0, 0, 0, 0, 0]

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():

        #init
        joy_msg.axes[0] = 0.0
        joy_msg.axes[1] = 0.0
        joy_msg.axes[2] = 0.0
        pub_joy.publish(joy_msg)
        time.sleep(5)    

        joy_msg.axes[1] = 1.0 #seq1
        pub_joy.publish(joy_msg)
        time.sleep(3.7)
        joy_msg.axes[1] = 0.0
        pub_joy.publish(joy_msg)

        time.sleep(2.0) #seq2
        joy_msg.axes[2] = 1.0 
        pub_joy.publish(joy_msg)
        time.sleep(1.8)
        joy_msg.axes[2] = 0.0
        pub_joy.publish(joy_msg)

        time.sleep(2) #seq3
        joy_msg.axes[1] = -1.0
        pub_joy.publish(joy_msg)
        time.sleep(4.3)
        joy_msg.axes[1] = 0.0
        pub_joy.publish(joy_msg)

        time.sleep(2.0) #seq4
        joy_msg.axes[2] = -1.0 
        pub_joy.publish(joy_msg)
        time.sleep(2.2)
        joy_msg.axes[2] = 0.0
        pub_joy.publish(joy_msg)

        time.sleep(2) #seq5
        joy_msg.axes[1] = 1.0
        pub_joy.publish(joy_msg)
        time.sleep(4.2)
        joy_msg.axes[1] = 0.0
        pub_joy.publish(joy_msg)

        time.sleep(2.0) #seq6
        joy_msg.axes[2] = -1.0 
        pub_joy.publish(joy_msg)
        time.sleep(2.1)
        joy_msg.axes[2] = 0.0
        pub_joy.publish(joy_msg)

        time.sleep(2) #seq7
        joy_msg.axes[1] = 1.0
        pub_joy.publish(joy_msg)
        time.sleep(4.2)
        joy_msg.axes[1] = 0.0
        pub_joy.publish(joy_msg)

        time.sleep(2.0) #seq8
        joy_msg.axes[2] = 1.0 
        pub_joy.publish(joy_msg)
        time.sleep(1.1)
        joy_msg.axes[2] = 0.0
        pub_joy.publish(joy_msg)

        time.sleep(2.0) #seq9
        joy_msg.axes[1] = -1.0 
        pub_joy.publish(joy_msg)
        time.sleep(12)
        joy_msg.axes[1] = 0.0
        pub_joy.publish(joy_msg)

        #stop signals
        joy_msg.axes[0] = 0.0
        joy_msg.axes[1] = 0.0
        joy_msg.axes[2] = 0.0
        pub_joy.publish(joy_msg)
        time.sleep(7)   

if __name__ == "__main__":
    try:
        open_loop()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

