#!/usr/bin/env python
#-*- coding:utf-8 -*-
 
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import Joy

topic_name = "/vive/joy3"

pub = rospy.Publisher("/tilt_controller/command", Float64)

def send_command(mode):
    rospy.init_node("vive2gripper")
    rospy.Subscriber(topic_name, Joy, catcher, queue_size=1)#要調整
    rospy.spin()

def catcher(data):
    if data.button[1] == 1:
       pub.publish(0.2)#閉じるときの数値を入れて
    else:
       pub.publish(0.9)#開くときの数値を入れて

if __name__ == '__main__':
   send_command()
