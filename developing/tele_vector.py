#!/usr/bin/env python
#-*- coding:utf-8 -*-

#from nextage_ros_bridge import nextage_client
from hrpsys import rtm
from hironx_ros_bridge.ros_client import ROS_Client
import argparse
import rospy
import math
import tf
from geometry_msgs.msg import Vector3

#(left_x,left_y,left_z,right_x,right_y,right_z)
ini_p = (0.3255, 0.1823, 0.0746, 0.3255, -0.1823, 0.0746)
ini_f = [-0.6,   0.0,   -100.0,  0.6,     0.0,   -100.0]

class Tele():
      def q2e(self,rot):
          self.eul = tf.transformations.euler_from_quaternion((rot[0],rot[1],rot[2],rot[3]))
          return Vector(x=self.eul[0],y=self.eul[1],z=self.eul[2])

      def __init__(self):
          listener=tf.TransformListener()
          rospy.sleep(1)
          now = rospy.Time(0)
          rospy.loginfo("waiting hands and head tf")
          try:
              listener.waitForTransform("/target", "/l_hand", now, rospy.Duration(1.0))
              listener.waitForTransform("/target", "/r_hand", now, rospy.Duration(1.0))
              listener.waitForTransform("/target", "/hmd_head", now, rospy.Duration(1.0))
              (l_trans,l_rot) = listener.lookupTransform('/target', '/l_hand', now)
              (r_trans,r_rot) = listener.lookupTransform('/target', '/r_hand', now)
              (h_trans,h_rot) = listener.lookupTransform("/target", "/hmd_head", now)
          except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
              rospy.loginfo("TF not broadcast")
          rospy.loginfo("complete")
          
          initial_left = (-round(l_trans[2],2),-round(l_trans[0],2),round(l_trans[1],2))
          initial_right= (-round(l_trans[2],2),-round(l_trans[0],2),round(l_trans[1],2))

　　　　　　　　　　while not rospy.is_shutdown():
                now = rospy.Time(0)
                try:
                    (l_trans,l_rot) = listener.lookupTransform('/target', '/l_hand', now)
                    (r_trans,r_rot) = listener.lookupTransform('/target', '/r_hand', now)
                    (h_trans,h_rot) = listener.lookupTransform("/target", "/hmd_head", now) 
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    continue

#---------------Vector------------------------------------------------------------------
                global ini_p
                r_cur_p = robot.getCurrentPosition("RARM_JOINT5")
                l_cur_p = robot.getCurrentPosition("LARM_JOINT5")
　　　　　　　　　　　　　　　　
                l_trans[2] + ini_p[0] #どれだけうごいたか 
                l_trans[0] + ini_p[1]
                l_trans[1] + ini_p[2]

                r_trans[2] + ini_p[3]
                r_trans[0] + ini_p[4]
                r_trans[1] + ini_p[5]

#---------------------------------------------------------------------------------
　　　　　　　　　　　　　　　　global ini_p
                LTP = [,,]
                RTP = [,,]
#---------------------------------------------------------------------------------
                    











