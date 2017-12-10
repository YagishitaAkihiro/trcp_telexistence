#!/usr/bin/env python
#-*- coding:utf-8 -*-

from nextage_ros_bridge import nextage_client
from hrpsys import rtm
from hironx_ros_bridge.ros_client import ROS_Client
import argparse
import rospy
import math
import tf
from geometry_msgs.msg import Vector3

base_head=[0.0,0.0]#ローパスフィルター用初期値

class Tele():
      def q2e(self,rot):
          self.eul = tf.transformations.euler_from_quaternion((rot[0],rot[1],rot[2],rot[3]))
          return Vector(x=self.eul[0],y=self.eul[1],z=self.eul[2])

      def __init__(self):
          robot.goInitial()
          listener =tf.TransformListener()
          listener2=tf.TransformListener()
          rospy.sleep(1)
          now = rospy.Time(0)
          rospy.loginfo("waiting hands and head tf")
          try:
              listener.waitForTransform("/Spine", "/Head", now, rospy.Duration(1.0))
              (h_trans,h_rot) = listener.lookupTransform("/Spine", "/Head", now)
          except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
              rospy.loginfo("TF not broadcast")
          rospy.loginfo("complete")

          ini_head = self.q2e(h_rot)
          initial_head = (ini_head[1],ini_head[0])
          while not rospy.is_shutdown():
                now = rospy.Time(0)
                try:
                    (h_trans2,h_rot2) = listener2.lookupTransform("/Spine", "/Head", now) 
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    continue

                global base_head
                N_head_d = self.q2e(h_rot2)
                N_head = [N_head_d[1],N_head_d[0]]
                HeaD= [round(H_head[1]-initial_head[1]+base_head[1]/2,1),
                       round(H_head[0]-initial_head[0]+base_head[0]/2,1)]
 
                ros.set_joint_angles_rad("head",[HeaD[0],HeaD[1]],duration=0.3,wait=False) 
                rospy.sleep(0.3)

                base_head = [HeaD[0],Head[1]]
if __name__ == '__main__':

#-------------------------------------------initial_setting------------------------------------------
    parser = argparse.ArgumentParser(description='NEXTAGE Open command line interpreters')
    parser.add_argument('--host', help='corba name server hostname')
    parser.add_argument('--port', help='corba name server port number')
    parser.add_argument('--modelfile', help='robot model file nmae')
    parser.add_argument('--robot', help='robot modlule name (RobotHardware0 for real robot, Robot()')
    args, unknown = parser.parse_known_args()
  
    if args.host:
       rtm.nshost = args.host
    if args.port:
       rtm.nsport = args.port
    if not args.robot:
       args.robot = "RobotHardware0" if args.host else "HiroNX(Robot)0"
    if not args.modelfile:
       args.modelfile = ""
  
    if len(unknown) >= 2:
       args.robot = unknown[0]
       args.modelfile = unknown[1]
    robot = nxc = nextage_client.NextageClient()
    robot.init(robotname=args.robot, url=args.modelfile)
    ros = ROS_Client()
#--------------------------------------------end_initial_setting------------------------------------------
#   主処理
    Tele()
