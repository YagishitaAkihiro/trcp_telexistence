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

#(left_x,left_y,left_z,right_x,right_y,right_z)
ini_p = (0.3255, 0.1823, 0.0746, 0.3255, -0.1823, 0.0746)
ini_f = [-0.6,   0.0,   -100.0,  0.6,     0.0,   -100.0]
ini_ang = [0.0, -1.6, -0.05] #タプルにせねば。

low_filter = [-0.6, 0.0, -100.0, 0.6, 0.0, -100.0]#ローパスフィルター用初期値

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
              listener.waitForTransform("/Spine", "/LeftHand", now, rospy.Duration(1.0))
              listener.waitForTransform("/Spine", "/RightHand",now, rospy.Duration(1.0))
              listener.waitForTransform("/Spine", "/Head", now, rospy.Duration(1.0))
              (l_trans,l_rot) = listener.lookupTransform('/Spine', '/LeftHand', now)
              (r_trans,r_rot) = listener.lookupTransform('/Spine', '/RightHand', now)
              (h_trans,h_rot) = listener.lookupTransform("/Spine", "/Head", now)
          except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
              rospy.loginfo("TF not broadcast")
          rospy.loginfo("complete")
          
          initial_left = (round(l_trans[2],2),round(l_trans[0],2),round(l_trans[1],2))
          initial_right= (round(r_trans[2],2),round(r_trans[0],2),round(r_trans[1],2))
          while not rospy.is_shutdown():
                now = rospy.Time(0)
                try:
                    (l_trans2,l_rot2) = listener2.lookupTransform('/Spine', '/LeftHand', now)
                    (r_trans2,r_rot2) = listener2.lookupTransform('/Spine', '/RightHand', now)
                    (h_trans2,h_rot2) = listener2.lookupTransform("/Spine", "/Head", now) 
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    continue

#---------------Vector------------------------------------------------------------------
                global ini_p

                L_dis = [(l_trans2[2] - initial_left[0]), #どれだけうごいたか 
                         (l_trans2[0] - initial_left[1]),
                         (l_trans2[1] - initial_left[2])]

                R_dis = [(r_trans2[2] - initial_right[0]), #どれだけうごいたか 
                         (r_trans2[0] - initial_right[1]),
                          (r_trans2[1] - initial_right[2])]

#---------------------------------------------------------------------------------
                global low_filter

                LTP = [round((ini_p[0]+L_dis[0]+low_filter[0])/2,2),
                       round((ini_p[1]+L_dis[1]+low_filter[1])/2,2),
                       round((ini_p[2]+L_dis[2]+low_filter[2])/2,2)]

                RTP = [round((ini_p[3]+R_dis[0]+low_filter[3])/2,2),
                       round((ini_p[4]+R_dis[1]+low_filter[4])/2,2),
                       round((ini_p[5]+R_dis[2]+low_filter[5])/2,2)]
 
#----------------------------------------------------
                if -0.292 > LTP[0]:
                  LTP[0] =-0.292
                if LTP[0] > 0.523:
                  LTP[0] =0.523

                if -0.150 > LTP[1]:
                  LTP[1] = -0.150
                if LTP[1] > 0.6:#0.501:
                  LTP[1] = 0.6 #0.501

                if -0.0 > LTP[2]:
                  LTP[2] = -0.0
                if LTP[2] > 0.600:
                  LTP[2] =0.600

                if -0.292 > RTP[0]:
                   RTP[0] = 0.292
                if RTP[0] > 0.523:
                   RTP[0] = 0.523

                if 0.150 < RTP[1]:
                   RTP[1] = 0.150
                if RTP[1] < -0.6:
                   RTP[1] = -0.6

                  
#---------------------------------------------------------------------------------
                global ini_ang
                robot.setTargetPose("larm",LTP, ini_ang,0.3)
                robot.setTargetPose("rarm",RTP, ini_ang,0.3)
                #ros.set_pose("rarm",RTP,ini_ang,0.2)
#                ros.set_pose("larm",LTP,ini_ang,1.0)
                rospy.sleep(0.3)

                #ローパスフィルター_アップデート
                r_cur_p = robot.getCurrentPosition("RARM_JOINT5")
                l_cur_p = robot.getCurrentPosition("LARM_JOINT5")
                low_filter = [l_cur_p[0], l_cur_p[1], l_cur_p[2], r_cur_p[0], r_cur_p[1], r_cur_p[2]]

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
