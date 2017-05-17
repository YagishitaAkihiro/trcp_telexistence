#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Author: Akihiro Yagishita

from nextage_ros_bridge import nextage_client
from hrpsys import rtm
import argparse
from hironx_ros_bridge.ros_client import ROS_Client

# depends
import rospy
import math
import tf
from geometry_msgs.msg import Vector3
#leap
#__author__ = 'flier'

#from leap_motion.msg import leap
#from leap_motion.msg import leapros


class Larm():
      def quaternion_to_euler(self,rot):
             self.e = tf.transformations.euler_from_quaternion((rot[0], rot[1], rot[2], rot[3]))
             return Vector3(x=self.e[0], y=self.e[1], z=self.e[2])


      def __init__(self):
         '''
         @param nxo_if: type nextage_client.NextageClient         
         _robot = nxo_if 今回"ros = ROS_Client()"で代用できた。
         '''
         listener = tf.TransformListener()
         rospy.sleep(1)
         #torsoから見たleft_hand_1の座標。ros::Time(0)指定して最新のtransformを取得。
         now = rospy.Time(0)
         try:
          listener.waitForTransform("/purpose", "/controller1", now, rospy.Duration(10.0))
          listener.waitForTransform("/purpose", "/hmd", now, rospy.Duration(10.0))
          (l_trans,l_rot) = listener.lookupTransform('/purpose', '/controller1', now)
          (h_trans,h_rot) = listener.lookupTransform("/purpose", "/hmd", now)
         except  (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
          print "tf error"
         kX = -round(l_trans[2],2)
         kY = -round(l_trans[0],2)
         kZ =  round(l_trans[1],2)

         HX = 0.3255
         HY = 0.1823
         HZ = 0.0746

         x = -0.6
         y = 0
         z = -100
#------------------------
         self.quaternion_to_euler(l_rot)
         dx = round(self.e[2],1)
         dy = round(self.e[1],1)
         dz = round(self.e[0],1)

         roll_x = 0.0
         pitch_y = -1.6
         yaw_z = -0.05
         #0, -1.6, -0.05
         self.quaternion_to_euler(h_rot)
         c_head0 = -round(self.e[1],2)
         c_head1 = -round(self.e[0],2)

         b_head0 = 0.0
         b_head1 = 0.0 #上下
#------------------------
         while not rospy.is_shutdown():
               try:
                  #torsoから見たleft_hand_1の座標。ros::Time(0)指定して最新のl_transformを取得。
#                  now = rospy.Time.now()
                  now = rospy.Time(0)
                  listener.waitForTransform("/purpose", "/controller1", now, rospy.Duration(100.0))
                  listener.waitForTransform("/purpose", "/hmd", now, rospy.Duration(100.0))
                  (l_trans,l_rot) = listener.lookupTransform("/purpose", "/controller1", now)
                  (h_trans,h_rot) = listener.lookupTransform("/purpose", "/hmd", now)
               except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                  continue


               l_transx = HX + (round(l_trans[2],2) + kX)
               l_transy = HY + (round(l_trans[0],2) + kY)
               l_transz = HZ + (round(l_trans[1],2) - kZ)
               self.quaternion_to_euler(l_rot)
               rotx = roll_x  + round(self.e[0],1) - dx
               roty = pitch_y -(round(self.e[1],1) - dy)
               rotz = yaw_z   + round(self.e[2],1) - dz

               self.quaternion_to_euler(h_rot)
               head0 = -round(self.e[1],2) - c_head0
               head1 = round(self.e[0],2) - c_head1
#              filter
#               """
               if -0.292 > l_transx:
                  l_transx=-0.292
#                  print "over x"
               if l_transx > 0.523:
                  l_transx=0.523
#                  print "over x"

               if -0.150 > l_transy:
                  l_transy=-0.150
#                  print "over y"
               if l_transy > 0.6:#0.501:
                  l_transy=0.6#0.501
#                  print "over y"

               if 0.050 > l_transz:
                  l_transz=0.050
#                  print "over z"
               if l_transz > 0.600:
                  l_transz=0.600
#                  print "over z"
               """
               if rotx > 0.5:
                  rotx=0.5
                  print "over rotx"
               if rotx < -0.5:
                  rotx=-0.5
                  print "over rotx"
               if roty > 0.0:
                  roty=0.0
                  print "over roty"
               if roty < -2.1:
                  roty=-2.1
                  print "over roty"
               if rotz > 0.5:
                  rotz=0.5
                  print "over rotz"
               if rotz < -0.5:
                  rotz=0.5
                  print "over rotz"

               """

               if head0 > 0.7:
                  head0 = 0.7
               if head0 < -0.7:
                  head0 = -0.7

               if head1 > 0.3:
                  head1 = 0.3
               if head1 < -0.3:
                  head1 = -0.3
               robot.setTargetPose("larm",[round((l_transx+x)/2,2),round((l_transy+y)/2,2),round((l_transz+z)/2,2)], [0.0, -1.6, -0.05],0.5)
#               robot.setTargetPose("larm",[round((l_transx+x)/2,2),round((l_transy+y)/2,2),round((l_transz+z)/2,2)], [rotx,roty,rotz],0.3)
               x = l_transx
               y = l_transy
               z = l_transz

#               ros.set_joint_angles_rad("head",[round((b_head0 + head0)/2,3),round((b_head1 + head1)/2,3)],duration=0.2,wait=True)           
               ros.set_joint_angles_rad("head",[-round((b_head0 + head0)/2,3),0],duration=1.0,wait=False)           
#               ros.set_joint_angles_rad("head",[0,round(b_head1 + head1)/2,2],duration=0.2,wait=False)
               b_head0 = head0
               b_head1 = head1
               rospy.sleep(0.5)
               #rospy.sleep(0.03)

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
#   24行目の呼び出しの代用品
    ros = ROS_Client()
#--------------------------------------------end_initial_setting------------------------------------------
#   "robot."が使えるかのテストとして、jointをリストアップ    
    robot.Groups
#   主処理
    Larm()
    rospy.spin()
