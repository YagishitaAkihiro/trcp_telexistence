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
#leap
#__author__ = 'flier'
#from leap_motion.msg import leap
#from leap_motion.msg import leapros

from sensor_msgs.msg import Joy
from std_msgs.msg import Float64
import sys
#global listener
yaw = 0
pich = 0 
roll = 0
def scanYPR(data):
    global yaw
    yaw = data.axes[2]
    global pich
    pich = data.axes[1]
    global roll
    roll = data.axes[0]
    rospy.sleep(0.2)
#    rospy.loginfo('YPR Y:>'+str(yaw)+' P: '+str(pich)+' R: '+str(roll))

class Larm():
      def __init__(self):
         '''
         @param nxo_if: type nextage_client.NextageClient         
         _robot = nxo_if 今回"ros = ROS_Client()"で代用できた。
         '''
         self.flag=0
         listener = tf.TransformListener()
         listener.waitForTransform("/torso_1", "/right_hand_1", rospy.Time(), rospy.Duration(1.0))
         x = -0.6
         y = 0
         z = -100
         self.l_hand = rospy.Publisher("/tilt_controller/command", Float64, queue_size=1)
         rospy.Subscriber("/joy", Joy, self.wii_callback, queue_size = 1) #wiimote/wiimote_node.pyからpublish
         rospy.Subscriber("/joy", Joy, scanYPR, queue_size = 1)

         f_yaw  = yaw
         f_pich = pich
         f_roll = roll
         while not rospy.is_shutdown():
            if self.flag == 0:
               continue

            try:
                  #torsoから見たleft_hand_1の座標。ros::Time(0)指定して最新のtransformを取得。
                  now = rospy.Time.now()
                  listener.waitForTransform("/torso_1", "/right_hand_1", now, rospy.Duration(1.0))
                  (trans,rot) = listener.lookupTransform('/torso_1', '/right_hand_1', now)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                  continue
            #trans[2](人のz座標、前後)をHIROのｘ座標（transx）負号反転あと人の値に＋0.050
            transx = round(trans[2],4)
            transx = transx * -1
            transx = transx + 0.05
            #trans[0](人のx座標、左右)をHIROのy座標（transy）負号反転
            transy = round(trans[0],4)
            transy = transy * -1
            #trans[1](人のy座標、上下)をHIROのz座標（transz）に、あと人の値に＋0.080(手を上に上げるとまあまああってるけど、腰に近いところだと値小さい)
            transz = round(trans[1],4)
            transz = transz + 0.08

#               """
            if -0.292 > transx:
                  transx=-0.292
                  print "over"
            if transx > 0.523:
                  transx=0.523
                  print "over"
            if -0.120 > transy:
                  transy=-0.120
                  print "over"
            if transy > 0.501:
                  transy=0.501
                  print "over"
            if 0.00 > transz: #0.002
                  transz=0.00
                  print "over"
            if transz > 0.800:
                  transz=0.800
                  print "over"

            h_yaw  = round(yaw  * math.pi/21, 1) - f_yaw
            h_pich = round(pich * math.pi/21, 1) - 1.6 - f_pich
            h_roll = -( round(roll * math.pi/21, 1) - f_roll )
            print h_roll

            if h_yaw < -0.5:
               h_yaw = -0.5
            elif h_yaw > 0.5:
                 h_yaw = 0.5

            if h_pich < -2.5:
               h_pich = -2.5
            elif h_pich > 0.5:
                 h_pich = 0.5
            
            
            if h_roll < -1.2:
               h_roll = -1.2
            elif h_roll > 0.5:
                 h_roll =0.5
 
#            robot.setTargetPose("larm",[transx,transy,transz], [-0.5, -1.25, -0.7],1)
#            robot.setTargetPose("larm",[round((transx+x)/2,2),round((transy+y)/2,2),round((transz+z)/2,2)], [0, -1.3, -0.05],0.3)#1.6
            robot.setTargetPose("larm",[round((transx+x)/2,4),round((transy+y)/2,4),round((transz+z)/2,4)], [h_yaw, h_pich, h_roll],0.5)
            x = transx
            y = transy
            z = transz

            rospy.sleep(0.3)
            self.flag = 0

      def wii_callback(self,key_data):
          print "A: gripper , B: move_arm , 1: initialize , 2: off_pose"
          print key_data.buttons
          if key_data.buttons == (0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0): #B

             self.flag=1
          elif key_data.buttons == (0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0): #A
#             print "つかむぅ"
             self.l_hand.publish(0.49)
          elif key_data.buttons == (0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0): #A&B
#             print "つかむぅ"
             self.l_hand.publish(0.49)
             self.flag=1
          elif key_data.buttons == (1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0): #1
             robot.goInitial()
          elif key_data.buttons == (0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0): #2
             robot.goOffPose()
             rospy.sleep(10)
             sys.exit()
          else:
             self.l_hand.publish(1.0)

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
#    robot.Groups
#   主処理
    Larm()
    rospy.spin()
