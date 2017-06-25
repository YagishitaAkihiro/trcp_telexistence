#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Author: Akihiro Yagishita

"""
about:Joint_anguler

initial_anguler
[0.0,   #不明  0
 0.0,　　　#torso 1
 0.0,   #head0 2
 0.0,   #head1 3
 -0.6,  #rarm0 4
 0.0,   #rarm1 5
 -100.0,#rarm2 6
 15.2,  #rarm3 7
 9.4,   #rarm4 8
 3.2,   #rarm5 9
 0.0,          10
 0.0,          11
 0.0,          12
 0.0,          13
 0.6,   #larm0 14
 0.0,   #larm1 15
 -100.0,#larm2 16
 -15.2, #larm3 17
 9.4,   #larm4 18
 -3.2,  #larm5 19
 0.0,          20
 0.0,          21
 0.0,          22
 0.0]          23
と思われる。
残りは、（r/l）handかと
"""

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
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Float64

#import sys
#sys.path.append("/home/roboworks/catkin_ws/src/msg_of_hironx/msg")
from telexistence.msg import JointAngle


class Larm():
      def __init__(self):
         '''
         @param nxo_if: type nextage_client.NextageClient         
         _robot = nxo_if 今回"ros = ROS_Client()"で代用できた。
         '''
         listener = tf.TransformListener()
         rospy.sleep(1)
#         listener.waitForTransform("/torso_1", "/right_hand_1", rospy.Time(), rospy.Duration(1.0))
         x = -0.6
         y = 0
         z = -100

         pub = rospy.Publisher('/angle_data', JointAngle, queue_size=1)
         while not rospy.is_shutdown():
               try:
                  #torsoから見たleft_hand_1の座標。ros::Time(0)指定して最新のtransformを取得。
                  now = rospy.Time.now()
                  listener.waitForTransform("/torso_1", "/right_hand_1", now, rospy.Duration(1000.0))
                  (trans,rot) = listener.lookupTransform('/torso_1', '/right_hand_1', now)
#                  rospy.Subscriber("/joint_states", JointState, self.callback, queue_size=1)
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

               if 0.292 > transx:
                  transx=0.292
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
               if 0.002 > transz:
                  transz=0.002
                  print "over"
               if transz > 0.600:
                  transz=0.600
                  print "over"

               robot.setTargetPose("larm",[round((transx+x)/2,4),round((transy+y)/2,4),round((transz+z)/2,4)], [0, -1.6, -0.05],0.3)
               
               x = transx
               y = transy
               z = transz

               rospy.sleep(0.3)

               jorot = robot.getJointAngles()
               angle_data = JointAngle()

               angle_data.torso[0] = jorot[0]

               angle_data.head[0] = jorot[1]
               angle_data.head[1] = jorot[2]

               angle_data.rangle[0] = jorot[3]
               angle_data.rangle[1] = jorot[4]
               angle_data.rangle[2] = jorot[5]
               angle_data.rangle[3] = jorot[6]
               angle_data.rangle[4] = jorot[7]
               angle_data.rangle[5] = jorot[8]

               angle_data.langle[0] = jorot[9]
               angle_data.langle[1] = jorot[10]
               angle_data.langle[2] = jorot[11]
               angle_data.langle[3] = jorot[12]
               angle_data.langle[4] = jorot[13]
               angle_data.langle[5] = jorot[14]

               pub.publish(angle_data)#for文で綺麗にしようze

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
