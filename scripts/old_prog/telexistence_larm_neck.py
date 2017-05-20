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
from geometry_msgs.msg import Pose

yaw = 0.0
pich=0.0

def quaternion_to_euler(data):
          """
          Convert Quaternion to Euler Angles
          quarternion: geometry_msgs/Quaternion
          euler: geometry_msgs/Vector3
          """
          e = tf.transformations.euler_from_quaternion((data.orientation.x, data.orientation.y,data.orientation.z, data.orientation.w))
          global yaw, pich
          yaw = e[1]
          pich = -e[0]
          if yaw < -1.20:
             yaw = -1.20
          elif yaw > 1.20:
             yaw = 1.20
          if pich < -0.20:
             pich = -0.20
          elif pich > 0.20:
             pich = 0.20

#          global b_yaw, b_pich
#          print b_yaw, b_pich
#          ros.set_joint_angles_rad("head",[(b_yaw +round(yaw,3))/2,(b_pich+round(pich,3))/2],duration=0.3,wait=False)
#          ros.set_joint_angles_rad("head",[round(yaw,3),round(pich,3)],duration=0.5,wait=False)
#          rospy.sleep(0.5)

#          b_yaw = (b_yaw +round(yaw,3))/2
#          b_pich= (b_pich+round(pich,3))/2

class Larm():
      def __init__(self):
         '''
         @param nxo_if: type nextage_client.NextageClient         
         _robot = nxo_if 今回"ros = ROS_Client()"で代用できた。
         '''
         listener = tf.TransformListener()
         rospy.sleep(1)
         #torsoから見たleft_hand_1の座標。ros::Time(0)指定して最新のtransformを取得。
         now = rospy.Time.now()
         listener.waitForTransform("/openni_depth_frame", "/right_hand_1", now, rospy.Duration(1.0))
         (trans,rot) = listener.lookupTransform('/openni_depth_frame', '/right_hand_1', now)
         kX = round(trans[0],4)
         kY = round(trans[1],4)
         kZ = round(trans[2],4)

         #初期姿勢(x,y,z)
         HX = 0.3255
         HY = 0.1823
         HZ = 0.0746
         #初期姿勢(Θx,Θy,Θz)
         IX =  0.00
         IY = -1.60
         IZ = -0.05
         #上限
         highX = 0.55
         highY = 0.50
         highZ = 0.40
         #下限
         lowX = 0.15
         lowY = -0.05
         lowZ = 0.07
         #周期
         Rtime = 0.35

         x = -0.6
         y = 0
         z = -100

         #首初期角度
         b_yaw = 0.0
         b_pich = 0.0

         while not rospy.is_shutdown():
               try:
                  #torsoから見たleft_hand_1の座標。ros::Time(0)指定して最新のtransformを取得。
                  now = rospy.Time.now()
                  listener.waitForTransform("/openni_depth_frame", "/right_hand_1", now, rospy.Duration(1.0))
                  (trans,rot) = listener.lookupTransform('/openni_depth_frame', '/right_hand_1', now)
               except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                  continue

               rospy.Subscriber("/oculus_ori", Pose, quaternion_to_euler, queue_size = 1)

               transx = HX - (round(trans[0],4) - kX)
               transy = HY - (round(trans[1],4) - kY)
               transz = HZ + (round(trans[2],4) - kZ)


#               """
               if lowX > transx:
                  transx=lowX
#                  print "over lowX"
               if transx > highX:
                  transx=highX
#                  print "over highX"
               if lowY > transy:
                  transy=lowY
#                  print "over lowY"
               if transy > highY:
                  transy=highY
#                  print "over highY"
               if lowZ > transz:
                  transz=lowZ
#                  print "over lowZ"
               if transz > highZ:
                  transz=highZ
#                  print "over highZ"
#               """
#              robot.setTargetPose("larm",[transx,transy,transz], [-0.5, -1.25, -0.7],1)
               robot.setTargetPose("larm",[round((transx+x)/2,2),round((transy+y)/2,2),round((transz+z)/2,2)], [IX, IY, -IZ], Rtime)

               x = transx
               y = transy
               z = transz

#               global yaw, pich
               ros.set_joint_angles_rad("head",[round((b_yaw + yaw)/2,3),round((b_pich + pich)/2,3)],duration=(Rtime - 0.1),wait=False)           

               b_yaw = round((b_yaw + yaw)/2, 3)
               b_pich= round((b_pich + pich)/2, 3)

               rospy.sleep(Rtime)

      

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
#   主処理
    Larm()
    rospy.spin()
