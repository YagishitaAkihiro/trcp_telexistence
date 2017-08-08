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

from std_msgs.msg import String

class Tele():
      def quaternion_to_euler(self,rot):
             self.e = tf.transformations.euler_from_quaternion((rot[0], rot[1], rot[2], rot[3]))
             return Vector3(x=self.e[0], y=self.e[1], z=self.e[2])


      def __init__(self):
         '''
         @param nxo_if: type nextage_client.NextageClient         
         _robot = nxo_if 今回"ros = ROS_Client()"で代用できた。
         '''
         #-----------------------------------         
         self.pub = rospy.Publisher("plan_pose",String,queue_size=1)
         #-----------------------------------
         listener = tf.TransformListener()
         rospy.sleep(1)
         #ros::Time(0)指定して最新のtransformを取得。
         #controller1が左腕、controller2が右腕 という体にしてます。
         now = rospy.Time(0)
         try:
             listener.waitForTransform("/target", "/l_hand", now, rospy.Duration(1.0))
             listener.waitForTransform("/target", "/r_hand", now, rospy.Duration(1.0))
             listener.waitForTransform("/target", "/hmd_head", now, rospy.Duration(1.0))
             (l_trans,l_rot) = listener.lookupTransform('/target', '/l_hand', now)
             (r_trans,r_rot) = listener.lookupTransform('/target', '/r_hand', now)
             (h_trans,h_rot) = listener.lookupTransform("/target", "/hmd_head", now)
         except  (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                  print "tf error"
         lkX = -round(l_trans[2],2)
         lkY = -round(l_trans[0],2)
         lkZ =  round(l_trans[1],2)

         rkX = -round(r_trans[2],2)
         rkY = -round(r_trans[0],2)
         rkZ =  round(r_trans[1],2)


         lHX = 0.3255
         lHY = 0.1823
         lHZ = 0.0746

         lx = -0.6
         ly = 0
         lz = -100

         rHX = 0.3255
         rHY = -0.1823
         rHZ = 0.0746

         rx = 0.6
         ry = 0
         rz = -100

#------------------------
         self.quaternion_to_euler(l_rot)
         ldx = round(self.e[2],1)
         ldy = round(self.e[1],1)
         ldz = round(self.e[0],1)

         self.quaternion_to_euler(r_rot)
         rdx = round(self.e[2],1)
         rdy = round(self.e[1],1)
         rdz = round(self.e[0],1)

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
                  listener.waitForTransform("/target", "/l_hand", now, rospy.Duration(10.0))
                  listener.waitForTransform("/target", "/r_hand", now, rospy.Duration(10.0))
                  listener.waitForTransform("/target", "/hmd_head", now, rospy.Duration(10.0))
                  (l_trans,l_rot) = listener.lookupTransform("/target", "/l_hand", now)
                  (r_trans,r_rot) = listener.lookupTransform("/target", "/r_hand", now)
                  (h_trans,h_rot) = listener.lookupTransform("/target", "/hmd_head", now)
               except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                  continue

#--------------generate_move_plan------------------------------
               l_transx = lHX + (round(l_trans[2],2) + lkX)
               l_transy = lHY + (round(l_trans[0],2) + lkY)
               l_transz = lHZ + (round(l_trans[1],2) - lkZ)

               self.quaternion_to_euler(l_rot)
               lrotx = roll_x  + round(self.e[0],1) - ldx
               lroty = pitch_y -(round(self.e[1],1) - ldy)
               lrotz = yaw_z   + round(self.e[2],1) - ldz

               r_transx = rHX + (round(r_trans[2],2) + rkX)
               r_transy = rHY + (round(r_trans[0],2) + rkY)
               r_transz = rHZ + (round(r_trans[1],2) - rkZ)

               self.quaternion_to_euler(r_rot)
               rrotx = roll_x  + round(self.e[0],1) - rdx
               rroty = pitch_y -(round(self.e[1],1) - rdy)
               rrotz = yaw_z   + round(self.e[2],1) - rdz


               self.quaternion_to_euler(h_rot)
               head0 = -round(self.e[1],2) - c_head0
               head1 =  round(self.e[0],2) - c_head1
#-------------------------------------------------------------
#filter

               if -0.292 > l_transx:
                  l_transx=-0.292
               if l_transx > 0.523:
                  l_transx=0.523

               if -0.150 > l_transy:
                  l_transy=-0.150
               if l_transy > 0.6:#0.501:
                  l_transy=0.6#0.501

               if -0.0 > l_transz:
                  l_transz= -0.0
               if l_transz > 0.600:
                  l_transz=0.600
#----------------------------------------
               if -0.292 > r_transx:
                  r_transx=-0.292
               if r_transx > 0.523:
                  r_transx=0.523

               if 0.150 < r_transy:
                  r_transy=0.150
               if r_transy < -0.6:#0.501:
                  r_transy=-0.6#0.501

               if -0.0 > l_transz:
                  r_transz=-0.0
               if r_transz > 0.600:
                  r_transz=0.600


#--------------head_filter--------
               if head0 > 0.7:
                  head0 = 0.7
               if head0 < -0.7:
                  head0 = -0.7

               if head1 > 0.3:
                  head1 = 0.3
               if head1 < -0.3:
                  head1 = -0.3

#--------------move_hand----------
               robot.setTargetPose("larm",[round((l_transx+lx)/2,2),round((l_transy+ly)/2,2),round((l_transz+lz)/2,2)], [0.0, -1.6, -0.05],0.5)
 #              robot.setTargetPose("rarm",[round((r_transx+rx)/2,2),round((r_transy+ry)/2,2),round((r_transz+rz)/2,2)], [0.0, -1.6, -0.05],0.5)

               pub_data = str(round((l_transx+lx)/2,2))+","+str(round((l_transy+ly)/2,2))+","+str(round((l_transz+lz)/2,2))+","+str(round((r_transx+rx)/2,2))+","+str(round((r_transy+ry)/2,2))+","+str(round((r_transz+rz)/2,2))+","
#--------------filter_update------
               lx = l_transx
               ly = l_transy
               lz = l_transz

               rx = r_transx
               ry = r_transy
               rz = r_transz
#-------------move_head-------------------------------
#               ros.set_joint_angles_rad("head",[-round((b_head0 + head0)/2,3),round((b_head1 + head1)/2,3)],duration=0.2,wait=True) 
               ros.set_joint_angles_rad("head",[-round((b_head0 + head0)/2,3),0],duration=0.4,wait=False)   
               b_head0 = head0
               b_head1 = head1
               
               pub_data = pub_data + str(-round((b_head0 + head0)/2,3))+","+str(0.0)

               self.pub.publish(pub_data)
#--------------wait_data-------
               rospy.sleep(0.5)

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
    Tele()
