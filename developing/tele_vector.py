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
ini_ang = [0.0, -1.6, -0.05]
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
          
          initial_left = (round(l_trans[2],2),round(l_trans[0],2),round(l_trans[1],2))
          initial_right= (round(l_trans[2],2),round(l_trans[0],2),round(l_trans[1],2))

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
　　　　　　　　　　　　　　　　
                L_dis = [l_trans[2] - ini_p[0], #どれだけうごいたか 
                         l_trans[0] - ini_p[1],
                         l_trans[1] - ini_p[2]]

                R_dis = [r_trans[2] - ini_p[3],
                         r_trans[0] - ini_p[4],
                         r_trans[1] - ini_p[5]]
                #ini_p[] + *_dis[] => Target_position
                luv = math.sqrt(math.pow(L_dis[0] + ini_p[0] - l_cur_p[0],2)
                               +math.pow(L_dis[1] + ini_p[1] - l_cur_p[1],2)
                               +math.pow(L_dis[2] + ini_p[2] - l_cur_p[2],2))
                ruv = math.sqrt(math.pow(R_dis[0] + ini_p[3] - r_cur_p[0],2)
                               +math.pow(R_dis[1] + ini_p[4] - r_cur_p[1],2)
                               +math.pow(R_dis[2] + ini_p[5] - r_cur_p[2],2))
#---------------------------------------------------------------------------------
                LTP = [ini_p[0]+(L_dis[0]/luv),ini_p[1]+(L_dis[1]/luv),ini_p[2]+(L_dis[2]/luv)]
                RTP = [ini_p[3]+(R_dis[0]/ruv),ini_p[4]+(R_dis[1]/ruv),ini_p[5]+(R_dis[2]/ruv)]
#---------------------------------------------------------------------------------
                global ini_ang
                ros.set_pose("rarm",RTP,ini_ang,0.2)
               #ros.set_pose("larm",LTP,ini_ang,0.2)
                rospy.sleep(0.2)

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
