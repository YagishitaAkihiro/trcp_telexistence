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
#leap
__author__ = 'flier'

#from leap_motion.msg import leap
#from leap_motion.msg import leapros
#oculus
import geometry_msgs.msg
import tf
from geometry_msgs.msg import Vector3

b_yow = 0.0
b_pich = 0.0

class Neck_move():
      def __init__(self):
         '''
         @param nxo_if: type nextage_client.NextageClient         
         _robot = nxo_if 今回"ros = ROS_Client()"で代用できた。
         '''
#        首の角度を取得(今回leap_motion使用)今後、oculus等で実験を行うこと。
         #rospy.Subscriber("/oculus_ori", geometry_msgs.msg.Pose, self.get_vectors, queue_size = 1)
         rospy.Subscriber("/oculus_ori", geometry_msgs.msg.Pose, self.quaternion_to_euler, queue_size = 1)
          
      def get_vectors(self,data):
#         デバック用ypr出力...導入後、コメントアウト推奨。
          rospy.loginfo("oculus ROS Data \n%s" % data)
#          robot.setTargetPose("head",[0.0, 0.0, 0.56950000000000001], [0.0, math.radians(data.ypr.z), math.radians(data.ypr.x)], 1)
#          robot.setTargetPose("head",[0.0, 0.0, 0.56950000000000001], [0.0, 0.0, math.radians(data.ypr.x)], 2)
          rospy.sleep(1)
      def quaternion_to_euler(self,data):
          """Convert Quaternion to Euler Angles

          quarternion: geometry_msgs/Quaternion
          euler: geometry_msgs/Vector3
          """
#          print data
          e = tf.transformations.euler_from_quaternion((data.orientation.x, data.orientation.y,data.orientation.z, data.orientation.w))
#          return Vector3(x=e[0], y=e[1], z=e[2])
#          print Vector3(x=e[0], y=e[1], z=e[2])
          yow = e[1]
          pich = -e[0]
          if yow < -1.20:
             yow = -1.20
          elif yow > 1.20:
             yow = 1.20
          if pich < -0.20:
             pich = -0.20
          elif pich > 0.20:
             pich = 0.20
#          robot.setTargetPose("head",[0.0, 0.0, 0.56950000000000001], [0.0, round(pich,1), round(yow,1)], 0.28)
#          robot.setTargetPose("head",[0.00, 0.00, 0.56950000000000001], [0.00, 0.00, round(yow,2)], 0.4)#横向きだけ
          global b_yow, b_pich
#          print b_yow, b_pich
          ros.set_joint_angles_rad("head",[(b_yow +round(yow,3))/2,(b_pich+round(pich,3))/2],duration=2.0,wait=False)
#          ros.set_joint_angles_rad("head",[round(yow,3),round(pich,3)],duration=0.5,wait=False)
          rospy.sleep(0.5)
          b_yow = (b_yow +round(yow,3))/2
          b_pich= (b_pich+round(pich,3))/2


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
    robot.goInitial()
#   主処理
    Neck_move()
    rospy.spin()
