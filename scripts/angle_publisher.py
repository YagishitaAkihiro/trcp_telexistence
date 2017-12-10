#!/usr/bin/env python
#-*- coding:utf-8 -*-
"""
initial_anguler
[0.0,　　　#torso 1  'CHEST_JOINT0'
 0.0,   #head0 2  'HEAD_JOINT0'
 0.0,   #head1 3  'HEAD_JOINT1'
 -0.6,  #rarm0 4  'RARM_JOINT0'
 0.0,   #rarm1 5  'RARM_JOINT1'
 -100.0,#rarm2 6  'RARM_JOINT2'
 15.2,  #rarm3 7  'RARM_JOINT3'
 9.4,   #rarm4 8  'RARM_JOINT4'
 3.2,   #rarm5 9  'RARM_JOINT5'
 0.0,          10 'RHAND_JOINT0'
 0.0,          11 'RHAND_JOINT1'
 0.0,          12 'RHAND_JOINT2'
 0.0,          13 'RHAND_JOINT3'
 0.6,   #larm0 14 'LARM_JOINT0'
 0.0,   #larm1 15 'LARM_JOINT1'
 -100.0,#larm2 16 'LARM_JOINT2'
 -15.2, #larm3 17 'LARM_JOINT3'
 9.4,   #larm4 18 'LARM_JOINT4'
 -3.2,  #larm5 19 'LARM_JOINT5'
 0.0,          20 'LHAND_JOINT0'
 0.0,          21 'LHAND_JOINT1'
 0.0,          22 'LHAND_JOINT2'
 0.0]          23 'LHAND_JOINT3'
"""
###############################################################
#これからやること
#・publish_dataを作る
#・首の部分を作成
#
#
###############################################################

import rospy
#from moveit_msgs.msg import MoveGroupActionGoal,PlanningScene
from std_msgs.msg import String, MultiArrayLayout, MultiArrayDimension, Float32MultiArray
from control_msgs.msg import FollowJointTrajectoryActionGoal
import sys

mode = String()
array = Float32MultiArray()

array.data = [0.0,0.0,0.0,
              0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,
              0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]

def main(mode):
    rospy.init_node("mvit2win")
    if mode == 1:
       rospy.Subscriber("/larm_controller/follow_joint_trajectory_action/goal",FollowJointTrajectoryActionGoal, debag, callback_args="left",  queue_size=1)
       rospy.Subscriber("/rarm_controller/follow_joint_trajectory_action/goal",FollowJointTrajectoryActionGoal, debag, callback_args="right", queue_size=1) 
    else:
       global pub
       pub = rospy.Publisher("angle_data",Float32MultiArray,queue_size=1)
       rospy.Subscriber("/larm_controller/follow_joint_trajectory_action/goal",FollowJointTrajectoryActionGoal, multiarray, callback_args="left",  queue_size=1)
       rospy.Subscriber("/rarm_controller/follow_joint_trajectory_action/goal",FollowJointTrajectoryActionGoal, multiarray, callback_args="right", queue_size=1) 
def debag(data,id):
    if id == "left":
       print "left angles is "  + str(data.goal.trajectory.points[-1].positions) #左腕の関節を表示
    else:
       print "right angles is " + str(data.goal.trajectory.points[-1].positions) #右腕の関節を表示

def multiarray(data,id):
    if id == "left":
       for i in xrange(len(data.goal.trajectory.points[-1].positions)):
           array.data[i+3] = data.goal.trajectory.points[-1].positions[i]
           print array.data
    elif id == "right":
       for i in xrange(len(data.goal.trajectory.points[-1].positions)):
           array.data[i+13] = data.goal.trajectory.points[-1].positions[i]
           print array.data
    print array.data
    global pub
    pub.publish(array)



if __name__ == "__main__":
   print "start"
   args = sys.argv
   if not len(args) == 2:
      print("args not found.")
      sys.exit()
   elif args[1] == "deb":
      print "debag mode"
      mode = 1
   elif args[1] == "main":
      print "main mode"
      mode = 2
   else:
      print("bad args. Starting debag mode")
      mode = 1
   main(mode)
   rospy.spin()
