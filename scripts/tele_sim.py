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

import rospy
from moveit_msgs.msg import MoveGroupActionGoal,PlanningScene
from std_msgs.msg import String, MultiArrayLayout, MultiArrayDimension, Float32MultiArray
import sys

mode = String()

def main(mode):
    rospy.init_node("mvit2win")
    if mode == 1:
       rospy.Subscriber("/move_group/goal",MoveGroupActionGoal, debag, queue_size=1)
    else:
       rospy.Subscriber("/move_group/goal",MoveGroupActionGoal, multiarray, queue_size=1)
def debag(data):
    print data.goal.request.goal_constraints[0].joint_constraints

def multiarray(data):
    if len(data.goal.request.goal_constraints[0].joint_constraints) != 23:
       angle_data = [0.0,0.0,0.0,
                     0.0,0.0,0.0,0.0,0.0,0.0,
                     0.0,0.0,0.0,0.0,0.0,0.0]
       """
          #data.goal.request.goal_constraints[0].joint_constraints[x]のxがそれぞれの関節角になると思われる。
          #全角度を出すためには、rvizのplannning requestを"upperbody"にしなければならない。
          if data.goal.request.goal_constraints[0].joint_constraints[0].joint_name == "CHEST_JOINT0":
             for i in xrange(len(data.goal.request.goal_constraints[0].joint_constraints)):
                self.angle_data[i] = data.goal.request.goal_constraints[0].joint_constraints[i].joint_name + ":"+ str(data.goal.request.goal_constraints[0].joint_constraints[i].position)
             print self.angle_data
             self.pub.publish(str(self.angle_data))
          else:
             print "rviz error! you must set upperbody of plannning request on rviz!"
          """   
if __name__ == "__main__":
   print "start"
   args = sys.argv
   if not len(args) == 2:
      print("args not found.")
      sys.exit()
   elif args[1] == "deb":
      print "debag mode"
      mode = 1
   elif args[1] == "play":
      print "main mode"
      mode = 2
   else:
      print("bad args. Starting debag mode")
      mode = 1
   main(mode)
   rospy.spin()
