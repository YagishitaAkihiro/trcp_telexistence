#!/usr/bin/env python
#-*- coding:utf-8 -*-

import rospy
from std_msgs.msg import Float64
from moveit_msgs.msg import PlanningScene
from telexistence.msg import JointAngle

class Compare():
      def __init__(self):
          rospy.init_node("compare")
          global s,r
          s = open("sim.txt","w")
          r = open("real.txt","w")
          rospy.loginfo("wait for topic")
          rospy.wait_for_message("/move_group/monitored_planning_scene", PlanningScene, timeout=1000)
#          rospy.wait_for_message("/angle_data"                         , JointAngle,    timeout=1000)
          rospy.loginfo("subscribe topic")


          rospy.Subscriber("/move_group/monitored_planning_scene", PlanningScene, self.rviz_angle, queue_size=1)
#          rospy.Subscriber("/angle_data",                          JointAngle,    self.real_angle, queue_size=1)

      def rviz_angle(self,data):
          global s
          s.write(str(data.robot_state.joint_state.position)+"\n")

      def real_angle(self,data):
          global r
          r.write(str(data.torso) + " " + str(data.head) + " "  + str(data.rangle) + " " + str(data.langle) + "\n")

if __name__ == "__main__":
   Compare()
   rospy.spin()
   global s,r
   s.close()
   r.close()
