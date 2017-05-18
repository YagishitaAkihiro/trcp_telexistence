#!/usr/bin/env python  
#-*- coding:utf-8 -*-

import rospy
import tf
import math

"""
tf_tree

base-|
     vive_base-|
               |-lh1:実装しているが、出力してない
               |-lh2:実装しているが、出力してない
               |-target-|
                        |-hmd_head
                        |-r_hand
                        |-l_hand
のつもり。
"""

class MakeGet():
      def __init__(self):
          rospy.init_node("Rebuild_vive_TF")
          self.br = tf.TransformBroadcaster()
          self.listener = tf.TransformListener()
          self.now = rospy.Time(0)

          try:
              self.listener.waitForTransform("/world",      "/world_vive",  self.now, rospy.Duration(1.0))
              self.listener.waitForTransform("/world_vive", "/lighthouse1", self.now, rospy.Duration(1.0))
              self.listener.waitForTransform("/world_vive", "/lighthouse2", self.now, rospy.Duration(1.0))
              self.listener.waitForTransform("/world_vive", "/hmd",         self.now, rospy.Duration(1.0))
              self.listener.waitForTransform("/world_vive", "/controller1", self.now, rospy.Duration(1.0))
              self.listener.waitForTransform("/world_vive", "/controller2", self.now, rospy.Duration(1.0))
          except (tf.LookupException, tf.ConnectivityException, tf.ExtrapokationException):
             print "error"
          print "connection complete"

          self.main()

      def main(self):
          while not rospy.is_shutdown():
             self.br.sendTransform((0,0,0),
                               tf.transformations.quaternion_from_euler(0,0,0),
                               self.now,
                               "/base",
                               "/world")
             try:
                (wld_ts,wld_rot) = self.listener.lookupTransform("/world", "/world_vive", self.now)
                self.br.sendTransform((wld_ts[0], wld_ts[1], wld_ts[2] ),
                                      (wld_rot[0],wld_rot[1],wld_rot[2],wld_rot[3]),
                                      self.now,
                                      "/vive_base",
                                      "/base")
                print "wld_ok"
             except:
                pass
             """
             try:
                (lh1_ts,lh1_rot) = self.listener.lookupTransform("/world_vive", "/lighthouse1", self.now)
                print "lh1_ok"
             except:
                pass
             try:
                (lh2_ts,lh2_rot) = self.listener.lookupTransform("/world_vive", "/lighthouse2", self.now)
                print "lh2_ok"
             except:
                pass
             try:
                (hmd_ts,hmd_rot) = self.listener.lookupTransform("/world_vive", "/hmd", self.now)
                print "hmd_ok"
             except:
                 pass
             try:
                (c1_ts,c1_rot) = self.listener.lookupTransform("/world_vive", "/controller1", self.now)
                print "c1_ok"
             except:
                 pass
             try:
                (c2_ts,c2_rot) = self.listener.lookupTransform("/world_vive", "/controller2", self.now)
                print "c2_ok"
             except:
                 pass
"""
          """
          while not rospy.is_shutdown():
                br.sendTransform((0,0,0),
                                  inirot,
                                  now,
                                  "/purpose",
                                  "/world")
#                rate.sleep()
"""
if __name__ == '__main__':
   MakeGet()
#   rospy.spin()
