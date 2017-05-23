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
               |-target
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
              self.listener.waitForTransform("/world",      "/world_vive",  self.now, rospy.Duration(10.0))
              self.listener.waitForTransform("/world_vive", "/lighthouse1", self.now, rospy.Duration(10.0))
              self.listener.waitForTransform("/world_vive", "/lighthouse2", self.now, rospy.Duration(10.0))
              self.listener.waitForTransform("/world_vive", "/hmd",         self.now, rospy.Duration(10.0))
              self.listener.waitForTransform("/world_vive", "/controller1", self.now, rospy.Duration(10.0))
              self.listener.waitForTransform("/world_vive", "/controller2", self.now, rospy.Duration(10.0))
          except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
             print "error"
          print "connection complete"

          self.main()

      def main(self):
          
          print "broadcasting start"

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
             except:
                pass
             """
             try:
                (lh1_ts,lh1_rot) = self.listener.lookupTransform("/world_vive", "/lighthouse1", self.now)
                self.br.sendTransform((lh1_ts[0], lh1_ts[1], lh1_ts[2] ),
                                      (lh1_rot[0],lh1_rot[1],lh1_rot[2],lh1_rot[3]),
                                      self.now,
                                      "/light1",
                                      "/vive_base")
             except:
                pass
             try:
                (lh2_ts,lh2_rot) = self.listener.lookupTransform("/world_vive", "/lighthouse2", self.now)
                self.br.sendTransform((lh2_ts[0], lh2_ts[1], lh2_ts[2] ),
                                      (lh2_rot[0],lh2_rot[1],lh2_rot[2],lh2_rot[3]),
                                      self.now,
                                      "/light2",
                                      "/vive_base")

             except:
                pass
             """
             try:
                (t_trans,t_rot) = self.listener.lookupTransform("/lighthouse1", "/lighthouse2", self.now)
                self.br.sendTransform((-t_trans[1], -1.0, t_trans[2]/3),
                                      (0, 0, 0, 1),
                                      self.now,
                                      "/target",
                                      "/vive_base")
             except:
                pass
             try:
                (hmd_ts,hmd_rot) = self.listener.lookupTransform("/world_vive", "/hmd", self.now)
                self.br.sendTransform((hmd_ts[0], hmd_ts[1], hmd_ts[2] ),
                                      (hmd_rot[0],hmd_rot[1],hmd_rot[2],hmd_rot[3]),
                                      self.now,
                                      "/hmd_head",
                                      "/vive_base")
             except:
                 pass
             try:
                (c1_ts,c1_rot) = self.listener.lookupTransform("/world_vive", "/controller1", self.now)
                self.br.sendTransform((c1_ts[0], c1_ts[1], c1_ts[2] ),
                                      (c1_rot[0],c1_rot[1],c1_rot[2],c1_rot[3]),
                                      self.now,
                                      "/r_hand",
                                      "/vive_base")
             except:
                 pass
             try:
                (c2_ts,c2_rot) = self.listener.lookupTransform("/world_vive", "/controller2", self.now)
                self.br.sendTransform((c2_ts[0], c2_ts[1], c2_ts[2] ),
                                      (c2_rot[0],c2_rot[1],c2_rot[2],c2_rot[3]),
                                      self.now,
                                      "/l_hand",
                                      "/vive_base")
             except:
                 pass

if __name__ == '__main__':
   MakeGet()
#   rospy.spin()
