#!/usr/bin/env python  
#-*- coding:utf-8 -*-

"""
#import roslib
#roslib.load_manifest('learning_tf')
"""
import rospy
import tf
import math

class MakeGet():
      def __init__(self):
          rospy.init_node("make_gettingTF")
          self.main()
      def main(self):
          br = tf.TransformBroadcaster()
          rate = rospy.Rate(2.0)
          inirot = tf.transformations.quaternion_from_euler(0,0,0)
          now = rospy.Time(0)
          listener = tf.TransformListener()
          
          try:
             listener.waitForTransform("/world_vive", "/lighthouse2", now, rospy.Duration(1.0))
             (trans,rot) = listener.lookupTransform("/world_vive", "/lighthouse2", now)
             print "getting complete"
          except (tf.LookupException, tf.ConnectivityException, tf.ExtrapokationException):
             print "error"
          listener.waitForTransform("/world_vive", "/world", now, rospy.Duration(1.0))
          while not rospy.is_shutdown():
                br.sendTransform((0,0,0),
                                  inirot,
                                  now,
                                  "/purpose",
                                  "/world")
#                rate.sleep()

if __name__ == '__main__':
   MakeGet()
