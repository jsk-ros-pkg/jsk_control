#!/usr/bin/env python

import rospy
import roslib


from geometry_msgs.msg import PoseArray
from jsk_footstep_msgs.msg import FootstepArray


def cb(msg):
  global p
  array = PoseArray()
  array.header = msg.header
  for footstep in msg.footsteps:
    array.poses.append(footstep.pose)
  p.publish(array)

rospy.init_node("foo")

p = rospy.Publisher("pose_array", PoseArray)
s = rospy.Subscriber("/footstep", FootstepArray, cb)

rospy.spin()
