#!/usr/bin/env python

import roslib
import rospy
import os
if os.environ["ROS_DISTRO"] == "groovy":
  roslib.load_manifest("jsk_footstep_planner")

from jsk_recognition_msgs.msg import PolygonArray
from geometry_msgs.msg import PolygonStamped, Point32

def makePolygonArray():
  global_frame_id = rospy.myargv()[1]
  ret = PolygonArray()
  ret.header.stamp = rospy.Time.now()
  ret.header.frame_id = global_frame_id

  for i in range(-1, 5):
    # i = 0,  1, 2, 3, 4, 5
    x1 = i * 3 - 0.5
    x2 = i * 3 + 1 - 0.5
    x3 = i * 3 + 2 - 0.5
    x4 = i * 3 + 3 - 0.5
    points_array = [[[x2, 2, 0],   [x1, 2, 0],   [x1, -2, 0],   [x2, -2, 0]],
                    [[x3, 2, 0.3], [x2, 2, 0.2], [x2, -2, 0.2], [x3, -2, 0.3]],
                    [[x4, 2, 0.1], [x3, 2, 0.1], [x3, -2, 0.1], [x4, -2, 0.1]]]
    for points in points_array:
      polygon = PolygonStamped()
      polygon.header.frame_id = global_frame_id
      polygon.header.stamp = rospy.Time.now()
      for p in points:
        polygon.polygon.points.append(Point32(*p))
      ret.polygons.append(polygon)
  return ret
            

def main():
  rospy.init_node("plane_publisher")
  
  pub = rospy.Publisher("planes", PolygonArray)
  while not rospy.is_shutdown():
    msg = makePolygonArray()
    pub.publish(msg)
    rospy.sleep(1)

if __name__ == "__main__":
  main()
