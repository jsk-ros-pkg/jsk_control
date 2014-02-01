#!/usr/bin/env python

import roslib
import rospy

roslib.load_manifest("jsk_footstep_planner")

from jsk_pcl_ros.msg import PolygonArray
from geometry_msgs.msg import PolygonStamped, Point32

def makePolygonArray():
  global_frame_id = "/odom"
  points_array = [[[1, 1, 0], [-1, 1, 0], [-1, -1, 0], [1, -1, 0]],
                  [[1.5, 1, 0.2], [1, 1, 0.2], [1, -1, 0.2], [1.5, -1, 0.2]],
                  [[2, 1, 0], [1.5, 1, 0], [1.5, -1, 0], [2, -1, 0]]]
  ret = PolygonArray()
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
