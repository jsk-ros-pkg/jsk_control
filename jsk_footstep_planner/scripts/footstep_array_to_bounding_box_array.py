#!/usr/bin/env python

import rospy
from jsk_footstep_msgs.msg import Footstep, FootstepArray
from jsk_recognition_msgs.msg import BoundingBox, BoundingBoxArray

def callback(msg):
    box_array = BoundingBoxArray()
    box_array.header = msg.header
    for footstep in msg.footsteps:
        box = BoundingBox()
        box.header = msg.header
        box.pose = footstep.pose
        box.dimensions = footstep.dimensions
        box.pose.position.z += (z_max + z_min) / 2.0
        box.dimensions.z = z_max - z_min
        box_array.boxes.append(box)
    pub.publish(box_array)

if __name__ == "__main__":
    rospy.init_node("footstep_array_to_bounding_box")
    z_max = rospy.get_param('~z_max',0.0005)
    z_min = rospy.get_param('~z_min',-0.0005)
    pub = rospy.Publisher("~output", BoundingBoxArray)
    sub = rospy.Subscriber("~input", FootstepArray, callback)
    rospy.spin()
    
