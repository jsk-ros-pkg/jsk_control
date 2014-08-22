#!/usr/bin/env python

import rospy
import struct
try:
    from sensor_msgs.msg import Joy
except:
    import roslib; roslib.load_manifest("joy_mouse")
    from sensor_msgs.msg import Joy

def main(device_name, rate, frame_id):
    rospy.loginfo("reading %s" % (device_name))
    joy_pub = rospy.Publisher('joy', Joy)
    with open(device_name, "rb" ) as tp_file:
        r = rospy.Rate(rate)
        while not rospy.is_shutdown():
            buf = tp_file.read(3)
            button = ord( buf[0] )
            bLeft = button & 0x1
            bMiddle = ( button & 0x4 ) > 0
            bRight = ( button & 0x2 ) > 0
            x,y = struct.unpack( "bb", buf[1:] )
            joy_msg = Joy()
            joy_msg.header.stamp = rospy.Time.now()
            joy_msg.header.frame_id = frame_id
            joy_msg.axes = [x, y]
            joy_msg.buttons = [bLeft, bMiddle, bRight]
            joy_pub.publish(joy_msg)
            r.sleep()

