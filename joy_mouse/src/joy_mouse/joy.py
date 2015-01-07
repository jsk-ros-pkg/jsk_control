#!/usr/bin/env python

import rospy
import struct
import select
import os
import pyudev

try:
    from sensor_msgs.msg import Joy
except:
    import roslib; roslib.load_manifest("joy_mouse")
    from sensor_msgs.msg import Joy

    #def main(device_name, rate, frame_id):

def lookupDeviceFileByNameAttribute(name, prefix="mouse"):
    """
    lookup device by name attribute. You can check name by
    udevadmin info -a -n /dev/input/mouse0
    """
    context = pyudev.Context()
    for device in context.list_devices(subsystem="input"):
        for a in device.attributes:
            if a == "name" and device.attributes[a] == name:
                for child in device.children:
                    if child.sys_name.startswith(prefix):
                        return os.path.join("/dev", "input", child.sys_name)
    
def main(device_name, autorepeat_rate, frame_id):
    rospy.loginfo("reading %s" % (device_name))
    joy_pub = rospy.Publisher('joy', Joy)
    with open(device_name, "rb" ) as tp_file:
        prev_bLeft = 0
        prev_bMiddle = 0
        prev_bRight = 0
        while not rospy.is_shutdown():
            if autorepeat_rate == 0:
                buf = tp_file.read(3)
            else:
                r,w,e = select.select([tp_file], [], [], 1.0 / autorepeat_rate)
                if tp_file in r:
                    buf = os.read(tp_file.fileno(), 3)
                else:
                    buf = None
            if buf == None:
                x, y = (0, 0)
                bLeft = prev_bLeft
                bMiddle = prev_bMiddle
                bRight = prev_bRight
            else:
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
            prev_bLeft = bLeft
            prev_bMiddle = bMiddle
            prev_bRight = bRight
