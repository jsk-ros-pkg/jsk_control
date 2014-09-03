#!/usr/bin/env python

import rospy

try:
    import joy_mouse.xinput
except:
    import roslib; roslib.load_manifest("joy_mouse")
    import joy_mouse.xinput

if __name__ == "__main__":
    rospy.init_node("enabale_mouse")
    joy_mouse.xinput.enableDeviceByRegexp(rospy.myargv()[1])
