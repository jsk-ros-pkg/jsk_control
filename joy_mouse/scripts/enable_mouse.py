#!/usr/bin/env python

import rospy

try:
    import joy_mouse.xinput
except:
    import roslib; roslib.load_manifest("joy_mouse")
    import joy_mouse.xinput
    
import joy_mouse.joy

if __name__ == "__main__":
    rospy.init_node("enabale_mouse")
    argv = rospy.myargv()
    if "--name" == argv[1]:      #usage is disable_mouse.py --name foo
        dev = joy_mouse.joy.lookupDeviceFileByNameAttribute(argv[2])
    else:
        dev = argv[1]
    joy_mouse.xinput.enableDeviceByRegexp(dev)
