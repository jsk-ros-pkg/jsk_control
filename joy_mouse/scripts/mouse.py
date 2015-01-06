#!/usr/bin/env python

import rospy

try:
    import joy_mouse.joy
except:
    import roslib; roslib.load_manifest("joy_mouse")
    import joy_mouse.joy
    
if __name__ == "__main__":
    rospy.init_node("joy_mouse")
    device_name = rospy.get_param("~dev_name", False)
    if device_name:
        dev = lookupDeviceFileByNameAttribute(device_name)
    else:
        dev = rospy.get_param("~dev", "/dev/input/mouse0")
    joy_mouse.joy.main(dev,
                       rospy.get_param("~autorepeat_rate", 0),
                       rospy.get_param("~frame_id", "mouse"))
    
