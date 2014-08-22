#!/usr/bin/env python

import rospy

try:
    import joy_mouse.joy
except:
    import roslib; roslib.load_manifest("joy_mouse")
    import joy_mouse.joy
    
if __name__ == "__main__":
    rospy.init_node("joy_mouse")
    joy_mouse.joy.main(rospy.get_param("~dev", "/dev/input/mouse0"),
                       rospy.get_param("~rate", 30),
                       rospy.get_param("~frame_id", "mouse"))
    
