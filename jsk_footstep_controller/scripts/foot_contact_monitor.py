#!/usr/bin/env python

import rospy
from jsk_footstep_msgs.msg import ExecFootstepsAction
from jsk_footstep_controller.msg import GroundContactState
from jsk_footstep_controller.srv import RequireMonitorStatus, RequireMonitorStatusResponse
from threading import Lock
from math import pi
import actionlib

g_lock = Lock()
g_contact_state = None

def requireMonitorCallback(req):
    requested_stamp = req.header.stamp
    # wait for newer message
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        with g_lock:
            if g_contact_state and requested_stamp < g_contact_state.header.stamp:
                break
            else:
                rospy.loginfo("Waiting for newer message")
        r.sleep()
    with g_lock:
        if abs(g_contact_state.error_pitch_angle) > req.threshold:
            rospy.loginfo("error: %f > %f" % (g_contact_state.error_pitch_angle,
                                              req.threshold))
            return RequireMonitorStatusResponse(False)
        else:
            rospy.loginfo("error: %f < %f" % (g_contact_state.error_pitch_angle,
                                              req.threshold))
            return RequireMonitorStatusResponse(True)

def contactGroundCallback(msg):
    global g_contact_state
    with g_lock:
        g_contact_state = msg

def main():
    global pitch_threshold, controller_ac
    #pitch_threshold = rospy.get_param("~pitch_threshold", 2 * pi / 180.0)
    pitch_threshold = rospy.get_param("~pitch_threshold", 0.02)
    # controller_ac = actionlib.SimpleActionClient(
    #     'footstep_controller', ExecFootstepsAction)
    sub = rospy.Subscriber("/footcoords/contact_state", GroundContactState, contactGroundCallback)
    srv = rospy.Service("require_foot_contact_monitor", RequireMonitorStatus, requireMonitorCallback)
    rospy.spin()

if __name__ == "__main__":
    rospy.init_node("foot_contact_monitor")
    main()

