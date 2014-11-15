#!/usr/bin/env python

import rospy
from jsk_footstep_controller.msg import GroundContactState
from jsk_rviz_plugins.msg import OverlayText

def callback(msg):
    text = OverlayText()
    if msg.contact_state == GroundContactState.CONTACT_BOTH_GROUND:
        text.text = "Double Stance Phase"
        text.fg_color.a = 1.0
        text.fg_color.r = 25 / 255.0
        text.fg_color.g = 1
        text.fg_color.b = 1
    elif msg.contact_state == GroundContactState.CONTACT_LLEG_GROUND or msg.contact_state == GroundContactState.CONTACT_RLEG_GROUND:
        text.text = "Single Stance Phase"
        text.fg_color.a = 1.0
        text.fg_color.r = 1
        text.fg_color.g = 0
        text.fg_color.b = 0
    elif msg.contact_state == GroundContactState.CONTACT_UNSTABLE:
        text.text = "Unstable"
        text.fg_color.a = 1.0
        text.fg_color.r = 1
        text.fg_color.g = 0
        text.fg_color.b = 0
    elif msg.contact_state == GroundContactState.CONTACT_AIR:
        text.text = "Air"
        text.fg_color.a = 1.0
        text.fg_color.r = 1
        text.fg_color.g = 0
        text.fg_color.b = 0
    pub.publish(text)

def main():
    global pub
    pub = rospy.Publisher("stance_phase_text", OverlayText)
    sub = rospy.Subscriber("/footcoords/contact_state", GroundContactState, callback)
    rospy.spin()


if __name__ == "__main__":
    rospy.init_node("stance_phase_text")
    main()
