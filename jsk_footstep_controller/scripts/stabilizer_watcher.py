#!/usr/bin/env python

# A script to watch stabilizer status

from hrpsys_ros_bridge.srv import OpenHRP_StabilizerService_getParameter as getParameter
import rospy
from std_msgs.msg import Empty
from sound_play.msg import SoundRequest

# global variable
g_previous_st_controller_mode = None

def controllerModeToString(mode):
    if mode == 0:
        return "MODE_IDLE"
    elif mode == 1:
        return "MODE_AIR"
    elif mode == 2:
        return "MODE_ST"
    elif mode == 3:
        return "MODE_SYNC_TO_IDLE"
    elif mode == 4:
        return "MODE_SYNC_TO_AIR"

def isChangedControllerMode(actual_from, actual_to, expected_from, expected_to):
    if (actual_from in expected_from and
        actual_to == expected_to):
        return True
    else:
        return False

def trig():
    g_odom_init_trigger_pub.publish(Empty())
    # Say something
    sound = SoundRequest()
    sound.sound = SoundRequest.SAY
    sound.command = SoundRequest.PLAY_ONCE
    sound.arg = "Robot stans on the ground."
    g_robotsound_pub.publish(sound)
    
    
def watch(event):
    global g_get_parameter_srv, g_previous_st_controller_mode
    global g_odom_init_trigger_pub, g_robotsound_pub
    st_param = g_get_parameter_srv().i_param
    controller_mode = controllerModeToString(st_param.controller_mode)
    if (controller_mode == "MODE_AIR" or 
        controller_mode == "MODE_IDLE" or 
        controller_mode == "MODE_ST"):
        if g_previous_st_controller_mode == None:
            g_previous_st_controller_mode = controller_mode
            if controller_mode == "MODE_ST":
                trig()
        else:
            if isChangedControllerMode(g_previous_st_controller_mode,
                                       controller_mode,
                                       ["MODE_AIR", "MODE_IDLE"],
                                       "MODE_ST"):
                trig()
            g_previous_st_controller_mode = controller_mode

if __name__ == "__main__":
    rospy.init_node("stabilizer_watcher")
    rate = rospy.get_param("~rate", 1.0) # 10Hz
    g_get_parameter_srv = rospy.ServiceProxy("/StabilizerServiceROSBridge/getParameter", getParameter)
    g_odom_init_trigger_pub = rospy.Publisher("/odom_init_trigger", Empty)
    g_robotsound_pub = rospy.Publisher("/robotsound", SoundRequest)
    timer = rospy.Timer(rospy.Duration(1.0 / rate), watch)
    rospy.spin()

