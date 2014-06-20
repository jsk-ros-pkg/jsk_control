#!/usr/bin/env python

import rospy
import roslib

try:
    from sensor_msgs.msg import Joy
except:
    import roslib; roslib.load_manifest("jsk_teleop_joy")
    from sensor_msgs.msg import Joy

class XBoxStatus():
    def __init__(self, msg):
        if msg.buttons[6] == 1:
            self.select = True
        else:
            self.select = False
        if msg.buttons[7] == 1:
            self.start = True
        else:
            self.start = False
        if msg.buttons[9] == 1:
            self.L3 = True
        else:
            self.L3 = False
        if msg.buttons[10] == 1:
            self.R3 = True
        else:
            self.R3 = False
        if msg.buttons[2] == 1:
            self.square = True
        else:
            self.square = False
        if msg.buttons[1] == 1:
            self.circle = True
        else:
            self.circle = False
        if msg.axes[7] > 0.1:
            self.up = True
        else:
            self.up = False
        if msg.axes[7] < -0.1:
            self.down = True
        else:
            self.down = False
        if msg.axes[6] > 0.1:
            self.left = True
        else:
            self.left = False
        if msg.axes[6] < -0.1:
            self.right = True
        else:
            self.right = False
        if msg.buttons[3] == 1:
            self.triangle = True
        else:
            self.triangle = False
        if msg.buttons[0] == 1:
            self.cross = True
        else:
            self.cross = False
        if msg.buttons[4] == 1:
            self.L1 = True
        else:
            self.L1 = False
        if msg.buttons[5] == 1:
            self.R1 = True
        else:
            self.R1 = False
        if msg.axes[2] < -0.5:
            self.L2 = True
        else:
            self.L2 = False
        if msg.axes[5] < -0.5:
            self.R2 = True
        else:
            self.R2 = False
        self.left_analog_x = msg.axes[0]
        self.left_analog_y = msg.axes[1]
        self.right_analog_x = msg.axes[3]
        self.right_analog_y = msg.axes[4]
        self.orig_msg = msg

class PS3Status():
    def __init__(self, msg):
        # creating from sensor_msgs/Joy
        if msg.buttons[0] == 1:
            self.select = True
        else:
            self.select = False
        if msg.buttons[3] == 1:
            self.start = True
        else:
            self.start = False
        if msg.buttons[1] == 1:
            self.L3 = True
        else:
            self.L3 = False
        if msg.buttons[2] == 1:
            self.R3 = True
        else:
            self.R3 = False
        if msg.axes[15] < 0:
            self.square = True
        else:
            self.square = False
        if msg.axes[4] < 0:
            self.up = True
        else:
            self.up = False
        if msg.axes[6] < 0:
            self.down = True
        else:
            self.down = False
        if msg.axes[7] < 0:
            self.left = True
        else:
            self.left = False
        if msg.axes[5] < 0:
            self.right = True
        else:
            self.right = False
        if msg.axes[12] < 0:
            self.triangle = True
        else:
            self.triangle = False
        if msg.axes[14] < 0:
            self.cross = True
        else:
            self.cross = False
        if msg.axes[13] < 0:
            self.circle = True
        else:
            self.circle = False
        if msg.axes[10] < 0:
            self.L1 = True
        else:
            self.L1 = False
        if msg.axes[11] < 0:
            self.R1 = True
        else:
            self.R1 = False
        if msg.axes[8] < 0:
            self.L2 = True
        else:
            self.L2 = False
        if msg.axes[9] < 0:
            self.R2 = True
        else:
            self.R2 = False
        self.left_analog_x = msg.axes[0]
        self.left_analog_y = msg.axes[1]
        self.right_analog_x = msg.axes[2]
        self.right_analog_y = msg.axes[3]
        self.orig_msg = msg

class PS3WiredStatus():
    def __init__(self, msg):
        # creating from sensor_msgs/Joy
        if msg.buttons[0] == 1:
            self.select = True
        else:
            self.select = False
        if msg.buttons[3] == 1:
            self.start = True
        else:
            self.start = False
        if msg.buttons[1] == 1:
            self.L3 = True
        else:
            self.L3 = False
        if msg.buttons[2] == 1:
            self.R3 = True
        else:
            self.R3 = False
        if msg.buttons[15] == 1:
            self.square = True
        else:
            self.square = False
        if msg.buttons[4] == 1:
            self.up = True
        else:
            self.up = False
        if msg.buttons[6] == 1:
            self.down = True
        else:
            self.down = False
        if msg.buttons[7] == 1:
            self.left = True
        else:
            self.left = False
        if msg.buttons[5] == 1:
            self.right = True
        else:
            self.right = False
        if msg.buttons[12] == 1:
            self.triangle = True
        else:
            self.triangle = False
        if msg.buttons[14] == 1:
            self.cross = True
        else:
            self.cross = False
        if msg.buttons[13] == 1:
            self.circle = True
        else:
            self.circle = False
        if msg.buttons[10] == 1:
            self.L1 = True
        else:
            self.L1 = False
        if msg.buttons[11] == 1:
            self.R1 = True
        else:
            self.R1 = False
        if msg.buttons[8] == 1:
            self.L2 = True
        else:
            self.L2 = False
        if msg.buttons[9] == 1:
            self.R2 = True
        else:
            self.R2 = False
        self.left_analog_x = msg.axes[0]
        self.left_analog_y = msg.axes[1]
        self.right_analog_x = msg.axes[2]
        self.right_analog_y = msg.axes[3]
        self.orig_msg = msg
    def toPS3Msg(self):
        joy = Joy()
        joy.header = self.orig_msg.header
        joy.buttons = [0] * 17
        joy.axes = [0] * 20
        if self.select:
            joy.buttons[0] = 1
        if self.start:
            joy.buttons[3] = 1
        if self.L3:
            joy.buttons[1] = 1
        if self.R3:
            joy.buttons[2] = 1
        if self.square:
            joy.axes[15] = -1.0
            joy.buttons[15] = 1
        if self.up:
            joy.axes[4] = -1.0
            joy.buttons[4] = 1
        if self.down:
            joy.axes[6] = -1.0
            joy.buttons[6] = 1
        if self.left:
            joy.axes[7] = -1.0
            joy.buttons[7] = 1
        if self.right:
            joy.axes[5] = -1.0
            joy.buttons[5] = 1
        if self.triangle:
            joy.axes[12] = -1.0
            joy.buttons[12] = 1
        if self.cross:
            joy.axes[14] = -1.0
            joy.buttons[14] = 1
        if self.circle:
            joy.axes[13] = -1.0
            joy.buttons[13] = 1
        if self.L1:
            joy.axes[10] = -1.0
            joy.buttons[10] = 1
        if self.R1:
            joy.axes[11] = -1.0
            joy.buttons[11] = 1
        if self.L2:
            joy.axes[8] = -1.0
            joy.buttons[8] = 1
        if self.R2:
            joy.axes[9] = -1.0
            joy.buttons[9] = 1
        joy.axes[0] = self.left_analog_x
        joy.axes[1] = self.left_analog_y
        joy.axes[2] = self.right_analog_x
        joy.axes[3] = self.right_analog_y
        return joy
