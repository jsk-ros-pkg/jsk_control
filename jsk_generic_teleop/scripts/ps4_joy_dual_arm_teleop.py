#!/usr/bin/env python3
"""
PS4/PS5 joystick -> 6-DOF Joy (spacenav format) for jsk-generic-teleop.

Raw axis layout (PS4 DualShock4 / PS5 DualSense via joy_node):
  [0] Left  stick X   : left=-1,         right=+1
  [1] Left  stick Y   : down=-1,         up=+1
  [2] L2 analog       : not-pressed=+1,  fully-pressed=-1
  [3] Right stick X   : left=-1,         right=+1
  [4] Right stick Y   : down=-1,         up=+1
  [5] R2 analog       : not-pressed=+1,  fully-pressed=-1

Button layout:
  [0] Cross  (x)
  [1] Circle (o)
  [2] Square ([])
  [3] Triangle (^)
  [4] L1
  [5] R1

Output topics (6-DOF Joy, same convention as /spacenav/joy):
  /joy_larm   (same content as /joy_rarm)
  /joy_rarm

  axes[0] X     = left stick Y   (forward/back)
  axes[1] Y     = left stick X   (left/right)
  axes[2] Z     = R2 - L2        (R2=up/+1, L2=down/-1)
  axes[3] Roll  = Cross - Square (Cross=+1, Square=-1)
  axes[4] Pitch = right stick Y
  axes[5] Yaw   = right stick X

  buttons[0] = L1  (select larm / first arm in jsk-generic-teleop)
  buttons[1] = R1  (select rarm / second arm in jsk-generic-teleop)

All 6 DOF are output simultaneously (no mode switching),
matching the /spacenav/joy format consumed by jsk-generic-teleop
with :device-type :spacenav or :joy.
"""

import rospy
from sensor_msgs.msg import Joy

_LS_X = 0
_LS_Y = 1
_L2   = 2
_RS_X = 3
_RS_Y = 4
_R2   = 5

_BTN_CROSS    = 0
_BTN_SQUARE   = 3
_BTN_L1       = 4
_BTN_R1       = 5


class PS4JoyDualArmTeleop(object):
    def __init__(self):
        rospy.init_node('ps4_joy_dual_arm_teleop')

        self.idx_ls_x = rospy.get_param('~axis_ls_x', _LS_X)
        self.idx_ls_y = rospy.get_param('~axis_ls_y', _LS_Y)
        self.idx_rs_x = rospy.get_param('~axis_rs_x', _RS_X)
        self.idx_l2   = rospy.get_param('~axis_l2',   _L2)
        self.idx_r2   = rospy.get_param('~axis_r2',   _R2)
        self.idx_rs_y = rospy.get_param('~axis_rs_y', _RS_Y)

        self.btn_cross  = rospy.get_param('~btn_cross',  _BTN_CROSS)
        self.btn_square = rospy.get_param('~btn_square', _BTN_SQUARE)
        self.btn_l1     = rospy.get_param('~btn_l1',     _BTN_L1)
        self.btn_r1     = rospy.get_param('~btn_r1',     _BTN_R1)

        self._l2_rest = None
        self._r2_rest = None

        in_topic   = rospy.get_param('~input_joy',       '/joy')
        larm_topic = rospy.get_param('~output_joy_larm', '/joy_larm')
        rarm_topic = rospy.get_param('~output_joy_rarm', '/joy_rarm')

        self._pub_larm = rospy.Publisher(larm_topic, Joy, queue_size=1)
        self._pub_rarm = rospy.Publisher(rarm_topic, Joy, queue_size=1)
        rospy.Subscriber(in_topic, Joy, self._cb, queue_size=1)
        rospy.loginfo('ps4_joy_dual_arm_teleop: %s -> %s, %s',
                      in_topic, larm_topic, rarm_topic)
        rospy.spin()

    def _get(self, axes, idx):
        if 0 <= idx < len(axes):
            return float(axes[idx])
        return 0.0

    def _btn(self, buttons, idx):
        if 0 <= idx < len(buttons):
            return bool(buttons[idx])
        return False

    def _trigger(self, axes, idx, rest):
        span = rest - (-1.0)
        if span < 1e-6:
            return 0.0
        return max(0.0, min(1.0, (rest - self._get(axes, idx)) / span))

    def _cb(self, msg):
        axes    = msg.axes
        buttons = msg.buttons

        if self._l2_rest is None:
            self._l2_rest = self._get(axes, self.idx_l2)
        if self._r2_rest is None:
            self._r2_rest = self._get(axes, self.idx_r2)

        ls_x = self._get(axes, self.idx_ls_x)
        ls_y = self._get(axes, self.idx_ls_y)
        rs_x = self._get(axes, self.idx_rs_x)
        rs_y = self._get(axes, self.idx_rs_y)

        l2 = self._trigger(axes, self.idx_l2, self._l2_rest)
        r2 = self._trigger(axes, self.idx_r2, self._r2_rest)

        l1_held     = self._btn(buttons, self.btn_l1)
        r1_held     = self._btn(buttons, self.btn_r1)
        square_held = self._btn(buttons, self.btn_square)
        cross_held  = self._btn(buttons, self.btn_cross)

        out_axes = [
            ls_y,                                          # [0] X
            ls_x,                                          # [1] Y
            r2 - l2,                                       # [2] Z
            float(cross_held) - float(square_held),        # [3] Roll
            rs_y,                                          # [4] Pitch
            rs_x,                                          # [5] Yaw
        ]
        out_buttons = [int(l1_held), int(r1_held)]

        out         = Joy()
        out.header  = msg.header
        out.axes    = out_axes
        out.buttons = out_buttons

        self._pub_larm.publish(out)
        self._pub_rarm.publish(out)


if __name__ == '__main__':
    PS4JoyDualArmTeleop()
