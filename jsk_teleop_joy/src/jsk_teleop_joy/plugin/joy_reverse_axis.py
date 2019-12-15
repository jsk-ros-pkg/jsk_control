from jsk_teleop_joy.joy_plugin import JSKJoyPlugin

from std_msgs.msg import String
from sensor_msgs.msg import Joy

import tf
import rospy
import numpy
import math
import time

class JoyReverseAxis(JSKJoyPlugin):
  '''
Usage:
This plugin reads in the analog axis value and reverses the selected ones.
It is supposed to be used together with the default joy teleop controls.

circle/cross/triangle: publish corresponding command

Args:
namespace [String, default: demojoy]: namespace for the new joy topic
reverse_lx_axis_mode [Boolean, default: True]: reverse left analog x value or not
reverse_ly_axis_mode [Boolean, default: True]: reverse left analog y value or not
reverse_rx_axis_mode [Boolean, default: False]: reverse right analog x value or not
reverse_ry_axis_mode [Boolean, default: False]: reverse right analog y value or not
command [String, default: command]: topic name for publishing the command
triangle_cmd' [String, default: TRIANGLE_CMD]: command text when triangle button is pressed
circle_cmd' [String, default: CIRCLE_CMD]: command text when circle button is pressed
cross_cmd' [String, default: CROSS_CMD]: command text when cross button is pressed
  '''
  #def __init__(self, name='JoyPose6D', publish_pose=True):
  def __init__(self, name, args):
    JSKJoyPlugin.__init__(self, name, args)
    self.new_joy = Joy()
    self.reverse_lx_axis_mode = self.getArg('reverse_lx_axis_mode', True)
    self.reverse_ly_axis_mode = self.getArg('reverse_ly_axis_mode', True)
    self.reverse_rx_axis_mode = self.getArg('reverse_rx_axis_mode', False)
    self.reverse_ry_axis_mode = self.getArg('reverse_ry_axis_mode', False)
    self.prev_time = rospy.Time.now()
    self.joy_pub = rospy.Publisher(self.getArg('namespace', 'demo_joy')+"/joy",
                                    Joy, queue_size = 20)
    self.command_pub = rospy.Publisher(self.getArg('command', 'command'),
                                    String, queue_size=1)
    self.triangle_cmd = self.getArg('triangle_cmd', 'TRIANGLE_CMD')
    self.cross_cmd = self.getArg('cross_cmd', 'CROSS_CMD')
    self.circle_cmd = self.getArg('circle_cmd', 'CIRCLE_CMD')
    
  def joyCB(self, status, history):
    self.new_joy = status.toPS3Msg()
    if history.length() > 0:
      latest = history.latest()
      if status.R3 and status.L2 and status.R2 and not (latest.R3 and latest.L2 and latest.R2):
        self.followView(not self.followView())
    # currently only support 2D plane movement
    if status.L1 and not status.R3:
      #L1 [10] [2]start/stop-grasp
      if self.reverse_lx_axis_mode:
        self.new_joy.axes[0] = - status.left_analog_x
      if self.reverse_ly_axis_mode:
        self.new_joy.axes[1] = - status.left_analog_y
      if self.reverse_rx_axis_mode:
        self.new_joy.axes[2] = - status.right_analog_x
      if self.reverse_ry_axis_mode:
        self.new_joy.axes[3] = - status.right_analog_y
    elif not status.R3:
      if status.circle and not latest.circle:
        self.command_pub.publish(self.circle_cmd)
      if status.triangle and not latest.triangle:
        self.command_pub.publish(self.triangle_cmd)
      if status.cross and not latest.cross:
        self.command_pub.publish(self.cross_cmd)

    # publish at 10hz
    now = rospy.Time.from_sec(time.time())
    # placement.time_from_start = now - self.prev_time
    if (now - self.prev_time).to_sec() > 1 / 30.0:
      self.joy_pub.publish(self.new_joy)
      self.prev_time = now
