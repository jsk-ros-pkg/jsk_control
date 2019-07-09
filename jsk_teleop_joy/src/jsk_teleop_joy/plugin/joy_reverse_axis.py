from jsk_teleop_joy.joy_plugin import JSKJoyPlugin

import imp
try:
  imp.find_module("geometry_msgs")
except:
  import roslib; roslib.load_manifest('jsk_teleop_joy')

from std_msgs.msg import String
from sensor_msgs.msg import Joy
import tf
import rospy
import numpy
import math
import time

class JoyReverseAxis(JSKJoyPlugin):
  #def __init__(self, name='JoyPose6D', publish_pose=True):
  def __init__(self, name, args):
    JSKJoyPlugin.__init__(self, name, args)
    self.new_joy = Joy()
    self.new_joy.axes = [0]*20
    self.new_joy.buttons = [0]*17
    self.reverse_x_axis_mode = self.getArg('reverse_x_axis_mode', True)
    self.reverse_y_axis_mode = self.getArg('reverse_y_axis_mode', True)
    self.reverse_r_axis_mode = self.getArg('reverse_r_axis_mode', False)
    self.prev_time = rospy.Time.now()
    self.frame_id = self.getArg('frame_id', 'BODY')
    self.joy_pub = rospy.Publisher(self.getArg('namespace', 'demo_joy')+"/joy",
                                    Joy, queue_size = 20)
    self.command_pub = rospy.Publisher(self.getArg('command', 'command'),
                                    String, queue_size=1)
    
  def joyCB(self, status, history):
    if history.length() > 0:
      latest = history.latest()
      if status.R3 and status.L2 and status.R2 and not (latest.R3 and latest.L2 and latest.R2):
        self.followView(not self.followView())
    # currently only support 2D plane movement
    if status.L1 and not status.R3:
      #L1 [10] [2]start/stop-grasp
      self.new_joy.buttons[10] = 1
      self.check_button(status.triangle, 12)
      self.check_button(status.circle, 13)
      self.check_button(status.cross, 14)
      self.check_button(status.square, 15)
      if self.reverse_x_axis_mode:
        self.new_joy.axes[0] = - status.left_analog_x
      else:
        self.new_joy.axes[0] = status.left_analog_x
      if self.reverse_y_axis_mode:
        self.new_joy.axes[1] = - status.left_analog_y
      else:
        self.new_joy.axes[1] = status.left_analog_y
      if self.reverse_r_axis_mode:
        self.new_joy.axes[2] = - status.right_analog_x
      else:
        self.new_joy.axes[2] = status.right_analog_x
    elif not status.R3:
      self.new_joy.buttons[10] = 0
      if status.circle and not latest.circle:
        self.command_pub.publish("MANIP")
      if status.triangle and not latest.triangle:
        self.command_pub.publish("RELEASE")

    # publish at 10hz
    now = rospy.Time.from_sec(time.time())
    # placement.time_from_start = now - self.prev_time
    if (now - self.prev_time).to_sec() > 1 / 30.0:
      self.joy_pub.publish(self.new_joy)
      self.prev_time = now

  def check_button(self, state, index):
    if state:
      self.new_joy.axes[index] = -1.0
      self.new_joy.buttons[index] = 1
    else:
      self.new_joy.axes[index] = 0.0
      self.new_joy.buttons[index] = 0
