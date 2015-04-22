import rospy

import actionlib
from jsk_teleop_joy.joy_plugin import JSKJoyPlugin
try:
  imp.find_module("std_msgs")
except:
  import roslib; roslib.load_manifest('jsk_teleop_joy')


from std_msgs.msg import String, Empty, Float64
from geometry_msgs.msg import PoseStamped
import xml.etree.ElementTree as ET

class VehicleJoyController(JSKJoyPlugin):
  def __init__(self, name, args):
    JSKJoyPlugin.__init__(self, name, args)
    self.current_handle_val = 0.0
    self.current_step_val = 0.0
    self.handle_publisher = rospy.Publisher("/drive/operation/handle_cmd_fast", Float64)
    self.step_publisher = rospy.Publisher("/drive/operation/accel_cmd_fast", Float64)
  def joyCB(self, status, history):
    latest = history.latest()
    handle_changed = False
    step_changed = False
    if not latest:
      return
    if status.left:
      self.current_handle_val = self.current_handle_val + 0.05
      handle_changed = True
    elif status.right:
      self.current_handle_val = self.current_handle_val - 0.05
      handle_changed = True
    if status.circle:
      self.current_step_val = self.current_step_val + 0.01
      if self.current_step_val > 1.0:
        self.current_step_val = 1.0
      step_changed = True
    elif status.cross:
      self.current_step_val = self.current_step_val - 0.01
      if self.current_step_val < 0.0:
        self.current_step_val = 0.0
      step_changed = True
    if handle_changed:
      self.handle_publisher.publish(Float64(data = self.current_handle_val))
    if step_changed:
      self.step_publisher.publish(Float64(data = self.current_step_val))
      
