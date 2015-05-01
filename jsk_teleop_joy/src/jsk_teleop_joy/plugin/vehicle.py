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
    self.current_accel_val = 0.0
    self.current_brake_val = 0.0
    self.handle_publisher = rospy.Publisher("drive/operation/handle_cmd_fast", Float64)
    self.accel_publisher = rospy.Publisher("drive/operation/accel_cmd_fast", Float64)
    self.brake_publisher = rospy.Publisher("drive/operation/brake_cmd_fast", Float64)

  def joyCB(self, status, history):
    latest = history.latest()
    handle_resolution = 0.02
    accel_resolution = 0.01
    brake_resolution = 1.0

    if not latest:
      return

    # handle command
    if status.left:
      self.current_handle_val = self.current_handle_val + handle_resolution
    elif status.right:
      self.current_handle_val = self.current_handle_val - handle_resolution
    # accel command
    if status.circle:
      self.current_accel_val = self.current_accel_val + accel_resolution
      if self.current_accel_val > 1.0:
        self.current_accel_val = 1.0
    else:
      self.current_accel_val = self.current_accel_val - accel_resolution
      if self.current_accel_val < 0.0:
        self.current_accel_val = 0.0
    # brake command
    if status.cross:
      self.current_brake_val = self.current_brake_val + brake_resolution
      if self.current_brake_val > 1.0:
        self.current_brake_val = 1.0
    else:
      self.current_brake_val = self.current_brake_val - brake_resolution
      if self.current_brake_val < 0.0:
        self.current_brake_val = 0.0

    self.handle_publisher.publish(Float64(data = self.current_handle_val))
    self.accel_publisher.publish(Float64(data = self.current_accel_val))
    self.brake_publisher.publish(Float64(data = self.current_brake_val))

