import rospy

import actionlib
from jsk_teleop_joy.joy_plugin import JSKJoyPlugin
from drc_task_common.srv import SetValue

try:
  imp.find_module("std_msgs")
except:
  import roslib; roslib.load_manifest('jsk_teleop_joy')


from std_msgs.msg import String, Empty, Float64, Bool, Float32
from geometry_msgs.msg import PoseStamped
import xml.etree.ElementTree as ET

class VehicleJoyController(JSKJoyPlugin):
  def __init__(self, name, args):
    JSKJoyPlugin.__init__(self, name, args)
    self.current_handle_val = 0.0
    self.current_accel_val = 0.0
    self.current_brake_val = 0.0
    self.current_neck_y_val = 0.0
    self.current_neck_p_val = 0.0

    self.handle_publisher = rospy.Publisher("drive/operation/handle_cmd_fast", Float64)
    self.accel_publisher = rospy.Publisher("drive/operation/accel_cmd_fast", Float64)
    self.brake_publisher = rospy.Publisher("drive/operation/brake_cmd_fast", Float64)
    self.neck_y_publisher = rospy.Publisher("drive/operation/neck_y_cmd_fast", Float64)
    self.neck_p_publisher = rospy.Publisher("drive/operation/neck_p_cmd_fast", Float64)

  def joyCB(self, status, history):
    latest = history.latest()
    handle_resolution = 0.025 # rad
    neck_y_resolution = 0.1 # deg
    neck_y_angle_max = 30.0
    neck_p_resolution = 0.1 # deg
    neck_p_angle_max = 20.0
    max_accel_resolution = 0.05
    max_brake_resolution = 1.0

    if not latest:
      return
    
    # handle command
    if status.left_analog_x:
      self.current_handle_val = self.current_handle_val + handle_resolution * status.left_analog_x
    # neck_y command
    if status.left:
      self.current_neck_y_val = self.commandJointAngle(self.current_neck_y_val, neck_y_resolution, neck_y_angle_max)
    elif status.right:
      self.current_neck_y_val = self.commandJointAngle(self.current_neck_y_val, -neck_y_resolution, neck_y_angle_max)
    # neck_p command (command is incleased by up and decleased by down. positive and negative direction of angle is considerd by eus)
    if status.up:
      self.current_neck_p_val = self.commandJointAngle(self.current_neck_p_val, neck_p_resolution, neck_p_angle_max)
    elif status.down:
      self.current_neck_p_val = self.commandJointAngle(self.current_neck_p_val, -neck_p_resolution, neck_p_angle_max)
    # accel command
    if status.right_analog_y:
      self.current_accel_val = max(status.right_analog_y, 0.0)
      if status.right_analog_y < -0.9:
        self.current_brake_val = 1.0
      else:
        self.current_brake_val = 0.0
    else:
      self.current_accel_val = 0.0
      self.current_brake_val = 0.0

    self.handle_publisher.publish(Float64(data = self.current_handle_val))
    self.accel_publisher.publish(Float64(data = self.current_accel_val))
    self.brake_publisher.publish(Float64(data = self.current_brake_val))
    self.neck_y_publisher.publish(Float64(data = self.current_neck_y_val))
    self.neck_p_publisher.publish(Float64(data = self.current_neck_p_val))
    
  def commandJointAngle(self, current_value, resolution, max_value): # max_value assumed to be positive
      next_value = current_value + resolution
      if next_value > max_value:
        next_value = max_value
      elif next_value < -max_value:
        next_value = -max_value
      return next_value
