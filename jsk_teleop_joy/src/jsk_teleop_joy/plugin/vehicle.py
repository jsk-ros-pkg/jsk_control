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
    self.max_step = 0.0
    self.min_step = 0.0
    self.handle_publisher = rospy.Publisher("drive/operation/handle_cmd_fast", Float64)
    self.accel_publisher = rospy.Publisher("drive/operation/accel_cmd_fast", Float64)
    self.brake_publisher = rospy.Publisher("drive/operation/brake_cmd_fast", Float64)
    self.neck_y_publisher = rospy.Publisher("drive/operation/neck_cmd_fast", Float64)

    self.set_min_step_sub = rospy.Subscriber("drive/controller/min_step", Float32, self.setMinStep)
    self.set_max_step_sub = rospy.Subscriber("drive/controller/max_step", Float32, self.setMaxStep)

  def joyCB(self, status, history):
    latest = history.latest()
    handle_resolution = 0.05
    neck_y_resolution = 0.1
    neck_y_angle_max = 30.0
    max_accel_resolution = 0.05
    max_brake_resolution = 1.0

    if not latest:
      return
    
    # handle command
    if status.left_analog_x:
      self.current_handle_val = self.current_handle_val + handle_resolution * status.left_analog_x
    # neck_y command
    if status.left:
      self.current_neck_y_val = self.current_neck_y_val + neck_y_resolution
      if self.current_neck_y_val > neck_y_angle_max:
        self.current_neck_y_val = neck_y_angle_max
    elif status.right:
      self.current_neck_y_val = self.current_neck_y_val - neck_y_resolution
      if self.current_neck_y_val < -neck_y_angle_max:
        self.current_neck_y_val = -neck_y_angle_max
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
      self.set_current_step_as_max_flag = False

    # set current step as max flag
    if status.triangle and not self.set_current_step_as_max_flag and self.current_accel_val > 0.0:
      current_value = (self.max_step - self.min_step) * self.current_accel_val + self.min_step
      try:
        update_value = rospy.ServiceProxy('drive/controller/set_max_step', SetValue)
        self.max_step = update_value(current_value)
        self.accel_publisher.publish(Float64(data = 0.0))
        self.set_current_step_as_max_flag = True
      except rospy.ServiceException, e:
        print "Failed to call drive/controller/set_max_step"
      
    self.handle_publisher.publish(Float64(data = self.current_handle_val))
    if not self.set_current_step_as_max_flag:
      self.accel_publisher.publish(Float64(data = self.current_accel_val))
    self.brake_publisher.publish(Float64(data = self.current_brake_val))
    self.neck_y_publisher.publish(Float64(data = self.current_neck_y_val))

  def setMinStep(self, msg):
    self.min_step = msg.data
  def setMaxStep(self, msg):
    self.max_step = msg.data
