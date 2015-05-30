import rospy

import actionlib
from jsk_teleop_joy.joy_plugin import JSKJoyPlugin
from drc_task_common.srv import StringRequest, StringRequestResponse

try:
  imp.find_module("std_msgs")
except:
  import roslib; roslib.load_manifest('jsk_teleop_joy')


from std_msgs.msg import String, Float64, Bool, Float32
from std_srvs.srv import Empty, EmptyResponse
from geometry_msgs.msg import PoseStamped
import xml.etree.ElementTree as ET
import os, sys, time, numpy

class VehicleJoyController(JSKJoyPlugin):
  def __init__(self, name, args):
    JSKJoyPlugin.__init__(self, name, args)
    self.command_states = {
      "handle": HandleCommandState("handle", Float64, Float32, "drive/controller/goal_handle_angle"),
      "accel": VehicleCommandState("accel", Float64, Float32, "drive/controller/step"),
      "brake": VehicleCommandState("brake", Float64, None, None), # do not support brake synchronize
      "neck_y": VehicleCommandState("neck_y", Float64, Float32, "drive/controller/neck_y_angle"),
      "neck_p": VehicleCommandState("neck_p", Float64, Float32, "drive/controller/neck_p_angle")
      }
    self.synchronizeAllCommand()
    self.execute_flag = False
    print >> sys.stderr, "Joystick initialization is finished"
    self.initialize_service = rospy.Service('drive/operation/initialize', Empty, self.initializeServiceCallback)
    self.synchronize_service = rospy.Service('drive/operation/synchronize', StringRequest, self.synchronizeServiceCallback)
    self.subscriber = rospy.Subscriber('drive/execute_flag', Bool, self.executeFlagCallback)

  def joyCB(self, status, history):
    latest = history.latest()
    handle_resolution = 0.025 # rad
    neck_y_resolution = 0.1 # deg
    neck_y_angle_max = 35.0
    neck_y_angle_min = -35.0
    neck_p_resolution = 0.1 # deg
    neck_p_angle_max = 35.0
    neck_p_angle_min = -35.0
    max_accel_resolution = 0.05
    max_brake_resolution = 1.0

    if not self.execute_flag or not latest:
      return
    
    # handle command
    if status.left_analog_x:
      self.command_states["handle"].command = self.command_states["handle"].command + handle_resolution * status.left_analog_x
    # neck_y command
    if status.left:
      self.command_states["neck_y"].command = self.commandJointAngle(self.command_states["neck_y"].command, neck_y_resolution, neck_y_angle_max, neck_y_angle_min)
    elif status.right:
      self.command_states["neck_y"].command = self.commandJointAngle(self.command_states["neck_y"].command, -neck_y_resolution, neck_y_angle_max, neck_y_angle_min)
    # neck_p command (head goes down when neck_p incleases)
    if status.up:
      self.command_states["neck_p"].command = self.commandJointAngle(self.command_states["neck_p"].command, -neck_p_resolution, neck_p_angle_max, neck_p_angle_min)
    elif status.down:
      self.command_states["neck_p"].command = self.commandJointAngle(self.command_states["neck_p"].command, neck_p_resolution, neck_p_angle_max, neck_p_angle_min)
    # accel command
    if status.right_analog_y:
      self.command_states["accel"].command = max(status.right_analog_y, 0.0)
      if status.right_analog_y < -0.5:
        self.command_states["brake"].command = 1.0
      else:
        self.command_states["brake"].command = 0.0
    else:
      self.command_states["accel"].command = 0.0
      self.command_states["brake"].command = 0.0

    for command in self.command_states.values():
      command.publishCommand()
    
  def commandJointAngle(self, current_value, resolution, max_value, min_value): # max_value assumed to be positive
    next_value = current_value + resolution
    if next_value > max_value:
      next_value = max_value
    elif next_value < min_value:
      next_value = min_value
    return next_value

  def initializeAllCommand(self):
    for command in self.command_states.values():
      command.initialize()

  def synchronizeCommand(self, key):
    print >> sys.stderr, "Sync " + key
    self.command_states[key].synchronize()

  def synchronizeAllCommand(self):
    for key in self.command_states.keys():
      self.synchronizeCommand(key)

  def executeFlagCallback(self, msg):
    self.execute_flag = msg.data
    print >> sys.stderr, "set execute_flag as " + str(self.execute_flag)

  def initializeServiceCallback(self, req):
    print >> sys.stderr, "initialize vehicle joy"
    self.initializeAllCommand()
    return EmptyResponse()
      
  def synchronizeServiceCallback(self, req):
    key = req.data.lower()
    if key in self.command_states:
      self.synchronizeCommand(key)
    elif key == "neck":
      self.synchronizeCommand("neck_p")
      self.synchronizeCommand("neck_y")
    elif key == "all":
      self.synchronizeAllCommand()
    else:
      print >> sys.stderr, "Invalid key"
    return StringRequestResponse()

class VehicleCommandState():
  def __init__(self, command_name, pub_type, sub_type, robot_topic):
    self.command_name = command_name
    self.pub_type = pub_type
    self.sub_type = sub_type
    self.wait_for_message_flag = True
    self.robot_topic = robot_topic
    self.publisher = rospy.Publisher("drive/operation/" + self.command_name + "_cmd_fast", self.pub_type)
    if robot_topic and self.sub_type:
      self.subscriber = rospy.Subscriber(robot_topic, self.sub_type, self.callback)
    else:
      self.subscriber = None
    self.initialize()
  def callback(self, msg):
    self.robot_value = msg.data
  def initialize(self):
    self.command = 0.0
    self.robot_value = 0.0
  def synchronize(self, timeout = 1.0):
    if self.robot_topic != None and self.sub_type != None:
      try:
        if self.wait_for_message_flag:
          rospy.wait_for_message(self.robot_topic, self.sub_type, timeout)
          self.wait_for_message_flag = False
        print >> sys.stderr, "Sync with " + self.robot_topic
        print "%s -> %s" % (str(self.command), str(self.robot_value))
        self.command = self.robot_value
      except rospy.ROSException, e:
        print >> sys.stderr, "Cannot subscribe " + self.robot_topic
  def publishCommand(self):
    pub_msg = self.pub_type(data = self.command)
    self.publisher.publish(pub_msg)

class HandleCommandState(VehicleCommandState):
  def callback(self, msg):
    self.robot_value = msg.data * numpy.pi / 180.0 # goal_handle_angle[deg] -> command[rad]
  # def synchronize(self):
  #   self.command = self.robot_value * numpy.pi / 180.0 # robot_value[deg] -> command[rad]
    
