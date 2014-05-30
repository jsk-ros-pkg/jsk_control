#!/usr/bin/env python

import math
import numpy

import rospy
import os
import sys

import imp
try:
  imp.find_module("sensor_msgs")
except:
  import roslib; roslib.load_manifest('jsk_teleop_joy')


from sensor_msgs.msg import Joy
from diagnostic_msgs.msg import DiagnosticStatus, DiagnosticArray
import tf.transformations
from joy_status import XBoxStatus, PS3Status, PS3WiredStatus
from plugin_manager import PluginManager
from status_history import StatusHistory


AUTO_DETECTED_CLASS = None

def autoJoyDetect(msg):
  global AUTO_DETECTED_CLASS
  if AUTO_DETECTED_CLASS:
    return
  if len(msg.axes) == 27 and len(msg.buttons) == 19:
    rospy.loginfo("auto detected as ps3wired")
    AUTO_DETECTED_CLASS = PS3WiredStatus
  elif len(msg.axes) == 8 and len(msg.buttons) == 11:
    rospy.loginfo("auto detected as xbox")
    AUTO_DETECTED_CLASS = XBoxStatus
  elif len(msg.axes) == 20 and len(msg.buttons) == 17:
    rospy.loginfo("auto detected as ps3")
    AUTO_DETECTED_CLASS = PS3Status
  else:
    AUTO_DETECTED_CLASS = "UNKNOWN"
    
class JoyManager():
  def publishDiagnostic(self, statuses):
    diagnostic = DiagnosticArray()
    diagnostic.header.stamp = rospy.Time.now()
    diagnostic.status = statuses
    self.diagnostic_pub.publish(diagnostic)
  def __init__(self):
    self.pre_status = None
    self.history = StatusHistory(max_length=10)
    self.controller_type = rospy.get_param('~controller_type', 'xbox')
    self.plugins = rospy.get_param('~plugins', [])
    self.current_plugin_index = 0
    self.diagnostic_pub = rospy.Publisher("/diagnostics", DiagnosticArray)
    if self.controller_type == 'xbox':
      self.JoyStatus = XBoxStatus
    elif self.controller_type == 'ps3':
      self.JoyStatus = PS3Status
    elif self.controller_type == 'ps3wired':
      self.JoyStatus = PS3WiredStatus
    elif self.controller_type == 'auto':
      s = rospy.Subscriber('/joy', Joy, autoJoyDetect)
      error_message_published = False
      while not rospy.is_shutdown():
        if AUTO_DETECTED_CLASS == "UNKNOWN":
          if not error_message_published:
            rospy.logfatal("unknown joy type")
            error_message_published = True
            # update diagnostic
          status = DiagnosticStatus(name = '%s: JSKTeleopJoy' % (rospy.get_name()), level = DiagnosticStatus.ERROR, message = "unknown joy type")
          self.publishDiagnostic([status])
          rospy.sleep(1)
        elif AUTO_DETECTED_CLASS:
          self.JoyStatus = AUTO_DETECTED_CLASS
          s.unregister()
          break
        else:
          rospy.sleep(1)
    self.plugin_manager = PluginManager('jsk_teleop_joy')
    self.loadPlugins()
  def loadPlugins(self):
    self.plugin_manager.loadPlugins()
    self.plugin_instances = self.plugin_manager.loadPluginInstances(self.plugins)
  def nextPlugin(self):
    rospy.loginfo('switching to next plugin')
    self.current_plugin_index = self.current_plugin_index + 1
    if len(self.plugin_instances) == self.current_plugin_index:
      self.current_plugin_index = 0
    self.current_plugin.disable()
    self.current_plugin = self.plugin_instances[self.current_plugin_index]
    self.current_plugin.enable()
  def start(self):
    if len(self.plugin_instances) == 0:
      rospy.logfatal('no valid plugins are loaded')
      return
    self.current_plugin = self.plugin_instances[0]
    self.current_plugin.enable()
    self.joy_subscriber = rospy.Subscriber('/joy', Joy, self.joyCB)
    
  def joyCB(self, msg):
    status = self.JoyStatus(msg)
    diag_status = DiagnosticStatus(name = '%s: JSKTeleopJoy' % (rospy.get_name()), level = DiagnosticStatus.OK)
    self.publishDiagnostic([diag_status])
    if self.pre_status and status.select and not self.pre_status.select:
      self.nextPlugin()
    else:
      self.current_plugin.joyCB(status, self.history)
    self.pre_status = status
    self.history.add(status)
    
def main():
  global g_manager
  rospy.sleep(1)
  rospy.init_node('jsk_teleop_joy')
  g_manager = JoyManager()
  g_manager.start()
  rospy.spin()
  
if __name__ == '__main__':
  main()
  
