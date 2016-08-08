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
from jsk_rviz_plugins.msg import OverlayMenu
from plugin_manager import PluginManager
from status_history import StatusHistory
from diagnostic_updater import Updater as DiagnosticUpdater

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
  STATE_INITIALIZATION = 1
  STATE_RUNNING = 2
  STATE_WAIT_FOR_JOY = 3

  MODE_PLUGIN = 0
  MODE_MENU = 1
  mode = 0
  
  plugin_instances = []
  def stateDiagnostic(self, stat):
    if self.state == self.STATE_INITIALIZATION:
      stat.summary(DiagnosticStatus.WARN,
                   "initializing JoyManager")
    elif self.state == self.STATE_RUNNING:
      stat.summary(DiagnosticStatus.OK,
                   "running")
      stat.add("Joy stick type", str(self.JoyStatus))
    elif self.state == self.STATE_WAIT_FOR_JOY:
      stat.summary(DiagnosticStatus.WARN,
                   "waiting for joy message to detect joy stick type")
    return stat
  def pluginStatusDiagnostic(self, stat):
    if len(self.plugin_instances) == 0:
        stat.summary(DiagnosticStatus.ERROR, "no plugin is loaded")
    else:
        stat.summary(DiagnosticStatus.OK, 
                     "%d plugins are loaded" % (len(self.plugin_instances)))
        stat.add("instances", ", ".join([p.name for p in self.plugin_instances]))
    return stat
  def __init__(self, plugin_package="jsk_teleop_joy"):
    self.state = self.STATE_INITIALIZATION
    self.pre_status = None
    self.history = StatusHistory(max_length=10)
    self.menu_pub = rospy.Publisher("/overlay_menu", OverlayMenu)
    self.controller_type = rospy.get_param('~controller_type', 'auto')
    self.plugins = rospy.get_param('~plugins', {})
    self.current_plugin_index = 0
    self.selecting_plugin_index = 0
    #you can specify the limit of the rate via ~diagnostic_period
    self.diagnostic_updater = DiagnosticUpdater()
    self.diagnostic_updater.setHardwareID("teleop_manager")
    self.diagnostic_updater.add("State", self.stateDiagnostic)
    self.diagnostic_updater.add("Plugin Status", self.pluginStatusDiagnostic)
    #self.diagnostic_updater.add("Joy Input", self.joyInputDiagnostic)
    self.diagnostic_updater.update()
    if self.controller_type == 'xbox':
      self.JoyStatus = XBoxStatus
    elif self.controller_type == 'ps3':
      self.JoyStatus = PS3Status
    elif self.controller_type == 'ps3wired':
      self.JoyStatus = PS3WiredStatus
    elif self.controller_type == 'auto':
      s = rospy.Subscriber('/joy', Joy, autoJoyDetect)
      self.state = self.STATE_WAIT_FOR_JOY
      error_message_published = False
      r = rospy.Rate(1)
      while not rospy.is_shutdown():
        self.diagnostic_updater.update()
        if AUTO_DETECTED_CLASS == "UNKNOWN":
          if not error_message_published:
            rospy.logfatal("unknown joy type")
            error_message_published = True
          r.sleep()
        elif AUTO_DETECTED_CLASS:
          self.JoyStatus = AUTO_DETECTED_CLASS
          s.unregister()
          break
        else:
          r.sleep()
    self.diagnostic_updater.update()
    self.plugin_manager = PluginManager(plugin_package)
    self.loadPlugins()
  def loadPlugins(self):
    self.plugin_manager.loadPlugins()
    self.plugin_instances = self.plugin_manager.loadPluginInstances(self.plugins, self)
  def switchPlugin(self, index):
    self.current_plugin_index = index
    if len(self.plugin_instances) <= self.current_plugin_index:
      self.current_plugin_index = 0
    elif self.current_plugin_index < 0:
      self.current_plugin_index = len(self.plugin_instances)
    self.current_plugin.disable()
    self.current_plugin = self.plugin_instances[self.current_plugin_index]
    self.current_plugin.enable()
  def nextPlugin(self):
    rospy.loginfo('switching to next plugin')
    self.switchPlugin(self, self.current_plugin_index + 1)
  def start(self):
    self.publishMenu(0, close=True) # close menu anyway
    self.diagnostic_updater.force_update()
    if len(self.plugin_instances) == 0:
      rospy.logfatal('no valid plugins are loaded')
      return False
    self.current_plugin = self.plugin_instances[0]
    self.current_plugin.enable()
    self.joy_subscriber = rospy.Subscriber('/joy', Joy, self.joyCB)
    self.state = self.STATE_RUNNING
    return True
  def publishMenu(self, index, close=False):
    menu = OverlayMenu()
    menu.menus = [p.name for p in self.plugin_instances]
    menu.current_index = index
    menu.title = "Joystick"
    if close:
      menu.action = OverlayMenu.ACTION_CLOSE
    self.menu_pub.publish(menu)
  def forceToPluginMenu(self, publish_menu=False):
    self.selecting_plugin_index = self.current_plugin_index
    if publish_menu:
      self.publishMenu(self.current_plugin_index)
    self.mode = self.MODE_MENU
  def processMenuMode(self, status, history):
    if history.new(status, "down") or history.new(status, "left_analog_down"):
      self.selecting_plugin_index = self.selecting_plugin_index + 1
      if self.selecting_plugin_index >= len(self.plugin_instances):
        self.selecting_plugin_index = 0
      self.publishMenu(self.selecting_plugin_index)
    elif history.new(status, "up") or history.new(status, "left_analog_up"):
      self.selecting_plugin_index = self.selecting_plugin_index - 1
      if self.selecting_plugin_index < 0:
        self.selecting_plugin_index = len(self.plugin_instances) - 1
      self.publishMenu(self.selecting_plugin_index)
    elif history.new(status, "cross") or history.new(status, "center"):
      self.publishMenu(self.selecting_plugin_index, close=True)
      self.mode = self.MODE_PLUGIN
    elif history.new(status, "circle"):
      self.publishMenu(self.selecting_plugin_index, close=True)
      self.mode = self.MODE_PLUGIN
      self.switchPlugin(self.selecting_plugin_index)
    else:
      self.publishMenu(self.selecting_plugin_index)
  def joyCB(self, msg):
    status = self.JoyStatus(msg)
    if self.mode == self.MODE_MENU:
      self.processMenuMode(status, self.history)
    else:
      if self.history.new(status, "center"):
        self.forceToPluginMenu()
      else:
        self.current_plugin.joyCB(status, self.history)
    self.pre_status = status
    self.history.add(status)
    self.diagnostic_updater.update()
    
def main():
  global g_manager
  rospy.sleep(1)
  rospy.init_node('jsk_teleop_joy')
  g_manager = JoyManager()
  result = g_manager.start()
  if not result:
    rospy.logfatal("Fatal Error")
    return False
  else:
    rospy.spin()
  
if __name__ == '__main__':
  main()
  
