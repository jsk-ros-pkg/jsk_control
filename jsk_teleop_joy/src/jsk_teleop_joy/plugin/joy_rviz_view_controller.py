# joy_rviz_view_controller

from jsk_teleop_joy.plugin.rviz_view_controller_singleton import RVizViewControllerManagerSingleton
from jsk_teleop_joy.joy_plugin import JSKJoyPlugin

import rospy

class RVizViewController(JSKJoyPlugin):
  '''
Usage:
Right Analog x/y: yaw/pitch of camera position

R3(Right Analog button): while holding down L3 button, buttons/sticks listed below work for RVizViewController
   L3: reset camera pose
   L1: set focus point to target pose of Pose6D
   Up/Down/Left/Right : move focus point x/y
   Left Analog y: move camera for eye direction (near/far)
   Left Analog x: move focus point for eye direction (near/far)
   R3 + L2 + R2: enable follow view mode

Args:
control_view [Boolean, default: True]: Use or not control rviz camera (this is for child class)

Note: code of joyCB is implemented in rviz_view_controller_singleton.py
      Use TabletViewController(jsk_rviz_plugin) in rviz (selected at 'Views' Panel)
  '''
  def __init__(self, name, args):
    JSKJoyPlugin.__init__(self, name, args)
    self.control_view = self.getArg('control_view', True)
    if not self.control_view:
      rospy.loginfo("Not using rviz view control")
      RVizViewControllerManagerSingleton.camera_pub.unregister()
      RVizViewControllerManagerSingleton.camera_sub.unregister()
  def joyCB(self, status, history):
    RVizViewControllerManagerSingleton.joyCB(status, history, self.pre_pose)
  def supportFollowView(self, val):
    RVizViewControllerManagerSingleton.support_follow_view = val
  def followView(self, val=None):
    if val != None:
      RVizViewControllerManagerSingleton.follow_view = val
    return RVizViewControllerManagerSingleton.follow_view
