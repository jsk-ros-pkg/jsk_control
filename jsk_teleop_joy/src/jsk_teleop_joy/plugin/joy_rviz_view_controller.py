# joy_rviz_view_controller

from jsk_teleop_joy.plugin.rviz_view_controller_singleton import RVizViewControllerManagerSingleton
from jsk_teleop_joy.joy_plugin import JSKJoyPlugin

  
class RVizViewController(JSKJoyPlugin):
  def __init__(self, name, args):
    JSKJoyPlugin.__init__(self, name, args)
  def joyCB(self, status, history):
    RVizViewControllerManagerSingleton.joyCB(status, history, self.pre_pose)
  def supportFollowView(self, val):
    RVizViewControllerManagerSingleton.support_follow_view = val
  def followView(self, val=None):
    if val != None:
      RVizViewControllerManagerSingleton.follow_view = val
    return RVizViewControllerManagerSingleton.follow_view
    
