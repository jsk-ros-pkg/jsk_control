import rospy
import actionlib
from joy_pose_6d import JoyPose6D
from std_msgs.msg import String, Empty
from geometry_msgs.msg import PoseStamped
import xml.etree.ElementTree as ET
from jsk_interactive_marker.msg import MarkerMenu
from jsk_rviz_plugins.msg import OverlayMenu

class EndEffector(JoyPose6D):
  mode = 0
  MENU_MODE = 1
  JOY_MODE = 2
  def __init__(self, name, args):
    JoyPose6D.__init__(self, name, args)
    self.support_follow_view = True
    self.frame_id = self.getArg('frame_id', 'base_link')
    self.marker_menu_pub = rospy.Publisher(self.getArg('marker_menu', 
                                                       'marker_menu'), 
                                                       MarkerMenu)
    self.menu_pub = rospy.Publisher("/overlay_menu", OverlayMenu)
    self.menus = ['LARM', 'RARM', 
                  'close hand', 'open hand', 'toggle ik rotation']
    self.mode = self.JOY_MODE
    self.current_index = 0
  def publishMenu(self, index, close=False):
    menu = OverlayMenu()
    menu.menus = self.menus
    menu.current_index = index
    menu.title = "JSK teleop menu"
    if close:
      menu.action = OverlayMenu.ACTION_CLOSE
    self.menu_pub.publish(menu)
  def publishMarkerMenu(self, menu):
    menu_msg = MarkerMenu()
    menu_msg.menu = menu
    self.marker_menu_pub.publish(menu_msg)
  def joyCB(self, status, history):
    if history.length() > 0:
      latest = history.latest()
    else:
      return

    if self.mode == self.MENU_MODE:
      if history.new(status, "triangle") or history.new(status, "cross"):
        self.mode = self.JOY_MODE
        self.publishMenu(self.current_index, True)
      elif history.new(status, "up")  or history.new(status, "left_analog_up"):
        self.current_index = self.current_index - 1
        if self.current_index < 0:
          self.current_index = len(self.menus) - 1
        self.publishMenu(self.current_index)
      elif history.new(status, "down") or history.new(status, "left_analog_down"):
        self.current_index = self.current_index + 1
        if self.current_index >= len(self.menus):
          self.current_index = 0
        self.publishMenu(self.current_index)
      elif history.new(status, "circle"):
        if self.menus[self.current_index] == "ARMS":
          self.publishMarkerMenu(MarkerMenu.SET_MOVE_ARMS)
        elif self.menus[self.current_index] == "RARM":
          self.publishMarkerMenu(MarkerMenu.SET_MOVE_RARM)
        elif self.menus[self.current_index] == "LARM":
          self.publishMarkerMenu(MarkerMenu.SET_MOVE_LARM)
        elif self.menus[self.current_index] == "close hand":
          self.publishMarkerMenu(MarkerMenu.START_GRASP)
        elif self.menus[self.current_index] == "open hand":
          self.publishMarkerMenu(MarkerMenu.STOP_GRASP)
        elif self.menus[self.current_index] == "toggle ik rotation":
          self.publishMarkerMenu(MarkerMenu.IK_ROTATION_AXIS_T)
        self.publishMenu(self.current_index, True)
        self.mode = self.JOY_MODE
      else:
        self.publishMenu(self.current_index)
    elif self.mode == self.JOY_MODE:
      if history.new(status, "triangle"):
        self.mode = self.MENU_MODE
      elif history.new(status, "circle"):
        self.publishMarkerMenu(MarkerMenu.MOVE)
      elif history.new(status, "start"):
        self.publishMarkerMenu(MarkerMenu.PLAN)
      elif history.new(status, "select"):
        self.publishMarkerMenu(MarkerMenu.RESET_JOINT)
      else:
        JoyPose6D.joyCB(self, status, history)
