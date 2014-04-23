import rospy
import actionlib
from joy_pose_6d import JoyPose6D
from std_msgs.msg import String, Empty
from geometry_msgs.msg import PoseStamped
import xml.etree.ElementTree as ET
from jsk_interactive_marker.msg import MarkerMenu

class EndEffector(JoyPose6D):
  def __init__(self):
    JoyPose6D.__init__(self, name='EndEffector')
    self.support_follow_view = True
    self.frame_id = rospy.get_param('~frame_id', 'base_link')
    self.marker_menu_pub = rospy.Publisher('marker_menu', MarkerMenu)

  def publishMarkerMenu(self, menu):
    menu_msg = MarkerMenu()
    menu_msg.menu = menu
    self.marker_menu_pub.publish(menu_msg)


  def joyCB(self, status, history):
    if history.length() > 0:
      latest = history.latest()
    else:
      return

    #move
    if status.circle and not latest.circle:
      self.publishMarkerMenu(MarkerMenu.MOVE)

    elif status.triangle:
      #change move arm
      if status.R1 and status.L1 and not (latest.R1 and latest.L1):
        self.publishMarkerMenu(MarkerMenu.SET_MOVE_ARMS)
      elif status.R1 and not latest.R1:
        self.publishMarkerMenu(MarkerMenu.SET_MOVE_RARM)
      elif status.L1 and not latest.L1:
        self.publishMarkerMenu(MarkerMenu.SET_MOVE_LARM)

    else:
      JoyPose6D.joyCB(self, status, history)
