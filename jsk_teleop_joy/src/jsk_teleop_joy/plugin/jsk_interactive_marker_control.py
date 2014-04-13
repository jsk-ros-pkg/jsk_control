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
  def joyCB(self, status, history):
    if status.circle:
      menu = MarkerMenu()
      menu.menu = MarkerMenu.MOVE
      self.marker_menu_pub.publish(menu)
      
    JoyPose6D.joyCB(self, status, history)
    
