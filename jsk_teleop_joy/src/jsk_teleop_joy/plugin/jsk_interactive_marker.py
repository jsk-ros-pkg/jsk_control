import rospy
import actionlib
from joy_pose_6d import JoyPose6D
from std_msgs.msg import String, Empty
from geometry_msgs.msg import PoseStamped
import xml.etree.ElementTree as ET

class EndEffector(JoyPose6D):
  def __init__(self):
    JoyPose6D.__init__(self, name='JoyGoPos')
    self.support_follow_view = True
    self.frame_id = rospy.get_param('~frame_id', '/base_link')
  def joyCB(self, status, history):
    JoyPose6D.joyCB(self, status, history)
    
