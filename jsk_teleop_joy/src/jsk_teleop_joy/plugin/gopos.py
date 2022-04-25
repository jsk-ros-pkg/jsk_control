import rospy
import actionlib
from jsk_teleop_joy.joy_pose_6d import JoyPose6D

try:
  imp.find_module("std_msgs")
except:
  import roslib; roslib.load_manifest('jsk_teleop_joy')


from std_msgs.msg import String, Empty
from geometry_msgs.msg import PoseStamped
import xml.etree.ElementTree as ET

class JoyGoPos(JoyPose6D):
  def __init__(self, name, args):
    JoyPose6D.__init__(self, name, args)
    self.supportFollowView(True)
    self.frame_id = self.getArg('frame_id', '/map')
    # parse srdf to get planning_groups
    self.goal_pub = rospy.Publisher("goal", PoseStamped)
  def joyCB(self, status, history):
    JoyPose6D.joyCB(self, status, history)
    latest = history.latest()
    if not latest:
      return
    if status.circle and not latest.circle:
        self.goal_pub.publish(self.pre_pose)
    if status.cross and not latest.cross:
        self.pre_pose = PoseStamped()
        self.pre_pose.pose.orientation.w = 1
        
