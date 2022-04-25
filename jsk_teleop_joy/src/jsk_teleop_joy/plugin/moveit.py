import rospy
from jsk_teleop_joy.joy_pose_6d import JoyPose6D

import imp
try:
  imp.find_module("actionlib")
except:
  import roslib; roslib.load_manifest('jsk_teleop_joy')

import actionlib
from std_msgs.msg import String, Empty
import xml.etree.ElementTree as ET

class JoyMoveIt(JoyPose6D):
  def __init__(self, name, args):
    JoyPose6D.__init__(self, name, args)
    self.supportFollowView(True)
    self.frame_id = self.getArg('frame_id', '/map')
    self.planning_group = self.getArg('planning_group')
    self.plan_group_pub = rospy.Publisher('/rviz/moveit/select_planning_group', String)
    self.plan_pub = rospy.Publisher("/rviz/moveit/plan", Empty)
    self.execute_pub = rospy.Publisher("/rviz/moveit/execute", Empty)
    self.update_start_state_pub = rospy.Publisher("/rviz/moveit/update_start_state", Empty)
    self.update_goal_state_pub = rospy.Publisher("/rviz/moveit/update_goal_state", Empty)
    self.counter = 0
  def enable(self):
    self.plan_group_pub.publish(self.planning_group)
  def joyCB(self, status, history):
    JoyPose6D.joyCB(self, status, history)
    latest = history.latest()
    if not latest:
      return
    if status.triangle and not latest.triangle:
      # self.current_planning_group_index = self.current_planning_group_index + 1
      # if self.current_planning_group_index >= len(self.planning_groups):
      #   self.current_planning_group_index = 0
      # self.plan_group_pub.publish(self.planning_groups[self.current_planning_group_index])
      self.update_goal_state_pub.publish(Empty())
    elif status.cross and not latest.cross:
      # self.current_planning_group_index = self.current_planning_group_index - 1
      # if self.current_planning_group_index < 0:
      #   self.current_planning_group_index = len(self.planning_groups) - 1
      # self.plan_group_pub.publish(self.planning_groups[self.current_planning_group_index])
      pass
    elif status.square and not latest.square:
      self.plan_pub.publish(Empty())
    elif status.circle and not latest.circle:
      self.execute_pub.publish(Empty())
    self.counter = self.counter + 1
    if self.counter % 10:
      self.update_start_state_pub.publish(Empty())
