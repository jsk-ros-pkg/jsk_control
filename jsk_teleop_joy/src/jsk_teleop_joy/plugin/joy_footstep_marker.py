import rospy

import imp
try:
  imp.find_module("actionlib")
except:
  import roslib; roslib.load_manifest('jsk_teleop_joy')

import actionlib
from joy_pose_6d import JoyPose6D
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Empty
import copy
import time

def isSamePose(pose1, pose2):
  return all([getattr(pose1.position, attr) == getattr(pose2.position, attr) for attr in ["x", "y", "z"]]) and all ([getattr(pose1.orientation, attr) == getattr(pose2.orientation, attr) for attr in ["x", "y", "z", "w"]])

class JoyFootstepMarker(JoyPose6D):
  def __init__(self, name, args):
    args['publish_pose'] = False # supress publishing pose of joy_pose_6d
    JoyPose6D.__init__(self, name, args)
    self.pose_pub = rospy.Publisher(self.getArg('pose', 'pose'), PoseStamped)
    self.current_goal_pose = None
  
    # make service proxy
    rospy.wait_for_service('/footstep_marker/reset_marker')
    self.reset_marker_srv = rospy.ServiceProxy('/footstep_marker/reset_marker', Empty)
    rospy.wait_for_service('/footstep_marker/execute_footstep')
    self.execute_footstep_srv = rospy.ServiceProxy('/footstep_marker/execute_footstep', Empty)
    
  def joyCB(self, status, history):
    JoyPose6D.joyCB(self, status, history)
    latest = history.latest()
    if not latest:
      return
    if status.circle and not latest.circle:
      self.execute_footstep_srv()
    elif status.cross and not latest.cross:
      self.reset_marker_srv()
      self.pre_pose = PoseStamped()
      self.pre_pose.pose.orientation.w = 1

    if self.current_goal_pose == None or not isSamePose(self.current_goal_pose.pose, self.pre_pose.pose):
      now = rospy.Time.from_sec(time.time())
      # placement.time_from_start = now - self.prev_time
      if (now - self.prev_time).to_sec() > 1 / 30.0:
        self.pose_pub.publish(self.pre_pose)
        self.prev_time = now
        self.current_goal_pose = copy.deepcopy(self.pre_pose)
