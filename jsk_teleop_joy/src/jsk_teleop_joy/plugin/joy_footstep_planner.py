import rospy

import imp
try:
  imp.find_module("actionlib")
except:
  import roslib; roslib.load_manifest('jsk_teleop_joy')

import actionlib
from joy_pose_6d import JoyPose6D
from jsk_footstep_msgs.msg import PlanFootstepsAction, PlanFootstepsGoal, Footstep, FootstepArray
from std_msgs.msg import UInt8, Empty
import tf
from tf.transformations import *

import jsk_teleop_joy.tf_ext as tf_ext

class JoyFootstepPlanner(JoyPose6D):
  
    
  def __init__(self):
    JoyPose6D.__init__(self, name='JoyFootstepPlanner')
    self.support_follow_view = True
    self.frame_id = rospy.get_param('~frame_id', '/map')
    self.lfoot_frame_id = rospy.get_param('~lfoot_frame_id', '/LLEG_LINK5')
    self.rfoot_frame_id = rospy.get_param('~rfoot_frame_id', '/RLEG_LINK5')
    self.lfoot_offset = tf_ext.xyzxyzwToMatrix(rospy.get_param('~lfoot_offset'))
    self.rfoot_offset = tf_ext.xyzxyzwToMatrix(rospy.get_param('~rfoot_offset'))
    
    self.command_pub = rospy.Publisher('/menu_command', UInt8)
    self.execute_pub = rospy.Publisher('execute', Empty)
    self.tf_listener = tf.TransformListener()
    # initialize self.pre_pose
    rospy.loginfo("waiting %s" % (self.lfoot_frame_id))
    self.tf_listener.waitForTransform(self.frame_id, self.lfoot_frame_id,
                                      rospy.Time(0.0), rospy.Duration(100.0))
    rospy.loginfo("waiting %s" % (self.rfoot_frame_id))
    self.tf_listener.waitForTransform(self.frame_id, self.rfoot_frame_id, 
                                      rospy.Time(0.0), rospy.Duration(100.0))
    self.resetGoalPose()
  def resetGoalPose(self):
    # initial pose will be the center 
    # of self.lfoot_frame_id and self.rfoot_frame_id
    lfoot_pose = tf_ext.transformToMatrix(self.tf_listener.lookupTransform(
      self.frame_id, 
      self.lfoot_frame_id,
      rospy.Time(0.0)))
    rfoot_pose = tf_ext.transformToMatrix(self.tf_listener.lookupTransform(
      self.frame_id, 
      self.rfoot_frame_id,
      rospy.Time(0.0)))
    # apply offset
    lfoot_with_offset = numpy.dot(lfoot_pose, self.lfoot_offset)
    rfoot_with_offset = numpy.dot(rfoot_pose, self.rfoot_offset)
    (lfoot_pos, lfoot_q) = tf_ext.decomposeMatrix(lfoot_with_offset)
    (rfoot_pos, rfoot_q) = tf_ext.decomposeMatrix(rfoot_with_offset)
    # compute the center of the two transformations
    mid_pos = (lfoot_pos + rfoot_pos) / 2.0
    mid_quaternion = quaternion_slerp(lfoot_q, rfoot_q,
                                      0.5)
    self.pre_pose.pose.position.x = mid_pos[0]
    self.pre_pose.pose.position.y = mid_pos[1]
    self.pre_pose.pose.position.z = mid_pos[2]
    self.pre_pose.pose.orientation.x = mid_quaternion[0]
    self.pre_pose.pose.orientation.y = mid_quaternion[1]
    self.pre_pose.pose.orientation.z = mid_quaternion[2]
    self.pre_pose.pose.orientation.w = mid_quaternion[3]
  def executePlan(self):
    # publish execute with std_msgs/empty 
    self.execute_pub.publish(Empty())
  def joyCB(self, status, history):    
    JoyPose6D.joyCB(self, status, history)
    latest = history.latest()
    if not latest:
      return
    if status.triangle and not latest.triangle:
      self.command_pub.publish(UInt8(1))
    elif status.cross and not latest.cross:   #reset
      self.resetGoalPose()
    elif status.circle and not latest.circle:   #execute
      self.executePlan()
    
