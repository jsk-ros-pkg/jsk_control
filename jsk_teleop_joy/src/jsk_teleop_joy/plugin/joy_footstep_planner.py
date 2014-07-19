import rospy

import imp
try:
  imp.find_module("actionlib")
except:
  import roslib; roslib.load_manifest('jsk_teleop_joy')
import threading
import actionlib
from joy_pose_6d import JoyPose6D
from actionlib_msgs.msg import GoalStatusArray
from jsk_footstep_msgs.msg import PlanFootstepsAction, PlanFootstepsGoal, Footstep, FootstepArray, ExecFootstepsAction, ExecFootstepsGoal
from jsk_rviz_plugins.msg import OverlayMenu
from std_msgs.msg import UInt8, Empty
import tf
from tf.transformations import *

import jsk_teleop_joy.tf_ext as tf_ext

class JoyFootstepPlanner(JoyPose6D):
  EXECUTING = 1
  PLANNING = 2
  WAIT_FOR_CANCEL = 3
  CANCELED = 4
  CANCELED_MENUS = ["cancel", 
                    "ignore and proceed",
                    "use larger successors",
                    "use middle successors",
                    "use smaller successors"]
  def __init__(self, name, args):
    JoyPose6D.__init__(self, name, args)
    self.supportFollowView(True)
    self.mode = self.PLANNING
    self.ignore_next_status_flag = False
    self.prev_mode = self.PLANNING
    self.frame_id = self.getArg('frame_id', '/map')
    self.lfoot_frame_id = rospy.get_param('~lfoot_frame_id', '/LLEG_LINK5')
    self.rfoot_frame_id = rospy.get_param('~rfoot_frame_id', '/RLEG_LINK5')
    self.lfoot_offset = tf_ext.xyzxyzwToMatrix(rospy.get_param('~lfoot_offset'))
    self.rfoot_offset = tf_ext.xyzxyzwToMatrix(rospy.get_param('~rfoot_offset'))
    
    self.command_pub = rospy.Publisher('/menu_command', UInt8)
    self.execute_pub = rospy.Publisher(self.getArg('execute', 'execute'), Empty)
    self.resume_pub = rospy.Publisher(self.getArg('resume', 'resume'), Empty)
    self.tf_listener = tf.TransformListener()
    self.menu_pub = rospy.Publisher("/overlay_menu", OverlayMenu)
    # initialize self.pre_pose
    rospy.loginfo("waiting %s" % (self.lfoot_frame_id))
    self.tf_listener.waitForTransform(self.frame_id, self.lfoot_frame_id,
                                      rospy.Time(0.0), rospy.Duration(100.0))
    rospy.loginfo("waiting %s" % (self.rfoot_frame_id))
    self.tf_listener.waitForTransform(self.frame_id, self.rfoot_frame_id, 
                                      rospy.Time(0.0), rospy.Duration(100.0))
    self.exec_client = actionlib.SimpleActionClient('/footstep_controller',
                                                    ExecFootstepsAction)
    self.status_sub = rospy.Subscriber("/footstep_controller/status", GoalStatusArray,
                                       self.statusCB, queue_size=1)
    
    self.status_lock = threading.Lock()
    self.current_selecting_index = 0
    self.resetGoalPose()
  def statusCB(self, msg):
    if len(msg.status_list) == 0:
      return                    # do nothing
    self.status_lock.acquire()
    # msg.status_list[0].status == 0 -> done
    if self.mode == self.EXECUTING and msg.status_list[0].status == 0: # ???
      # done
      if self.ignore_next_status_flag: # hack!!
        self.ignore_next_status_flag = False
      else:
        self.mode = self.PLANNING
    elif self.mode == self.WAIT_FOR_CANCEL and msg.status_list[0].status == 0:
      self.mode = self.CANCELED
    self.status_lock.release()
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
  def resumePlan(self):
    # publish execute with std_msgs/empty 
    self.resume_pub.publish(Empty())
  def publishMenu(self, close=False):
    menu = OverlayMenu()
    menu.menus = self.CANCELED_MENUS
    menu.current_index = self.current_selecting_index
    menu.title = "Canceled"
    if close:
      menu.action = OverlayMenu.ACTION_CLOSE
    self.menu_pub.publish(menu)
  def procCancelMenu(self, index):
    selected_title = self.CANCELED_MENUS[index]
    if selected_title == "cancel":
      self.status_lock.acquire()
      self.mode = self.PLANNING
      self.status_lock.release()
      self.publishMenu(close=True)
    elif selected_title == "ignore and proceed":
      # re-execute the plan left
      self.resumePlan()
      self.status_lock.acquire()
      self.mode = self.EXECUTING
      self.status_lock.release()
      self.publishMenu(close=True)
  def joyCB(self, status, history):
    if self.prev_mode != self.mode:
      print self.prev_mode, " -> ", self.mode
    if self.mode == self.PLANNING:
      JoyPose6D.joyCB(self, status, history)
      if history.new(status, "circle"):
        self.status_lock.acquire()
        self.mode = self.EXECUTING
        self.executePlan()
        self.ignore_next_status_flag = True
        self.status_lock.release()
      elif history.new(status, "cross"):
        self.resetGoalPose()
    elif self.mode == self.CANCELED:
      # show menu
      if history.new(status, "circle"):
        # choosing
        self.procCancelMenu(self.current_selecting_index)
      else:
        if history.new(status, "up"):
          self.current_selecting_index = self.current_selecting_index - 1
        elif history.new(status, "down"):
          self.current_selecting_index = self.current_selecting_index + 1
          # menu is only cancel/ignore
        if self.current_selecting_index < 0:
          self.current_selecting_index = len(self.CANCELED_MENUS) - 1
        elif self.current_selecting_index > len(self.CANCELED_MENUS) - 1:
          self.current_selecting_index = 0
        self.publishMenu()
    elif self.mode == self.EXECUTING:
      if history.new(status, "triangle"):
        self.command_pub.publish(UInt8(1))
      elif history.new(status, "cross"):
        self.status_lock.acquire()
        self.exec_client.cancel_all_goals()
        self.mode = self.WAIT_FOR_CANCEL
        self.status_lock.release()
    self.prev_mode = self.mode
