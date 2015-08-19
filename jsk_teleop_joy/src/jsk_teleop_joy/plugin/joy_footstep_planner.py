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
from jsk_rviz_plugins.msg import OverlayMenu, OverlayText
from std_msgs.msg import UInt8, Empty
from std_srvs.srv import Empty as EmptySrv
import std_srvs.srv
import tf
from tf.transformations import *
from geometry_msgs.msg import PoseStamped
import jsk_teleop_joy.tf_ext as tf_ext
from jsk_footstep_planner.srv import ChangeSuccessor
class JoyFootstepPlanner(JoyPose6D):
  EXECUTING = 1
  PLANNING = 2
  WAIT_FOR_CANCEL = 3
  CANCELED = 4
  CANCELED_MENUS = ["Cancel", 
                    "Ignore and proceed",
                    "Use larger footsteps",
                    "Use middle footsteps",
                    "Use smaller footsteps"]
  def __init__(self, name, args):
    JoyPose6D.__init__(self, name, args)
    self.usage_pub = rospy.Publisher("/joy/usage", OverlayText)
    self.supportFollowView(True)
    self.mode = self.PLANNING
    self.snapped_pose = None
    self.ignore_next_status_flag = False
    self.prev_mode = self.PLANNING
    self.frame_id = self.getArg('frame_id', '/map')
    self.lfoot_frame_id = rospy.get_param('~lfoot_frame_id', '/LLEG_LINK5')
    self.rfoot_frame_id = rospy.get_param('~rfoot_frame_id', '/RLEG_LINK5')
    self.lfoot_offset = tf_ext.xyzxyzwToMatrix(rospy.get_param('~lfoot_offset'))
    self.rfoot_offset = tf_ext.xyzxyzwToMatrix(rospy.get_param('~rfoot_offset'))
    self.lock_furutaractive = self.getArg("lock_furutaractive", False)
    self.furutaractive_namespace = self.getArg("furutaractive_namespace", "urdf_model_marker")
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
    self.snap_sub = rospy.Subscriber(self.getArg("snapped_pose", "/footstep_marker/snapped_pose"), 
                                     PoseStamped,
                                     self.snapCB,
                                     queue_size=1)
    self.cancel_menu_sub = rospy.Subscriber("/footstep_cancel_broadcast", Empty, 
                                            self.cancelMenuCB, queue_size=1)
    self.status_lock = threading.Lock()
    self.current_selecting_index = 0
    self.resetGoalPose()
  def lockFurutaractive(self):
    try:
      lock = rospy.ServiceProxy(self.furutaractive_namespace + '/lock_joint_states', EmptySrv)
      lock()
    except rospy.ServiceException, e:
      rospy.logerror("failed to call service: %s" % (e.message))
  def unlockFurutaractive(self):
    try:
      unlock = rospy.ServiceProxy(self.furutaractive_namespace + '/unlock_joint_states', EmptySrv)
      unlock()
    except rospy.ServiceException, e:
      rospy.logerror("failed to call service: %s" % (e.message))
  def cancelMenuCB(self, msg):
    with self.status_lock:
      self.mode = self.CANCELED
  def snapCB(self, msg):
    self.snapped_pose = msg
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
        if self.lock_furutaractive:
          self.unlockFurutaractive()
    elif self.mode == self.WAIT_FOR_CANCEL and msg.status_list[0].status == 0:
      self.mode = self.CANCELED
    self.status_lock.release()
  def publishUsage(self):
    overlay_text = OverlayText()
    overlay_text.text = """
Left Analog: x/y
L2/R2      : +-z
L1/R1      : +-yaw
Left/Right : +-roll
Up/Down    : +-pitch
circle     : Go
cross      : Reset/Cancel
triangle   : Clear maps and look around ground
up/down    : Move menu cursors
"""
    overlay_text.width = 500
    overlay_text.height = 500
    overlay_text.text_size = 12
    overlay_text.left = 10
    overlay_text.top = 10
    overlay_text.font = "Ubuntu Mono Regular"
    overlay_text.bg_color.a = 0
    overlay_text.fg_color.r = 25 / 255.0
    overlay_text.fg_color.g = 1
    overlay_text.fg_color.b = 1
    overlay_text.fg_color.a = 1
    self.usage_pub.publish(overlay_text)
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
  def changePlanningSuccessor(self, successor_type):
    try:
      change_successor = rospy.ServiceProxy('/change_successor', ChangeSuccessor)
      change_successor(successor_type)
    except rospy.ServiceException, e:
      rospy.logerror("failed to call service: %s" % (e.message))
  def procCancelMenu(self, index):
    selected_title = self.CANCELED_MENUS[index]
    if selected_title == "Cancel":
      self.status_lock.acquire()
      self.mode = self.PLANNING
      self.status_lock.release()
      self.publishMenu(close=True)
    elif selected_title == "Ignore and proceed":
      # re-execute the plan left
      self.resumePlan()
      self.status_lock.acquire()
      self.mode = self.EXECUTING
      self.status_lock.release()
      self.publishMenu(close=True)
    elif selected_title == "Use smaller footsteps":
      self.status_lock.acquire()
      self.mode = self.PLANNING
      self.changePlanningSuccessor("small")
      self.status_lock.release()
      self.publishMenu(close=True)
    elif (selected_title == "Use larger footsteps" or 
          selected_title == "Use middle footsteps"):
      self.status_lock.acquire()
      self.mode = self.PLANNING
      self.changePlanningSuccessor("normal")
      self.status_lock.release()
      self.publishMenu(close=True)
  def lookAround(self):
    try:
      clear_maps = rospy.ServiceProxy('/env_server/clear_maps', std_srvs.srv.Empty)
      clear_maps()
      look_around = rospy.ServiceProxy('/lookaround_ground', std_srvs.srv.Empty)
      look_around()
    except Exception, e:
      rospy.logerr("error when lookaround ground: %s", e.message)
  def joyCB(self, status, history):
    self.publishUsage()
    if self.prev_mode != self.mode:
      print self.prev_mode, " -> ", self.mode
    if self.mode == self.PLANNING:
      JoyPose6D.joyCB(self, status, history)
      if history.new(status, "circle"):
        self.status_lock.acquire()
        self.mode = self.EXECUTING
        if self.lock_furutaractive:
          self.lockFurutaractive()
        self.executePlan()
        self.ignore_next_status_flag = True
        self.status_lock.release()
      elif history.new(status, "cross"):
        self.resetGoalPose()
      elif history.new(status, "triangle"):
        self.lookAround()
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
      # if history.new(status, "triangle"):
      #   self.command_pub.publish(UInt8(1))
      if history.new(status, "cross"):
        self.status_lock.acquire()
        self.exec_client.cancel_all_goals()
        self.mode = self.WAIT_FOR_CANCEL
        self.status_lock.release()
    self.prev_mode = self.mode
