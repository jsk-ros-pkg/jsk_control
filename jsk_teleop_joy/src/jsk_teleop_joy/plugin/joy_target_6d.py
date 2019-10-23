from joy_rviz_view_controller import RVizViewController

import imp
import copy
try:
  imp.find_module("geometry_msgs")
except:
  import roslib; roslib.load_manifest('jsk_teleop_joy')


from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

import tf
import rospy
import numpy
import math
import tf
import numpy
import time

def signedSquare(val):
 if val > 0:
   sign = 1
 else:
   sign = -1
 return val * val * sign

class JoyTarget6D(RVizViewController):
  '''
Usage:
Left Analog x/y: translate x/y
Up/Down/Right/Left: rotate pitch/roll
L1/R1: rotate yaw
L2/R2: translate z
square: move faster

Right Analog x/y: yaw/pitch of camera position (see parent class, RVizViewController)
R3(Right Analog button): suppressing buttons/sticks for controlling pose
   R3 + L2 + R2: enable follow view mode

L3+circle: save current pose
L3+square: load saved pose

circle/cross/triangle: publish cooperating command

Args:
publish_pose [Boolean, default: True]: Publish or not pose
frame_id [String, default: map]: frame_id of publishing pose, this is overwritten by parameter, ~frame_id
pose [String, default: pose]: topic name for publishing pose
target_pose [String, default: target_pose]: topic name to pubish current pose when button is pressed
set_pose [String, default: set_pose]: topic name for setting pose by topic
command [String, default: command]: topic name for publishing the command
triangle_cmd'　[String, default: TRIANGLE_CMD]: command text when triangle button is pressed
circle_cmd'　[String, default: CIRCLE_CMD]: command text when triangle button is pressed
cross_cmd'　[String, default: CROSS_CMD]: command text when triangle button is pressed
  '''
  def __init__(self, name, args):
    RVizViewController.__init__(self, name, args)
    self.pre_pose = PoseStamped()
    self.pre_pose.pose.orientation.w = 1
    self.saved_pose = PoseStamped()
    self.saved_pose.pose.orientation.w = 1
    self.prev_time = rospy.Time.from_sec(time.time())
    self.publish_pose = self.getArg('publish_pose', True)
    self.frame_id = self.getArg('frame_id', 'map')
    
    if self.publish_pose:
      self.pose_pub = rospy.Publisher(self.getArg('pose', 'pose'),
                                      PoseStamped, queue_size=10)
    self.target_pub = rospy.Publisher(self.getArg('target_pose', 'target_pose'),
                                    PoseStamped, queue_size=1)
    self.command_pub = rospy.Publisher(self.getArg('command', 'command'),
                                    String, queue_size=1)
    self.triangle_cmd = self.getArg('triangle_cmd', 'TRIANGLE_CMD')
    self.cross_cmd = self.getArg('cross_cmd', 'CROSS_CMD')
    self.circle_cmd = self.getArg('circle_cmd', 'CIRCLE_CMD')
    self.supportFollowView(True)

    self.pose_sub = rospy.Subscriber(self.getArg('set_pose', 'set_pose'),
                                     PoseStamped, self.setPoseCB)
    if rospy.has_param('~frame_id'):
      self.frame_id = rospy.get_param('~frame_id')
    self.tf_listener = tf.TransformListener()

  def setPoseCB(self, pose):
    pose.header.stamp = rospy.Time(0)
    self.pre_pose = self.tf_listener.transformPose(self.frame_id, pose)

    if self.publish_pose:
      self.pose_pub.publish(self.pre_pose)

  def publish_pose_command(self, pose, command):
    self.target_pub.publish(pose)
    self.command_pub.publish(command)

  def joyCB(self, status, history):
    pre_pose = self.pre_pose
    saved_pose = self.saved_pose
    if history.length() > 0:
      latest = history.latest()
      if status.R3 and status.L2 and status.R2 and not (latest.R3 and latest.L2 and latest.R2):
        self.followView(not self.followView())
    if self.control_view:
      RVizViewController.joyCB(self, status, history)
    new_pose = PoseStamped()
    new_pose.header.frame_id = self.frame_id
    new_pose.header.stamp = rospy.Time(0.0)
    # move in local
    if not status.R3:
      # xy
      if status.square:
        scale = 20.0
      else:
        dist = status.left_analog_y * status.left_analog_y + status.left_analog_x * status.left_analog_x
        if dist > 0.9:
          scale = 50.0
        else:
          scale = 80.0
      x_diff = signedSquare(status.left_analog_y) / scale
      y_diff = signedSquare(status.left_analog_x) / scale
      # z
      if status.L2:
        z_diff = 0.005
      elif status.R2:
        z_diff = -0.005
      else:
        z_diff = 0.0
      if status.square:
        z_scale = 5.0
      elif history.all(lambda s: s.L2) or history.all(lambda s: s.R2):
        z_scale = 2.0
      else:
        z_scale = 1.0
      local_move = numpy.array((x_diff, y_diff,
                                z_diff * z_scale, 
                                1.0))
    else:
      local_move = numpy.array((0.0, 0.0, 0.0, 1.0))
    q = numpy.array((pre_pose.pose.orientation.x,
                     pre_pose.pose.orientation.y,
                     pre_pose.pose.orientation.z,
                     pre_pose.pose.orientation.w))
    xyz_move = numpy.dot(tf.transformations.quaternion_matrix(q),
                         local_move)
    new_pose.pose.position.x = pre_pose.pose.position.x + xyz_move[0]
    new_pose.pose.position.y = pre_pose.pose.position.y + xyz_move[1]
    new_pose.pose.position.z = pre_pose.pose.position.z + xyz_move[2]
    roll = 0.0
    pitch = 0.0
    yaw = 0.0
    DTHETA = 0.02
    if not status.R3:
      if status.L1:
        if status.square:
          yaw = yaw + DTHETA * 5
        elif history.all(lambda s: s.L1):
          yaw = yaw + DTHETA * 2
        else:
          yaw = yaw + DTHETA
      elif status.R1:
        if status.square:
          yaw = yaw - DTHETA * 5
        elif history.all(lambda s: s.R1):
          yaw = yaw - DTHETA * 2
        else:
          yaw = yaw - DTHETA
      if status.up:
        if status.square:
          pitch = pitch + DTHETA * 5
        elif history.all(lambda s: s.up):
          pitch = pitch + DTHETA * 2
        else:
          pitch = pitch + DTHETA
      elif status.down:
        if status.square:
          pitch = pitch - DTHETA * 5
        elif history.all(lambda s: s.down):
          pitch = pitch - DTHETA * 2
        else:
          pitch = pitch - DTHETA
      if status.right:
        if status.square:
          roll = roll + DTHETA * 5
        elif history.all(lambda s: s.right):
          roll = roll + DTHETA * 2
        else:
          roll = roll + DTHETA
      elif status.left:
        if status.square:
          roll = roll - DTHETA * 5
        elif history.all(lambda s: s.left):
          roll = roll - DTHETA * 2
        else:
          roll = roll - DTHETA
      if status.triangle and not latest.triangle:
        self.publish_pose_command(new_pose, self.triangle_cmd)
      if status.cross and not latest.cross:
        self.publish_pose_command(new_pose, self.cross_cmd)
    diff_q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    new_q = tf.transformations.quaternion_multiply(q, diff_q)
    new_pose.pose.orientation.x = new_q[0]
    new_pose.pose.orientation.y = new_q[1]
    new_pose.pose.orientation.z = new_q[2]
    new_pose.pose.orientation.w = new_q[3]
    if not (status.R3 and status.R2 and status.L2) and status.circle and not latest.circle:
      self.publish_pose_command(new_pose, self.circle_cmd)

    # publish at 10hz
    if self.publish_pose:
      now = rospy.Time.from_sec(time.time())
      # placement.time_from_start = now - self.prev_time
      if (now - self.prev_time).to_sec() > 1 / 30.0:
        self.pose_pub.publish(new_pose)
        self.prev_time = now

    self.pre_pose = new_pose
