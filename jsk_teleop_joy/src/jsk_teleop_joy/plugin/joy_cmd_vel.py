from jsk_teleop_joy.joy_plugin import JSKJoyPlugin

import imp
try:
  imp.find_module("geometry_msgs")
except:
  import roslib; roslib.load_manifest('jsk_teleop_joy')


from geometry_msgs.msg import Twist
import tf
import rospy
import numpy
import math
import tf
import numpy
import time

class JoyCmdVel(JSKJoyPlugin):
  #def __init__(self, name='JoyPose6D', publish_pose=True):
  def __init__(self, name, args):
    JSKJoyPlugin.__init__(self, name, args)
    self.cmd_vel = Twist()
    self.publish_cmd_vel = self.getArg('publish_cmd_vel', True)
    self.max_vel = self.getArg('max_vel', 0.2)
    self.max_omega = self.getArg('max_omega', 0.17) # 10[deg]
    self.orthogonal_axis_mode = self.getArg('orthogonal_axis_mode', True)
    self.prev_time = rospy.Time.now()
    if self.publish_cmd_vel:
      self.twist_pub = rospy.Publisher(self.getArg('cmd_vel', 'cmd_vel'), Twist, queue_size = 1)
    
  def joyCB(self, status, history):
    if history.length() > 0:
      latest = history.latest()
      if status.R3 and status.L2 and status.R2 and not (latest.R3 and latest.L2 and latest.R2):
        self.followView(not self.followView())
    cmd_vel = Twist()
    # currently only support 2D plane movement
    if not status.R3:
      # xy
      dist = numpy.sqrt(status.left_analog_y * status.left_analog_y + status.left_analog_x * status.left_analog_x) # dist is assumed to be 0 < dist < 1      
      scale_v = self.max_vel * dist
      if self.orthogonal_axis_mode:
        if abs(status.left_analog_y) - abs(status.left_analog_x) > 0.2:
          x_diff = status.left_analog_y * scale_v
          y_diff = 0.0
        elif abs(status.left_analog_y) - abs(status.left_analog_x) < -0.2:
          x_diff = 0.0
          y_diff = status.left_analog_x * scale_v
        else:
          x_diff = 0.0
          y_diff = 0.0
      else:
        x_diff = status.left_analog_y * scale_v
        y_diff = status.left_analog_x * scale_v
    else:
      x_diff = 0.0
      y_diff = 0.0

    cmd_vel.linear.x = x_diff
    cmd_vel.linear.y = y_diff
    cmd_vel.linear.z = 0.0
    
    dyaw = 0.0
    if not status.R3:
      if status.L1:
        dyaw = self.max_omega
      elif status.R1:
        dyaw = -self.max_omega

    cmd_vel.angular.x = 0.0
    cmd_vel.angular.y = 0.0
    cmd_vel.angular.z = dyaw
    
    # publish at 10hz
    if self.publish_cmd_vel:
      now = rospy.Time.from_sec(time.time())
      # placement.time_from_start = now - self.prev_time
      if (now - self.prev_time).to_sec() > 1 / 30.0:
        self.twist_pub.publish(cmd_vel)
        self.prev_time = now
