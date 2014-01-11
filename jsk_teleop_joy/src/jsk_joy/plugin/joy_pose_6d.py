from joy_rviz_view_controller import RVizViewController
from geometry_msgs.msg import PoseStamped
import tf
import rospy
import numpy
import math

class JoyPose6D(RVizViewController):
  def __init__(self, name='JoyPose6D', publish_pose=True):
    RVizViewController.__init__(self, name)
    self.pre_pose = PoseStamped()
    self.publish_pose = publish_pose
    if self.publish_pose:
      self.pose_pub = rospy.Publisher('pose', PoseStamped)
    self.support_follow_view = True
  def joyCB(self, status, history):
    pre_pose = self.pre_pose
    if status.triangle and not history.latest().triangle:
      self.follow_view = not self.follow_view
    RVizViewController.joyCB(self, status, history)
    new_pose = PoseStamped()
    new_pose.header.frame_id = '/map'
    new_pose.header.stamp = rospy.Time(0.0)
    # move in local
    if not status.R3:
      if status.square:
        scale = 10.0
      elif status.left_analog_y * status.left_analog_y + status.left_analog_x * status.left_analog_x == 1.0:
        scale = 20.0
      else:
        scale = 60.0
      x_sign = 1.0
      if status.left_analog_x < 0:
        x_sign = -1.0
      y_sign = 1.0
      if status.left_analog_y < 0:
        y_sign = -1.0
      local_xy_move = numpy.array((status.left_analog_y * status.left_analog_y * y_sign / scale,
                                   status.left_analog_x * status.left_analog_x * x_sign / scale,
                                   0.0, 
                                   1.0))
    else:
      local_xy_move = numpy.array((0.0, 0.0, 0.0, 1.0))
    new_pose.pose.position.z = pre_pose.pose.position.z
    q = numpy.array((pre_pose.pose.orientation.x,
                     pre_pose.pose.orientation.y,
                     pre_pose.pose.orientation.z,
                     pre_pose.pose.orientation.w))
    xy_move = numpy.dot(tf.transformations.quaternion_matrix(q),
                        local_xy_move)
    new_pose.pose.position.x = pre_pose.pose.position.x + xy_move[0]
    new_pose.pose.position.y = pre_pose.pose.position.y + xy_move[1]
    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(q)
    DTHETA = 0.02
    D = 0.005
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
      if status.L2:
        if status.square:
          new_pose.pose.position.z = pre_pose.pose.position.z + D * 10.0
        elif history.all(lambda s: s.L2):
          new_pose.pose.position.z = pre_pose.pose.position.z + D * 4.0
        else:
          new_pose.pose.position.z = pre_pose.pose.position.z + D * 2.0
      elif status.R2:
        if status.square:
          new_pose.pose.position.z = pre_pose.pose.position.z - D * 10.0
        elif history.all(lambda s: s.R2):
          new_pose.pose.position.z = pre_pose.pose.position.z - D * 4.0
        else:
          new_pose.pose.position.z = pre_pose.pose.position.z - D * 2.0
    new_q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    new_pose.pose.orientation.x = new_q[0]
    new_pose.pose.orientation.y = new_q[1]
    new_pose.pose.orientation.z = new_q[2]
    new_pose.pose.orientation.w = new_q[3]
    if self.publish_pose:
      self.pose_pub.publish(new_pose)
    self.pre_pose = new_pose

