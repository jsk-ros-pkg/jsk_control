#!/usr/bin/env python

import math
import numpy

import rospy
import roslib

roslib.load_manifest('jsk_joy')

from sensor_msgs.msg import Joy
from view_controller_msgs.msg import CameraPlacement
from visualization_msgs.msg import InteractiveMarkerUpdate, InteractiveMarkerPose, InteractiveMarkerInit
from geometry_msgs.msg import PoseStamped
import tf.transformations

from joy_status import XBoxStatus
INTERACTIVE_MARKER_TOPIC = '/goal_marker'

pre_pose = None

class ViewProperty():
  def __init__(self):
    self.yaw = 0.0
    self.pitch = 0.0
    self.distance = 1.0
    self.focus = numpy.array((0, 0, 0))
  def viewPoint(self):
    p = numpy.array((self.distance * math.cos(self.yaw) * math.cos(self.pitch) + self.focus[0],
                     self.distance * math.sin(self.yaw) * math.cos(self.pitch) + self.focus[1],
                     self.distance *                      math.sin(self.pitch) + self.focus[2]))
    return p
  def cameraOrientation(self):
    OE = self.viewPoint()
    f = self.focus - OE # z
    f = f / tf.transformations.vector_norm(f)
    u = numpy.array((0, 0, 1))            #not aligned y
    r = numpy.cross(u, f) # x
    r = r / tf.transformations.vector_norm(r)
    uy = numpy.cross(f, r)
    uy = uy / tf.transformations.vector_norm(uy)
    m = tf.transformations.identity_matrix()[:3, :3]   #rotation matrix
    m[0, 0] = r[0]
    m[1, 0] = r[1]
    m[2, 0] = r[2]
    m[0, 1] = uy[0]
    m[1, 1] = uy[1]
    m[2, 1] = uy[2]
    m[0, 2] = f[0]
    m[1, 2] = f[1]
    m[2, 2] = f[2]
    return m
  def cameraPlacement(self):
    TIME = 0.05
    view_point = self.viewPoint()
    placement = CameraPlacement()
    placement.interpolation_mode = CameraPlacement.LINEAR
    placement.time_from_start = rospy.Duration(TIME)
    placement.eye.header.stamp = rospy.Time(0.0)
    placement.eye.header.frame_id = "/map"
    placement.eye.point.x = view_point[0]
    placement.eye.point.y = view_point[1]
    placement.eye.point.z = view_point[2]
    placement.focus.header.stamp = rospy.Time(0.0)
    placement.focus.header.frame_id = "/map"
    placement.focus.point.x = self.focus[0]
    placement.focus.point.y = self.focus[1]
    placement.focus.point.z = self.focus[2]
    placement.up.header.stamp = rospy.Time(0.0)
    placement.up.header.frame_id = "/map"
    placement.up.vector.z = 1.0
    placement.mouse_interaction_mode = CameraPlacement.ORBIT
    return placement


def joyCB(msg):
  global pre_view, g_seq_num, pre_pose
  if not pre_pose:
    pre_pose = PoseStamped()
  status = XBoxStatus(msg)

  #rospy.loginfo(status)
  # move interactive marker
  new_pose = PoseStamped()
  new_pose.header.frame_id = '/map'
  new_pose.header.stamp = rospy.Time(0.0)
  # move in local
  if not status.R3:
    if status.square:
      scale = 10.0
    else:
      scale = 30.0
    local_xy_move = numpy.array((status.left_analog_y / scale,
                                 status.left_analog_x / scale,
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
      else:
        yaw = yaw + DTHETA
    elif status.R1:
      if status.square:
        yaw = yaw - DTHETA * 5
      else:
        yaw = yaw - DTHETA
    if status.L2:
      if status.square:
        new_pose.pose.position.z = pre_pose.pose.position.z + D * 5.0
      else:
        new_pose.pose.position.z = pre_pose.pose.position.z + D
    elif status.R2:
      if status.square:
        new_pose.pose.position.z = pre_pose.pose.position.z - D * 5.0
      else:
        new_pose.pose.position.z = pre_pose.pose.position.z - D
  new_q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
  new_pose.pose.orientation.x = new_q[0]
  new_pose.pose.orientation.y = new_q[1]
  new_pose.pose.orientation.z = new_q[2]
  new_pose.pose.orientation.w = new_q[3]
  g_interactive_pub.publish(new_pose)
  pre_pose = new_pose
  
  # view update
  view = ViewProperty()
  view.focus = numpy.copy(pre_view.focus)
  view.yaw = pre_view.yaw
  view.pitch = pre_view.pitch
  view.distance = pre_view.distance
  if status.R3:
    view.distance = view.distance - status.left_analog_y * 0.1
    # calc camera orietation
    if status.left:
      view_x = 1.0
    elif status.right:
      view_x = -1.0
    else:
      view_x = 0.0
    if status.up:
      view_y = 1.0
    elif status.down:
      view_y = -1.0
    else:
      view_y = 0.0
    focus_diff = numpy.dot(view.cameraOrientation(),
                           numpy.array((view_x / 10.0 / view.distance,
                                        view_y / 10.0 / view.distance,
                                        0)))
    view.focus = view.focus + focus_diff
    if status.L2 and status.R2:           #align to marker
      view.distance = 1.0
      view.focus = numpy.array((new_pose.pose.position.x,
                                new_pose.pose.position.y,
                                new_pose.pose.position.z))
      #view.yaw = math.pi
      view.yaw = yaw + math.pi
      view.pitch = math.pi / 2.0 - 0.01
  else:
    view.yaw = view.yaw - 0.5 * status.right_analog_x
    view.pitch = view.pitch + 0.5 * status.right_analog_y
    if view.pitch > math.pi / 2.0 - 0.01:
      view.pitch = math.pi / 2.0 - 0.01
    elif view.pitch < - math.pi / 2.0 + 0.01:
      view.pitch = - math.pi / 2.0 + 0.01
  g_camera_pub.publish(view.cameraPlacement())
  pre_view = view


def publishInit():
  pass

def main():
  global g_camera_pub, pre_view, g_interactive_pub, g_seq_num, g_interactive_full_pub
  g_seq_num = 0
  rospy.sleep(1)
  rospy.init_node('jsk_joy')
  pre_view = ViewProperty()
  g_camera_pub = rospy.Publisher('/rviz/camera_placement', CameraPlacement)
  g_interactive_pub = rospy.Publisher(INTERACTIVE_MARKER_TOPIC + '/move_marker', PoseStamped)
  publishInit()
  s = rospy.Subscriber('/joy', Joy, joyCB)
  
  rospy.spin()
  
if __name__ == '__main__':
  main()
  
