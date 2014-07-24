from jsk_teleop_joy.camera_view import CameraView

import imp
try:
  imp.find_module("view_controller_msgs")
except:
  import roslib; roslib.load_manifest('jsk_teleop_joy')

from view_controller_msgs.msg import CameraPlacement

import tf
import rospy

import numpy
import math

import time

def signedSquare(val):
  if val > 0:
    sign = 1
  else:
    sign = -1
  return val * val * sign


class RVizViewControllerManager():
  def __init__(self):
    self.camera_pub = rospy.Publisher('/rviz/camera_placement', CameraPlacement)
    self.pre_view = CameraView()
    self.follow_view = False
    self.counter = 0
    self.prev_time = rospy.Time.from_sec(time.time())
  def joyCB(self, status, history, pre_pose):
    self.counter = self.counter + 1
    if self.counter > 1024:
      self.counter = 0
    pre_view = self.pre_view
    view = CameraView()
    view.focus = numpy.copy(pre_view.focus)
    view.yaw = pre_view.yaw
    view.pitch = pre_view.pitch
    view.distance = pre_view.distance
    view_updated = False
    if status.R3:
      if not status.left_analog_y == 0.0:
        view.distance = view.distance - signedSquare(status.left_analog_y) * 0.05
        view_updated = True
      # calc camera orietation
      if status.left:
        view_updated = True
        view_x = 1.0
      elif status.right:
        view_updated = True
        view_x = -1.0
      else:
        view_x = 0.0
      if status.up:
        view_updated = True
        view_y = 1.0
      elif status.down:
        view_updated = True
        view_y = -1.0
      else:
        view_y = 0.0
      focus_diff = numpy.dot(view.cameraOrientation(),
                             numpy.array((view_x / 20.0 / view.distance,
                                          view_y / 20.0 / view.distance,
                                          0)))
      view.focus = view.focus + focus_diff
    else:
      if status.right_analog_x != 0.0:
        view_updated = True
      if status.right_analog_y != 0.0:
        view_updated = True
      
      view.yaw = view.yaw - 0.05 * signedSquare(status.right_analog_x)
      view.pitch = view.pitch + 0.05 * signedSquare(status.right_analog_y)

    if self.follow_view and self.support_follow_view:
      view_updated = True
      view.focus = numpy.array((pre_pose.pose.position.x,
                                pre_pose.pose.position.y,
                                pre_pose.pose.position.z))
      #view.yaw = math.pi
      q = numpy.array((pre_pose.pose.orientation.x,
                       pre_pose.pose.orientation.y,
                       pre_pose.pose.orientation.z,
                       pre_pose.pose.orientation.w))
      mat = tf.transformations.quaternion_matrix(q)
      camera_local_pos = numpy.dot(mat, numpy.array((0, 0, 1, 1)))[:3]
      pitch = math.asin(camera_local_pos[2])
      # calc pitch quadrant
      if camera_local_pos[1] < 0:
        pitch = math.pi - pitch
      if math.fabs(math.cos(pitch)) < 0.01:
        yaw = 0.0
      else:
        cos_value = camera_local_pos[0] / math.cos(pitch)
        if cos_value > 1.0:
          cos_value = 1.0
        elif cos_value < -1.0:
          cos_value = -1.0
        yaw = math.acos(cos_value)
      view.pitch = pitch
      view.yaw = yaw
      z_up = numpy.dot(mat, numpy.array((1, 0, 0, 1)))
      view.z_up = z_up[:3]
    now = rospy.Time.from_sec(time.time())
    placement = view.cameraPlacement()
    placement.time_from_start = now - self.prev_time
    if (now - self.prev_time).to_sec() > 1 / 10.0:
      self.camera_pub.publish(placement)
      self.prev_time = now
    self.pre_view = view


RVizViewControllerManagerSingleton = RVizViewControllerManager()


