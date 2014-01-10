# joy_rviz_view_controller

from jsk_joy.joy_plugin import JSKJoyPlugin
from jsk_joy.camera_view import CameraView
from view_controller_msgs.msg import CameraPlacement

import tf
import rospy

import numpy
import math

class RVizViewController(JSKJoyPlugin):
  def __init__(self, name):
    JSKJoyPlugin.__init__(self, name)
    self.camera_pub = rospy.Publisher('/rviz/camera_placement', CameraPlacement)
    self.follow_view = rospy.get_param('~follow_view', False)
    self.pre_view = CameraView()
  def joyCB(self, status, history):
    pre_view = self.pre_view
    view = CameraView()
    view.focus = numpy.copy(pre_view.focus)
    view.yaw = pre_view.yaw
    view.pitch = pre_view.pitch
    view.distance = pre_view.distance
    view_updated = False
    if status.R3:
      if not status.left_analog_y == 0.0:
        view.distance = view.distance - status.left_analog_y * 0.1
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
      if status.L2 and status.R2:           #align to marker
        view_updated = True
        view.distance = 1.0
        view.focus = numpy.array((self.pre_pose.pose.position.x,
                                  self.pre_pose.pose.position.y,
                                  self.pre_pose.pose.position.z))
        #view.yaw = math.pi
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(numpy.array((self.pre_pose.pose.orientation.x,
                                                                                   self.pre_pose.pose.orientation.y,
                                                                                   self.pre_pose.pose.orientation.z,
                                                                                   self.pre_pose.pose.orientation.w)))
        view.yaw = yaw + math.pi
        view.pitch = math.pi / 2.0 - 0.01
    else:
      if status.right_analog_x != 0.0:
        view_updated = True
      if status.right_analog_y != 0.0:
        view_updated = True
      view.yaw = view.yaw - 0.2 * status.right_analog_x
      view.pitch = view.pitch + 0.2 * status.right_analog_y
      if view.pitch > math.pi / 2.0 - 0.01:
        view.pitch = math.pi / 2.0 - 0.01
      elif view.pitch < - math.pi / 2.0 + 0.01:
        view.pitch = - math.pi / 2.0 + 0.01

    if self.follow_view:
      view_updated = True
      view.distance = 0.8
      view.focus = numpy.array((self.pre_pose.pose.position.x,
                                self.pre_pose.pose.position.y,
                                self.pre_pose.pose.position.z))
      #view.yaw = math.pi
      (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(numpy.array((self.pre_pose.pose.orientation.x,
                                                                                 self.pre_pose.pose.orientation.y,
                                                                                 self.pre_pose.pose.orientation.z,
                                                                                 self.pre_pose.pose.orientation.w)))
      view.yaw = yaw + math.pi
      view.pitch = math.pi / 2.0 - 0.01
    if view_updated:
      self.camera_pub.publish(view.cameraPlacement())
    self.pre_view = view
