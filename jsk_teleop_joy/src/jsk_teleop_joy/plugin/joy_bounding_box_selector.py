from joy_rviz_view_controller import RVizViewController

import imp
try:
  imp.find_module("geometry_msgs")
except:
  import roslib; roslib.load_manifest('jsk_teleop_joy')


from geometry_msgs.msg import PoseStamped
import tf
import rospy
import numpy
import math
import time

from jsk_recognition_msgs.msg import BoundingBox
from jsk_recognition_msgs.msg import BoundingBoxArray

def signedSquare(val):
 if val > 0:
   sign = 1
 else:
   sign = -1
 return val * val * sign

class JoyBoundingBoxSelector(RVizViewController):
  '''
Usage:
Left Analog x/y: 

Args:
frame_id [String, default: map]: frame_id of publishing pose, overwritten by parameter ~frame_id
  '''
  def __init__(self, name, args):
    RVizViewController.__init__(self, name, args)
    self.pre_pose = PoseStamped()
    self.pre_pose.pose.orientation.w = 1
    self.prev_time = rospy.Time.from_sec(time.time())
    self.bb_topic = self.getArg('input_boxes', 'cluster_point_indices/boxes')
    self.index = 0
    self.frame_id = self.getArg('frame_id', 'map')
    bb = rospy.Subscriber(self.bb_topic, BoundingBoxArray, self.bboxSB)
    self.bbox_pub = rospy.Publisher(self.getArg('output', 'selected_bbox'),
                                    BoundingBox, queue_size=1)
    self.supportFollowView(True)

    if rospy.has_param('~frame_id'):
      self.frame_id = rospy.get_param('~frame_id')
    self.tf_listener = tf.TransformListener()

  def joyCB(self, status, history):
    if history.length() > 0:
      latest = history.latest()
    if self.control_view:
      RVizViewController.joyCB(self, status, history)
    # renew index
    # self.index = rospy.get_param('~index', self.index)
    if not status.R3:
      if (status.left_analog_y > 0.5 and latest.left_analog_y < 0.5) or (status.left_analog_x > 0.5 and latest.left_analog_x < 0.5):
        self.index -= 1
      elif (status.left_analog_y < -0.5 and latest.left_analog_y > -0.5) or (status.left_analog_x < -0.5 and latest.left_analog_x > -0.5):
        self.index += 1

    # publish at 10hz
    now = rospy.Time.from_sec(time.time())
    # placement.time_from_start = now - self.prev_time
    if (now - self.prev_time).to_sec() > 1 / 30.0:
      self.bbox_pub.publish(self.bbox)
      self.prev_time = now

  def bboxSB(self, msg):
    bboxes = msg.boxes
    if self.index < bboxes.__len__():
      if self.index < 0:
        self.index = bboxes.__len__() - 1
    else:
      self.index = 0
    # rospy.set_param('~index', self.index)
    self.bbox = bboxes[self.index]
    
