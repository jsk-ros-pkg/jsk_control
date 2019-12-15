from joy_rviz_view_controller import RVizViewController

from std_msgs.msg import String
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
up/left: go to the previous bounding box
down/right: go to the next bounding box

Right Analog x/y: yaw/pitch of camera position (see parent class, RVizViewController)
R3(Right Analog button): suppressing buttons/sticks for controlling pose
   R3 + L2 + R2: enable follow view mode

circle/cross/triangle: publish cooperating command

Args:
frame_id [String, default: map]: frame_id of publishing pose, overwritten by parameter ~frame_id
input_boxes [String, default: cluster_point_indices/boxes]: topic name of the bounding box array
output [String, default: selected_bbox]: topic name to pubish the selected bounding box
command [String, default: command]: topic name to publish commands
triangle_cmd [String, default: TRIANGLE_CMD]: command text when triangle button is pressed
circle_cmd [String, default: CIRCLE_CMD]: command text when triangle button is pressed
cross_cmd [String, default: CROSS_CMD]: command text when triangle button is pressed
  '''
  def __init__(self, name, args):
    RVizViewController.__init__(self, name, args)
    self.pre_pose = PoseStamped()
    self.bbox = None
    self.pre_pose.pose.orientation.w = 1
    self.prev_time = rospy.Time.from_sec(time.time())
    self.bb_topic = self.getArg('input_boxes', 'cluster_point_indices/boxes')
    self.index = 0
    self.frame_id = self.getArg('frame_id', 'map')
    bb = rospy.Subscriber(self.bb_topic, BoundingBoxArray, self.bboxSB)
    self.bbox_pub = rospy.Publisher(self.getArg('output', 'selected_bbox'),
                                    BoundingBox, queue_size=1)
    self.command_pub = rospy.Publisher(self.getArg('command', 'command'),
                                    String, queue_size=1)
    self.triangle_cmd = self.getArg('triangle_cmd', 'TRIANGLE_CMD')
    self.cross_cmd = self.getArg('cross_cmd', 'CROSS_CMD')
    self.circle_cmd = self.getArg('circle_cmd', 'CIRCLE_CMD')

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
      if (history.new(status, "up")
          or history.new(status, "left")
          or history.new(status, "left_analog_up")
          or history.new(status, "left_analog_left")):
        self.index -= 1
      elif (history.new(status, "down")
            or history.new(status, "right")
            or history.new(status, "left_analog_down")
            or history.new(status, "left_analog_right")):
        self.index += 1
      if status.circle and not latest.circle:
        self.command_pub.publish(self.circle_cmd)
      if status.triangle and not latest.triangle:
        self.command_pub.publish(self.triangle_cmd)
      if status.cross and not latest.cross:
        self.command_pub.publish(self.cross_cmd)

    # publish at 10hz
    now = rospy.Time.from_sec(time.time())
    # placement.time_from_start = now - self.prev_time
    if (now - self.prev_time).to_sec() > 1 / 30.0:
      if self.bbox is not None:
        self.bbox_pub.publish(self.bbox)
      self.prev_time = now

  def bboxSB(self, msg):
    bboxes = msg.boxes
    if bboxes.__len__() > 0:
      if self.index < bboxes.__len__():
        if self.index < 0:
          self.index = bboxes.__len__() - 1
      else:
        self.index = 0
      # rospy.set_param('~index', self.index)
      self.bbox = bboxes[self.index]
    else:
      self.bbox = None
