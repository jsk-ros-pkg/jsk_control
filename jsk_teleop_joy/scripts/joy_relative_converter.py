#!/usr/bin/env python
import rospy

try:
  from sensor_msgs.msg import Joy, JoyFeedbackArray
except:
  roslib.load_manifest('jsk_teleop_joy')
  from sensor_msgs.msg import Joy, JoyFeedbackArray
from std_msgs.msg import Int8, Empty

class AxesConverter:
  def __init__(self):
    self.input_origins = []
    self.output_origins = []
    self.prev_output_axes = [i for i in self.output_origins]

  def adjustAxesSize(self, axes_size):
    if len(self.input_origins) != axes_size:
      if len(self.input_origins) == 0:
        self.input_origins = [None] * axes_size
        self.output_origins = [0.5] * axes_size
        rospy.loginfo("[midi_relative_converter] adjust axes size.")
      else:
        rospy.logwarn("[midi_relative_converter] different axes size!!")

  def convert(self, abs_axes):
    self.adjustAxesSize(len(abs_axes))
    self.updateInputOrigins(abs_axes)
    rel_axes = self.__convertAxes(abs_axes, self.input_origins, self.output_origins)
    self.__updatePrevOutputAxes(rel_axes)
    return rel_axes

  def resetAllOrigins(self):
    self.input_origins = []
    self.output_origins = []
    self.prev_output_axes = [i for i in self.output_origins]

  def resetInputOrigins(self):
    for i in range(len(self.input_origins)):
      self.input_origins[i] = None

  def updateInputOrigins(self, abs_axes):
    for i in range(len(abs_axes)):
      if self.input_origins[i] is None and abs_axes[i] != 0:
        self.input_origins[i] = abs_axes[i]

  def __updatePrevOutputAxes(self, rel_axes):
    self.prev_output_axes = list(rel_axes)

  def updateOutputOrigins(self):
    self.output_origins = list(self.prev_output_axes)

  def __convertAxes(self, abs_axes, in_origins, out_origins):
    rel_axes = [None] * len(abs_axes)
    for i in range(len(abs_axes)):
      rel_axes[i] = self.__convertAxis(abs_axes[i], in_origins[i], out_origins[i])
    return rel_axes

  def __convertAxis(self, abs_axis, in_origin, out_origin):
    if abs_axis == 1:
      rel_axis = abs_axis
    elif in_origin is None:
      rel_axis = out_origin
    elif abs_axis < in_origin:
      rel_axis = (((abs_axis - 0.0) /  (in_origin - 0.0)) * (out_origin - 0.0)) + 0.0
    elif abs_axis == in_origin:
      rel_axis = out_origin
    elif abs_axis > in_origin:
      rel_axis = (((abs_axis - in_origin) /  (1.0 - in_origin)) * (1.0 - out_origin)) + out_origin
    return rel_axis

class AxesConverterArray:
  def __init__(self, max_page=1):
    self.ac = [AxesConverter() for i in range(max_page)]
    self.crt_page = 0
    self.prev_input_axes = []
    for page in range(max_page):
      joy_pub = rospy.Publisher("/joy_relative/page_" + str(page), Joy)
    rospy.Subscriber("/joy", Joy, self.joy_callback)
    rospy.Subscriber("/midi_relative_converter/command/switch_page", Int8, self.switch_page_cmd_cb)
    rospy.Subscriber("/midi_relative_converter/command/reset", Empty, self.reset_cmd_cb)

  def convert(self, abs_axes):
    return self.ac[self.crt_page].convert(abs_axes)

  def switchPage(self, page):
    if page >= 0 and page < len(self.ac):
      if page != self.crt_page:
        self.__updateReserveInputOrigins(self.prev_input_axes)
        self.__updateCurrentOutputOrigins()
        self.crt_page = page
      return page
    else:
      return -1

  def resetAllOrigins(self):
    for i in range(len(self.ac)):
      self.ac[i].resetAllOrigins()

  def __updateReserveInputOrigins(self, abs_axes):
    for i in range(len(self.ac)):
      if i != self.crt_page:
        self.ac[i].adjustAxesSize(len(abs_axes))
        self.ac[i].resetInputOrigins()
        self.ac[i].updateInputOrigins(abs_axes)

  def __updateCurrentOutputOrigins(self):
    self.ac[self.crt_page].updateOutputOrigins()

  def joy_callback(self, data):
    joy = Joy()
    joy.header.stamp = rospy.Time.now()
    joy.axes = self.convert(data.axes)
    joy.buttons = data.buttons
    self.prev_input_axes = list(data.axes)
    joy_pub = rospy.Publisher("/joy_relative/page_" + str(self.crt_page), Joy)
    joy_pub.publish(joy)

  def switch_page_cmd_cb(self, msg):
    self.switchPage(msg.data)

  def reset_cmd_cb(self, msg):
    self.resetAllOrigins()

def main():
  rospy.init_node('joy_relative_converter')
  aca = AxesConverterArray(3)
  rospy.spin()

if __name__ == '__main__':
  main()
