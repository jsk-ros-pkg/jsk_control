#!/usr/bin/env python
import rospy

try:
  from sensor_msgs.msg import Joy, JoyFeedbackArray
except:
  roslib.load_manifest('jsk_teleop_joy')
  from sensor_msgs.msg import Joy, JoyFeedbackArray
from std_msgs.msg import Int8

class AxesConverter:
  def __init__(self, axes):
    self.input_origins = [None] * len(axes)
    self.output_origins = [0.5] * len(axes)

  def convert(self, abs_axes):
    self.__updateInputOrigins(abs_axes)
    return self.__convertAxes(abs_axes, self.input_origins, self.output_origins)

  def __updateInputOrigins(self, abs_axes):
    for i in range(len(abs_axes)):
      if self.input_origins[i] is None and abs_axes[i] != 0 and abs_axes[i] != 1:
        self.input_origins[i] = abs_axes[i]

  def __convertAxes(self, abs_axes, in_origins, out_origins):
    rel_axes = [None] * len(abs_axes)
    for i in range(len(abs_axes)):
      rel_axes[i] = self.__convertAxis(abs_axes[i], in_origins[i], out_origins[i])
    return rel_axes

  def __convertAxis(self, abs_axis, in_origin, out_origin):
    if in_origin is None:
      rel_axis = out_origin
    elif abs_axis < in_origin:
      rel_axis = (((abs_axis - 0.0) /  (in_origin - 0.0)) * (out_origin - 0.0)) + 0.0
    elif abs_axis == in_origin:
      rel_axis = out_origin
    elif abs_axis > in_origin:
      rel_axis = (((abs_axis - in_origin) /  (1.0 - in_origin)) * (1.0 - out_origin)) + out_origin
    return rel_axis

class AxesConverterArray:
  def __init__(self, axes, max_page=1):
    self.ac = [AxesConverter(axes) for i in range(max_page)]
    self.crt_page = 0

  def convert(self, abs_axes, page=None):
    if page is None:
      return self.ac[self.crt_page].convert(abs_axes)
    else:
      return self.ac[page].convert(abs_axes)

  def switch_page(self, page):
    if page >= 0 and page < len(self.ac):
      self.crt_page = page
      return page
    else:
      return -1

def joy_callback(data):
  global aca
  try:
    aca
  except NameError:
    aca = AxesConverterArray(data.axes, 2)
  joy_pub = rospy.Publisher("/joy_relative/page_" + str(aca.crt_page), Joy)
  joy = Joy()
  joy.header.stamp = rospy.Time.now()
  joy.axes = aca.convert(data.axes)
  joy.buttons = data.buttons
  joy_pub.publish(joy)

def switch_page_cmd_cb(msg):
  aca.switch_page(msg.data)

def main():
  rospy.init_node('joy_relative_converter')
  rospy.Subscriber("/joy", Joy, joy_callback)
  rospy.Subscriber("/midi_relative_converter/command/switch_page", Int8, switch_page_cmd_cb)
  rospy.spin()

if __name__ == '__main__':
  main()
