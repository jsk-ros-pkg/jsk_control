from jsk_teleop_joy.joy_plugin import JSKJoyPlugin
try:
    from jsk_rviz_plugins.msg import OverlayText
except:
    import roslib; roslib.load_manifest("jsk_teleop_joy")
    from jsk_rviz_plugins.msg import OverlayText
from std_msgs.msg import ColorRGBA
import rospy

class Usage(JSKJoyPlugin):
  def __init__(self, name, args):
    JSKJoyPlugin.__init__(self, name, args)
    self.pub = rospy.Publisher("/overlay_text", OverlayText)
    self.usage = self.getArg("text")
    self.width = self.getArg("width", 400)
    self.height = self.getArg("width", 600)
    self.left = self.getArg("left", 10)
    self.top = self.getArg("top", 10)
    self.line_width = self.getArg("line_width", 2)
    self.text_size = self.getArg("text_size", 20)
    self.font = self.getArg("font", "DejaVu Sans Mono")
  def joyCB(self, status, history):
    msg = OverlayText()
    if status.left:
      self.left = self.left - 10
    if status.right:
      self.left = self.left + 10
    if status.up:
      self.top = self.top - 10
    if status.down:
      self.top = self.top + 10
    msg.text_size = self.text_size
    msg.width = self.width
    msg.height = self.height
    msg.left = self.left
    msg.top = self.top
    msg.line_width = self.line_width
    msg.font = self.font
    msg.fg_color = ColorRGBA(25 / 255.0, 1.0, 240.0 / 255.0, 1.0)
    msg.bg_color = ColorRGBA(0.0, 0.0, 0.0, 0.2)
    msg.text = self.usage
    self.pub.publish(msg)
  def disable(self):
    msg = OverlayText()
    msg.action = OverlayText.DELETE
    self.pub.publish(msg)


