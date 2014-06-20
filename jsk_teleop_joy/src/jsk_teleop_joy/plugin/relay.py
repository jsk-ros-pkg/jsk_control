from jsk_teleop_joy.joy_plugin import JSKJoyPlugin
try:
    from sensor_msgs.msg import Joy
except:
    import roslib; roslib.load_manifest("jsk_teleop_joy")
    from sensor_msgs.msg import Joy
    
import rospy

class Relay(JSKJoyPlugin):
  def __init__(self, name, args):
    JSKJoyPlugin.__init__(self, name, args)
    self.output_topic = self.getArg("output_topic", "output")
  def enable(self):
    self.pub = rospy.Publisher(self.output_topic, Joy)
  def disable(self):
    self.pub.unregister()
  def joyCB(self, status, history):
    self.pub.publish(status.orig_msg)
 

