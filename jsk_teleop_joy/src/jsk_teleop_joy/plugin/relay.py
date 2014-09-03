from jsk_teleop_joy.joy_plugin import JSKJoyPlugin
try:
    from sensor_msgs.msg import Joy
except:
    import roslib; roslib.load_manifest("jsk_teleop_joy")
    from sensor_msgs.msg import Joy
    
import rospy
from topic_tools.srv import MuxSelect

class Relay(JSKJoyPlugin):
  def __init__(self, name, args):
    JSKJoyPlugin.__init__(self, name, args)
    self.output_topic = self.getArg("output_topic", "output")
    self.org_topic = self.getArg("joy_original_topic", "/joy_org")
    self.joy_mux_srv_name = self.getArg("joy_mux", "mux")
    self.joy_mux_select_srv = rospy.ServiceProxy(self.joy_mux_srv_name + "/select" , MuxSelect)
  def enable(self):
    self.pub = rospy.Publisher(self.output_topic, Joy)
    try:
        self.joy_mux_select_srv(self.output_topic)
    except:
        rospy.logerr("Missing joy mux: %s", self.joy_mux_srv_name)
  def disable(self):
    self.pub.unregister()
    try:
        self.joy_mux_select_srv(self.org_topic)
    except:
        rospy.logerr("Missing joy mux: %s", self.joy_mux_srv_name)
  def joyCB(self, status, history):
    self.pub.publish(status.orig_msg)
 

class RelayAndConvertToPS3(Relay):
  def __init__(self, name, args):
    Relay.__init__(self, name, args)
  def joyCB(self, status, history):
    self.pub.publish(status.toPS3Msg())
 

