#!/usr/bin/env python
import pygame
import pygame.midi
import select
import sys
import yaml
import rospy
import roslib

try:
  from sensor_msgs.msg import Joy, JoyFeedbackArray
  from jsk_teleop_joy.midi_util import MIDIParse, MIDICommand, MIDIException, openMIDIInputByName, openMIDIOutputByName
except:
  roslib.load_manifest('jsk_teleop_joy')
  from sensor_msgs.msg import Joy, JoyFeedbackArray
  from jsk_teleop_joy.midi_util import MIDIParse, MIDICommand, MIDIException, openMIDIInputByName, openMIDIOutputByName

def feedback_array_cb(out_controller, config, msg_arr):
  global joy
  output_config = config["output"]
  for msg in msg_arr.array:
    if len(output_config) <= msg.id:
      rospy.logerr("%d is out of output configuration (%d configurations)" % (msg.id, len(output_config)))
      return
    the_config = output_config[msg.id]
    command = the_config[0]
    channel = the_config[1]
    val = int(msg.intensity * 127)
    if val < 0:
      val = 0
    elif val > 127:
      val = 127
    if the_config[2]:
      out_controller.write_short(command | channel, val, 0)
    else:
      param1 = the_config[3]
      out_controller.write_short(command | channel, param1, val)
      try:
        index = config["analogs"].index((MIDICommand.CONTINUOUS_CONTROLLER,param1))
        joy.axes[index] = msg.intensity
      except:
        pass

def main():
  global joy
  pygame.midi.init()
  rospy.init_node('midi_joy')
  # parse the arg
  argv = rospy.myargv()
  if len(argv) == 0:
    rospy.logfatal("You need to specify config yaml file")
    sys.exit(1)
  config_file = argv[1]
  joy_pub = rospy.Publisher("/joy", Joy, queue_size=10)
  autorepeat_rate = rospy.get_param("~autorepeat_rate", 0)
  if autorepeat_rate == 0:
    r = rospy.Rate(1000)
  else:
    r = rospy.Rate(autorepeat_rate)
  with open(config_file, "r") as f:
    config = yaml.load(f)
    # open the device
    controller = openMIDIInputByName(config["device_name"])
    
    joy = Joy()
    joy.axes = [0.0] * len(config["analogs"])
    # automatically mapping to buttons from axes if it has NOTE_OFF or NOTE_ON MIDI commands
    button_configs = [c for c in config["analogs"]
                      if c[0] == MIDICommand.NOTE_ON or c[0] == MIDICommand.NOTE_OFF]
    if "output" in config:
      out_controller = openMIDIOutputByName(config["device_name"])
      s = rospy.Subscriber("~set_feedback", JoyFeedbackArray, lambda msg: feedback_array_cb(out_controller, config, msg))
    joy.buttons = [0] * len(button_configs)
    while not rospy.is_shutdown():
      joy.header.stamp = rospy.Time.now()
      p = False
      while controller.poll():
        data = controller.read(1)
        for elem_set in data:
          p = True
          (command, ind, val) = MIDIParse(elem_set)
          try:
            index = config["analogs"].index((command, ind))
            joy.axes[index] = val
            if command == MIDICommand.NOTE_ON or command == MIDICommand.NOTE_OFF:
              button_index = button_configs.index((command, ind))
              if val == 0.0:
                joy.buttons[button_index] = 0
              else:
                joy.buttons[button_index] = 1
          except:
            rospy.logwarn("unknown MIDI message: (%d, %d, %f)" % (command, ind, val))
      if (autorepeat_rate != 0) or p:
        joy_pub.publish(joy)
      r.sleep()
if __name__ == '__main__':
  main()
  
