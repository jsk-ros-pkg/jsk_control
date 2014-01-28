#!/usr/bin/env python
import pygame
import pygame.midi
import select
import sys
import yaml
import rospy
import roslib

from sensor_msgs.msg import Joy

roslib.load_manifest('jsk_joy')

from jsk_joy.midi_util import MIDIParse, MIDICommand, MIDIException, openMIDIInputByName

def main():
  pygame.midi.init()
  rospy.init_node('midi_joy')
  # parse the arg
  argv = rospy.myargv()
  if len(argv) == 0:
    rospy.logfatal("You need to specify config yaml file")
    sys.exit(1)
  config_file = argv[1]
  joy_pub = rospy.Publisher("/joy", Joy)
  with open(config_file, "r") as f:
    config = yaml.load(f)
    # open the device
    controller = openMIDIInputByName(config["device_name"])
    joy = Joy()
    joy.axes = [0.0] * len(config["analogs"])
    # automatically mapping to buttons from axes if it has NOTE_OFF or NOTE_ON MIDI commands
    button_configs = [c for c in config["analogs"]
                      if c[0] == MIDICommand.NOTE_ON or c[0] == MIDICommand.NOTE_OFF]
    joy.buttons = [0] * len(button_configs)
    while not rospy.is_shutdown():
      joy.header.stamp = rospy.Time.now()
      while controller.poll():
        data = controller.read(1)
        for elem_set in data:
          (command, ind, val) = MIDIParse(elem_set)
          index = config["analogs"].index((command, ind))
          joy.axes[index] = val
          if command == MIDICommand.NOTE_ON or command == MIDICommand.NOTE_OFF:
            button_index = button_configs.index((command, ind))
            if val == 0.0:
              joy.buttons[button_index] = 0
            else:
              joy.buttons[button_index] = 1
      joy_pub.publish(joy)
      rospy.sleep(1.0 / 100.0)
if __name__ == '__main__':
  main()
  
