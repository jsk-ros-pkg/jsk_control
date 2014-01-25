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

def openMidiDeviceWithName(device_name):
  devices = pygame.midi.get_count()
  for i in range(devices):
    info = pygame.midi.get_device_info(i)
    if info[1] == device_name and info[2] == 1:
      return pygame.midi.Input(i)
  raise Exception("Cannot find the device: %s" % (device_name))
    
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
    controller = openMidiDeviceWithName(config["device_name"])
    joy = Joy()
    joy.axes = [0.0] * len(config["analogs"])
    #joy.buttons = [0] * len(config["buttons"])
    while not rospy.is_shutdown():
      joy.header.stamp = rospy.Time.now()
      while controller.poll():
        data = controller.read(1)
        for elem_set in data:
          elem = elem_set[0]
          major_id = elem[0]
          minor_id = elem[1]
          value = elem[2]
          if major_id == 144 or major_id == 128:              #button
            if (144, minor_id) in config["analogs"]:
              index = config["analogs"].index((144, minor_id))
              if major_id == 128:
                joy.axes[index] = 0
              else:
                joy.axes[index] = value / 127.0
          elif (major_id, minor_id) in config["analogs"]:
            index = config["analogs"].index((major_id, minor_id))
            joy.axes[index] = value / 127.0
      joy_pub.publish(joy)
      rospy.sleep(1.0 / 100.0)
if __name__ == '__main__':
  main()
  
