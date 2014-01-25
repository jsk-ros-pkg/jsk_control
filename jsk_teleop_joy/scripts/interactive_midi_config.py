#!/usr/bin/env python

import pygame
import pygame.midi
import select
import sys
import yaml

G_DEVICE_INFO = {
  "device_name": "",
  "analogs": []                             #[[major_id, minor_id], [major_id, minor_id], ...
  }

class ParseException(Exception):
  pass

def parseDeviceName():
  global G_DEVICE_INFO
  devices = pygame.midi.get_count()
  print "First, we choose device name:"
  for d in range(devices):
    info = pygame.midi.get_device_info(d)
    if info[2] == 1:
      print "  [%d] %s (%s)" % (d, info[1], "input")
    else:
      print "  [%d] %s (%s)" % (d, info[1], "output")
  val = raw_input("Please select the device by number[%d-%d]:" % (0, d))
  try:
    parsed_number = int(val)
    if parsed_number >= 0 and parsed_number <= d:
      name = pygame.midi.get_device_info(parsed_number)[1]
      G_DEVICE_INFO["device_name"] = name
      print ""
      print "device_name: %s"  % (name)
      return parsed_number
    else:
      raise ParseException("please input number bewtween %d to %d" % (0, d))
  except ValueError:
    raise ParseException("please input number")

def configAnalogInputs(controller):
  global G_DEVICE_INFO
  print "Please move analog inputs"
  print "The order you move them will be mapped into Joy/axes."
  print "If you want to finish analog mapping, please type 'q'"
  analog_configs = []
  while True:
    ready = select.select([sys.stdin], [], [], 0.1)[0]
    if ready:
      line = sys.stdin.readline()
      if line.startswith("q"):
        print "We installed %d analog inputs" % (len(analog_configs))
        G_DEVICE_INFO["analogs"] = analog_configs
        return
    while controller.poll():
      data = controller.read(1)
      for elem_set in data:
        elem = elem_set[0]
        major_id = elem[0]
        minor_id = elem[1]
        # check that is already installed or not
        if major_id == 240 or major_id == 1 or major_id == 79:
            print "we ignore this button because it has %d major_id" % (major_id)
            print "please tell the code master it"
        elif major_id == 144 or major_id == 128:
          # this might be a same button
          if (144, minor_id) not in analog_configs:
            print "(144/128, %d) installing into %d" % (minor_id, len(analog_configs))
            analog_configs.append((144, minor_id))
        elif (major_id, minor_id) not in analog_configs:
          print "(%d, %d) installing into %d" % (major_id, minor_id, len(analog_configs))
          analog_configs.append((major_id, minor_id))
          
def configButtonInputs(controller):
  global G_DEVICE_INFO
  print "Please push the buttons"
  print "The order you move them will be mapped into Joy/buttons."
  print "If you want to finish buttons mapping, please type 'q'"
  button_configs = []
  while True:
    ready = select.select([sys.stdin], [], [], 0.1)[0]
    if ready:
      line = sys.stdin.readline()
      if line.startswith("q"):
        print "We installed %d button inputs" % (len(button_configs))
        G_DEVICE_INFO["buttons"] = button_configs
        return
    while controller.poll():
      data = controller.read(1)
      for elem_set in data:
        elem = elem_set[0]
        major_id = elem[0]
        minor_id = elem[1]
        # check that is already installed or not
        if major_id == 144:
          if minor_id not in button_configs:
            print "%dinstalling into %d" % (minor_id, len(button_configs))
            button_configs.append(minor_id)
  
def main():
  pygame.midi.init()
  while True:
    try:
      device_num = parseDeviceName()
      break
    except ParseException, e:
      print e.message
      print ""
      continue
  controller = pygame.midi.Input(device_num)
  configAnalogInputs(controller)
  # configButtonInputs(controller)
  f = open('/tmp/midi.yaml', 'w')
  f.write(yaml.dump(G_DEVICE_INFO))
  f.close()
  print "writing the configuration to /tmp/midi.yaml"
  
if __name__ == "__main__":
  main()

  
