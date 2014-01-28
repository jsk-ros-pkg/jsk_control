#!/usr/bin/env python
import pygame
import pygame.midi
import select
import sys
import yaml
import rospy
import roslib
import time
import re
from optparse import OptionParser

roslib.load_manifest('jsk_joy')
from jsk_joy.midi_util import MIDICommand, MIDIParse

class MIDIWriteError(Exception):
  pass

def parseRangedNumber(string):
  """
  string = 0-9 or [number-number] and this returns list of the numbers
  """
  try:                                    #0-0
    val = int(string)
    return [val]
  except ValueError:                      
    if string.startswith("[") and string.endswith("]"):
      # range representation
      r = re.compile("\[([0-9]+)-([0-9]+)\]")
      m = r.search(string)
      if m:
        first_number = m.group(1)
        last_number = m.group(2)
        return range(int(first_number), int(last_number) + 1)
      else:
        raise MIDIWriteError("failed to parse string: %s" % (string))
    else:
      raise MIDIWriteError("failed to parse string: %s" % (string))
    

def main():

  all_midi_commands = ""
  for c in MIDICommand.allCommands():
    midi_command_str = "%s (%d)" % (MIDICommand.toStr(c), c)
    if len(all_midi_commands) > 0:
      all_midi_commands = "%s, %s (%d)" % (all_midi_commands, MIDICommand.toStr(c),
                                           c)
    else:
      all_midi_commands = midi_command_str
  parser = OptionParser()
  parser.add_option("--device_index", "-d", help="device index", type="int")
  parser.add_option("--midi_command", "-m", 
                    help="midi command: candidates are: " + all_midi_commands)
  parser.add_option("--channel", "-c", help="channel", default="0")
  parser.add_option("--param1", "-p", help="param1")
  parser.add_option("--param2", "-P", help="param2")
  parser.add_option("--interval", "-i", help="interval between each command", default=0.1, type="float")
  parser.add_option("--write", "-w", help="insert this output configuration into the configuration yaml")
  (options, args) = parser.parse_args()

  print (parseRangedNumber(options.channel))
  
  # check the argument
  if options.device_index == None:
    raise MIDIWriteError("You need to specify --device_index option")
  if options.midi_command == None:
    raise MIDIWriteError("You need to specify --midi_command option")
  if options.param1 == None:
    raise MIDIWriteError("You need to specify --param1 option")
  if options.param2 == None:
    raise MIDIWriteError("You need to specify --param2 option")

  # parse midi command
  midi_command = None
  try:
    midi_command = int(options.midi_command)
  except ValueError:
    # failed to parse integer, it might be a string
    for candidate in MIDICommand.allCommands():
      if MIDICommand.toStr(candidate).lower() == options.midi_command.lower():
        midi_command = candidate

  if midi_command == None:
    raise MIDIWriteError("Unknown midi command: %s" % (options.midi_command))

  # adding channel to midi_command
  pygame.midi.init()
  controller = pygame.midi.Output(options.device_index)
  for channel in parseRangedNumber(options.channel):
    midi_command_w_channel = midi_command | channel
    for param1 in parseRangedNumber(options.param1):
      for param2 in parseRangedNumber(options.param2):
        print "Writing [%d, %d, %d] ([0x%X, 0x%X, 0x%X])" % (midi_command_w_channel, param1, param2,
                                                             midi_command_w_channel, param1, param2)
        controller.write_short(midi_command_w_channel, param1, param2)
        rospy.sleep(options.interval)
main()
