#!/usr/bin/env python
from __future__ import print_function

import pygame
import pygame.midi
import sys
import time
import roslib
roslib.load_manifest('jsk_teleop_joy')
from jsk_teleop_joy.midi_util import MIDICommand, MIDIParse

def checkUpperByte(ref, val):
  return ((ref | val) >> 4 == (ref >> 4))

def main():
  pygame.midi.init()
  controller = pygame.midi.Input(int(sys.argv[1]))
  while True:
    while controller.poll():
      data = controller.read(1)
      for elem_set in data:
        midi_command = elem_set[0][0]
        print(elem_set[0], end=' ')
        print("(0x%X, 0x%X, 0x%X)" % (midi_command, elem_set[0][1], elem_set[0][2]), end=' ')
        print(MIDICommand.toStr(MIDICommand.detect(midi_command)))
        print(MIDIParse(elem_set))
    time.sleep(0.1)
  
if __name__ == "__main__":
  main()
