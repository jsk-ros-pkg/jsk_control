#!/usr/bin/env python
import pygame
import pygame.midi
import sys
import time

def main():
  pygame.midi.init()
  controller = pygame.midi.Input(int(sys.argv[1]))
  while True:
    while controller.poll():
      data = controller.read(1)
      for elem_set in data:
        print elem_set
    time.sleep(0.1)
  
if __name__ == "__main__":
  main()
