#!/usr/bin/env python

# only if groovy
import os
import roslib
if os.environ["ROS_DISTRO"] == "groovy":
  roslib.load_manifest('jsk_teleop_joy')

import jsk_teleop_joy.joy

if __name__ == '__main__':
  jsk_teleop_joy.joy.main()
