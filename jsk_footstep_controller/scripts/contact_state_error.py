#!/usr/bin/env python
from jsk_footstep_controller.msg import GroundContactState
import rospy
import numpy
from math import pi
global data_pitch
global data_roll
global data_z
data_pitch = []
data_roll = []
data_z = []
def callback(msg):
    # if msg.contact_state == GroundContactState.CONTACT_BOTH_GROUND:
    #     print "ok"
    # else:
    #     print "no"
    data_pitch.append(msg.error_pitch_angle)
    data_roll.append(msg.error_roll_angle)
    data_z.append(msg.error_z)
    print("samples: %d" % len(data_pitch))
    print(" pitch:")
    print("   average: %f rad (%f deg)" % (numpy.average(data_pitch), numpy.average(data_pitch) / pi * 180))
    print("   stddev: %f" % numpy.std(data_pitch))
    print(" roll:")
    print("   average: %f rad (%f deg)" % (numpy.average(data_roll), numpy.average(data_roll) / pi * 180))
    print("   stddev: %f" % numpy.std(data_roll))
    print(" z:")
    print("   average: %f m" % (numpy.average(data_z)))
    print("   stddev: %f" % numpy.std(data_z))
    

def main():
    s = rospy.Subscriber("/footcoords/contact_state", GroundContactState, callback)
    rospy.spin()

if __name__ == "__main__":
  rospy.init_node("contact_state_error")
  main()
