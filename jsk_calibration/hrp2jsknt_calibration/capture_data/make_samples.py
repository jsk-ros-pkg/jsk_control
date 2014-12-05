#!/usr/bin/env python

# capture samples!!!!1!one

# this script should eventually be replaced by something that finds
# samples automatically

import rospy
from sensor_msgs.msg import JointState

import string, os

header1 = """camera_measurements:
- {cam_id: head_camera, config: small_cb_4x5}
joint_commands:
"""

header2_arm = """- controller: arm_controller
  segments:
  - duration: 2.0
    positions: """

header2_head = """- controller: head_controller
  segments:
  - duration: 2.0
    positions: """

header3 = """joint_measurements:
- {chain_id: arm_chain, config: tight_tol}
- {chain_id: head_chain, config: tight_tol}

sample_id: arm_"""

header4 = """target: {chain_id: arm_chain, target_id: small_cb_4x5}"""

class SampleMaker:
    
    def __init__(self):
        rospy.init_node("make_samples")
        rospy.Subscriber("joint_states", JointState, self.callback)
        self.arm_joints = ["RARM_JOINT0", "RARM_JOINT1", "RARM_JOINT2", "RARM_JOINT3", "RARM_JOINT4", "RARM_JOINT5", ]
        self.arm_state = [0.0 for joint in self.arm_joints]
        self.head_joints = ["HEAD_JOINT0", "HEAD_JOINT1", ]
        self.head_state = [0.0 for joint in self.head_joints]

        self.count = 0

        while not rospy.is_shutdown():
            print "Move arm/head to a new sample position."
            resp = raw_input("press <enter> ")
            if string.upper(resp) == "EXIT":
                break
            else:
                # save a sample:
                count = str(self.count).zfill(4)
                f = open(os.path.dirname(__file__)+"/samples/arm/arm_"+count+".yaml", "w")
                f.write(header1)
                print('saving ... {0}'.format(self.count))
                print('  arm_state: {0}'.format(self.arm_state))

                f.write(header2_arm)
                print>>f, self.arm_state
                print('  head_state: {0}'.format(self.head_state))

                f.write(header2_head)
                print>>f, self.head_state

                f.write(header3)
                print>>f, count
                f.write(header4)
            self.count += 1

    def callback(self, msg):

        for i in range(len(self.arm_joints)):
            try:
                idx = msg.name.index(self.arm_joints[i])
                self.arm_state[i] = msg.position[idx]
            except: 
                pass

        for i in range(len(self.head_joints)):
            try:
                idx = msg.name.index(self.head_joints[i])
                self.head_state[i] = msg.position[idx]
            except: 
                pass


if __name__=="__main__":
    SampleMaker()
