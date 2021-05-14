#!/usr/bin/env python
#####################################################################
# Software License Agreement (BSD License)
#
#  Copyright (c) 2015, JSK Lab
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/o2r other materials provided
#     with the distribution.
#   * Neither the name of the JSK Lab nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
#####################################################################


import rospy
from sensor_msgs.msg import JointState

prev_joint_states = None
pub = None

def callback(msg):
    global prev_joint_states, pub
    if not prev_joint_states:
        # initial time
        # just pass the input message to output
        pub.publish(msg)
        prev_joint_states = msg
        # force to use list instead of tuple.
        prev_joint_states.name = list(prev_joint_states.name)
        prev_joint_states.position = list(prev_joint_states.position)
        if len(prev_joint_states.velocity) > 0:
            prev_joint_states.velocity = list(prev_joint_states.velocity)
        else:
            prev_joint_states.velocity = [0] * len(prev_joint_states.name)
        if len(prev_joint_states.effort) > 0:
            prev_joint_states.effort = list(prev_joint_states.effort)
        else:
            prev_joint_states.effort = [0] * len(prev_joint_states.name)
    else:
        if len(msg.velocity) > 0:
            velocity = msg.velocity
        else:
            velocity = [0] * len(msg.name)
        if len(msg.effort) > 0:
            effort = msg.effort
        else:
            effort = [0] * len(msg.name)

        for name, pos, vel, ef in zip(msg.name, msg.position, velocity, effort):
            if name in prev_joint_states.name:
                # if name is already listed in prev_joint_states,
                # update value
                index = prev_joint_states.name.index(name)
                prev_joint_states.position[index] = pos
                prev_joint_states.velocity[index] = vel
                prev_joint_states.effort[index] = ef
            else:
                # if name is not yet listed in prev_joint_states,
                # append name and position to prev_joint_states
                prev_joint_states.name.append(name)
                prev_joint_states.position.append(pos)
                prev_joint_states.effort.append(ef)
                prev_joint_states.velocity.append(vel)
        if prev_joint_states.header.stamp < msg.header.stamp:
            prev_joint_states.header.stamp = msg.header.stamp
        pub.publish(prev_joint_states)
        
if __name__ == "__main__":
    rospy.init_node("joint_states_appender")
    pub = rospy.Publisher("joint_states_appended", JointState, queue_size=1)
    sub = rospy.Subscriber("joint_states", JointState, callback, queue_size=1)
    rospy.spin()
