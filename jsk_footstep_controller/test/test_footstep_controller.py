#!/usr/bin/env python

# 1. roslaunch hrpsys_gazebo_tutorials gazebo_jaxon_no_controllers.launch 
# 2. rtmlaunch hrpsys_gazebo_tutorials jaxon_hrpsys_bringup.launch KINEMATICS_MODE:=true
# 3. rosrun jsk_footstep_controller footstep-controller.l __name:=footstep_controller
# 4. rosrun jsk_footstep_controller test_footstep_controller.py

import rospy
import actionlib
import tf

from jsk_footstep_msgs.msg import Footstep, FootstepArray, ExecFootstepsAction, ExecFootstepsGoal
from geometry_msgs.msg import Pose
# pose ... matrix

from tf.transformations import *

def matToMsg(mat):
    q = quaternion_from_matrix(mat)
    p = translation_from_matrix(mat)
    pose = Pose()
    pose.orientation.x = q[0]
    pose.orientation.y = q[1]
    pose.orientation.z = q[2]
    pose.orientation.w = q[3]
    pose.position.x = p[0]
    pose.position.y = p[1]
    pose.position.z = p[2]
    return pose

def msgToMat(pose):
    return concatenate_matrices(translation_matrix([pose.position.x,
                                                    pose.position.y,
                                                    pose.position.z]),
                                quaternion_matrix([pose.orientation.x,
                                                   pose.orientation.y,
                                                   pose.orientation.z,
                                                   pose.orientation.w]))
def transRotToMat(trans, rot):
    return concatenate_matrices(translation_matrix([trans[0],
                                                    trans[1],
                                                    trans[2]]),
                                quaternion_matrix([rot[0],
                                                   rot[1],
                                                   rot[2],
                                                   rot[3]]))

def sendFootstep(footstep):
    pub_footsteps.publish(footstep)
    goal = ExecFootstepsGoal(footstep=footstep)
    client.send_goal(goal)
    client.wait_for_result()

def oneFootstep(origin, leg, x=0, y=0, z=0, roll=0, pitch=0, yaw=0):
    msg = Footstep()
    msg.leg = leg
    translation = [x, y, z]
    q = quaternion_from_euler(roll, pitch, yaw)
    offset = transRotToMat(translation, q)
    target = concatenate_matrices(origin, offset)
    print(origin)
    msg.pose = matToMsg(target)
    msg.dimensions.x = 0.23
    msg.dimensions.y = 0.13
    msg.dimensions.z = 0.001
    return msg

def getCurrentLeftLeg():
    tf_listener.waitForTransform('lleg_end_coords', 'odom', rospy.Time(0), rospy.Duration(10))
    (trans, rot) = tf_listener.lookupTransform('odom', 'lleg_end_coords', rospy.Time(0))
    return transRotToMat(trans, rot)

def currentSpot():
    origin = getCurrentLeftLeg()
    footsteps = FootstepArray()
    footsteps.footsteps = [oneFootstep(origin, Footstep.LEFT, 0, 0),
                           oneFootstep(origin, Footstep.RIGHT, 0, -0.2),
                           oneFootstep(origin, Footstep.LEFT, 0, 0),
                           oneFootstep(origin, Footstep.RIGHT, 0, -0.2),
                           oneFootstep(origin, Footstep.LEFT, 0, 0),
                           oneFootstep(origin, Footstep.RIGHT, 0, -0.2),
                           oneFootstep(origin, Footstep.LEFT, 0, 0),
                           oneFootstep(origin, Footstep.RIGHT, 0, -0.2)]
    footsteps.header.frame_id = "odom"
    footsteps.header.stamp = rospy.Time.now()
    sendFootstep(footsteps)

def straightSteps():
    origin = getCurrentLeftLeg()
    footsteps = FootstepArray()
    footsteps.footsteps = [oneFootstep(origin, Footstep.LEFT, 0, 0),
                           oneFootstep(origin, Footstep.RIGHT, 0.2, -0.2),
                           oneFootstep(origin, Footstep.LEFT, 0.4, 0),
                           oneFootstep(origin, Footstep.RIGHT, 0.6, -0.2),
                           oneFootstep(origin, Footstep.LEFT, 0.8, 0),
                           oneFootstep(origin, Footstep.RIGHT, 1.0, -0.2),
                           oneFootstep(origin, Footstep.LEFT, 1.2, 0),
                           oneFootstep(origin, Footstep.RIGHT, 1.2, -0.2)]
    footsteps.header.frame_id = "odom"
    footsteps.header.stamp = rospy.Time.now()
    sendFootstep(footsteps)

def backSteps():
    origin = getCurrentLeftLeg()
    footsteps = FootstepArray()
    footsteps.footsteps = [oneFootstep(origin, Footstep.LEFT, 0, 0),
                           oneFootstep(origin, Footstep.RIGHT, -0.2, -0.2),
                           oneFootstep(origin, Footstep.LEFT, -0.4, 0),
                           oneFootstep(origin, Footstep.RIGHT, -0.6, -0.2),
                           oneFootstep(origin, Footstep.LEFT, -0.8, 0),
                           oneFootstep(origin, Footstep.RIGHT, -1.0, -0.2),
                           oneFootstep(origin, Footstep.LEFT, -1.2, 0),
                           oneFootstep(origin, Footstep.RIGHT, -1.2, -0.2)]
    footsteps.header.frame_id = "odom"
    footsteps.header.stamp = rospy.Time.now()
    sendFootstep(footsteps)
    
def stairUp():
    origin = getCurrentLeftLeg()
    footsteps = FootstepArray()
    footsteps.footsteps = [oneFootstep(origin, Footstep.LEFT, 0, 0, 0),
                           oneFootstep(origin, Footstep.RIGHT, 0.2, -0.2, 0.0),
                           oneFootstep(origin, Footstep.LEFT, 0.4, 0, 0.1),
                           oneFootstep(origin, Footstep.RIGHT, 0.6, -0.2, 0.1),
                           oneFootstep(origin, Footstep.LEFT, 0.8, 0, 0.2),
                           oneFootstep(origin, Footstep.RIGHT, 1.0, -0.2, 0.2),
                           oneFootstep(origin, Footstep.LEFT, 1.2, 0, 0.3),
                           oneFootstep(origin, Footstep.RIGHT, 1.2, -0.2, 0.3)]
    footsteps.header.frame_id = "odom"
    footsteps.header.stamp = rospy.Time.now()
    sendFootstep(footsteps)
    
def stairDown():
    origin = getCurrentLeftLeg()
    footsteps = FootstepArray()
    footsteps.footsteps = [oneFootstep(origin, Footstep.LEFT, 0, 0, 0),
                           oneFootstep(origin, Footstep.RIGHT, 0.2, -0.2, 0.0),
                           oneFootstep(origin, Footstep.LEFT, 0.4, 0, -0.1),
                           oneFootstep(origin, Footstep.RIGHT, 0.6, -0.2, -0.1),
                           oneFootstep(origin, Footstep.LEFT, 0.8, 0, -0.2),
                           oneFootstep(origin, Footstep.RIGHT, 1.0, -0.2, -0.2),
                           oneFootstep(origin, Footstep.LEFT, 1.2, 0, -0.3),
                           oneFootstep(origin, Footstep.RIGHT, 1.2, -0.2, -0.3)]
    footsteps.header.frame_id = "odom"
    footsteps.header.stamp = rospy.Time.now()
    sendFootstep(footsteps)

def slopeUp():
    origin = getCurrentLeftLeg()
    footsteps = FootstepArray()
    footsteps.footsteps = [oneFootstep(origin, Footstep.LEFT, 0, 0, 0),
                           oneFootstep(origin, Footstep.RIGHT, 0.2, -0.2, 0.0),
                           oneFootstep(origin, Footstep.LEFT, 0.4, 0, 0.1, pitch=-0.2),
                           oneFootstep(origin, Footstep.RIGHT, 0.6, -0.2, 0.1, pitch=-0.2),
                           oneFootstep(origin, Footstep.LEFT, 0.8, 0, 0.2, pitch=-0.2),
                           oneFootstep(origin, Footstep.RIGHT, 1.0, -0.2, 0.2, pitch=-0.2),
                           oneFootstep(origin, Footstep.LEFT, 1.2, 0, 0.3, pitch=0.0),
                           oneFootstep(origin, Footstep.RIGHT, 1.2, -0.2, 0.3, pitch=0.0)]
    footsteps.header.frame_id = "odom"
    footsteps.header.stamp = rospy.Time.now()
    sendFootstep(footsteps)
    
def slopeDown():
    origin = getCurrentLeftLeg()
    footsteps = FootstepArray()
    footsteps.footsteps = [oneFootstep(origin, Footstep.LEFT, 0, 0, 0),
                           oneFootstep(origin, Footstep.RIGHT, 0.2, -0.2, 0.0),
                           oneFootstep(origin, Footstep.LEFT, 0.4, 0, -0.1, pitch=0.2),
                           oneFootstep(origin, Footstep.RIGHT, 0.6, -0.2, -0.1, pitch=0.2),
                           oneFootstep(origin, Footstep.LEFT, 0.8, 0, -0.2, pitch=0.2),
                           oneFootstep(origin, Footstep.RIGHT, 1.0, -0.2, -0.2, pitch=0.2),
                           oneFootstep(origin, Footstep.LEFT, 1.2, 0, -0.3, pitch=0.0),
                           oneFootstep(origin, Footstep.RIGHT, 1.2, -0.2, -0.3, pitch=0.0)]
    footsteps.header.frame_id = "odom"
    footsteps.header.stamp = rospy.Time.now()
    sendFootstep(footsteps)

    
if __name__ == "__main__":
    rospy.init_node("test_footstep_controller")
    tf_listener = tf.TransformListener()
    client = actionlib.SimpleActionClient('footstep_controller', ExecFootstepsAction)
    pub_footsteps = rospy.Publisher('footstep', FootstepArray, latch=True, queue_size=1)
    rospy.loginfo("Waiting for footstep controller")
    client.wait_for_server()
    rospy.loginfo("Found footstep controller")
    currentSpot()
    straightSteps()
    backSteps()
    stairUp()
    stairDown()
    slopeUp()
    slopeDown()
