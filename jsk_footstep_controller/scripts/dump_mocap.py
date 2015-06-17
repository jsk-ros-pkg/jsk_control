#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from tf.transformations import *
import os
import tf

initial_pose = None
world_pose = None
def poseMsgToMatrix(pose):
    return concatenate_matrices(translation_matrix([pose.position.x,
                                                    pose.position.y,
                                                    pose.position.z]),
                                quaternion_matrix([pose.orientation.x,
                                                   pose.orientation.y,
                                                   pose.orientation.z,
                                                   pose.orientation.w]))




def callback(msg):
    global f, initial_pose, world_pose
    if world_pose == None:
        rospy.loginfo("calibrating world coordinates")
        A = identity_matrix()
        B = poseMsgToMatrix(msg.pose)
        C = concatenate_matrices(A, inverse_matrix(B))
        world_pose = C
    # x, y, z, qx, qy, qz, qw
    pose = poseMsgToMatrix(msg.pose)
    new_A = concatenate_matrices(world_pose, pose)
    diff = new_A
    diff_pos = translation_from_matrix(diff)
    diff_rot = quaternion_from_matrix(diff)
    f.write("%f, %f, %f, %f, %f, %f, %f\n" % (diff_pos[0], diff_pos[1],
                                              diff_pos[2],
                                              diff_rot[0],
                                              diff_rot[1],
                                              diff_rot[2],
                                              diff_rot[3]))

if __name__ == "__main__":
    rospy.init_node("dump_mocap")
    file_name = rospy.get_param("~file_name", "output.csv")
    tf_listener = tf.TransformListener()
    try:
        f = open(file_name, "w")
        sub = rospy.Subscriber("/Robot_1/pose", PoseStamped, callback)
        rospy.spin()
    finally:
        f.close()
        
        
    
