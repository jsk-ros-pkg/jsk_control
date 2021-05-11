#!/usr/bin/env python

import rospy
import sys
from jsk_footstep_msgs.msg import Footstep, FootstepArray
import tf
from tf.transformations import *

def callback(msg):
    global tf_listener, frame_id, pub
    try:
        if strict_tf:
            tf_listener.waitForTransform(frame_id, msg.header.frame_id,
                                         msg.header.stamp,
                                         rospy.Duration(1.0))
        if strict_tf:
            stamp = msg.header.stamp
        else:
            stamp = rospy.Time(0.0)
        # frame_id -> header
        (pos, rot) = tf_listener.lookupTransform(frame_id, msg.header.frame_id,
                                                 stamp)
        trans = concatenate_matrices(translation_matrix(pos), quaternion_matrix(rot))
        new_msg = FootstepArray()
        new_msg.header.frame_id = frame_id
        new_msg.header.stamp = msg.header.stamp
        for footstep in msg.footsteps:
            new_footstep = Footstep()
            new_footstep.leg = footstep.leg
            new_footstep.duration = footstep.duration
            new_footstep.footstep_group = footstep.footstep_group
            new_footstep.swing_height = footstep.swing_height
            new_footstep.dimensions = footstep.dimensions
            old_pose = concatenate_matrices(translation_matrix([
                footstep.pose.position.x,
                footstep.pose.position.y,
                footstep.pose.position.z]),
                                            quaternion_matrix([
                                                footstep.pose.orientation.x,
                                                footstep.pose.orientation.y,
                                                footstep.pose.orientation.z,
                                                footstep.pose.orientation.w]),
            )
            new_pose = concatenate_matrices(trans, old_pose)
            translation = translation_from_matrix(new_pose)
            rotation = quaternion_from_matrix(new_pose)
            new_footstep.pose.position.x = translation[0]
            new_footstep.pose.position.y = translation[1]
            new_footstep.pose.position.z = translation[2]
            new_footstep.pose.orientation.x = rotation[0]
            new_footstep.pose.orientation.y = rotation[1]
            new_footstep.pose.orientation.z = rotation[2]
            new_footstep.pose.orientation.w = rotation[3]
            new_msg.footsteps.append(new_footstep)
        pub.publish(new_msg)
    except tf.Exception as e:
        rospy.logerr("[transform_footstep_array] Failed to lookup transform: %s" % (e.message))
    

if __name__ == "__main__":
    rospy.init_node("transform_footstep_array")
    tf_listener = tf.TransformListener() # Should we use tf2?
    pub = rospy.Publisher("~output", FootstepArray)
    frame_id = rospy.get_param("~frame_id", "odom")
    strict_tf = rospy.get_param("~strict_tf", True)
    s = rospy.Subscriber("~input", FootstepArray, callback)
    rospy.spin()
    
