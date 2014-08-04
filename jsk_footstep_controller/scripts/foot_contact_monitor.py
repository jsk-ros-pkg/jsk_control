#!/usr/bin/env python

import rospy
from geometry_msgs.msg import WrenchStamped, PoseStamped
import message_filters
import tf
import numpy
from tf.transformations import *
import diagnostic_updater
from diagnostic_msgs.msg import DiagnosticStatus

g_contact_state = None

def offsetPose(offset):
    pose = PoseStamped()
    pose.pose.position.x = offset[0]
    pose.pose.position.y = offset[1]
    pose.pose.position.z = offset[2]
    pose.pose.orientation.x = offset[3]
    pose.pose.orientation.y = offset[4]
    pose.pose.orientation.z = offset[5]
    pose.pose.orientation.w = offset[6]
    return pose

def publishMidCoords(stamp):
    l_pose = computeEndCoords("left", stamp)
    r_pose = computeEndCoords("right", stamp)
    # compute the midcoords between l_pose and r_pose
    l_pos = numpy.array((l_pose.pose.position.x,
                         l_pose.pose.position.y,
                         l_pose.pose.position.z))
    r_pos = numpy.array((r_pose.pose.position.x,
                         r_pose.pose.position.y,
                         r_pose.pose.position.z))
    l_rot = numpy.array((l_pose.pose.orientation.x,
                         l_pose.pose.orientation.y,
                         l_pose.pose.orientation.z,
                         l_pose.pose.orientation.w))
    r_rot = numpy.array((r_pose.pose.orientation.x,
                         r_pose.pose.orientation.y,
                         r_pose.pose.orientation.z,
                         r_pose.pose.orientation.w))
    mid_pos = (l_pos + r_pos) / 2.0
    mid_rot = quaternion_slerp(l_rot, r_rot, 0.5)
    br = tf.TransformBroadcaster()
    br.sendTransform(mid_pos, mid_rot,
                     stamp,
                     ground_frame,
                     parent_frame)

def computeEndCoords(leg, stamp):
    if leg == "left":
        foot_frame_id = lfoot_frame
        foot_offset_pose = offsetPose(lfoot_offset)
    elif leg == "right":
        foot_frame_id = rfoot_frame
        foot_offset_pose = offsetPose(rfoot_offset)
    else:
        raise Exception("unknown leg specified")
    foot_offset_pose.header.frame_id = foot_frame_id
    foot_offset_pose.header.stamp = stamp
    tf_listener.waitForTransform(parent_frame, foot_offset_pose.header.frame_id, 
                                 foot_offset_pose.header.stamp,
                                 rospy.Duration(1.0))
    pose_stamped = tf_listener.transformPose(parent_frame, foot_offset_pose)
    return pose_stamped

def publishSingleCoords(leg, stamp):
    pose_stamped = computeEndCoords(leg, stamp)
    br = tf.TransformBroadcaster()
    br.sendTransform((pose_stamped.pose.position.x,
                      pose_stamped.pose.position.y,
                      pose_stamped.pose.position.z),
                     (pose_stamped.pose.orientation.x,
                      pose_stamped.pose.orientation.y,
                      pose_stamped.pose.orientation.z,
                      pose_stamped.pose.orientation.w),
                     pose_stamped.header.stamp,
                     ground_frame,
                     pose_stamped.header.frame_id)

def callback(lfoot_force_msg, rfoot_force_msg):
    global force_thr, g_contact_state
    if lfoot_force_msg.wrench.force.z > force_thr:
        lfoot_contact = True
    else:
        lfoot_contact = False
    if rfoot_force_msg.wrench.force.z > force_thr:
        rfoot_contact = True
    else:
        rfoot_contact = False
    try:
        if lfoot_contact and rfoot_contact:
            g_contact_state = "both"
            publishMidCoords(lfoot_force_msg.header.stamp)
        elif lfoot_contact:
            g_contact_state = "lleg"
            publishSingleCoords("left", lfoot_force_msg.header.stamp)
        elif rfoot_contact:
            g_contact_state = "rleg"
            publishSingleCoords("right", rfoot_force_msg.header.stamp)
        else:
            g_contact_state = "air"
            publishMidCoords(lfoot_force_msg.header.stamp)
        updater.update()
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.logerr("tf error, ignored")


def contactStateDiagnostic(stats):
    if g_contact_state == "both":
        stats.summary(DiagnosticStatus.OK,
                      "BOTH legs on the floor")
    elif g_contact_state == "lleg":
        stats.summary(DiagnosticStatus.OK,
                      "LEFT leg on the floor")
    elif g_contact_state == "rleg":
        stats.summary(DiagnosticStatus.OK,
                      "RIGHT leg on the floor")
    elif g_contact_state == "air":
        stats.summary(DiagnosticStatus.WARN,
                      "robot on the AIR")
    else:
        stats.summary(DiagnosticStatus.ERROR,
                      "failed to estimate contact state")
    stats.add("contact state", g_contact_state)

if __name__ == "__main__":
    rospy.init_node("foot_contact_monitor")
    tf_listener = tf.TransformListener()
    updater = diagnostic_updater.Updater()
    updater.setHardwareID(rospy.get_name())
    updater.add('ContactState', contactStateDiagnostic)
    force_thr = rospy.get_param('~force_thr', 100)
    ground_frame = rospy.get_param('~ground_frame', 'ground')
    parent_frame = rospy.get_param('~parent_frame', 'BODY')
    lfoot_frame = rospy.get_param('~lfoot_frame_id')
    rfoot_frame = rospy.get_param('~rfoot_frame_id')
    lfoot_offset = rospy.get_param('~lfoot_endcoords_offset')
    rfoot_offset = rospy.get_param('~rfoot_endcoords_offset')
    ground_parent_frame = rospy.get_param('')
    lfoot_sub = message_filters.Subscriber('/off_lfsensor', WrenchStamped)
    rfoot_sub = message_filters.Subscriber('/off_rfsensor', WrenchStamped)
    ts = message_filters.TimeSynchronizer([lfoot_sub, rfoot_sub], 100)
    ts.registerCallback(callback)
    rospy.spin()

