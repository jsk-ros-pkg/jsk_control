#!/usr/bin/env python
import rospy
import numpy
import math
import tf
import numpy
import time

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy

class MoveMode:
    WorldMode = 1
    LocalMode = 2

class Spacenav2Pose():
    def __init__(self):
        rospy.init_node("spacenav2pose")
        self.pre_pose = PoseStamped()
        self.pre_pose.pose.orientation.w = 1
        self.prev_time = rospy.Time.from_sec(time.time())
        self.prev_joy = None

        self.publish_pose = True
        self.move_mode = MoveMode.WorldMode

        self.frame_id = rospy.get_param('~frame_id', '/map')
        if self.publish_pose:
            self.pose_pub = rospy.Publisher(rospy.get_param('~pose', 'pose'),
                                            PoseStamped)
        self.pose_sub = rospy.Subscriber(rospy.get_param('~set_pose', 'set_pose'), PoseStamped, self.setPoseCB)

        joy_topic = rospy.get_param('~joy', '/spacenav/joy')
        self.joy_sub = rospy.Subscriber(joy_topic, Joy, self.joyCB)
        rospy.logwarn("subscribe " + joy_topic)

        self.x_scale = rospy.get_param('~x_scale', 0.03)
        self.y_scale = rospy.get_param('~y_scale', 0.03)
        self.z_scale = rospy.get_param('~z_scale', 0.03)

        self.r_scale = rospy.get_param('~r_scale', 0.01)
        self.p_scale = rospy.get_param('~p_scale', 0.01)
        self.y_scale = rospy.get_param('~y_scale', 0.01)

        self.tf_listener = tf.TransformListener()

        rospy.spin()

    def setPoseCB(self, pose):
        pose.header.stamp = rospy.Time(0)
        self.pre_pose = self.tf_listener.transformPose(self.frame_id, pose)

        if self.publish_pose:
            self.pose_pub.publish(self.pre_pose)

    def joyCB(self, joy):
        b0 = joy.buttons[0]
        b1 = joy.buttons[1]
        if self.prev_joy:
            # change move mode
            if b0 == 1 and self.prev_joy.buttons[0] == 0:
                if self.move_mode == MoveMode.WorldMode:
                    self.move_mode = MoveMode.LocalMode
                    rospy.logwarn("change mode to local mode")
                else:
                    self.move_mode = MoveMode.WorldMode
                    rospy.logwarn("change mode to world mode")

        self.prev_joy = joy

        pre_pose = self.pre_pose

        new_pose = PoseStamped()
        new_pose.header.frame_id = self.frame_id
        new_pose.header.stamp = rospy.Time(0.0)

        # calculate position
        q = numpy.array((pre_pose.pose.orientation.x,
                         pre_pose.pose.orientation.y,
                         pre_pose.pose.orientation.z,
                         pre_pose.pose.orientation.w))

        x_diff = joy.axes[0] * self.x_scale
        y_diff = joy.axes[1] * self.y_scale
        z_diff = joy.axes[2] * self.z_scale
        local_move = numpy.array((x_diff, y_diff, z_diff, 1.0))

        if self.move_mode == MoveMode.WorldMode:
            move_q = numpy.array((0.0, 0.0, 0.0, 1.0))
        else:
            move_q = q

        xyz_move = numpy.dot(tf.transformations.quaternion_matrix(move_q),
                             local_move)

        new_pose.pose.position.x = pre_pose.pose.position.x + xyz_move[0]
        new_pose.pose.position.y = pre_pose.pose.position.y + xyz_move[1]
        new_pose.pose.position.z = pre_pose.pose.position.z + xyz_move[2]

        # calculate orientation
        roll =  self.r_scale * joy.axes[3]
        pitch =  self.p_scale * joy.axes[4]
        yaw =  self.y_scale * joy.axes[5]

        diff_q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

        if self.move_mode == MoveMode.WorldMode:
            new_q = tf.transformations.quaternion_multiply(diff_q, q)
        else:
            new_q = tf.transformations.quaternion_multiply(q, diff_q)

        new_pose.pose.orientation.x = new_q[0]
        new_pose.pose.orientation.y = new_q[1]
        new_pose.pose.orientation.z = new_q[2]
        new_pose.pose.orientation.w = new_q[3]

        # publish at 10hz
        if self.publish_pose:
            now = rospy.Time.from_sec(time.time())
            # placement.time_from_start = now - self.prev_time
            if (now - self.prev_time).to_sec() > 1 / 30.0:
                self.pose_pub.publish(new_pose)
                self.prev_time = now

        self.pre_pose = new_pose


if __name__ == '__main__':
    Spacenav2Pose()
