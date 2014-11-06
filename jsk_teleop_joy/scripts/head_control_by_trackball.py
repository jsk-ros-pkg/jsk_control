#!/usr/bin/env python

import threading
from operator import add
import math
import rospy
from sensor_msgs.msg import Joy, CameraInfo, JointState
import actionlib
import control_msgs.msg
import hrpsys_ros_bridge.srv
# try:
#     from image_view2.msg import ImageMarker2
# except:
#     import roslib; roslib.load_manifest("jsk_teleop_joy")
#     from image_view2.msg import ImageMarker2

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal 
from trajectory_msgs.msg import JointTrajectoryPoint
    
def inRange(val, min, max):
    if val < min:
        return min
    elif val > max:
        return max
    else:
        return val
    
class TrackballController:
    camera_info = None
    joint_state = None
    current_cursor = (0, 0)
    camera_info_lock = threading.Lock()
    joint_state_lock = threading.Lock()
    def camCB(self, msg):
        with self.camera_info_lock:
            if self.camera_info == None:
                # initialize current_cursor
                self.current_cursor = (msg.width / 2, msg.height / 2)
            self.camera_info = msg
    def getCameraInfo(self):
        with self.camera_info_lock:
            return self.camera_info
    def procSequence(self, sequence):
        if len(sequence) > 0:
            # integral the sequence
            dx = - reduce(add, [msg.axes[0] for msg in sequence])
            dy = reduce(add, [msg.axes[1] for msg in sequence])
            print (dx, dy)
            dpitch_deg = dx / 2.0
            dyaw_deg = dy / 2.0
            dpitch_rad = dpitch_deg / 180.0 * math.pi
            dyaw_rad = dyaw_deg / 180.0 * math.pi
            joint_state = self.getJointState()
            if not joint_state:           #not yet /joint_state is ready
                rospy.logwarn("joint_state is not yet available")
                return
            if (self.pitch_joint_name in joint_state.name and
                self.yaw_joint_name in joint_state.name):
                pitch_index = joint_state.name.index(self.pitch_joint_name)
                yaw_index = joint_state.name.index(self.yaw_joint_name)
                current_pitch = joint_state.position[pitch_index]
                current_yaw = joint_state.position[yaw_index]
                print (current_pitch, current_yaw)
                next_pitch = current_pitch + dpitch_rad
                next_yaw = current_yaw + dyaw_rad
                print (next_pitch, next_yaw)
                # move
                goal = FollowJointTrajectoryGoal()
                goal.trajectory.header.stamp = joint_state.header.stamp
                goal.trajectory.joint_names = [self.pitch_joint_name, 
                                               self.yaw_joint_name]
                p = JointTrajectoryPoint()
                p.positions = [next_pitch, next_yaw]
                p.time_from_start = rospy.Duration(0.2)   #0.2 sec
                goal.trajectory.points = [p]
                self.ac_client.send_goal(goal)
                self.ac_client.wait_for_result()
            else:
                rospy.logwarn("the joint for pitch and yaw cannot be found in joint_states")
                return
    def enableHeadGroupControl(self):
        print 'enable head joint group'
        self.enable_head_joint_group_srv(gname='head', jnames=[self.pitch_joint_name, self.yaw_joint_name])
        self.enable_head_control_flag = True
    def disableHeadGroupControl(self):
        print 'disable head joint group'
        self.disable_head_joint_group_srv(gname='head')
        self.enable_head_control_flag = False
    def jointCB(self, msg):
        with self.joint_state_lock:
            self.joint_state = msg
    def getJointState(self):
        with self.joint_state_lock:
            return self.joint_state
    def joyCB(self, msg):
        if not self.prev_buttons:
            self.prev_buttons = msg.buttons
        if (msg.buttons[0] == 1 and self.prev_buttons[0] == 0):
            self.enableHeadGroupControl()
        elif (msg.buttons[2] == 1 and self.prev_buttons[2] == 0):
            self.disableHeadGroupControl()
        elif (abs(msg.axes[0]) > 1.0 or abs(msg.axes[1]) > 1.0):
            self.procSequence([msg])
        self.prev_buttons = msg.buttons
    def joyCB2(self, msg):
        camera_info = self.getCameraInfo()
        if not camera_info:
            rospy.logfatal("camera_info is not yet available")
            return
        (raw_dx, raw_dy) = (msg.axes[0], msg.axes[1])
        current_x = self.current_cursor[0]
        current_y = self.current_cursor[1]
        next_x = inRange(raw_dx + current_x, 0, camera_info.width)
        next_y = inRange(-raw_dy + current_y, 0, camera_info.height)
        self.current_cursor = (next_x, next_y)
        self.publishCursor(msg.header.stamp, self.current_cursor)
    def publishCursor(self, stamp, pos):
        marker = ImageMarker2()
        marker.header.stamp = stamp
        marker.type = ImageMarker2.CIRCLE
        marker.position.x = pos[0]
        marker.position.y = pos[1]
        marker.scale = 10                  #5 px
        self.marker_pub.publish(marker)
    def main(self):
        self.pitch_joint_name = rospy.get_param("~pitch_joint", "head_pan_joint")
        self.yaw_joint_name = rospy.get_param("~yaw_joint", "head_tilt_joint")
        self.enable_head_joint_group_srv = rospy.ServiceProxy('/SequencePlayerServiceROSBridge/addJointGroup',
                                                         hrpsys_ros_bridge.srv.OpenHRP_SequencePlayerService_addJointGroup)
        self.disable_head_joint_group_srv = rospy.ServiceProxy('/SequencePlayerServiceROSBridge/removeJointGroup',
                                                         hrpsys_ros_bridge.srv.OpenHRP_SequencePlayerService_removeJointGroup)
        self.enableHeadGroupControl()
        self.prev_buttons = False
        self.joint_trajectory_action_name = rospy.get_param("~joint_trajectory_action", "/head_traj_controller/follow_joint_trajectory")
        self.ac_client = actionlib.SimpleActionClient(self.joint_trajectory_action_name,
                                                      control_msgs.msg.FollowJointTrajectoryAction)
        self.ac_client.wait_for_server()
        #self.marker_pub = rospy.Publisher("/image_marker", ImageMarker2)
        s = rospy.Subscriber("/trackball_joy", Joy, self.joyCB, queue_size=1)
        scam = rospy.Subscriber("camera_info", CameraInfo, self.camCB)
        sjs = rospy.Subscriber("/joint_states", JointState, self.jointCB)
        rospy.spin()

    

def main():
    controller = TrackballController()
    controller.main()
    

if __name__ == "__main__":
    rospy.init_node("head_control_by_trackball")
    main()
    
  
