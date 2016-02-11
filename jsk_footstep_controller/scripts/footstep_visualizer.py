#!/usr/bin/env python

from hrpsys_ros_bridge.srv import OpenHRP_StabilizerService_getParameter as getParameter
import rospy
from tf2_py import ExtrapolationException
import tf2_ros
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Image

from cv_bridge import CvBridge
from tf.transformations import *
import cv2 as cv
import numpy as np
from threading import Lock
from math import pi, isnan
# hrpsys robot only
from hrpsys_ros_bridge.msg import ContactStatesStamped, ContactStateStamped, ContactState
from jsk_footstep_controller.cfg import FootstepVisualizerConfig
from dynamic_reconfigure.server import Server

msg_lock = Lock()
def matrixFromTranslationQuaternion(trans, q):
    return  concatenate_matrices(translation_matrix(trans),
                                 quaternion_matrix(q))

def verticesPoints(original_vertices, origin_pose, scale, resolution_size):
    original_vertices_3d_local = [[v[0], v[1], 0.0] for v in original_vertices]
    vertices_3d_pose = [concatenate_matrices(origin_pose, translation_matrix(v)) for v in original_vertices_3d_local]
    vertices_3d = [translation_from_matrix(v) for v in vertices_3d_pose]
    
    return [(int(v[0] / (scale / 2.0) * resolution_size / 2.0 + resolution_size / 2.0),
             int(v[1] / (scale / 2.0) * resolution_size / 2.0 + resolution_size / 2.0))
            for v in vertices_3d]

def verticesPoints3D(original_vertices, origin_pose, scale, resolution_size):
    for v in original_vertices:
        if isnan(v[0]) or isnan(v[1]) or isnan(v[2]):
            return [False]
    original_vertices_3d_local = original_vertices
    vertices_3d_pose = [concatenate_matrices(origin_pose, translation_matrix(v)) for v in original_vertices_3d_local]
    vertices_3d = [translation_from_matrix(v) for v in vertices_3d_pose]
    
        
    return [(int(v[0] / (scale / 2.0) * resolution_size / 2.0 + resolution_size / 2.0),
             int(v[1] / (scale / 2.0) * resolution_size / 2.0 + resolution_size / 2.0))
            for v in vertices_3d]

def transformToMatrix(transform):
    return matrixFromTranslationQuaternion([transform.translation.x,
                                            transform.translation.y,
                                            transform.translation.z],
                                           [transform.rotation.x,
                                            transform.rotation.y,
                                            transform.rotation.z,
                                            transform.rotation.w])

ACT_CP_COLOR=(192, 202, 164)
REF_CP_COLOR=(167, 60, 151)

def drawPoint(image, point, size, color, text):
    cv.circle(image, point, 7, color + (255,), -1)
    cv.putText(image, text, (point[0] + 5, point[1] + 5),
               cv.FONT_HERSHEY_PLAIN, 1.0,  color + (255,))


#def cop_callback(lleg_cop, rleg_cop):
def periodicCallback(event):
    global tf_buffer, lleg_cop_msg, rleg_cop_msg, zmp_msg, act_cp_msg, ref_cp_msg, act_contact_states_msg
    print "run"
    with msg_lock:
     try:
        _lleg_pose = tf_buffer.lookup_transform(root_link, lleg_end_coords, rospy.Time())
        _rleg_pose = tf_buffer.lookup_transform(root_link, rleg_end_coords, rospy.Time())
        if lleg_cop_msg:
            _lleg_cop_origin = tf_buffer.lookup_transform(root_link, lleg_cop_msg.header.frame_id, rospy.Time())
            lleg_cop_origin = transformToMatrix(_lleg_cop_origin.transform)
            lleg_cop_point = np.array([lleg_cop_msg.point.x,
                                       lleg_cop_msg.point.y,
                                       lleg_cop_msg.point.z])
        if rleg_cop_msg:
            _rleg_cop_origin = tf_buffer.lookup_transform(root_link, rleg_cop_msg.header.frame_id, rospy.Time())
            rleg_cop_origin = transformToMatrix(_rleg_cop_origin.transform)
            rleg_cop_point = np.array([rleg_cop_msg.point.x,
                                       rleg_cop_msg.point.y,
                                       rleg_cop_msg.point.z])
        if zmp_msg:
            _zmp_origin = tf_buffer.lookup_transform(root_link, zmp_msg.header.frame_id, rospy.Time())
            zmp_origin = transformToMatrix(_zmp_origin.transform)
            zmp_point = np.array([zmp_msg.point.x,
                                  zmp_msg.point.y,
                                  zmp_msg.point.z])
        if act_cp_msg:
            _act_cp_origin = tf_buffer.lookup_transform(root_link, act_cp_msg.header.frame_id, rospy.Time())
            act_cp_origin = transformToMatrix(_act_cp_origin.transform)
            act_cp_point = np.array([act_cp_msg.point.x,
                                     act_cp_msg.point.y,
                                     act_cp_msg.point.z])
        if ref_cp_msg:
            _ref_cp_origin = tf_buffer.lookup_transform(root_link, ref_cp_msg.header.frame_id, rospy.Time())
            ref_cp_origin = transformToMatrix(_ref_cp_origin.transform)
            ref_cp_point = np.array([ref_cp_msg.point.x,
                                       ref_cp_msg.point.y,
                                       ref_cp_msg.point.z])
        lleg_pos = np.array([_lleg_pose.transform.translation.x,
                             _lleg_pose.transform.translation.y,
                             _lleg_pose.transform.translation.z])
        lleg_rot = np.array([_lleg_pose.transform.rotation.x,
                             _lleg_pose.transform.rotation.y,
                             _lleg_pose.transform.rotation.z,
                             _lleg_pose.transform.rotation.w])
        rleg_pos = np.array([_rleg_pose.transform.translation.x,
                             _rleg_pose.transform.translation.y,
                             _rleg_pose.transform.translation.z])
        rleg_rot = np.array([_rleg_pose.transform.rotation.x,
                             _rleg_pose.transform.rotation.y,
                             _rleg_pose.transform.rotation.z,
                             _rleg_pose.transform.rotation.w])
        
        lleg_pose = matrixFromTranslationQuaternion(lleg_pos, lleg_rot)
        rleg_pose = matrixFromTranslationQuaternion(rleg_pos, rleg_rot)
        mid_coords = concatenate_matrices(matrixFromTranslationQuaternion((lleg_pos + rleg_pos) / 2.0,
                                                                          quaternion_slerp(lleg_rot, rleg_rot, 0.5)),
                                          euler_matrix(pi, 0, 0))
        lleg_from_mid = concatenate_matrices(inverse_matrix(mid_coords),
                                             lleg_pose)
        rleg_from_mid = concatenate_matrices(inverse_matrix(mid_coords),
                                             rleg_pose)
        lleg_points = verticesPoints(lleg_vertices, lleg_from_mid, scale, image_size)
        rleg_points = verticesPoints(rleg_vertices, rleg_from_mid, scale, image_size)
        lleg_margined_points = verticesPoints(lleg_margined_vertices, lleg_from_mid, scale, image_size)
        rleg_margined_points = verticesPoints(rleg_margined_vertices, rleg_from_mid, scale, image_size)
        image = np.zeros((image_size, image_size, 4), dtype=np.uint8)
        lleg_contact = False
        rleg_contact = False
        lleg_color = (0, 255, 0, 255)
        rleg_color = (0, 0, 255, 255)
        lleg_width = 2
        rleg_width = 2
        if act_contact_states_msg:
            for state in act_contact_states_msg.states:
                if state.header.frame_id == "lfsensor":
                    if state.state.state == ContactState.ON:
                        lleg_color = (0, 255, 0, 255)
                        lleg_width = 3
                        lleg_contact = True
                    else:
                        lleg_color = (150, 255, 150, 255)
                        lleg_width = 1
                if state.header.frame_id == "rfsensor":
                    if state.state.state == ContactState.ON:
                        rleg_color = (0, 0, 255, 255)
                        rleg_width = 3
                        rleg_contact = True
                    else:
                        rleg_color = (150, 150, 255, 255)
                        rleg_width = 1
            # act_contact_states_msg = None
        cv.line(image, lleg_points[0], lleg_points[1], lleg_color, lleg_width)
        cv.line(image, lleg_points[1], lleg_points[2], lleg_color, lleg_width)
        cv.line(image, lleg_points[2], lleg_points[3], lleg_color, lleg_width)
        cv.line(image, lleg_points[3], lleg_points[0], lleg_color, lleg_width)
        cv.line(image, rleg_points[0], rleg_points[1], rleg_color, rleg_width)
        cv.line(image, rleg_points[1], rleg_points[2], rleg_color, rleg_width)
        cv.line(image, rleg_points[2], rleg_points[3], rleg_color, rleg_width)
        cv.line(image, rleg_points[3], rleg_points[0], rleg_color, rleg_width)
        if lleg_contact and rleg_contact:
            hull = cv.convexHull(np.array(lleg_points + rleg_points))
            cv.drawContours(image, [hull], -1, (155, 155, 155, 255), 2)
            margined_hull = cv.convexHull(np.array(lleg_margined_points + rleg_margined_points))
        elif lleg_contact:
            margined_hull = cv.convexHull(np.array(lleg_margined_points))
        elif rleg_contact:
            margined_hull = cv.convexHull(np.array(rleg_margined_points))
        if (lleg_contact or rleg_contact) and g_config.cp_safe_area:
            cv.drawContours(image, [margined_hull], -1, (255, 255, 255, 255), 2)
        if lleg_cop_msg and g_config.lcop:
            lleg_cop_point_2d = verticesPoints3D([lleg_cop_point],
                                                 concatenate_matrices(inverse_matrix(mid_coords),
                                                                      lleg_cop_origin),
                                                 scale,
                                                 image_size)[0]
            if lleg_cop_point_2d:
                drawPoint(image, lleg_cop_point_2d, 5, (0, 255, 0), "LCoP")
            # lleg_cop_msg = None
        if rleg_cop_msg and g_config.rcop:
            rleg_cop_point_2d = verticesPoints3D([rleg_cop_point],
                                                 concatenate_matrices(inverse_matrix(mid_coords),
                                                                      rleg_cop_origin),
                                                 scale,
                                                 image_size)[0]
            if rleg_cop_point_2d:
                drawPoint(image, rleg_cop_point_2d, 5, (0, 0, 255), "RCoP")
            # rleg_cop_msg = None
        if zmp_msg and g_config.zmp:
            zmp_point_2d = verticesPoints3D([zmp_point],
                                            concatenate_matrices(inverse_matrix(mid_coords),
                                                                 zmp_origin),
                                            scale,
                                            image_size)[0]
            if zmp_point_2d:
                drawPoint(image, zmp_point_2d, 5, (0, 255, 255), "ZMP")
            # zmp_msg = None
        if ref_cp_msg and g_config.ref_cp:
            ref_cp_point_2d = verticesPoints3D([ref_cp_point],
                                               concatenate_matrices(inverse_matrix(mid_coords),
                                                                    ref_cp_origin),
                                               scale,
                                               image_size)[0]
            if ref_cp_point_2d:
                drawPoint(image, ref_cp_point_2d, 7, REF_CP_COLOR, "RCP")
            # ref_cp_msg = None
        if act_cp_msg and g_config.act_cp:
            act_cp_point_2d = verticesPoints3D([act_cp_point],
                                               concatenate_matrices(inverse_matrix(mid_coords),
                                                                    act_cp_origin),
                                               scale,
                                               image_size)[0]
            if act_cp_point_2d:
                drawPoint(image, act_cp_point_2d, 7, ACT_CP_COLOR, "ACP")
            # act_cp_msg = None
        bridge = CvBridge()
        pub.publish(bridge.cv2_to_imgmsg(image, "bgra8"))
     except ExtrapolationException:
         rospy.logwarn("tf error")
     finally:
        pass
     # except Exception, e:
     #     print e

lleg_cop_msg = None
rleg_cop_msg = None
zmp_msg = None
act_cp_msg = None
ref_cp_msg = None
act_contact_states_msg = None
def llegCopCallback(msg):
    global lleg_cop_msg
    with msg_lock:
        lleg_cop_msg = msg


def rlegCopCallback(msg):
    global rleg_cop_msg
    with msg_lock:
        rleg_cop_msg = msg

def actCPCallback(msg):
    global act_cp_msg
    with msg_lock:
        act_cp_msg = msg

def refCPCallback(msg):
    global ref_cp_msg
    with msg_lock:
        ref_cp_msg = msg

def contactStatesCallback(msg):
    global act_contact_states_msg
    with msg_lock:
        act_contact_states_msg = msg

def zmpCallback(msg):
    global zmp_msg
    with msg_lock:
        zmp_msg = msg

def config_callback(config, level):
    global g_config
    g_config = config
    return config

if __name__ == "__main__":
    rospy.init_node("footstep_visualizer")
    pub = rospy.Publisher("~output", Image)
    srv = Server(FootstepVisualizerConfig, config_callback)
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    rospy.loginfo("wait for st server")
    rospy.wait_for_service("/StabilizerServiceROSBridge/getParameter")
    rospy.loginfo("st server found")
    g_get_parameter_srv = rospy.ServiceProxy("/StabilizerServiceROSBridge/getParameter", getParameter)
    lleg_end_coords = rospy.get_param("~lleg_end_coords", "lleg_end_coords")
    rleg_end_coords = rospy.get_param("~rleg_end_coords", "rleg_end_coords")
    image_size = rospy.get_param("~image_size", 400)
    scale = rospy.get_param("~scale", 0.6)
    i_param = g_get_parameter_srv().i_param
    leg_front_margin = i_param.eefm_leg_front_margin
    leg_rear_margin = i_param.eefm_leg_rear_margin
    leg_inside_margin = i_param.eefm_leg_inside_margin
    leg_outside_margin = i_param.eefm_leg_outside_margin
    cp_check_margin = i_param.cp_check_margin
    lleg_vertices = [[leg_front_margin, leg_outside_margin],
                     [leg_front_margin, -1*leg_inside_margin],
                     [-1*leg_rear_margin, -1*leg_inside_margin],
                     [-1*leg_rear_margin, leg_outside_margin]]
    rleg_vertices = [[leg_front_margin, leg_inside_margin],
                     [leg_front_margin, -1*leg_outside_margin],
                     [-1*leg_rear_margin, -1*leg_outside_margin],
                     [-1*leg_rear_margin, leg_inside_margin]]
    lleg_margined_vertices = [[leg_front_margin-cp_check_margin[0], leg_outside_margin-cp_check_margin[3]],
                              [leg_front_margin-cp_check_margin[0], -1*(leg_inside_margin-cp_check_margin[2])],
                              [-1*(leg_rear_margin-cp_check_margin[1]), -1*(leg_inside_margin-cp_check_margin[2])],
                              [-1*(leg_rear_margin-cp_check_margin[1]), leg_outside_margin-cp_check_margin[3]]]
    rleg_margined_vertices = [[leg_front_margin-cp_check_margin[0], leg_inside_margin-cp_check_margin[2]],
                              [leg_front_margin-cp_check_margin[0], -1*(leg_outside_margin-cp_check_margin[3])],
                              [-1*(leg_rear_margin-cp_check_margin[1]), -1*(leg_outside_margin-cp_check_margin[3])],
                              [-1*(leg_rear_margin-cp_check_margin[1]), leg_inside_margin-cp_check_margin[2]]]
    root_link = rospy.get_param("~root_link", "BODY")
    lleg_sub = rospy.Subscriber("/lfsensor_cop", PointStamped, llegCopCallback)
    rleg_sub = rospy.Subscriber("/rfsensor_cop", PointStamped, rlegCopCallback)
    zmp_sub = rospy.Subscriber("/zmp", PointStamped, zmpCallback)
    act_cp_point_sub = rospy.Subscriber("/act_capture_point", PointStamped, actCPCallback)
    ref_cp_point_sub = rospy.Subscriber("/ref_capture_point", PointStamped, refCPCallback)
    contact_states_sub = rospy.Subscriber("/act_contact_states", ContactStatesStamped, contactStatesCallback)
    # lleg_cop_sub = Subscriber("/lfsensor_cop", PointStamped)
    # rleg_cop_sub = Subscriber("/rfsensor_cop", PointStamped)
    # ts = TimeSynchronizer([lleg_cop_sub, rleg_cop_sub], 10)
    # ts.registerCallback(cop_callback)
    rospy.Timer(rospy.Duration(0.1), periodicCallback)
    rospy.spin()
    
