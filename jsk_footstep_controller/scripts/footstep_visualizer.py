#!/usr/bin/env python

import rospy
import tf2_ros
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Image

from cv_bridge import CvBridge
from tf.transformations import *
import cv2 as cv
import numpy as np
from threading import Lock
from math import pi
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

def transformToMatrix(transform):
    return matrixFromTranslationQuaternion([transform.translation.x,
                                            transform.translation.y,
                                            transform.translation.z],
                                           [transform.rotation.x,
                                            transform.rotation.y,
                                            transform.rotation.z,
                                            transform.rotation.w])

#def cop_callback(lleg_cop, rleg_cop):
def periodicCallback(event):
    global tf_buffer, lleg_cop_msg, rleg_cop_msg, zmp_msg
    try:
        msg_lock.acquire()
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
        image = np.zeros((image_size, image_size, 4), dtype=np.uint8)
        cv.line(image, lleg_points[0], lleg_points[1], (0, 255, 0, 255), 2)
        cv.line(image, lleg_points[1], lleg_points[2], (0, 255, 0, 255), 2)
        cv.line(image, lleg_points[2], lleg_points[3], (0, 255, 0, 255), 2)
        cv.line(image, lleg_points[3], lleg_points[0], (0, 255, 0, 255), 2)
        cv.line(image, rleg_points[0], rleg_points[1], (0, 0, 255, 255), 2)
        cv.line(image, rleg_points[1], rleg_points[2], (0, 0, 255, 255), 2)
        cv.line(image, rleg_points[2], rleg_points[3], (0, 0, 255, 255), 2)
        cv.line(image, rleg_points[3], rleg_points[0], (0, 0, 255, 255), 2)
        if lleg_cop_msg:
            lleg_cop_point_2d = verticesPoints([lleg_cop_point],
                                               concatenate_matrices(inverse_matrix(mid_coords),
                                                                    lleg_cop_origin),
                                               scale,
                                               image_size)[0]
            cv.circle(image, lleg_cop_point_2d, 5, (0, 255, 0, 255), -1)
            lleg_cop_msg = None
        if rleg_cop_msg:
            rleg_cop_point_2d = verticesPoints([rleg_cop_point],
                                               concatenate_matrices(inverse_matrix(mid_coords),
                                                                    rleg_cop_origin),
                                               scale,
                                               image_size)[0]
            cv.circle(image, rleg_cop_point_2d, 5, (0, 0, 255, 255), -1)
            rleg_cop_msg = None
        if zmp_msg:
            zmp_point_2d = verticesPoints([zmp_point],
                                               concatenate_matrices(inverse_matrix(mid_coords),
                                                                    zmp_origin),
                                               scale,
                                               image_size)[0]
            cv.circle(image, zmp_point_2d, 5, (0, 255, 255, 255), -1)
            zmp_msg = None
        bridge = CvBridge()
        pub.publish(bridge.cv2_to_imgmsg(image, "bgra8"))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        # rospy.logerr("Failed to lookup transform")
        pass
    finally:
        msg_lock.release()

lleg_cop_msg = None
rleg_cop_msg = None
zmp_msg = None
def llegCopCallback(msg):
    global lleg_cop_msg
    with msg_lock:
        lleg_cop_msg = msg


def rlegCopCallback(msg):
    global rleg_cop_msg
    with msg_lock:
        rleg_cop_msg = msg


def zmpCallback(msg):
    global zmp_msg
    with msg_lock:
        zmp_msg = msg

        
if __name__ == "__main__":
    rospy.init_node("footstep_visualizer")
    pub = rospy.Publisher("~output", Image)
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    lleg_end_coords = rospy.get_param("~lleg_end_coords", "lleg_end_coords")
    rleg_end_coords = rospy.get_param("~rleg_end_coords", "rleg_end_coords")
    image_size = rospy.get_param("~image_size", 400)
    scale = rospy.get_param("~scale", 0.6)
    lleg_vertices = rospy.get_param("~lleg_vertices", [[0.137525, -0.070104],
                                                       [-0.106925, -0.070104],
                                                       [-0.106925,  0.070104],
                                                       [ 0.137525,  0.070104]])
    rleg_vertices = rospy.get_param("~rleg_vertices", [[0.137525, -0.070104],
                                                       [-0.106925, -0.070104],
                                                       [-0.106925, 0.070104],
                                                       [0.137525, 0.070104]])
                                                       
    root_link = rospy.get_param("~root_link", "BODY")
    lleg_sub = rospy.Subscriber("/lfsensor_cop", PointStamped, llegCopCallback)
    rleg_sub = rospy.Subscriber("/rfsensor_cop", PointStamped, rlegCopCallback)
    zmp_sub = rospy.Subscriber("/zmp", PointStamped, zmpCallback)
    # lleg_cop_sub = Subscriber("/lfsensor_cop", PointStamped)
    # rleg_cop_sub = Subscriber("/rfsensor_cop", PointStamped)
    # ts = TimeSynchronizer([lleg_cop_sub, rleg_cop_sub], 10)
    # ts.registerCallback(cop_callback)
    rospy.Timer(rospy.Duration(0.1), periodicCallback)
    rospy.spin()
    
