#!/usr/bin/env roseus

(ros::load-ros-manifest "jsk_ik_server")
;;(require "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2w-interface.l")
(require :hrp2w "package://hrpsys_ros_bridge_tutorials/models/hrp2w.l")
(when (probe-file (ros::resolve-ros-path "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2w-utils.l"))
  (require :hrp2w-utils "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2w-utils.l"))
(require "package://jsk_ik_server/euslisp/ik-server.l")

(ros::roseus "hrp2w_ik_server")

(defvar *hrp2w-ik-server* (instance ik-server :init :robot (hrp2w)))
(send *hrp2w-ik-server* :start-ik-server)
