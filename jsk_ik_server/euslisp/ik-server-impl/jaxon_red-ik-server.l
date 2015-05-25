#!/usr/bin/env roseus

(ros::load-ros-manifest "jsk_ik_server")

;;(require "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon_red-interface.l")
(require :jaxon_red "package://hrpsys_ros_bridge_tutorials/models/jaxon_red.l")
(when (probe-file (ros::resolve-ros-path "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon_red-utils.l"))
  (require :jaxon_red-utils "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon_red-utils.l"))
(require "package://jsk_ik_server/euslisp/ik-server.l")

(ros::roseus "jaxon_red_ik_server")

(defvar *jaxon_red-ik-server* (instance ik-server :init :robot (jaxon_red)))
(send *jaxon_red-ik-server* :start-ik-server)
