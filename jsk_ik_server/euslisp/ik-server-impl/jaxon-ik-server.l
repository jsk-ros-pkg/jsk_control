#!/usr/bin/env roseus

(ros::load-ros-manifest "jsk_ik_server")

;;(require "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon-interface.l")
(require :jaxon "package://hrpsys_ros_bridge_tutorials/models/jaxon.l")
(when (probe-file (ros::resolve-ros-path "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon-utils.l"))
  (require :jaxon-utils "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon-utils.l"))
(require "package://jsk_ik_server/euslisp/ik-server.l")

(ros::roseus "jaxon_ik_server")

(defvar *jaxon-ik-server* (instance ik-server :init :robot (jaxon)))
(send *jaxon-ik-server* :start-ik-server)
