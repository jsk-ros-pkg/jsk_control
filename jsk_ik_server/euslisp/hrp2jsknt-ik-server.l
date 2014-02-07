#!/usr/bin/env roseus

(ros::load-ros-manifest "jsk_ik_server")

;;(require "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknt-interface.l")
(require :hrp2jsknt "package://hrpsys_ros_bridge_tutorials/models/hrp2jsknt.l")
(when (probe-file (ros::resolve-ros-path "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknt-utils.l"))
  (require :hrp2jsknt-utils "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknt-utils.l"))
(require "package://jsk_ik_server/euslisp/ik-server.l")

(ros::roseus "hrp2jsknt_ik_server")


(defclass hrp2jsknt-ik-server
  :super ik-server
  )

(defmethod hrp2jsknt-ik-server
  (:init
   (&rest args)

   (setq *robot* (hrp2jsknt))

   (setq *root-link* "/pelvis")
   (send-super* :init args)
  )
  )

(instance hrp2jsknt-ik-server :init)
