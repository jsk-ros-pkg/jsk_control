#!/usr/bin/env roseus

(ros::load-ros-manifest "jsk_ik_server")

;;(require "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsk-interface.l")
(require :hrp2jsk "package://hrpsys_ros_bridge_tutorials/models/hrp2jsk.l")
(when (probe-file (ros::resolve-ros-path "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsk-utils.l"))
  (require :hrp2jsk-utils "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsk-utils.l"))
(require "package://jsk_ik_server/euslisp/ik-server.l")

(ros::roseus "hrp2jsk_ik_server")


(defclass hrp2jsk-ik-server
  :super ik-server
  )

(defmethod hrp2jsk-ik-server
  (:init
   (&rest args)

   (setq robot (hrp2jsk))

   (setq root-link-frame-id "/pelvis")
   (send-super* :init args)
  )
  )

(instance hrp2jsk-ik-server :init)
