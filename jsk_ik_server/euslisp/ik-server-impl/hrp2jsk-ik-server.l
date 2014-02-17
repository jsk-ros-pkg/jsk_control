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
   (setq ik-server-name "hrp2jsk_ik_server")
   (setq ik-server-service-name "/solve_ik")
   (setq root-link-frame-id nil)
   (send self :update-support-links '(:rleg :lleg))
   (send self :make-foot-convex :force? t)
   (send self :make-centroid-object)
   (send-super* :init args)
   )
  )

(defvar *hrp2jsk-ik-server* (instance hrp2jsk-ik-server :init))
(send *hrp2jsk-ik-server* :start-ik-server)
