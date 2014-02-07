#!/usr/bin/env roseus

(ros::load-ros-manifest "jsk_ik_server")

(require "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknt-interface.l")
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
