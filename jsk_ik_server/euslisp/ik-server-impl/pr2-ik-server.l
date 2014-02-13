#!/usr/bin/env roseus

(ros::load-ros-manifest "jsk_ik_server")

;;(require "package://pr2eus/pr2-interface.l")
(require :pr2 "package://pr2eus/pr2.l")
(require :pr2-utils "package://pr2eus/pr2-utils.l")

(require :ik-server "package://jsk_ik_server/euslisp/ik-server.l")

(ros::roseus "pr2_ik_server")

(defclass pr2-ik-server
  :super ik-server
  )

(defmethod pr2-ik-server
  (:init
   (&rest args)

   (setq robot (pr2))
   (setq ik-server-name "pr2_ik_server")
   (setq ik-server-service-name "/solve_ik")
   (setq root-link-frame-id "/base_link")
   (send-super* :init args)
   )
  )

(instance pr2-ik-server :init)


