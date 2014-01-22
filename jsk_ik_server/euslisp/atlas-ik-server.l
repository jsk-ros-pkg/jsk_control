#!/usr/bin/env roseus

(ros::load-ros-manifest "jsk_ik_server")
(ros::load-ros-manifest "hrpsys_gazebo_atlas")
(ros::load-ros-manifest "sandia_hand_msgs")
(ros::load-ros-manifest "atlas_msgs")
(ros::load-ros-manifest "arm_navigation_msgs")

(require "package://hrpsys_gazebo_atlas/euslisp/atlas-interface.l")
(require "package://jsk_ik_server/euslisp/atlas-fullbody-ik.l")
(require "package://jsk_ik_server/euslisp/atlas-end-coords.l")
(require :robot-im "package://jsk_ik_server/euslisp/ik-server.l")

(ros::roseus "atlas_ik_server")


(defclass atlas-ik-server
  :super ik-server
  )

(defmethod atlas-ik-server
  (:init
   (&rest args)

   (setq *atlas* (atlas-with-hand))

   (send *atlas* :reset-manip-pose)
   (send *atlas* :fix-leg-to-coords (make-coords) :both)
   (send-all (send *atlas* :joint-list) :max-joint-velocity 0.3)

   (send *atlas* :torso :waist-r :max-angle 10)
   (send *atlas* :torso :waist-r :min-angle -10)
   ;;
   (send *atlas* :torso :waist-p :max-angle 10)
   (send *atlas* :torso :waist-p :min-angle -10)
   
   (make-foot-convex)
   (make-centroid-object)

   (init-fullbody-ik-main)
   (send self :switch-end-coords :id 1)
   (setq *root-link* "/pelvis")
   (setq *robot* *atlas*)
   (send-super* :init args)
  )
  )

(instance atlas-ik-server :init)

;;
;; roseus "(setq *setup-end-coords-for-hku-flag* nil)" atlas-eus-ik.l
;;
