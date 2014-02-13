#!/usr/bin/env roseus

(require "package://jsk_ik_server/euslisp/fullbody-ik-client.l")

(defmethod fullbody-ik-client
  (:test-fullbody-ik
   (&rest
    args
    &key
    (robot *robot*)
    (target-limbs '(:rarm :larm :rleg :lleg))
    (move-target (mapcar #'(lambda (k) (send robot k :end-coords)) target-limbs))
    (target-coords (send-all move-target :copy-worldcoords))
    &allow-other-keys)
   (mapcar
    #'(lambda (tc)
	(send tc :transform
	      (make-coords
	       :pos (scale 300 (random-vector 1.0))
	       :rpy (scale 1.57 (random-vector 1.0)))))
    (subseq target-coords 0 2))
   (apply
    #'send self :ik-server-call
    (append
     (list
      :robot robot
      :target-limbs target-limbs
      :move-target move-target
      :target-coords target-coords
      :root-link-virtual-joint-weight #F(1 1 1 1 1 1)
      ;;:centroid-thre 100
      :stop 50
      :use-toes '(nil nil)
      )
     args)))
  (:hrp2jsknt-test
   nil
   (require :hrp2jsknt "package://hrpsys_ros_bridge_tutorials/models/hrp2jsknt.l")
   (when (probe-file (ros::resolve-ros-path "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknt-utils.l"))
     (require :hrp2jsknt-utils "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknt-utils.l"))
   (defvar *robot* (hrp2jsknt))
   (objects (list *robot*))
   (ros::roseus "fullbody_ik_client_test")
   (defvar *end-coords-without-toe*
     (mapcar
      #'(lambda (k)
	  (make-cascoords
	   :name
	   (read-from-string (format nil "~A-end-coords-without-toe" k))
	   :parent
	   (send (send *robot* k :end-coords :parent) :parent)
	   :coords
	   (send *robot* k :end-coords :copy-worldcoords)))
      '(:rleg :lleg)))
   (do-until-key
    (send *robot* :reset-manip-pose)
    (send *robot* :fix-leg-to-coords (make-coords))
    (send self :test-fullbody-ik
	  :move-target
	  (append
	   (mapcar #'(lambda (k) (send *robot* k :end-coords)) '(:rarm :larm))
	   *end-coords-without-toe*))
    ))
  (:hrp2jsk-test
   nil
   (require "package://hrpsys_ros_bridge_tutorials/models/hrp2jsk.l")
   (defvar *hrp2jsk* (hrp2jsk))
   (objects (list *hrp2jsk*))
   (ros::roseus "fullbody_ik_client_test")
   (do-until-key
    (send *hrp2jsk* :reset-manip-pose)
    (send *hrp2jsk* :fix-leg-to-coords (make-coords))
    (send self :test-fullbody-ik :robot *hrp2jsk*))
   )
  (:atlas-test
   nil
   (require "package://hrpsys_gazebo_atlas/euslisp/atlas-model.l")
   (defvar *atlas* (atlas))
   (ros::roseus "fullbody_ik_client_test")
   (objects (list *atlas*))
   (do-until-key
    (send *atlas* :reset-manip-pose)
    (send *atlas* :fix-leg-to-coords (make-coords))
    (send self :test-fullbody-ik :robot *atlas*)
    ))
  (:pr2-test
   nil
   (require :pr2 "package://pr2eus/pr2.l")
   (defvar *pr2* (pr2))
   (ros::roseus "fullbody_ik_client_test")
   (objects (list *pr2*))
   (defvar *pr2-move-target*
     (list (make-cascoords
	    :parent
	    (send (send (send *pr2* :rarm :end-coords :parent) :parent) :parent)
	    :coords
	    (copy-object
	     (send (send (send (send *pr2* :rarm :end-coords :parent) :parent) :parent)
		   :worldcoords))
	    :name :rarm-pr2-end-coords)
	   (make-cascoords
	    :parent
	    (send (send (send *pr2* :larm :end-coords :parent) :parent) :parent)
	    :coords
	    (copy-object
	     (send (send (send (send *pr2* :larm :end-coords :parent) :parent) :parent)
		   :worldcoords))
	    :name :larm-pr2-end-coords)))
   (do-until-key
    (send *pr2* :reset-manip-pose)
    (send self :test-fullbody-ik
	  :robot *pr2*
	  :group-name ":inverse-kinematics"
	  ;;:torso-fix t
	  ;;:use-torso 0
	  :target-limbs '(:rarm :larm)
	  :centroid-thre nil
	  :target-centroid-pos nil
	  ;;:min-vec #F(0 0 0 0 0 0)
	  ;;:max-vec #F(0 0 0 0 0 0)
	  :cog-gain 0
	  ;;:root-link-virtual-joint-weight #F(0 0 0 0 0 0)
	  :move-target *pr2-move-target*
	  :rotation-axis (list t t)
	  )))
  )

(let ((ik-client-test (instance fullbody-ik-client :init))
      (test (unix::getenv "IK_CLIENT_TEST")))
  (cond
   ((null test))
   ((substringp "hrp2jsknt" test)
    (send ik-client-test :hrp2jsknt-test))
   ((substringp "hrp2jsk" test)
    (send ik-client-test :hrp2jsk-test))
   ((substringp "pr2" test)
    (send ik-client-test :pr2-test))
   ((substringp "atlas" test)
    (send ik-client-test :atlas-test))
   ))

