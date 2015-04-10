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
    (foot? nil)
    &allow-other-keys)
   (mapcar
    #'(lambda (tc)
	(send tc :transform
	      (make-coords
	       :pos (scale 300 (random-vector 1.0))
	       :rpy (scale 1.57 (random-vector 1.0)))))
    (subseq target-coords 0 2))
   (if foot?
       (mapcar
	#'(lambda (tc)
	    (send tc :transform
		  (make-coords
		   :pos
		   (map float-vector
			#'*
			#F(1 1 1)
			(scale 100 (random-vector 1.0))))))
	(if (> (length target-coords) 2) (subseq target-coords 2 4))))
   (apply
    #'send self :ik-server-call
    (append
     (list
      :robot robot
      :target-limbs target-limbs
      :move-target move-target
      :target-coords target-coords
      :root-link-virtual-joint-weight #F(1 1 1 1 1 1)
      ;;:fix-limbs '(:rleg :lleg)
      :centroid-thre :support-convex
      :stop 50
      :use-toes '(nil nil)
      )
     args)))
  (:hrp2jsknt-test
   (&rest args)
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
   (defvar *end-coords-from-hand*
     (mapcar
      #'(lambda (k)
	  (if (find-method *robot* :hand)
	      (make-cascoords
	       :name
	       (read-from-string (format nil "~A-end-coords-without-toe" k))
	       :parent (car (last (send *robot* :hand k :links)))
	       :coords
	       (copy-object
		(send (car (last (send *robot* :hand k :links))) :worldcoords)))
	    (send *robot* k :end-coords)))
      '(:rarm :larm)))
   (do-until-key
    (send *robot* :reset-manip-pose)
    (send *robot* :fix-leg-to-coords (make-coords))
    (cond
     ((find-method *robot* :hand)
      (send *robot* :hand :rarm :close-pose)
      (send *robot* :hand :larm :close-pose)))
    (send* self :test-fullbody-ik
	   :all-links
	   (remove-if
	    #'(lambda (l)
		(not (and (find-method l :joint)
			  (send l :joint))))
	    (flatten
	     (append
	      (send *robot* :links)
	      (if (find-method *robot* :hand)
		  (list
		   (send (send *robot* :hand :rarm) :links)
		   (send (send *robot* :hand :larm) :links))))))
	   :move-target
	   (append
	    ;;(mapcar #'(lambda (k) (send *robot* k :end-coords)) '(:rarm :larm))
	    *end-coords-from-hand*
	    *end-coords-without-toe*)
	   args))
   )
  (:hrp2jsknts-test
   (&rest args)
   (require :hrp2jsknts "package://hrpsys_ros_bridge_tutorials/models/hrp2jsknts.l")
   (when (probe-file (ros::resolve-ros-path "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknts-utils.l"))
     (require :hrp2jsknts-utils "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknts-utils.l"))
   (defvar *robot* (hrp2jsknts))
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
   (defvar *end-coords-from-hand*
     (mapcar
      #'(lambda (k)
	  (if (find-method *robot* :hand)
	      (make-cascoords
	       :name
	       (read-from-string (format nil "~A-end-coords-without-toe" k))
	       :parent (car (last (send *robot* :hand k :links)))
	       :coords
	       (copy-object
		(send (car (last (send *robot* :hand k :links))) :worldcoords)))
	    (send *robot* k :end-coords)))
      '(:rarm :larm)))
   (do-until-key
    (send *robot* :reset-manip-pose)
    (send *robot* :fix-leg-to-coords (make-coords))
    (cond
     ((find-method *robot* :hand)
      (send *robot* :hand :rarm :close-pose)
      (send *robot* :hand :larm :close-pose)))
    (send* self :test-fullbody-ik
	   :all-links
	   (remove-if
	    #'(lambda (l)
		(not (and (find-method l :joint)
			  (send l :joint))))
	    (flatten
	     (append
	      (send *robot* :links)
	      (if (find-method *robot* :hand)
		  (list
		   (send (send *robot* :hand :rarm) :links)
		   (send (send *robot* :hand :larm) :links))))))
	   :move-target
	   (append
	    ;;(mapcar #'(lambda (k) (send *robot* k :end-coords)) '(:rarm :larm))
	    *end-coords-from-hand*
	    *end-coords-without-toe*)
	   args))
   )
  (:hrp2jsk-test
   (&rest args)
   (require "package://hrpsys_ros_bridge_tutorials/models/hrp2jsk.l")
   (defvar *hrp2jsk* (hrp2jsk))
   (objects (list *hrp2jsk*))
   (ros::roseus "fullbody_ik_client_test")
   (do-until-key
    (send *hrp2jsk* :reset-manip-pose)
    (send *hrp2jsk* :fix-leg-to-coords (make-coords))
    (send* self :test-fullbody-ik :robot *hrp2jsk* args))
   )
  (:staro-test
   (&rest args)
   (require "package://hrpsys_ros_bridge_tutorials/models/staro.l")
   (defvar *staro* (staro))
   (objects (list *staro*))
   (ros::roseus "fullbody_ik_client_test")
   (do-until-key
    (send *staro* :reset-manip-pose)
    (send *staro* :fix-leg-to-coords (make-coords))
    (send* self :test-fullbody-ik :robot *staro* args))
   )
  (:jaxon-test
    (&rest args)
    (require "package://hrpsys_ros_bridge_tutorials/models/jaxon.l")
    (defvar *jaxon* (jaxon))
    (objects (list *jaxon*))
    (ros::roseus "fullbody_ik_client_test")
    (do-until-key
     (send *jaxon* :reset-manip-pose)
     (send *jaxon* :fix-leg-to-coords (make-coords))
     (send* self :test-fullbody-ik :robot *jaxon* args))
    )
  (:atlas-test
   (&rest args)
   (require "package://hrpsys_gazebo_atlas/euslisp/atlas-model.l")
   (defvar *atlas* (atlas))
   (ros::roseus "fullbody_ik_client_test")
   (objects (list *atlas*))
   (do-until-key
    (send *atlas* :reset-manip-pose)
    (send *atlas* :fix-leg-to-coords (make-coords))
    (send* self :test-fullbody-ik :robot *atlas* args)
    ))
  (:sample-robot-test
   (&rest args)
   (require :samplerobot "package://hrpsys_ros_bridge_tutorials/models/samplerobot.l")
   (defvar *sample-robot* (instance samplerobot-robot :init))
   (send *sample-robot* :fix-leg-to-coords (make-coords))
   (ros::roseus "fullbody_ik_client_test")
   (objects (list *sample-robot*))
   (do-until-key
    (send *sample-robot* :reset-pose)
    (send *sample-robot* :fix-leg-to-coords (make-coords))
    (send* self :test-fullbody-ik :robot *sample-robot* args)
    ))
  (:pr2-test
   (&rest args)
   (require :pr2 "package://pr2eus/pr2.l")
   (defvar *pr2* (pr2))
   (ros::roseus "fullbody_ik_client_test")
   (objects (list *pr2*))
   (defvar *pr2-move-target*
     (mapcar
      #'(lambda (k)
	  (make-cascoords
	   :parent (send *pr2* k :end-coords :parent)
	   :coords (send *pr2* k :end-coords :copy-worldcoords)
	   :name k))
      '(:rarm :larm)))
   (do-until-key
    (send *pr2* :reset-manip-pose)
    (send* self :test-fullbody-ik
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
	   args
	   )))
  )

(let ((ik-client-test (instance fullbody-ik-client :init))
      (test (unix::getenv "IK_CLIENT_TEST"))
      (foot? (unix::getenv "IK_CLIENT_TEST_WITH_LEG"))
      )
  (if (and (stringp foot?) (plusp (length foot?)))
      (setq foot? (string-equal foot? "true")))
  (cond
   ((null test))
   ((substringp "hrp2jsknt" test)
    (send ik-client-test :hrp2jsknt-test :foot? foot?))
   ((substringp "hrp2jsknts" test)
    (send ik-client-test :hrp2jsknts-test :foot? foot?))
   ((substringp "hrp2jsk" test)
    (send ik-client-test :hrp2jsk-test :foot? foot?))
   ((substringp "staro" test)
    (send ik-client-test :staro-test :foot? foot?))
   ((substringp "jaxon" test)
    (send ik-client-test :jaxon-test :foot? foot?))
   ((substringp "pr2" test)
    (send ik-client-test :pr2-test :foot? foot?))
   ((substringp "atlas" test)
    (send ik-client-test :atlas-test :foot? foot?))
   ((substringp "sample-robot" test)
    (send ik-client-test :sample-robot-test :foot? foot?))
   ))
