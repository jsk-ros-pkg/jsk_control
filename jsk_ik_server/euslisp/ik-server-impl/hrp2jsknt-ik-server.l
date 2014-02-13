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
   (setq robot (hrp2jsknt))
   (mapcar
    #'(lambda (k)
	(let* ((mt (send robot k :end-coords))
	       (l (send mt :parent)))
	  (send l :dissoc mt)
	  (send (send l :parent) :assoc mt)))
    '(:rleg :lleg))
   (setq ik-server-name "hrp2jsknt_ik_server")
   (setq ik-server-service-name "/solve_ik")
   (setq root-link-frame-id "/pelvis")
   (send self :update-support-links '(:rleg :lleg))
   (send self :make-foot-convex :force? t)
   (send self :make-centroid-object)
   (send-super* :init args)
   )
  )

(defmethod ik-server
  (:make-foot-convex
  (&key
   (support-links support-links)
   (support-vertices
    (mapcar
     #'(lambda (l)
	 (cond
	  ((find l (subseq
		    (reverse (send robot :rleg :links))
		    0 2))
	   (flatten
	    (send-all
	     (flatten
	      (send-all
	       (subseq
		(reverse (send robot :rleg :links))
		0 2)
	       :faces))
	     :vertices)))
	  ((find l (subseq
		    (reverse (send robot :lleg :links))
		    0 2))
	   (flatten
	    (send-all
	     (flatten
	      (send-all
	       (subseq
		(reverse (send robot :lleg :links))
		0 2)
	       :faces))
	     :vertices)))
	  (t (flatten (send-all (send l :faces) :vertices)))))
     support-links))
   (support-height
    (mapcar #'(lambda (hl) (apply #'min hl))
	    (mapcar #'(lambda (vl)
			(mapcar #'(lambda (v) (aref v 2)) vl))
		    support-vertices)))
   (force? nil))
  ;;
  ;;(when (position nil (mapcar #'(lambda (l) (send robot l)) foot-key))
  ;;(return-from :make-foot-convex nil))
  (when (null support-links) (return-from :make-foot-convex nil))
  ;;
  (format t "[foot-convex] ")
  (cond
   ((or
     force?
     (null foot-convex))
    (format t "update~%")
    (setq foot-convex
	  (make-prism
	   (quickhull
	    (flatten
	     (mapcar
	      #'(lambda (vl h)
		  (mapcar
		   #'(lambda (v)
		       (float-vector
			(aref v 0)
			(aref v 1)
			(- h 10)))
		   vl))
	      support-vertices
	      support-height)))
	   5))
    (send foot-convex :set-color #F(0 0 1))
    (gl::transparent foot-convex 0.3)
    )
   (t (format t "reject~%"))))
  )

(defvar *hrp2jsknt-ik-server* (instance hrp2jsknt-ik-server :init))
(send *hrp2jsknt-ik-server* :start-ik-server)
