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
   (mapcar
    #'(lambda (k hl)
	(mapcar
	 #'(lambda (h)
	     (cond
	      ((null (send h :parent-link)))
	      ((null (send (send h :parent-link) :parent-link))
	       (send h :add-parent-link
		     (send robot k :end-coords :parent))
	       (send (send robot k :end-coords :parent)
		     :add-child-links h))))
	 hl))
    '(:rarm :larm)
    (list (send robot :hand :rarm :links)
	  (send robot :hand :larm :links)))
   (send self :update-support-links '(:rleg :lleg))
   (send self :make-foot-convex :force? t)
   (send self :make-centroid-object)
   (send-super*
    :init
    :ik-server-name "hrp2jsknt_ik_server"
    :ik-server-service-name "/solve_ik"
    :link-list
    (flatten
     (append
      (send robot :links)
      (send (send robot :hand :rarm) :links)
      (send (send robot :hand :larm) :links)))
    :root-link-frame-id nil
    args)
   )
  )

(defmethod ik-server
  (:make-foot-convex
  (&key
   (support-links support-links)
   (support-link-target-transform-coords
    (make-list (length support-links) :initial-element (make-coords)))
   (support-vertices
    (mapcar
     #'(lambda (l c)
	 (let (ll ll2)
	   (setq ll
		 (find-if
		  #'(lambda (ll) (find l ll))
		  (list (subseq
			 (reverse (send robot :rleg :links))
			 0 2)
			(subseq
			 (reverse (send robot :lleg :links))
			 0 2))))
	   (cond
	    (ll
	     (setq ll2 (copy-object ll))
	     (send-all ll2 :transform c)
	     (flatten
	      (send-all (flatten (send-all ll2 :faces)) :vertices)))
	    (t
	     (setq ll2 (copy-object l))
	     (send ll2 :transform c)
	     (flatten (send-all (send ll2 :faces) :vertices))))))
     support-links support-link-target-transform-coords))
   (support-height
    (mapcar #'(lambda (hl) (apply #'min hl))
	    (mapcar #'(lambda (vl)
			(mapcar #'(lambda (v) (aref v 2)) vl))
		    support-vertices)))
   (force? nil))
  (when (null support-links) (return-from :make-foot-convex nil))
  (format t "[foot-convex] ")
  (cond
   ((or
     force?
     (null foot-convex)
     )
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
