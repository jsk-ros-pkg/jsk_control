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
   (setq root-link-frame-id "/pelvis")
   (send self :make-foot-convex :force? t)
   (send self :make-centroid-object)
   (send-super* :init args)
   )
  )

(defmethod ik-server
  (:make-foot-convex
  (&key
   (foot-key '(:rleg :lleg))
   (force? nil))
  (when (position nil (mapcar #'(lambda (l) (send robot l)) foot-key))
    (return-from :make-foot-convex nil))
  (format t "[foot-convex] ")
  (cond
   ((or
     force?
     (null foot-convex)
     (and *fix-limb-coords*
	  (flatten
	   (mapcar
	    #'(lambda (k)
		(or
		 (not (assoc k *fix-limb-coords*))
		 (> (norm
		     (send
		      (send robot k :end-coords :worldcoords)
		      :difference-position
		      (cdr (assoc k *fix-limb-coords*))))
		    50)
		 (> (norm
		     (send
		      (send robot k :end-coords :worldcoords)
		      :difference-rotation
		      (cdr (assoc k *fix-limb-coords*))))
		    (deg2rad 10))))
	    foot-key))))
    (format t "update~%")
    (setq foot-convex
	  (make-prism
	   (quickhull
	    (mapcar
	     #'(lambda (v)
		 (concatenate float-vector
			      (subseq v 0 2)
			      (list
			       (+
				-10
				(aref
				 (send robot :rleg :end-coords :worldpos)
				 2)))))
	     (flatten
	      (mapcar
	       #'(lambda (k)
		   (send-all
		    (append
		     (if (and (eps= (send robot :rleg :toe-p :joint-angle) 0 1)
			      (eps= (send robot :lleg :toe-p :joint-angle) 0 1))
			 (send
			  (send (send robot k :end-coords :parent) :parent)
			  :faces))
		     (send
		      (send robot k :end-coords :parent)
		      :faces))
		    :vertices))
	       foot-key))))
	   5))
    (send foot-convex :set-color #F(0 0 1))
    (setq *fix-limb-coords*
	  (append
	   (remove-if
	    #'(lambda (kf) (find (car kf) '(:rleg :lleg)))
	    *fix-limb-coords*)
	   (mapcar
	    #'(lambda (k)
		(cons
		 k
		 (send robot k :end-coords :copy-worldcoords)))
	    foot-key)))
    (gl::transparent foot-convex 0.3)
    )
   (t (format t "reject~%"))))
  )

(setq a (instance hrp2jsknt-ik-server :init))
