#!/usr/bin/env roseus

(ros::load-ros-manifest "jsk_ik_server")
;;(require "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknts-interface.l")
(require :hrp2jsknts "package://hrpsys_ros_bridge_tutorials/models/hrp2jsknts.l")
(when (probe-file (ros::resolve-ros-path "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknts-utils.l"))
  (require :hrp2jsknts-utils "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknts-utils.l"))
(require "package://jsk_ik_server/euslisp/ik-server.l")

(ros::roseus "hrp2jsknts_ik_server")

(defclass hrp2jsknts-ik-server
  :super ik-server
  )

(defmethod hrp2jsknts-ik-server
  (:init
   (&rest args)
   (setq robot (hrp2jsknts))
   (if (find-method robot :hand)
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
	      (send robot :hand :larm :links))))
   ;;
   (send self :update-support-links
	 nil
	 :link-list
	 (append
	  (subseq (reverse (send robot :rleg :links)) 0 2)
	  (subseq (reverse (send robot :lleg :links)) 0 2)))
   (send self :make-foot-convex :force? t)
   (send self :make-centroid-object)
   ;;(setq additional-ik-options (list :centroid-thre 10)) ;; sample for changing default centroid threhold
   (send-super*
    :init
    :ik-server-name "hrp2jsknts_ik_server"
    :ik-server-service-name "/solve_ik"
    :link-list
    (flatten
     (append
      (send robot :links)
      (if (find-method robot :hand)
	  (list
	   (send (send robot :hand :rarm) :links)
	   (send (send robot :hand :larm) :links)))))
    :root-link-frame-id nil
    :default-end-coords
    (append
     (mapcar
      #'(lambda (k) (cons k (send robot k :end-coords)))
      '(:rarm :larm))
     (mapcar
      #'(lambda (k)
	  (cons
	   k
	   (make-cascoords
	    :coords (send (send robot k :end-coords) :copy-worldcoords)
	    :parent (send (send (send robot k :end-coords) :parent) :parent)
	    :name (read-from-string (format nil "~a-default-end-coords" k)))))
      '(:rleg :lleg)))
    args)
   )
  )

(defvar *hrp2jsknts-ik-server* (instance hrp2jsknts-ik-server :init))
(send *hrp2jsknts-ik-server* :start-ik-server)

