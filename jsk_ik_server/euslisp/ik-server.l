#!/usr/bin/env roseus

(ros::load-ros-manifest "jsk_ik_server")
;(ros::load-ros-manifest "arm_navigation_msgs")
(ros::load-ros-manifest "moveit_msgs")

(require :rtm-ros-robotics-interface
	 "package://hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l")
(require "models/arrow-object.l")

(ros::roseus "ik_server")

(defclass ik-server
  :slots
  (*robot*
   *arrow*
   *centroid-object*
   *foot-convex*
   *root-link*
   *success-state-collection*
   *fix-limb-coords*
   ;;
   viewer
   move-target-hash
   ))

(defmethod ik-server
  (:init
   nil
   (setq move-target-hash (make-hash-table))
   (unless (boundp '*do-not-start-ik-server*)
     (send self :start-ik-server)))
  (:ik-service-cb
   (req)
   (or
    (send self :fullbody-inverse-kinematics-service-cb req)
    (send self :old-ik-service-cb)))
  (:start-ik-server
   nil
   ;; initialization
   (unless *root-link* (setq *root-link* "/base_link"))
   (setq *arrow* (arrow))
   ;; check viewer
   (cond
    ((and (boundp '*irtviewer*) *irtviewer*)
     (setq viewer *irtviewer*))
    ((and (boundp '*pickview*) *pickview*)
     (setq viewer *pickview*))
    ((functionp 'pickview)
     (pickview :no-menu t :width 640 :height 640)
     (send *viewer* :viewsurface :bg-color #F(0.9 1 0.9 0))
     (send *pickview* :title "IK server")
     (setq viewer *pickview*))
    (t
     (make-irtviewer :width 640 :height 640)
     (send *irtviewer* :change-background #f(0.9 1.0 0.9))
     (send *irtviewer* :title "IK server")
     (setq viewer *irtviewer*))
    )
   (unless *foot-convex* (setq *foot-convex* (make-coords)))
   (unless *centroid-object* (setq *centroid-object* (make-coords)))
   (cond
    (viewer
     (send viewer :objects
	   (list *robot* *foot-convex* *arrow*))
     (send viewer :look-all)
     (send viewer :draw-objects)))
   ;;
   (setq *tf* (instance ros::transform-listener :init))
   ;;
   (ros::advertise-service "/solve_ik" moveit_msgs::GetPositionIK #'send self :ik-service-cb)
   (ros::rate 10)
   (send self :loop-ik-server)
   )
  (:loop-ik-server
   nil
   (do-until-key
    (if (not (ros::ok)) (return-from nil nil))
    (unless (or (null x::*display*) (= x::*display* 0))
      (x::window-main-one))
    (ros::sleep)
    (ros::spin-once))
   )
  ;;
  ;;
  ;;
  (:update-joint-states
   (js &optional opt buf)
   (setq buf (concatenate float-vector
			  (send (car (send *robot* :links)) :worldpos)
			  (matrix-log
			   (send (car (send *robot* :links)) :worldrot))))
   (dotimes (i (length (send js :name)))
     (if (or (>= i (length (send js :name)))
	     (>= i (length (send js :position))))
	 (return-from nil nil))
     (let ((name (remove #\: (elt (send js :name) i)))
	   (pos (elt (send js :position) i)) j)
       ;;(format t "[:update-joint-states] ~A ~Arad~%" name pos)
       (cond
	((substringp "base" name)
	 (cond
	  ((find #\x name)
	   (setf (aref buf 0) (* 1e+3 pos)))
	  ((find #\y name)
	   (setf (aref buf 1) (* 1e+3 pos)))
	  ((find #\z name)
	   (setf (aref buf 2) (* 1e+3 pos)))
	  ((find #\r name)
	   (setf (aref buf 3) pos))
	  ((find #\p name)
	   (setf (aref buf 4) pos))
	  ((find #\w name)
	   (setf (aref buf 5) pos))))
	((setq j (find-if
		  #'(lambda (j) (string-equal
				 (remove #\: (format nil "~A" (send j :name)))
				 name))
		  (send *robot* :joint-list)))
	 (send j :joint-angle (rad2deg pos)))
	((setq j (or j (find-if
			#'(lambda (j) (string-equal
				       (remove #\: (format nil "~A" (send j :name)))
				       name))
			(send *robot* :links))))
	 (if (find-method j :joint) (setq j (send j :joint)))
	 (send j :joint-angle (rad2deg pos))))))
   (if (and (cadr (member :angle-vector opt))
	    (eq (length (cadr (member :angle-vector opt)))
		(length (send *robot* :angle-vector))))
       (send *robot* :angle-vector (cadr (member :angle-vector opt))))
   (setq
    buf
    (if (cadr (member :worldcoords opt))
	(cond
	 ((subclassp (class (cadr (member :worldcoords opt))) coordinates)
	  (cadr (member :worldcoords opt)))
	 ((functionp (car (cadr (member :worldcoords opt))))
	  (eval (cadr (member :worldcoords opt)))))
      (make-coords :pos (subseq buf 0 3)
		   :rpy (reverse (subseq buf 3 6)))))
   (dotimes (i 3)
     (send *robot* :transform
	   (send
	    (copy-object
	     (send (car (send *robot* :links)) :worldcoords))
	    :transformation
	    buf)))
   (send-all (send *robot* :links) :worldcoords)
   (if viewer (send viewer :draw-objects))
   ;;(read-line)
   )
  ;;
  (:update-move-target
   (poscon oricon &optional opt)
   (let (name link cs (mt-coords (make-hash-table)))
     (dolist (pc poscon)
       (setq name (remove #\: (send pc :link_name)))
       (setq link
	     (find-if
	      #'(lambda (j) (string-equal
			     (remove #\: (format nil "~A" (send j :name)))
			     name))
	      (send *robot* :links)))
       (cond
	((null link)
	 (format t "[:update-move-target] unknown link ~A~%" name))
	(t
	 (send mt-coords :enter
	       link
	       (list :pos
		     (float-vector
		      (or (* 1e+3 (send (send pc :target_point_offset) :x)) 0)
		      (or (* 1e+3 (send (send pc :target_point_offset) :y)) 0)
		      (or (* 1e+3 (send (send pc :target_point_offset) :z)) 0))))
	 )))
     (dolist (oc oricon)
       (setq name (remove #\: (send oc :link_name)))
       (setq link
	     (find-if
	      #'(lambda (j) (string-equal
			     (remove #\: (format nil "~A" (send j :name)))
			     name))
	      (send *robot* :links)))
       (cond
	((null link)
	 (format t "[:update-move-target] unknown link~A ~%" name))
	(t
	 (send mt-coords :enter
	       link
	       (append
		(send mt-coords :get link)
		(list
		 :rot (quaternion2matrix
		       (float-vector
			(or (send (send oc :orientation) :w) 0)
			(or (send (send oc :orientation) :x) 0)
			(or (send (send oc :orientation) :y) 0)
			(or (send (send oc :orientation) :z) 0))))))
	 (setq cs (send move-target-hash :get link))
	 (if cs (send (send cs :parent) :dissoc cs))
	 (setq cs
	       (make-cascoords
		:name name
		:parent link
		:coords (copy-object (send link :worldcoords))))
	 (send cs
	       :transform
	       (apply
		#'make-coords
		(send mt-coords :get link))
	       :local)
	 (send move-target-hash :enter link cs)
	 )))
     (dolist (cs (cadr (member :move-target opt)))
       (if (functionp (car cs)) (setq cs (eval cs)))
       (if (subclassp (class cs) cascaded-coords)
	   (send move-target-hash :enter (send cs :parent) cs)))
     ))
  (:parse-joint-weight-list
   (jocon)
   (let (name joint link ret)
     (dolist (jc jocon)
       (setq name (remove #\: (send jc :joint_name)))
       ;;(print name)
       (setq link
	     (find-if
	      #'(lambda (j) (string-equal
			     (remove #\: (format nil "~A" (send j :name)))
			     name))
	      (send *robot* :links)))
       (cond
	((and
	  (null link)
	  (null
	   (setq joint
		 (find-if
		  #'(lambda (j) (string-equal
				 (remove #\: (format nil "~A" (send j :name)))
				 name))
		  (send-all (send *robot* :links) :joint))))))
	(t
	 (setq link (or link (send joint :child-link)))
	 (push (list link (send jc :weight)) ret))))
     ret))
  (:parse-collision-link-pair
   (collisions &optional opt)
   (let (buf link name)
     (dolist (col collisions)
       (setq name (remove #\: (send col :link_name)))
       (setq link
	     (find-if
	      #'(lambda (j) (string-equal
			     (remove #\: (format nil "~A" (send j :name)))
			     name))
	      (send *robot* :links)))
       (if link (push link buf)))
     (setq buf (union buf nil))
     (append
      (apply
       #'append
       (mapcar
	#'(lambda (a)
	    (mapcar #'(lambda (b) (list a b)) (cdr (member a buf))))
	buf))
      (cadr (member :collision-avoidance-link-pair opt)))))
  (:parse-ik-param
   (ik-link-name pose-stamped
    ik-link-name-list pose-stamped-list
    &optional opt)
   (let (mt ll tc link name buf (id -1)
	 goal goal-in-world goal-in-pelvis
	 (ik-link-name-list (cons ik-link-name ik-link-name-list)))
     (dolist (ps (cons pose-stamped pose-stamped-list))
       (incf id)
       (setq name (remove #\: (car ik-link-name-list)))
       (setq ik-link-name-list (cdr ik-link-name-list))
       (setq link
	     (find-if
	      #'(lambda (j) (string-equal
			     (remove #\: (format nil "~A" (send j :name)))
			     name))
	      (send *robot* :links)))
       (cond
	((null link)
	 (format t "[:parse-ik-param] invalid link ~A~%" name))
	(t
	 (setq buf (send move-target-hash :get link))
	 (if (null buf)
	     (setq buf (make-cascoords
			:name name
			:parent link
			:coords (copy-object (send link :worldcoords)))))
	 (send move-target-hash :enter link buf)
	 (push buf mt)
	 (push (send *robot* :link-list (send buf :parent)) ll)
	 ;;
	 ;;(setq goal ps)
	 (if *root-link*
	     (setq
	      goal-in-pelvis
	      (send *tf* :transform-pose *root-link* ps)))
	 (setq goal-in-pelvis
	       (or
		goal-in-pelvis
		(make-coords
		 :pos (float-vector
		       (or (* 1e+3 (send (send (send ps :pose) :position) :x)) 0)
		       (or (* 1e+3 (send (send (send ps :pose) :position) :y)) 0)
		       (or (* 1e+3 (send (send (send ps :pose) :position) :z)) 0))
		 :rot
		 (quaternion2matrix
		  (float-vector
		   (or (send (send (send ps :pose) :orientation) :w) 0)
		   (or (send (send (send ps :pose) :orientation) :x) 0)
		   (or (send (send (send ps :pose) :orientation) :y) 0)
		   (or (send (send (send ps :pose) :orientation) :z) 0))))))
	 (if (not (string-equal *root-link* (send (send ps :header) :frame_id)))
	     (format t "[:parse-ik-param] frame_id must be ~A~%" *root-link*))
	 (setq goal-in-world
	       (send (send (car (send *robot* :links)) :copy-worldcoords)
		     :transform goal-in-pelvis))
	 (if (zerop id)
	     (send *arrow* :newcoords goal-in-world))
	 (push goal-in-world tc))))
     (append
      (list
       :target-coords (reverse tc)
       :move-target (reverse mt)
       :link-list (reverse ll))
      opt)
     ))
  ;; :fullbody-inverse-kinematics
  (:fullbody-inverse-kinematics-service-cb ;; [m] or [rad] expected
   (msg)
   (cond
    ((not (substringp
	   ":fullbody-inverse-kinematics"
	   (send (send msg :ik_request) :group_name)))
     (format t "[:fik-service-cb] invalid group ~A~%"
	     (send (send msg :ik_request) :group_name))
     nil)
    (t
     (let* ((res (send msg :response))
	    (req (send msg :ik_request))
	    (opt
	     (cadr (read-from-string (format nil "(~A)"
					     (send req :group_name)))))
	    (rsd (send req :robot_state))
	    (js (send rsd :joint_state))
	    ik-param ret additional-weight-list
	    buf )
       ;;
       (format t "[:fik-service-cb] options: ~A~%" opt)
       ;; joint angle update
       (send self :update-joint-states js opt)
       ;; move-target update
       (send self :update-move-target
	     (send (send req :constraints) :position_constraints)
	     (send (send req :constraints) :orientation_constraints)
	     opt)
       ;; foot convex udpate
       (send self :make-foot-convex :force? t)
       (cond
	(viewer
	 (send viewer :objects
	       (list *robot* *centroid-object* *foot-convex* *arrow*))
	 (send viewer :draw-objects)))
       ;;
       (send res :error_code :val moveit_msgs::MoveItErrorCodes::*FAILURE*)
       (setq ik-param
	     (append
	      (send self :parse-ik-param
		    (send req :ik_link_name)
		    (send req :pose_stamped)
		    (send req :ik_link_names)
		    (send req :pose_stamped_vector)
		    opt)
	      (list
	       :collision-pair
	       (send self :parse-collision-link-pair
		     (send rsd :attached_collision_objects)))
	      (list
	       :additional-weight-list
	       (send self :parse-joint-weight-list
		     (send (send req :constraints) :joint_constraints)))
	      ))
       (cond
	((or
	  (zerop (length (cadr (member :target-coords ik-param))))
	  (zerop (length (cadr (member :move-target ik-param))))
	  (zerop (length (cadr (member :link-list ik-param))))
	  (not
	   (eq
	    (length (cadr (member :target-coords ik-param)))
	    (length (cadr (member :move-target ik-param)))))
	  (not
	   (eq
	    (length (cadr (member :move-target ik-param)))
	    (length (cadr (member :link-list ik-param))))))
	 (format t "[:fik-service-cb] invalid args length/tc=~A mt=~A ll=~A~%"
		 (length (cadr (member :target-coords ik-param)))
		 (length (cadr (member :move-target ik-param)))
		 (length (cadr (member :link-list ik-param))))
	 res)
	((progn
	   (format t "[:fik-service-cb] args=~A~%" ik-param)
	   (setq ret
		 (send* self :fullbody-ik-main
			ik-param))
	   (vectorp ret))
	 ;;(format t "[ik-server] SUCCESS!!~%")
	 (send res :error_code :val moveit_msgs::MoveItErrorCodes::*SUCCESS*)
	 (if viewer (send viewer :draw-objects))
	 (send res :solution :joint_state :name
	       (append
		(mapcar
		 #'(lambda (n) (format nil "~A" n))
		 (send-all (send *robot* :joint-list) :name))
		(list "basex" "basey" "basez" "baser" "basep" "basew")))
	 (send res :solution :joint_state :position
	       (concatenate
		float-vector
		(map float-vector #'deg2rad
		     (send-all (send *robot* :joint-list) :joint-angle))
		(scale 1e-3 (send (car (send *robot* :links)) :worldpos))
		(matrix-log (send (car (send *robot* :links)) :worldrot))))
	 res)
	(t res))))))
  )


;; IK methods definition
(require "package://jsk_ik_server/euslisp/eus-fullbody-ik-ex.l")
(require "package://jsk_ik_server/euslisp/old-ik-server.l")
