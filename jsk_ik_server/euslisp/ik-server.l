#!/usr/bin/env roseus

(ros::load-ros-manifest "jsk_ik_server")
;(ros::load-ros-manifest "arm_navigation_msgs")
(ros::load-ros-manifest "moveit_msgs")
(ros::load-ros-manifest "geometry_msgs")

(require :rtm-ros-robotics-interface
	 "package://hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l")
(require "models/arrow-object.l")
(require "fullbody-ik-client.l")

(defmethod cascaded-link
  (:find-link-route
   (to &optional from)
   (let ((pl (send to :parent-link)))
     (cond
      ;;((and pl (not (find to (send self :links))))
      ;;(send self :find-link-route pl from))
      ((and pl (not (eq to from)))
       (append (send self :find-link-route pl from) (list to)))
      ((and pl (eq to from))
       (list from))
      )))
  )

(defclass ik-server
  :slots
  (robot
   link-list
   arrow
   centroid-object
   foot-convex
   root-link-frame-id
   success-state-collection
   ;;
   ik-server-name
   ik-server-service-name
   ik-server-client
   ;;
   support-links
   support-centroid-pos
   viewer
   move-target-hash
   ))

(defmethod ik-server
  (:init
   (&rest args)
   (while args
     (send self (car args) (cadr args))
     (setq args (cddr args)))
   (setq move-target-hash (make-hash-table))
   (setq link-list
	 (union
	  nil
	  (remove-if
	   #'(lambda (a)
	       (not
		(and (class a)
		     (subclassp (class a) bodyset-link)
		     (find-method a :joint)
		     (send a :joint))))
	   (flatten (send robot :slots)))))
   (setq ik-server-client
	 (instance fullbody-ik-client :init))
   )
  (:ik-service-cb
   (req)
   (or
    (send self :inverse-kinematics-service-cb req)
    (send self :old-ik-service-cb)))
  (:start-ik-server
   nil
   ;; initialization
   (unless root-link-frame-id (setq root-link-frame-id "/base_link"))
   (setq arrow (arrow))
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
   ;; (unless
   ;;     (and (or (null support-links) (null support-centroid-pos))
   ;; 	    (find-method robot :rleg))
   ;;   (send self :update-support-links '(:rleg :lleg)))
   (unless ik-server-name (setq ik-server-name "/ik_server"))
   (unless ik-server-service-name
     (setq ik-server-service-name (format nil "~A/~A" ik-server-name "solve")))
   (unless foot-convex (setq foot-convex (make-coords)))
   (unless centroid-object (setq centroid-object (make-coords)))
   (cond
    (viewer
     (send viewer :objects
	   (list robot foot-convex arrow))
     (send viewer :look-all)
     (send viewer :draw-objects)))
   ;;
   (setq *tf* (instance ros::transform-listener :init))
   ;;
   (ros::roseus ik-server-name)
   (ros::advertise-service ik-server-service-name
			   moveit_msgs::GetPositionIK
			   #'send self :ik-service-cb)
   (ros::rate 10)
   (send self :loop-ik-server)
   )
  (:update-support-links
   (name-list
    &key
    (link-list
     (flatten
      (mapcar
       #'(lambda (name)
	   (cond
	    ((and (keywordp name)
		  (find-method robot name))
	     (send robot name :end-coords :parent))
	    (t
	     (let ((n (remove #\: (format nil "~A" name))))
	       (find-if #'(lambda (l)
			    (substringp n (format nil "~A" (send l :name))))
			link-list)))))
       name-list))))
   ;;(print name-list)
   (setq support-links link-list)
   (setq support-centroid-pos
	 (cond
	  ((null support-links) nil)
	  ((eq 1 (length support-links))
	   (send (car support-links) :worldpos))
	  (t
	   (scale (/ 1.0 (length support-links))
		  (reduce #'v+
			  (send-all support-links :worldpos)))))))
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
   (js &key
       opt
       (name-list (send js :name))
       (position-list (send js :position))
       position rpy quaternion rpy-flag)
   (send ik-server-client
	 :update-joint-states
	 js
	 :name-list name-list
	 :position-list position-list
	 :opt opt
	 :robot robot)
   (if (and (boundp '*viewer*) *viewer*) (send *viewer* :draw-objects))
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
	      link-list))
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
	      link-list))
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
   (jocon &optional opt)
   (let (name joint link ret)
     (dolist (jc jocon)
       (setq name (remove #\: (send jc :joint_name)))
       ;;(print name)
       (setq link
	     (find-if
	      #'(lambda (j) (string-equal
			     (remove #\: (format nil "~A" (send j :name)))
			     name))
	      link-list))
       (cond
	((and
	  (null link)
	  (null
	   (setq joint
		 (find-if
		  #'(lambda (j) (string-equal
				 (remove #\: (format nil "~A" (send j :name)))
				 name))
		  (send-all link-list :joint))))))
	(t
	 (setq link (or link (send joint :child-link)))
	 (push (list link (send jc :weight)) ret))))
     (append
      (cadr (member :additional-weight-list opt)) ret)))
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
	      link-list))
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
	      link-list))
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
	 (push (send robot :link-list (send buf :parent)) ll)
	 ;;
	 ;;(setq goal ps)
	 (if root-link-frame-id
	     (setq
	      goal-in-pelvis
	      (send *tf* :transform-pose root-link-frame-id ps)))
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
	 (if (not (string-equal root-link-frame-id (send (send ps :header) :frame_id)))
	     (format t "[:parse-ik-param] frame_id must be ~A~%" root-link-frame-id))
	 (setq goal-in-world
	       (send (send (car (send robot :links)) :copy-worldcoords)
		     :transform goal-in-pelvis))
	 (if (zerop id)
	     (send arrow :newcoords goal-in-world))
	 (push goal-in-world tc))))
     (setq buf
	   (mapcar
	    #'(lambda (k) (send robot k :end-coords))
	    (cadr (member :fix-limbs opt))))
     (append
      (list
       :target-coords (append (send-all buf :copy-worldcoords) (reverse tc))
       :move-target (append buf (reverse mt))
       :link-list (append
		   (mapcar
		    #'(lambda (mt) (send robot :link-list (send mt :parent)))
		    buf)
		   (reverse ll)))
      opt)
     ))
  (:inverse-kinematics-group-options
   (&optional opt)
   (append
    (if (cadr (member :torso-fix opt))
	(list
	 :additional-weight-list
	 (mapcar #'(lambda (l) (list l 0))
		 (send robot :torso :links))))
    (list
     :min-vec #F(0 0 0 0 0 0)
     :max-vec #F(0 0 0 0 0 0)
     :root-link-virtual-joint-weight #F(0 0 0 0 0 0)
     )))
  ;; :fullbody-inverse-kinematics
  (:inverse-kinematics-service-cb ;; [m] or [rad] expected
   (msg)
   (cond
    ((and
      (not (substringp
	    ":fullbody-inverse-kinematics"
	    (send (send msg :ik_request) :group_name)))
      (not (substringp
	    ":inverse-kinematics"
	    (send (send msg :ik_request) :group_name))))
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
	    (mjs (send rsd :multi_dof_joint_state))
	    transform-key
	    ik-param ret additional-weight-list
	    buf pos rot)
       (if (substringp
	    ":inverse-kinematics"
	    (send (send msg :ik_request) :group_name))
	   (setq opt
		 (append
		  (send self :inverse-kinematics-group-options opt)
		  opt)))
       ;;
       (format t "[:fik-service-cb] options: ~A~%" opt)
       ;; joint angle update
       (send self :update-joint-states js
	     :name-list (send js :name)
	     :position-list (send js :position)
	     :opt opt)
       (send self :update-joint-states mjs
	     :name-list (send mjs :joint_names)
	     :position-list (send mjs :joint_transforms)
	     :opt opt)
       ;; move-target update
       (send self :update-move-target
	     (send (send req :constraints) :position_constraints)
	     (send (send req :constraints) :orientation_constraints)
	     opt)
       ;; foot convex udpate
       (if (cadr (member :support-links opt))
	   (send self :update-support-links
		 (cadr (member :support-links opt))))
       (send self :make-foot-convex :force? t)
       (cond
	(viewer
	 (send viewer :objects
	       (list robot centroid-object foot-convex arrow))
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
		     (send (send req :constraints) :joint_constraints)
		     opt))
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
	 (send-all link-list :worldcoords)
	 (send res :solution :joint_state :name
	       (append
		(mapcar
		 #'(lambda (n) (format nil "~A" n))
		 (send-all link-list :name))
		;;(list "BASE_POSITION_X" "BASE_POSITION_Y" "BASE_POSITION_Z")
		;;(list "BASE_EULER_X" "BASE_EULER_Y" "BASE_EULER_Z")
		;;(list "BASE_QUAT_W" "BASE_QUAT_X" "BASE_QUAT_Y" "BASE_QUAT_Z")
		))
	 (send res :solution :joint_state :position
	       (concatenate
		float-vector
		(map float-vector #'deg2rad
		     (send-all (send-all link-list :joint) :joint-angle))
		;;(scale 1e-3 (send (car (send robot :links)) :worldpos))
		;;(matrix-log (send (car (send robot :links)) :worldrot))
		;;(matrix2quaternion (send (car (send robot :links)) :worldrot))
		))
	 (send res :solution :multi_dof_joint_state :joint_names
	       (list "VIRTUAL_JOINT"))
	 (setq pos (send (car (send robot :links)) :worldpos))
	 (setq rot (matrix2quaternion
		    (send (car (send robot :links)) :worldrot)))
	 (if (and mjs ;; for hrdro
		  (or (find-method mjs (setq transform-key :joint_transforms))
		      (find-method mjs (setq transform-key :transforms))))
	     (send res :solution :multi_dof_joint_state transform-key
		   (list
		    (instance geometry_msgs::Transform
			      :init
			      :translation
			      (instance geometry_msgs::Vector3
					:init
					:x (* 1e-3 (aref pos 0))
					:y (* 1e-3 (aref pos 1))
					:z (* 1e-3 (aref pos 2)))
			      :rotation
			      (instance geometry_msgs::Quaternion
					:init
					:w (aref rot 0)
					:x (aref rot 1)
					:y (aref rot 2)
					:z (aref rot 3)))))
	   (format t "[:fik-cb] unknown type of multi_dof_joint_state~%"))
	 res)
	(t res))))))
  (:nomethod
   (&rest args)
   (let (sym val)
     (cond
      ((keywordp (car args))
       (setq sym (read-from-string (send (car args) :pname)))
       (setq val (assoc sym (send self :slots)))))
     (cond
      ((or (null sym) (null val)) nil)
      ((> (length args) 1)
       (eval (list 'setq sym '(cadr args))))
      (t (cdr val)))))
  )




;; IK methods definition
(require "package://jsk_ik_server/euslisp/ik-server-util.l")
(require "package://jsk_ik_server/euslisp/old-ik-server.l")
