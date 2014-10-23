#!/usr/bin/env roseus

(ros::load-ros-manifest "jsk_ik_server")
;;(ros::load-ros-manifest "arm_navigation_msgs")
(ros::load-ros-manifest "sensor_msgs")
(ros::load-ros-manifest "moveit_msgs")
(ros::load-ros-manifest "geometry_msgs")

(require "models/arrow-object.l")
(require "fullbody-ik-client.l")

(defclass ik-server
  :slots
  (robot
   link-list
   joint-list
   arrow
   centroid-object
   foot-convex
   root-link-frame-id
   success-state-collection
   ;;
   default-end-coords
   default-collision-avoidance-link-pair
   ;;
   ik-server-name
   ik-server-service-name
   ik-server-client
   ;;
   support-links
   support-centroid-pos
   viewer
   move-target-hash
   additional-ik-options
   ))

(defmethod ik-server
  (:init
   (&rest
    args
    &key
    robot
    (link-list
     (union
      nil
      (remove-if
       #'(lambda (a)
	   (not
	    (and (class a)
		 (subclassp (class a) bodyset-link))))
       (flatten (send robot :slots)))))
    (joint-list
     (flatten
      (mapcar
       #'(lambda (l)
	   (if (and (find-method l :joint)
		    (class (send l :joint))
		    (subclassp (class (send l :joint)) joint))
	       (send l :joint)))
       link-list)))
    (move-target-hash (make-hash-table))
    (default-end-coords
      (apply
       #'append
       (mapcar
	#'(lambda (k) (if (and (find-method robot k) (send robot k))
			  (list (cons k (send robot k :end-coords)))))
	'(:rarm :larm :rleg :lleg))))
    (ik-server-client
     (instance fullbody-ik-client :init))
    (ik-server-name (format nil "~A_ik_server" (send robot :name)))
    (ik-server-service-name "/solve_ik")
    &allow-other-keys
    )
   (while args
     (send self (car args) (cadr args))
     (setq args (cddr args)))
   (send self :link-list link-list)
   (send self :joint-list joint-list)
   (send self :move-target-hash move-target-hash)
   (send self :ik-server-client ik-server-client)
   (send self :set-default-end-coords default-end-coords)
   (send self :ik-server-name ik-server-name)
   (send self :ik-server-service-name ik-server-service-name)
   (cond
    ((find-method robot :rleg)
     (send self :update-support-links '(:rleg :lleg))
     (send self :make-foot-convex :force? t)
     (send self :make-centroid-object)))
   )
  (:ik-service-cb
   (req)
   (or
    (send self :inverse-kinematics-with-both-leg-balancing-service-cb req)
    (send self :inverse-kinematics-service-cb req)
    (send self :old-ik-service-cb req)))
  (:start-ik-server
   nil
   ;; initialization
   ;;(unless root-link-frame-id (setq root-link-frame-id "/base_link"))
   (setq arrow (arrow))
   ;; check viewer
   (cond
    ((or (null x::*display*) (= x::*display* 0)) nil)
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
   (when (ros::has-param "~additional_ik_options")
     (let ((str (ros::get-param "~additional_ik_options")))
       (if (> (length str) 1)
           (setq additional-ik-options
                 (if (= (elt str 0) #\()
                     (read-from-string str)
                   (read-from-string (format nil "(~A)" str)))))
       ))
   (ros::advertise-service ik-server-service-name
			   moveit_msgs::GetPositionIK
			   #'send self :ik-service-cb)
   (ros::rate 10)
   (send self :loop-ik-server)
   )
  (:set-default-end-coords (c) (setq default-end-coords c))
  (:default-end-coords (k) (cdr (assoc k default-end-coords)))
  (:update-support-links
   (name-list
    &key
    (link-list
     (flatten
      (mapcar
       #'(lambda (name)
	   (cond
	    ((and (keywordp name)
		  (send self :default-end-coords name))
	     (send (send self :default-end-coords name) :parent))
	    (t (find-link-from-name name :link-list
				    (send-all joint-list :child-link)))))
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
			  (send-all support-links :worldpos))))))
   )
  (:loop-ik-server
   nil
   (send self :publish-default-end-coords :init? t)
   (do-until-key
    (if (not (ros::ok)) (return-from nil nil))
    (unless (or (null x::*display*) (= x::*display* 0))
      (x::window-main-one))
    (ros::spin-once)
    (send self :publish-default-end-coords :init? nil)
    ;;(ros::sleep)
   ( unix:usleep (* 10 1000))
    )
   )
  ;;
  ;;
  ;;
  (:publish-default-end-coords
   (&key (init? nil))
   (mapcar
    #'(lambda (dec)
	(if init?
	    (ros::advertise
	     (format nil "~A/default_end_coords/~A"
		     ik-server-name
		     (remove #\: (format nil "~A" (car dec))))
	     geometry_msgs::posestamped 1))
	(ros::publish
	 (format nil "~A/default_end_coords/~A"
		 ik-server-name
		 (remove #\: (format nil "~A" (car dec))))
	 (send ik-server-client :coords2posestamped
	       (send (cdr dec) :copy-worldcoords)
	       :robot robot))
	)
    default-end-coords))
  (:local-transform
   (frame-id
    pose
    &key
    (coords
     (make-coords
      :pos (float-vector
	    (or (* 1e+3 (send (send pose :position) :x)) 0)
	    (or (* 1e+3 (send (send pose :position) :y)) 0)
	    (or (* 1e+3 (send (send pose :position) :z)) 0))
      :rot
      (quaternion2matrix
       (float-vector
	(or (send (send pose :orientation) :w) 0)
	(or (send (send pose :orientation) :x) 0)
	(or (send (send pose :orientation) :y) 0)
	(or (send (send pose :orientation) :z) 0)))))
    (from-link
     (find-link-from-name
      (remove #\/ frame-id) :link-list link-list))
    (from-coords (if from-link (send from-link :copy-worldcoords)
		   (make-coords)))
    (to-coords
     (send (car (send robot :links)) :copy-worldcoords)))
   (format t "link name=~A, link=~A~%" frame-id from-link)
   (send to-coords :transformation
	 (send from-coords :transform coords :local)))
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
	 :joint-list joint-list
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
       (setq link (find-link-from-name name :link-list
				       (send-all joint-list :child-link)))
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
       (setq link (find-link-from-name name :link-list
				       (send-all joint-list :child-link)))
       (cond
	((null link)
	 (format t "[:update-move-target] unknown link ~A ~%" name))
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
       (setq link (find-link-from-name name :link-list
				       (send-all joint-list :child-link)))
       (cond
	((and
	  (null link)
	  (null
	   (setq joint
		 (find-link-from-name
		  name :link-list joint-list)))))
	(t
	 (setq link (or link (send joint :child-link)))
	 (push (list link (send jc :weight)) ret))))
     (append
      (cadr (member :additional-weight-list opt)) ret)))
  (:parse-collision-link-pair
   (collisions &optional opt default-collisions)
   (let (buf link name)
     (dolist (col collisions)
       (setq name (remove #\: (send col :link_name)))
       (setq link (find-link-from-name name :link-list
				       (send-all joint-list :child-link)))
       (if link (push link buf)))
     (setq buf (union buf nil))
     (append
      (apply
       #'append
       (mapcar
	#'(lambda (a)
	    (mapcar #'(lambda (b) (list a b)) (cdr (member a buf))))
	buf))
      (cadr (member :collision-avoidance-link-pair opt))
      default-collisions
      )))
  (:parse-ik-param
   (ik-link-name pose-stamped
    ik-link-name-list pose-stamped-list
    &optional opt)
   (let (mt ll tc link name buf tmp (id -1)
	 goal goal-in-world goal-in-pelvis
	 (ik-link-name-list (cons ik-link-name ik-link-name-list)))
     (dolist (ps (cons pose-stamped pose-stamped-list))
       (incf id)
       (setq name (remove #\: (car ik-link-name-list)))
       (setq ik-link-name-list (cdr ik-link-name-list))
       (setq link (find-link-from-name name :link-list
				       (send-all joint-list :child-link)))
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
	 (setq tmp
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
		  (or (send (send (send ps :pose) :orientation) :z) 0)))))
	 (setq
	  goal-in-pelvis
	  (cond
	   ((and root-link-frame-id
		 (send ps :header :frame_id)
		 (send ps :header :stamp)
		 (setq goal-in-pelvis
		       (send *tf* :lookup-transform
			     (send ps :header :frame_id)
			     root-link-frame-id
			     (send ps :header :stamp))))
	    (send tmp :transform goal-in-pelvis))
	   (t (send self :local-transform
		    (send ps :header :frame_id)
		    (send ps :pose)
		    :coords tmp))))
	 (setq goal-in-world
	       (send (send (car (send robot :links)) :copy-worldcoords)
		     :transform goal-in-pelvis))
	 (if (zerop id)
	     (send arrow :newcoords goal-in-world))
	 (push goal-in-world tc))))
     (setq buf
	   (flatten
	    (mapcar
	     #'(lambda (k) (send self :default-end-coords k))
	     (cadr (member :fix-limbs opt)))))
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
   (msg
    &key
    (group_name (send (send msg :ik_request) :group_name))
    (opt (if (plusp (length group_name))
	     (cadr (read-from-string
		    (format nil "(~A)" group_name))))))
   (cond
    ((and
      (not (substringp ":fullbody-inverse-kinematics" group_name))
      (not (substringp ":inverse-kinematics" group_name)))
     (format t "[:fik-service-cb] invalid group ~A~%" group_name)
     nil)
    (t
     (let* ((res (send msg :response))
	    (req (send msg :ik_request))
	    (rsd (send req :robot_state))
	    (js (send rsd :joint_state))
	    (mjs (send rsd :multi_dof_joint_state))
	    transform-key
	    ik-param ret additional-weight-list
	    buf pos rot)
       (if (substringp ":inverse-kinematics" group_name)
	   (setq opt
		 (append
		  (send self :inverse-kinematics-group-options opt)
		  opt)))
       ;; merge additional-ik-options, additional options might be overwritten by opt
       (let (tmp (aop additional-ik-options))
         (while aop
           (let* ((key (car aop))
                  (val (assoc key opt)))
             (unless val
               (push (cadr aop) tmp)
               (push key tmp))
             (setq aop (cddr aop))
             ))
         (setq additional-ik-options tmp))
       (setq opt (append opt additional-ik-options))
       ;;
       (format t "[:fik-service-cb] options: ~A~%" opt)
       ;; joint angle update
       (send self :update-joint-states js
	     :name-list (send js :name)
	     :position-list (send js :position)
	     :opt opt)
       (if (and mjs ;; for hrdro
		(or (find-method mjs (setq transform-key :joint_transforms))
		    (find-method mjs (setq transform-key :transforms))))
	   (send self :update-joint-states mjs
		 :name-list (send mjs :joint_names)
		 :position-list (send mjs transform-key)
		 :opt opt))
       ;; move-target update
       (send self :update-move-target
	     (send (send req :constraints) :position_constraints)
	     (send (send req :constraints) :orientation_constraints)
	     opt)
       ;; foot convex udpate
       (if (cadr (member :support-links opt))
	   (send self :update-support-links
		 (cadr (member :support-links opt))))
       ;;
       (send res :error_code :val
	     (cond
	      ((boundp 'moveit_msgs::MoveItErrorCodes::*FAILURE*)
	       moveit_msgs::MoveItErrorCodes::*FAILURE*)
	      ((boundp 'moveit_msgs::MoveItErrorCodes::*NO_IK_SOLUTION*)
	       moveit_msgs::MoveItErrorCodes::*NO_IK_SOLUTION*)))
       (setq ik-param
	     (append
	      (send self :parse-ik-param
		    (send req :ik_link_name)
		    (send req :pose_stamped)
		    (send req :ik_link_names)
		    (send req :pose_stamped_vector)
		    opt)
	      (let ((collision-pair
		     (send self :parse-collision-link-pair
			   (send rsd :attached_collision_objects)
			   nil
			   (if (send req :avoid_collisions)
			       default-collision-avoidance-link-pair)
			   )))
		(list
		 :collision-avoidance-link-pair collision-pair
		 ;;:union-link-list #'(lambda nil (union nil (flatten collision-pair)))
		 ))
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
	   (send self :make-foot-convex
		 :support-links support-links
		 :support-link-target-transform-coords
		 (let ((mt (cadr (member :move-target ik-param)))
		       (tc (cadr (member :target-coords ik-param)))
		       target)
		   (format t "[:support-link] ~A~%"
			   (send-all support-links :name))
		   (format t "[:move-target] ~A~%"
			   (send-all (send-all mt :parent) :name))
		   (mapcar
		    #'(lambda (l)
			(cond
			 ((setq target
				(or
				 (find-if
				  #'(lambda (mttc)
				      (eq (send (car mttc) :parent) l))
				  (mapcar #'cons mt tc))
				 (find-if
				  #'(lambda (mttc)
				      (find l (send robot
						    :link-list
						    (send (car mttc) :parent))))
				  (mapcar #'cons mt tc))
				 (find-if
				  #'(lambda (mttc)
				      (find (send (car mttc) :parent)
					    (send robot :link-list l)))
				  (mapcar #'cons mt tc))))
			  (send (send l :copy-worldcoords)
				:transformation
				(send
				 (send (copy-object (cdr target))
				       :transform
				       (send
					(send (car target) :copy-worldcoords)
					:transformation
					(send (send (car target) :parent)
					      :copy-worldcoords))
				       :local)
				 :transform
				 (send (send (send (car target) :parent)
					     :copy-worldcoords)
				       :transformation
				       (send l :copy-worldcoords))
				 :local)))
			 (t (make-coords))))
		    support-links))
		 :force? t)
	   ;;
	   (cond
	    (viewer
	     (send viewer :objects
		   (list robot centroid-object foot-convex arrow))
	     (send viewer :draw-objects)
	     ;;(unix:sleep 3)
	     ))
	   ;;
	   (setq ret
		 (send* self :fullbody-ik-main
			ik-param))
	   (vectorp ret))
	 (format t "[:fik-service-cb] output ...~%")
	 ;;(format t "[ik-server] SUCCESS!!~%")
	 (send res :error_code :val moveit_msgs::MoveItErrorCodes::*SUCCESS*)
	 (if viewer (send viewer :draw-objects))
	 (send-all link-list :worldcoords)
	 (send res :solution :joint_state :header :frame_id "/odom")
	 (send res :solution :joint_state :name
	       (append
		(mapcar
		 #'(lambda (n) (format nil "~A" n))
		 (send-all joint-list :name))
		;;(list "BASE_POSITION_X" "BASE_POSITION_Y" "BASE_POSITION_Z")
		;;(list "BASE_EULER_X" "BASE_EULER_Y" "BASE_EULER_Z")
		;;(list "BASE_QUAT_W" "BASE_QUAT_X" "BASE_QUAT_Y" "BASE_QUAT_Z")
		))
	 (send res :solution :joint_state :position
	       (concatenate
		float-vector
		(map float-vector #'deg2rad
		     (send-all joint-list :joint-angle))
		;;(scale 1e-3 (send (car (send robot :links)) :worldpos))
		;;(matrix-log (send (car (send robot :links)) :worldrot))
		;;(matrix2quaternion (send (car (send robot :links)) :worldrot))
		))
	 (send res :solution :multi_dof_joint_state :joint_names
	       ;;(list "VIRTUAL_JOINT")
	       (list "virtual_joint")
	       )
	 (send res :solution :multi_dof_joint_state :header :frame_id "/odom")
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
  (:inverse-kinematics-with-both-leg-balancing-service-cb ;; [m] or [rad] expected
   (msg)
   (cond
    ((substringp "whole_body" (send (send msg :ik_request) :group_name))
     (let ((tag (read-from-string
		 (format nil "(~A)" (send (send msg :ik_request) :group_name)))))
       (send self :inverse-kinematics-service-cb
	     msg
	     :group_name ":fullbody-inverse-kinematics"
	     :opt (append (cadr tag)
			  (list :fix-limbs '(:rleg :lleg)
				:debug-view nil)))))
    (t nil)))
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



#|

(ros::load-ros-manifest "moveit_msgs")
(ros::load-ros-manifest "sensor_msgs")
(ros::load-ros-manifest "geometry_msgs")

(ros::roseus "test")

(ros::service-call
 "/solve_ik"
 (instance
  moveit_msgs::GetPositionIKRequest
  :init
  :ik_request
  (instance
   moveit_msgs::PositionIKRequest
   :init
   :group_name ":fullbody-inverse-kinematics (:fix-limbs (:rleg :lleg))"
   :robot_state
   (instance
    moveit_msgs::robotstate
    :init
    :joint_state
    (instance sensor_msgs::jointstate
	      :init
	      :name
	      (list
	       "CHEST_JOINT0"
	       "CHEST_JOINT1"
	       "HEAD_JOINT0"
	       "HEAD_JOINT1"
	       "LARM_JOINT0"
	       "LARM_JOINT1"
	       "LARM_JOINT2"
	       "LARM_JOINT3"
	       "LARM_JOINT4"
	       "LARM_JOINT5"
	       "LARM_JOINT6"
	       "LARM_JOINT7"
	       "L_INDEXMP_R"
	       "L_INDEXMP_P"
	       "L_INDEXPIP_R"
	       "L_MIDDLEPIP_R"
	       "L_THUMBCM_Y"
	       "L_THUMBCM_P"
	       "RARM_JOINT0"
	       "RARM_JOINT1"
	       "RARM_JOINT2"
	       "RARM_JOINT3"
	       "RARM_JOINT4"
	       "RARM_JOINT5"
	       "RARM_JOINT6"
	       "RARM_JOINT7"
	       "R_INDEXMP_R"
	       "R_INDEXMP_P"
	       "R_INDEXPIP_R"
	       "R_MIDDLEPIP_R"
	       "R_THUMBCM_Y"
	       "R_THUMBCM_P"
	       "LLEG_JOINT0"
	       "LLEG_JOINT1"
	       "LLEG_JOINT2"
	       "LLEG_JOINT3"
	       "LLEG_JOINT4"
	       "LLEG_JOINT5"
	       "LLEG_JOINT6"
	       "RLEG_JOINT0"
	       "RLEG_JOINT1"
	       "RLEG_JOINT2"
	       "RLEG_JOINT3"
	       "RLEG_JOINT4"
	       "RLEG_JOINT5"
	       "RLEG_JOINT6")
	      :position
	      (list 0.0449376
		    0.322729
		    0
		    0
		    -0.0230563
		    0.0411221
		    -0.0796949
		    -0.562652
		    0.0482643
		    0
		    0
		    0
		    0
		    0
		    0
		    0
		    0
		    0
		    -0.281266
		    -0.020549
		    -0.0296894
		    -0.20813
		    -0.0598932
		    -0.0318
		    0.658795
		    0
		    0
		    0
		    0
		    0
		    0
		    0
		    -0.00580568
		    -0.0439399
		    -0.312627
		    0.473204
		    -0.0634008
		    0.00540425
		    0
		    -0.00579787
		    -0.0440984
		    -0.340275
		    0.530151
		    -0.092688
		    0.00558468)
	      )
    :multi_dof_joint_state
    (instance moveit_msgs::MultiDOFJointState
	      :init
	      :joint_names
	      (list "virtual_joint")
	      :joint_transforms
	      (list
	       (instance geometry_msgs::transform
			 :init
			 :translation
			 (instance geometry_msgs::Vector3
				   :init
				   :x 0
				   :y 0
				   :z 0)
			 :rotation
			 (instance geometry_msgs::Quaternion
				   :init
				   :x 0.00188104
				   :y 0.151814
				   :z -0.0122459
				   :w 0.988331)
			 )))
    )
   :ik_link_names
   (list "RARM_LINK6" "LARM_LINK6")
   :pose_stamped_vector
   (list
    (instance geometry_msgs::posestamped
	      :init
	      :pose
	      (instance geometry_msgs::pose
			:init
			:position
			(instance geometry_msgs::Vector3
				  :init
				  :x 0.303323
				  :y -0.179039
				  :z -0.00450461)
			:orientation
			(instance geometry_msgs::Quaternion
				  :init
				  :x -0.0394373
				  :y -0.581649
				  :z -0.00731869
				  :w 0.81245)))
    (instance geometry_msgs::posestamped
	      :init
	      :pose
	      (instance geometry_msgs::pose
			:init
			:position
			(instance geometry_msgs::Vector3
				  :init
				  :x 0.0909181
				  :y 0.258434
				  :z 0.0149688)
			:orientation
			(instance geometry_msgs::Quaternion
				  :init
				  :x 0.00226681
				  :y 0.021845
				  :z -0.0112564
				  :w 0.999695)))
    )
   )))

