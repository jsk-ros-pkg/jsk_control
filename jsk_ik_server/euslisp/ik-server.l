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
    (let* ((res (send req :response))
	   (arm (if (or (substringp "left_arm" (send req :ik_request :group_name))
			(substringp "larm" (send req :ik_request :group_name))
			(substringp "lhand" (send req :ik_request :group_name)))
		    :larm
		  :rarm))
	   (torsop (substringp "torso" (send req :ik_request :group_name)))
	   (torso2p (substringp "torso2" (send req :ik_request :group_name)))
	   (torso3p (substringp "torso3" (send req :ik_request :group_name)))
	   (fullbodyp (substringp "fullbody" (send req :ik_request :group_name)))
	   (opt-str (send (send (send req :ik_request) :constraints) :name))
	   (opt
	    (cond
	     ((and (stringp opt-str)
		   (plusp (length opt-str))
		   (listp (setq opt-str (read-from-string opt-str))))
	      opt-str)
	     (t (list :update-convex t)))))
      ;; taking joint state into account
      (cond
       ((and (send req :ik_request :robot_state)
	     (send req :ik_request :robot_state :joint_state)
	     (send req :ik_request :robot_state :joint_state :name)
	     (send req :ik_request :robot_state :joint_state :position)
	     (= (length (send req :ik_request :robot_state :joint_state :name))
		(length (send req :ik_request :robot_state :joint_state :position))))
	(dotimes (i (length (send req :ik_request :robot_state :joint_state :name)))
	  (let ((joint-name (elt (send req :ik_request :robot_state :joint_state :name) i)))
	    (let ((j (find joint-name (send *robot* :joint-list)
			   :test #'string= :key #'(lambda (x) (send x :name)))))
	      (if j
		  (send j :joint-angle
			(rad2deg
			 (elt
			  (send req :ik_request :robot_state :joint_state :position)
			  i)))
		(warn "we cannot find ~A~%" joint-name)
		)))))
       (t (send *robot* :reset-manip-pose)))
      (format t "[ik-server] options: ~A~%" opt)
      (format t "            target: limb=~A, torso=~A:~A:~A, fullbody=~A~%"
	      arm torsop torso2p torso3p fullbodyp)

      (send *robot* :fix-leg-to-coords (make-coords) :both)

      (cond
       ((cadr (member :update-convex opt))

	(send self :make-foot-convex :force? t)
	(send *irtviewer* :objects
	      (list *robot* *centroid-object* *foot-convex* *arrow*))))
      ;;(make-foot-convex)
      ;;(send *irtviewer* :objects
      ;;(list *robot* *foot-convex* *centroid-object* *arrow*))
      (send *viewer* :draw-objects)
      (let ((goal (send req :ik_request :pose_stamped))
	    ;; check that goal is malformed or not
	    (q (float-vector (send req :ik_request :pose_stamped :pose :orientation :w)
			     (send req :ik_request :pose_stamped :pose :orientation :x)
			     (send req :ik_request :pose_stamped :pose :orientation :y)
			     (send req :ik_request :pose_stamped :pose :orientation :z))))
	(send res :error_code :val moveit_msgs::MoveItErrorCodes::*FAILURE*)
	(cond
	 ((not (eps= (norm q) 1.0))
	  (ros::ros-error "Invalid quotanion detected ~A~%" q)
	  res)
	 ((not (equal *root-link* (send goal :header :frame_id)))
	  (ros::ros-error "root-link(~A) and frame_id(~A) must be same" *root-link* (send goal :header :frame_id))
	  res)
	 ((let* ((goal-in-pelvis (send *tf* :transform-pose *root-link* goal))
		 (goal-in-world
		  (send (send *robot* :copy-worldcoords)
			:transform goal-in-pelvis))
		 (draw-arrow
		  (send *arrow* :newcoords goal-in-world))
		 (ik-ret (cond
			  (fullbodyp
			   (apply
			    #'send self :fullbody-ik-with-collision
			    (append
			     (list arm goal-in-world)
			     opt))
			   )
			  (torso2p
			   (apply
			    #'send self :torso-ik-with-collision
			    (append
			     (list arm goal-in-world)
			     (list :root-link-virtual-joint-weight
				   (float-vector 0.02 0.02 0.1 0.0 0.0 0.04))
			     opt)))
			  (torso3p
			   (apply
			    #'send self :torso-ik-with-collision
			    (append
			     (list arm goal-in-world)
			     (list :waist-gain '(0 0 1))
			     opt)))
			  (torsop
			   (apply
			    #'send self :torso-ik-with-collision
			    (append
			     (list arm goal-in-world)
			     opt)))
			  (t
			   (apply
			    #'send self :normal-ik-with-collision
			    (append
			     (list arm goal-in-world)
			     opt))))))
	    ;; (print (list 'original goal))
	    ;; (print (list 'original
	    ;;              (send goal :header :frame_id)
	    ;;              (send goal :pose :position :x)
	    ;;              (send goal :pose :position :y)
	    ;;              (send goal :pose :position :z)
	    ;;              (send goal :pose :orientation :x)
	    ;;              (send goal :pose :orientation :y)
	    ;;              (send goal :pose :orientation :z)
	    ;;              (send goal :pose :orientation :w)))
	    ;; (print (list 'goal-in-pelvis goal-in-pelvis))
	    ;; (print (list 'goal-in-world goal-in-world))
	    (vectorp ik-ret))
	  ;;(format t "[ik-server] SUCCESS!!~%")
	  (send res :error_code :val moveit_msgs::MoveItErrorCodes::*SUCCESS*)
	  (send *viewer* :draw-objects)
	  (send res :solution :joint_state :name
		(send-all (send *robot* :joint-list) :name))
	  (send res :solution :joint_state :position
		(map float-vector #'deg2rad (send *robot* :angle-vector)))
	  res)
	 (t res)))))
   )
  (:start-ik-server
   nil
   ;; initialization
   (unless *root-link* (setq *root-link* "/base_link"))
   (setq *arrow* (arrow))

   (cond
    ((not (and (boundp '*irtviewer*) *irtviewer*))
     (make-irtviewer :width 640 :height 640)
     (send *irtviewer* :change-background #f(0.9 1.0 0.9))
     (send *irtviewer* :title "IK server")))
   (send *irtviewer*
	 :objects
	 (list *robot* *arrow*))

   (unless *foot-convex* (setq *foot-convex* (make-coords)))
   (unless *centroid-object* (setq *centroid-object* (make-coords)))
   (send *irtviewer*
	 :objects
	 (list *robot* *foot-convex* *arrow*))
   (send *irtviewer* :look-all)
   (send *viewer* :draw-objects)
   ;;
   (setq *tf* (instance ros::transform-listener :init))
   ;;
   (send *irtviewer*
	 :objects
	 (list *robot* *foot-convex* *arrow* *centroid-object*))

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
   (send *viewer* :draw-objects)
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
	 (format t "[:update-move-target] unknown link~A~%" name))
	(t
	 (send mt-coords :enter
	       link
	       (list :pos
		     (float-vector
		      (or (send (send pc :target_point_offset) :x) 0)
		      (or (send (send pc :target_point_offset) :y) 0)
		      (or (send (send pc :target_point_offset) :z) 0))))
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
	 (format t "[:update-move-target] unknown link~A~%" name))
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
	 (setq goal-in-pelvis
	       (make-coords
		:pos (float-vector
		      (or (send (send (send ps :pose) :position) :x) 0)
		      (or (send (send (send ps :pose) :position) :y) 0)
		      (or (send (send (send ps :pose) :position) :z) 0))
		:rot
		(quaternion2matrix
		 (float-vector
		  (or (send (send (send ps :pose) :orientation) :w) 0)
		  (or (send (send (send ps :pose) :orientation) :x) 0)
		  (or (send (send (send ps :pose) :orientation) :y) 0)
		  (or (send (send (send ps :pose) :orientation) :z) 0)))))
	 ;;(send *tf* :transform-pose *root-link* goal))
	 (if (not (string-equal *root-link* (send (send ps :header) :frame_id)))
	     (format t "[:parse-ik-param] frame_id must be ~A~%" *root-link*))
	 (setq goal-in-world
	       (send (send (car (send *robot* :links)) :copy-worldcoords)
		     :transform goal-in-pelvis))
	 (if (zerop id)
	     (send *arrow* :newcoords goal-in-world))
	 (push goal-in-world tc))))
     (append
      opt
      (list
       :move-target (reverse mt)
       :target-coords (reverse tc)
       :link-list (reverse ll)))
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
       (send *irtviewer* :objects
	     (list *robot* *centroid-object* *foot-convex* *arrow*))
       (send *viewer* :draw-objects)
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
       (format t "[:fik-service-cb] args=~A~%" ik-param)
       (setq ret
	     (send* self :fullbody-ik-main
		    ik-param))
       (cond
	((vectorp ret)
	 ;;(format t "[ik-server] SUCCESS!!~%")
	 (send res :error_code :val moveit_msgs::MoveItErrorCodes::*SUCCESS*)
	 (send *viewer* :draw-objects)
	 (send res :solution :joint_state :name
	       (append
		(send-all (send *robot* :joint-list) :name)
		(list "basex" "basey" "basez" "baser" "basep" "basew")))
	 (send res :solution :joint_state :position
	       (concatenate
		float-vector
		(map float-vector #'deg2rad (send *robot* :angle-vector))
		(scale 1e-3 (send (car (send *robot* :links)) :worldpos))
		(matrix-log (send (car (send *robot* :links)) :worldrot))))
	 res)
	(t res))))))
  )


;; IK methods definition
(require "package://jsk_ik_server/euslisp/eus-fullbody-ik-ex.l")

