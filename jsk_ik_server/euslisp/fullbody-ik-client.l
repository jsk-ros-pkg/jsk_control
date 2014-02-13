#!/usr/bin/env roseus

;;o(ros::load-ros-manifest "hrpsys_gazebo_atlas")
(ros::load-ros-manifest "moveit_msgs")

;;(ros::roseus "fullbody_ik_client_test")

(defclass fullbody-ik-client)
(defmethod fullbody-ik-client
  (:init nil)
  (:coords2posestamped
   (coords
    &key
    (robot *robot*)
    (frame_id "/pelvis")
    (origin
     (copy-object
      (send (car (send robot :links)) :worldcoords)))
    (coords-from-origin
     (send origin :transformation coords))
    (q (matrix2quaternion (send coords-from-origin :worldrot))))
   (instance
    geometry_msgs::PoseStamped
    :init
    :header (instance std_msgs::Header :init
		      :stamp (ros::time 0.0)
		      :frame_id "/pelvis")
    :pose
    (instance geometry_msgs::Pose
	      :init
	      :position
	      (instance geometry_msgs::Point :init
			:x (* 1e-3 (aref (send coords-from-origin :worldpos) 0))
			:y (* 1e-3 (aref (send coords-from-origin :worldpos) 1))
			:z (* 1e-3 (aref (send coords-from-origin :worldpos) 2)))
	      :orientation
	      (instance geometry_msgs::Quaternion :init
			:x (elt q 1)
			:y (elt q 2)
			:z (elt q 3)
			:w (elt q 0))
	      )))
  ;;
  (:update-joint-states
   (js &key
       (robot *robot*)
       (joint-list (send robot :joint-list))
       buf)
   (setq buf (concatenate float-vector
			  (send (car (send robot :links)) :worldpos)
			  (matrix-log
			   (send (car (send robot :links)) :worldrot))))
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
		  joint-list))
	 (setq joint-list (remove j joint-list))
	 (send j :joint-angle (rad2deg pos)))
	((and
	  (setq j (or j (find-if
			 #'(lambda (j) (string-equal
					(remove #\: (format nil "~A" (send j :name)))
					name))
			 (send robot :links))))
	  (find-method j :joint)
	  (setq j (send j :joint))
	  (not (find j joint-list)))
	 (setq joint-list (remove j joint-list))
	 (send j :joint-angle (rad2deg pos))))))
   (setq
    buf
    (make-coords :pos (subseq buf 0 3)
		 :rpy (reverse (subseq buf 3 6))))
   (dotimes (i 3)
     (send robot :transform
	   (send
	    (copy-object
	     (send (car (send robot :links)) :worldcoords))
	    :transformation
	    buf)))
   (send-all (send robot :links) :worldcoords)
   (if (and (boundp '*viewer*) *viewer*) (send *viewer* :draw-objects))
   ;;(read-line)
   )
  ;;
  (:ik-server-call
   (&rest
    args
    &key
    (frame-id "/pelvis")
    (ik-server-service-name "/solve_ik")
    (group-name ":fullbody-inverse-kinematics")
    (robot *robot*)
    (target-limbs '(:rarm :larm :rleg :lleg))
    (move-target (mapcar #'(lambda (k) (send robot k :end-coords)) target-limbs))
    (target-coords (send-all move-target :copy-worldcoords))
    (additional-weight-list nil)
    ll
    collision-pair
    &allow-other-keys)
   (let* ((req (instance moveit_msgs::GetPositionIKRequest :init))
	  pc pc-list oc oc-list tf q res
	  (opt (list :worldcoords
		     (list 'make-coords
			   :pos (send (car (send robot :links)) :worldpos)
			   :rot (send (car (send robot :links)) :worldrot)))))
     ;; options
     (while (not (null args))
       (if (or (numberp (cadr args))
	       (symbolp (cadr args))
	       (vectorp (cadr args))
	       (and (listp (cadr args))
		    (or (find-if #'numberp (cadr args))
			(not (zerop (count nil (cadr args))))
			(find-if #'symbolp (cadr args)))))
	   (setq opt (append (list (car args) (cadr args)) opt)))
       (setq args (cddr args)))
     ;;(print opt)
     ;; group_name
     (send req :ik_request :group_name
	   (format nil "~A ~A" group-name opt))
     ;; joint state
     (send req :ik_request :robot_state :joint_state
	   (instance sensor_msgs::JointState
		     :init
		     :header
		     (instance std_msgs::Header :init
			       :stamp (ros::time 0.0)
			       :frame_id frame-id)
		     :name
		     (mapcar #'(lambda (j) (format nil "~A" (send j :name)))
			     (send-all (send robot :joint-list) :child-link))
		     :position
		     (map float-vector #'deg2rad
			  (send-all (send robot :joint-list) :joint-angle))))
     ;; move-target
     (dolist (mt move-target)
       (setq pc (instance moveit_msgs::PositionConstraint :init))
       (send pc :link_name
	     (format nil "~A" (send (send mt :parent) :name)))
       (setq tf
	     (send
	      (copy-object (send (send mt :parent) :worldcoords))
	      :transformation
	      (send mt :copy-worldcoords)))
       (send pc :target_point_offset
	     (instance geometry_msgs::vector3
		       :init
		       :x (* 1e-3 (aref (send tf :worldpos) 0))
		       :y (* 1e-3 (aref (send tf :worldpos) 1))
		       :z (* 1e-3 (aref (send tf :worldpos) 2))))
       (push pc pc-list)
       ;;
       (setq oc (instance moveit_msgs::OrientationConstraint :init))
       (send oc :link_name
	     (format nil "~A" (send (send mt :parent) :name)))
       (setq q (matrix2quaternion (send tf :worldrot)))
       (send oc :orientation
	     (instance geometry_msgs::Quaternion
		       :init
		       :w (aref q 0)
		       :x (aref q 1)
		       :y (aref q 2)
		       :z (aref q 3)))
       (push oc oc-list))
     (send req :ik_request :constraints :position_constraints pc-list)
     (send req :ik_request :constraints :orientation_constraints oc-list)
     ;; additional-weight-list
     (send req :ik_request :constraints :joint_constraints
	   (mapcar
	    #'(lambda (aw)
		(instance moveit_msgs::JointConstraint
			  :init
			  :joint_name
			  (format nil "~A" (send (car aw) :name))
			  :weight (cadr aw)))
	    additional-weight-list))
     ;; target coords
     (if move-target
	 (send req :ik_request :ik_link_name
	       (format nil "~A"
		       (send (send (car move-target) :parent) :name))))
     (send req :ik_request :ik_link_names
	   (mapcar #'(lambda (mt)
		       (format nil "~A" (send (send mt :parent) :name)))
		   (cdr move-target)))
     (if target-coords
	 (send req :ik_request :pose_stamped
	       (send self :coords2posestamped (car target-coords) :robot robot)))
     (send req :ik_request :pose_stamped_vector
	   (mapcar
	    #'(lambda (tc) (send self :coords2posestamped tc :robot robot))
	    (cdr target-coords)))
     ;;
     (setq res (ros::service-call ik-server-service-name req))
     (cond
      ((= (send res :error_code :val) moveit_msgs::MoveItErrorCodes::*SUCCESS*)
       (send self :update-joint-states (send res :solution :joint_state) :robot robot)
       (ros::ros-info " success ~A"  (send res :solution :joint_state :position)))
      (t (ros::ros-error " failed ")))
     res))
  )

(defun ik-server-call
  (&rest args &key target-coords &allow-other-keys)
  (apply
   #'send
   (append
    (list (instance fullbody-ik-client :init) :ik-server-call)
    args)))

