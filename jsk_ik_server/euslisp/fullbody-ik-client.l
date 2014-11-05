#!/usr/bin/env roseus

;;o(ros::load-ros-manifest "hrpsys_gazebo_atlas")
(ros::load-ros-manifest "moveit_msgs")
(ros::load-ros-manifest "geometry_msgs")
(ros::load-ros-manifest "sensor_msgs")

;;(ros::roseus "fullbody_ik_client_test")

(defclass fullbody-ik-client)
(defmethod fullbody-ik-client
  (:init nil)
  (:coords2posestamped
   (coords
    &key
    (robot *robot*)
    (origin-link (car (send robot :links)))
    (frame_id (format nil "/~A" (send origin-link :name)))
    (origin (send origin-link :copy-worldcoords))
    (coords-from-origin
     (send origin :transformation coords))
    (q (matrix2quaternion (send coords-from-origin :worldrot))))
   (instance
    geometry_msgs::PoseStamped
    :init
    :header (instance std_msgs::Header :init
		      :stamp (ros::time 0.0)
		      :frame_id frame_id)
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
   (js
    &key
    (name-list (send js :name))
    (position-list (send js :position))
    opt
    (robot *robot*)
    (joint-list (send robot :joint-list))
    ;;(viewer (if (and (boundp '*viewer*) *viewer*) *viewer*))
    position rpy quaternion rpy-flag)
   (setq base-pos
	 (copy-object (send (car (send robot :links)) :worldpos)))
   (setq base-rpy
	 (matrix-log
	  (copy-object (send (car (send robot :links)) :worldrot))))
   (setq base-quaternion
	 (matrix2quaternion
	  (copy-object (send (car (send robot :links)) :worldrot))))
   (dotimes (i (length name-list))
     (if (or (>= i (length name-list))
	     (>= i (length position-list)))
	 (return-from nil nil))
     (let ((name (string-downcase (remove #\: (elt name-list i))))
	   (pos (elt position-list i)) j)
       ;;(format t "[:update-joint-states] ~A ~Arad~%" name pos)
       (cond
	((setq j (find-link-from-name name :link-list joint-list))
	 (setq joint-list (remove j joint-list))
	 (cond
	  ((and (numberp pos)
		(class j)
		(or (subclassp (class j) linear-joint)
		    (subclassp (class j) rotational-joint)))
	   (send j :joint-angle (rad2deg pos)))
	  ((and (class pos)
		(class j)
		(subclassp (class pos) geometry_msgs::Transform)
		(subclassp (class j) 6dof-joint))
	   (send j :joint-angle
		 (concatenate
		  float-vector
		  (scale 1e+3 (float-vector (send pos :translation :x)
					    (send pos :translation :y)
					    (send pos :translation :z)))
		  (matrix-log
		   (quaternion2matrix
		    (float-vector (send pos :rotation :w)
				  (send pos :rotation :x)
				  (send pos :rotation :y)
				  (send pos :rotation :z)))))))
	  )
	 )
	((and (setq j (find-link-from-name
		       name :link-list
		       (send-all joint-list :child-link)))
	      (find-method j :joint)
	      (setq j (send j :joint)))
	 (setq joint-list (remove j joint-list))
	 (cond
	  ((and (numberp pos)
		(class j)
		(or (subclassp (class j) linear-joint)
		    (subclassp (class j) rotational-joint)))
	   (send j :joint-angle (rad2deg pos)))
	  ((and (class pos)
		(class j)
		(subclassp (class pos) geometry_msgs::Transform)
		(subclassp (class j) 6dof-joint))
	   (send j :joint-angle
		 (concatenate
		  float-vector
		  (scale 1e+3 (float-vector (send pos :translation :x)
					    (send pos :translation :y)
					    (send pos :translation :z)))
		  (matrix-log
		   (quaternion2matrix
		    (float-vector (send pos :rotation :w)
				  (send pos :rotation :x)
				  (send pos :rotation :y)
				  (send pos :rotation :z)))))))
	  )
	 )
	((or (substringp "base" name)
	     (substringp "root" name)
	     (substringp "virtual" name))
	 (cond
	  ((and
	    (class pos)
	    (subclassp (class pos) geometry_msgs::Transform))
	   (setf (aref base-pos 0) (* 1e+3 (send pos :translation :x)))
	   (setf (aref base-pos 1) (* 1e+3 (send pos :translation :y)))
	   (setf (aref base-pos 2) (* 1e+3 (send pos :translation :z)))
	   (setf (aref base-quaternion 0) (send pos :rotation :w))
	   (setf (aref base-quaternion 1) (send pos :rotation :x))
	   (setf (aref base-quaternion 2) (send pos :rotation :y))
	   (setf (aref base-quaternion 3) (send pos :rotation :z)))
	  ((or (substringp "pos" name)
	       (substringp "trans" name))
	   (cond
	    ((find #\x name)
	     ;;(format t "[:udpate-joint-state] base_pose_x ~A~%" pos)
	     (setf (aref base-pos 0) (* 1e+3 pos)))
	    ((find #\y name)
	     ;;(format t "[:udpate-joint-state] base_pose_y ~A~%" pos)
	     (setf (aref base-pos 1) (* 1e+3 pos)))
	    ((find #\z name)
	     ;;(format t "[:udpate-joint-state] base_pose_z ~A~%" pos)
	     (setf (aref base-pos 2) (* 1e+3 pos)))))
	  ((or (substringp "eul" name)
	       (substringp "rot" name))
	   (setq rpy-flag t)
	   (cond
	    ((find #\x name)
	     ;;(format t "[:udpate-joint-state] base_eul_x ~A~%" pos)
	     (setf (aref base-rpy 0) pos))
	    ((find #\y name)
	     ;;(format t "[:udpate-joint-state] base_eul_y ~A~%" pos)
	     (setf (aref base-rpy 1) pos))
	    ((find #\z name)
	     ;;(format t "[:udpate-joint-state] base_eul_z ~A~%" pos)
	     (setf (aref base-rpy 2) pos))))
	  ((substringp "qua" name)
	   (cond
	    ((find #\w name)
	     (setf (aref base-quaternion 0) pos))
	    ((find #\x name)
	     (setf (aref base-quaternion 1) pos))
	    ((find #\y name)
	     (setf (aref base-quaternion 2) pos))
	    ((find #\z name)
	     (setf (aref base-quaternion 3) pos))))
	  (t (format t "[:update-joint-state] unknown link ~A ~%" name))
	  )))))
   (if (and (cadr (member :angle-vector opt))
	    (eq (length (cadr (member :angle-vector opt)))
		(length (send robot :angle-vector))))
       (send robot :angle-vector (cadr (member :angle-vector opt))))
   (setq
    buf
    (cond
     ((cadr (member :worldcoords opt))
      (cond
       ((subclassp (class (cadr (member :worldcoords opt))) coordinates)
	(cadr (member :worldcoords opt)))
       ((functionp (car (cadr (member :worldcoords opt))))
	(eval (cadr (member :worldcoords opt))))))
     (rpy-flag
      (make-coords :pos base-pos
		   :rpy (reverse base-rpy)))
     (t
      (make-coords :pos base-pos
		   :rot (quaternion2matrix base-quaternion)))))
   (dotimes (i 3)
     (send robot :transform
	   (send
	    (copy-object
	     (send (car (send robot :links)) :worldcoords))
	    :transformation
	    buf)))
   (send-all (send robot :links) :worldcoords)
   ;;(if viewer (send viewer :draw-objects))
   ;;(read-line)
   )
  ;;
  (:ik-server-call
   (&rest
    args
    &key
    (ik-server-service-name "/solve_ik")
    (group-name ":fullbody-inverse-kinematics") ;;"whole_body")
    (robot *robot*)
    (all-links (remove-if #'(lambda (l) (not (and (find-method l :joint)
						  (send l :joint))))
			  (send robot :links)))
    (target-limbs '(:rarm :larm :rleg :lleg))
    (move-target (mapcar #'(lambda (k) (send robot k :end-coords)) target-limbs))
    (target-coords (send-all move-target :copy-worldcoords))
    (additional-weight-list nil)
    ll
    collision-avoidance-link-pair
    (collision-link-names
     (union nil (flatten collision-avoidance-link-pair)))
    &allow-other-keys)
   (let* ((req (instance moveit_msgs::GetPositionIKRequest :init))
	  pc pc-list oc oc-list tf q res mdj transform-key
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
			       :frame_id "/odom")
		     :name
		     (mapcar #'(lambda (j) (format nil "~A" (send j :name)))
			     all-links)
		     :position
		     (map float-vector #'deg2rad
			  (send-all (send-all all-links :joint) :joint-angle))))
     ;; collision-avoidance-link-pair
     (send req :ik_request :robot_state :attached_collision_objects
	   (mapcar
	    #'(lambda (name)
		(instance moveit_msgs::AttachedCollisionObject
			  :init
			  :link_name (format nil "~A" name)))
	    collision-link-names))
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
       (send pc :header :frame_id
	     (format nil "/~A" (send (send (send mt :parent) :parent) :name)))
       (push pc pc-list)
       ;;
       (setq oc (instance moveit_msgs::OrientationConstraint :init))
       (send oc :link_name
	     (format nil "~A" (send (send mt :parent) :name)))
       (send oc :header :frame_id
	     (format nil "/~A" (send (send (send mt :parent) :parent) :name)))
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
     (cond
      (move-target
       (send req :ik_request :ik_link_name
	     (format nil "~A"
		     (send (send (car move-target) :parent) :name)))
       (send req :ik_request :pose_stamped
	     (send self :coords2posestamped
		   (car target-coords)
		   :origin-link (car (send robot :links))
		   ;;:frame_id "/odom"
		   ;;:origin (make-coords)
		   :robot robot))
       ;;(send req :ik_request :pose_stamped :header :frame_id
       ;;(format nil "/~A" (send (car (send robot :links)) :name))
       ;;"/odom"
       ))
     (send req :ik_request :ik_link_names
	   (mapcar #'(lambda (mt)
		       (format nil "~A" (send (send mt :parent) :name)))
		   (cdr move-target)))
     (send req :ik_request :pose_stamped_vector
	   (mapcar
	    #'(lambda (tc) (send self :coords2posestamped
				 tc
				 :origin-link (car (send robot :links))
				 ;;:frame_id "/odom"
				 ;;:origin (make-coords)
				 :robot robot))
	    (cdr target-coords)))
     ;;(send-all
     ;;(send req :ik_request :pose_stamped_vector)
     ;;:header :frame_id "/odom"
     ;;(format nil "/~A" (send (car (send robot :links)) :name))
     ;;)
     ;;
     (setq res (ros::service-call ik-server-service-name req t))
     (cond
      ((= (send res :error_code :val) moveit_msgs::MoveItErrorCodes::*SUCCESS*)
       (send self :update-joint-states
	     (send res :solution :joint_state)
	     :joint-list (send-all all-links :joint)
	     :name-list (send (send res :solution :joint_state) :name)
	     :position-list (send (send res :solution :joint_state) :position)
	     :robot robot)
       (setq mdj (send res :solution :multi_dof_joint_state))
       (if (and mdj ;; for hydro
		(or (find-method mdj (setq transform-key :joint_transforms))
		    (find-method mdj (setq transform-key :transforms))))
	   (send self :update-joint-states
		 mdj
		 :name-list (send mdj :joint_names)
		 :position-list (send mdj transform-key)
		 :robot robot)
	 (format t "[:update-joint-state] unknown type of multi_dof_joint_state~%"))
       (if (and (boundp '*viewer*) *viewer*) (send *viewer* :draw-objects))
       (ros::ros-info " success ~A"  (send res :solution :joint_state :position)))
      (t (ros::ros-error " failed ")))
     res))
  )

(defun find-link-from-name
  (name &key
	link-list
	(target-name
	 (remove-if #'(lambda (c) (find c (list #\:))) (format nil "~A" name))))
  (find-if
   #'(lambda (j) (string-equal
		  (remove #\: (format nil "~A" (send j :name)))
		  target-name))
   link-list))

(defun ik-server-call
  (&rest args &key target-coords &allow-other-keys)
  (apply
   #'send
   (append
    (list (instance fullbody-ik-client :init) :ik-server-call)
    args)))

#|

(let* ((key '(:rarm :rleg :lleg))
       (mt
	(mapcar
	 #'(lambda (k) (send *robot* k :end-coords))
	 key))
       (tc (send-all mt :copy-worldcoords))
       (ll
	(mapcar
	 #'(lambda (mt)
	     (send *robot* :link-list (send mt :parent)))
	 mt)))
  (send (car tc) :translate #F(150 0 0) :world)
  (ik-server-call
   :target-coords tc
   :move-target mt
   :link-list ll
   :debug-view :no-message
   ))
