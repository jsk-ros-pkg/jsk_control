#!/usr/bin/env roseus

(ros::roseus-add-msgs "sensor_msgs")
(ros::roseus "spacenav_controller")

(defvar *target-limb* :rarm)

(defun publish-posestamped
  (coords
   &key
   (pos (send coords :worldpos))
   (rot (send coords :worldrot))
   (q (matrix2quaternion rot))
   )
  (format t "[publish-posestamped] pos=~A/q=~A~%" pos q)
  (ros::publish
   (if (eq *target-limb* :rarm)
       "/robot_joint_command/relative2_rarm_posestamped"
     "/robot_joint_command/relative2_larm_posestamped")
   (instance geometry_msgs::posestamped
	     :init
	     :header
	     (instance std_msgs::header
		       :init
		       :stamp (ros::time 0)
		       :frame_id (format nil "~A" *target-limb*))
	     :pose
	     (instance geometry_msgs::pose
		       :init
		       :position
		       (instance geometry_msgs::Point :init
				 :x (* 1e-3 (aref pos 0))
				 :y (* 1e-3 (aref pos 1))
				 :z (* 1e-3 (aref pos 2)))
		       :orientation
		       (instance geometry_msgs::Quaternion :init
				 :w (aref q 0)
				 :x (aref q 1)
				 :y (aref q 2)
				 :z (aref q 3))))))

(defvar *spacenav-cnt* 0)
(defun spacenav-controller-callback
  (msg)
  (cond
   ((and (> (aref (send msg :buttons) 0) 0.1)
	 (> (aref (send msg :buttons) 1) 0.1))
    (cond
     ((eq *target-limb* :rarm)
      (setq *target-limb* :larm))
     (t (setq *target-limb* :rarm)))
    (format t "[spacenav2posestamped] ~A mode~%" *target-limb*)
    (publish-posestamped (make-coords))
    (unix::usleep (round (* 1000 1000 0.5)))
    )
   ((> (aref (send msg :buttons) 0) 0.1)
    (setq *updated* t)
    (format t "[spacenav2posestamped] grasp~%")
    (ros::publish
     "/robot_joint_command/string_command"
     (instance std_msgs::string
	       :init
	       :data
	       (format nil "(send *robot* ~A :hand :index-avoid-grasp-pose)" *target-limb*))))
   ((> (aref (send msg :buttons) 1) 0.1)
    (setq *updated* t)
    (format t "[spacenav2posestamped] hook~%")
    (ros::publish
     "/robot_joint_command/string_command"
     (instance std_msgs::string
	       :init
	       :data
	       (format nil "(send *robot* ~A :hand :index-avoid-hook-pose)" *target-limb*)))
    )
   ((> (norm (send msg :axes)) 0.1)
    (format t "[spacenav2posestamped] ~A move ~A x~A~%"
	    *target-limb*
	    (send msg :axes)
	    *spacenav-cnt*)
    (let* ((move
	    (scale
	     30
	     (coerce
	      (subseq (send msg :axes) 0 3)
	      float-vector)))
	   (rpy
	    (reverse
	     (scale
	      0.25
	      (coerce
	       (subseq (send msg :axes) 3 6)
	       float-vector))))
	   (pos
	    (if (> (norm move) 30)
		(setq move (scale 30 (normalize-vector move)))
	      move))
	   (rot
	    (send
	     (make-coords
	      :rpy
	      (if (> (norm rpy) (min 0.08
				     (* *spacenav-cnt* 0.01)))
		  (setq rpy (scale (min 0.08
					(* *spacenav-cnt* 0.01))
				   (normalize-vector rpy)))
		rpy))
	     :worldrot)))
      (publish-posestamped
       (make-coords :pos pos :rot rot))
      (incf *spacenav-cnt*)
      (setq *updated* t)))
   (t (setq *spacenav-cnt* 0)
      (format t "[spacenav-controller] spacenav skip~%"))))

(ros::subscribe
 "/spacenav/joy"
 sensor_msgs::Joy
 #'spacenav-controller-callback)

(ros::advertise
 "/robot_joint_command/relative2_rarm_posestamped"
 geometry_msgs::posestamped)
(ros::advertise
 "/robot_joint_command/relative2_larm_posestamped"
 geometry_msgs::posestamped)
(ros::advertise
 "/robot_joint_command/relative2_lleg_posestamped"
 geometry_msgs::posestamped)
(ros::advertise
 "/robot_joint_command/relative2_rleg_posestamped"
 geometry_msgs::posestamped)

(ros::advertise
 "/robot_joint_command/string_command"
 std_msgs::string)

(ros::rate 10)
(do-until-key
 (ros::spin-once)
 (ros::sleep))

