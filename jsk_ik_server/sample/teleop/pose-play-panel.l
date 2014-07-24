#!/usr/bin/env roseus

(ros::load-ros-manifest "sensor_msgs")
(ros::roseus "pose_play_panel")

(ros::advertise
 "/robot_joint_command/joint_state"
 sensor_msgs::JointState)
(ros::advertise
 "/robot_joint_command/string_command"
 std_msgs::string)


(require "util/button-sample.l")

(defvar *data-path*
  (or (unix::getenv "POSE_PLAY_PANEL_DATA_PATH")
      (format nil "~A/~A" "." "img")))

(require (format nil "~A/data.l" *data-path*))

(defun click-callback
  (obj &rest args)
  (let* ((data
	  (find-if
	   #'(lambda (l) (string= (cdr (assoc :name l)) (send obj :name)))
	   *data-list*))
	 (js (cdr (assoc :joint_states data)))
	 (key (cdr (assoc :pose_key data))))
    (cond
     (js
      (ros::publish
       "/robot_joint_command/joint_state"
       (instance sensor_msgs::JointState :init
		 :header
		 (instance
		  std_msgs::header
		  :init
		  :stamp (ros::time 0)
		  :frame_id "/map")
		 :name (cdr (assoc :name js))
		 :position
		 (coerce (cdr (assoc :position js)) float-vector))))
     (key
      (ros::publish
       "/robot_joint_command/string_command"
       (instance std_msgs::string :init
		 :data (format nil "(send *robot* ~A)" key)))))))

(defvar *image-panel*
  (instance button-sample :create
	    :window-name "Pose Player"
	    :name-list
	    (mapcar #'(lambda (l) (cdr (assoc :name l))) *data-list*)
	    :w 102 :h 102
	    :image-list
	    (mapcar
	     #'(lambda (path)
		 (reverse-pixel
		  (resize-image
		   (img::read-image-file
		    (format nil "~A/~A" *data-path* path))
		   100 100)))
	     (mapcar #'(lambda (l) (cdr (assoc :path l))) *data-list*))))
(send *image-panel* :flush)
