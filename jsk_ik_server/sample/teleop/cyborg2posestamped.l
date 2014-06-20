#!/usr/bin/env roseus

(ros::load-ros-manifest "roseus")
(ros::load-ros-manifest "geometry_msgs")
(ros::load-ros-manifest "sensor_msgs")

(ros::roseus "cyborg_joy")

(defun joy2posestamped
  (msg)
  (let* ((pose (instance geometry_msgs::posestamped :init))
	 (coords (make-coords
		  :rpy
		  (list (* +3. (aref (send msg :axes) 3))
			(* -3. (aref (send msg :axes) 1))
			(* +3. (aref (send msg :axes) 0)))))
	 (q (matrix2quaternion (send coords :worldrot))))
    (send (send pose :header) :stamp (ros::time 0))
    (send (send pose :header) :frame_id "/base_link")
    (send (send (send pose :pose) :position) :x 0)
    (send (send (send pose :pose) :position) :y 0)
    (send (send (send pose :pose) :position) :z 0)
    (send (send (send pose :pose) :orientation) :x (aref q 1))
    (send (send (send pose :pose) :orientation) :y (aref q 2))
    (send (send (send pose :pose) :orientation) :z (aref q 3))
    (send (send (send pose :pose) :orientation) :w (aref q 0))
    (ros::publish "/robot_joint_command/relative_head_posestamped" pose))
  )

(ros::advertise "/robot_joint_command/relative_head_posestamped"
                geometry_msgs::posestamped)
(ros::subscribe "/cyborg/joy"
		sensor_msgs::Joy
                #'joy2posestamped
		1)

(ros::rate 10)
(do-until-key
 (if (not (ros::ok)) (return-from nil nil))
 (ros::spin-once)
 (ros::sleep))
