#!/usr/bin/env roseus

(load "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknts-interface.l")
(ros::roseus "base_controller")

(hrp2jsknts-init)
(setq *robot* *hrp2jsknts*)


(setq *head-av-list* (list (float-vector 0 0)
                           (float-vector 40 20)
                           (float-vector 0 40)
                           (float-vector -40 20)))
(setq *av-id* 0)


(ros::load-ros-manifest "geometry_msgs")
(setq *vel* #F(0 0 0))
(ros::subscribe "/cmd_vel"
               geometry_msgs::Twist
               #'(lambda (m)
                   (setelt *vel* 0 (send m :linear :x))
                   (setelt *vel* 1 (send m :linear :y))
                   (setelt *vel* 2 (send m :angular :z)))
               1)


(ros::rate 5)
(while (ros::ok)
  (ros::spin-once)
  (send *ri* :spin-once)
  (if (= (norm *vel*) 0)
      (send *ri* :go-stop)
      (send *ri* :go-velocity (elt *vel* 0) (elt *vel* 1) (rad2deg (elt *vel* 2))))
  (unless (send *ri* :interpolatingp :head-controller)
    (incf *av-id*)
    (when (>= *av-id* (length *head-av-list*))
      (setq *av-id* 0))
    (send *robot* :head :angle-vector (elt *head-av-list* *av-id*))
    (send *ri* :angle-vector (send *robot* :angle-vector) 5000 :head-controller)
    )
  (ros::sleep)
  )
