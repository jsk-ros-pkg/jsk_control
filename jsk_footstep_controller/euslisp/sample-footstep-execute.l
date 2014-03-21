#!/usr/bin/env roseus

(ros::roseus-add-msgs "std_msgs")
(ros::roseus-add-msgs "geometry_msgs")
(ros::roseus-add-msgs "jsk_footstep_msgs")

(ros::roseus "sample_footstep")

(ros::advertise "/footstep_marker/execute" std_msgs::Empty)
(ros::advertise "/footstep_marker/pose" geometry_msgs::PoseStamped)

(setq *tf* (instance ros::transform-listener :init))

(defun reset-goal-pose ()
  )

(let ((prev-coords (make-coords))
      (xrange '(-2000 2000))
      (yrange '(-2000 2000)))
  (ros::sleep 10)
  ;;(while (ros::ok)
    (ros::spin-once)
    (let ((next-coords (make-coords)))
      (send next-coords :locate (float-vector
                                 (+ (car xrange)
                                    (random (- (cadr xrange) (car xrange))))
                                 (+ (car yrange)
                                    (random (- (cadr yrange) (car yrange))))
                                 0))
      (send next-coords :rotate (deg2rad (random 360.0)) :z)
      
      (let ((pose (ros::coords->tf-pose-stamped
                    (send prev-coords :transformation next-coords) "/odom")))
        (ros::publish "/footstep_marker/pose" pose)
        )
      (setq prev-coords next-coords))
    (ros::sleep 5)
    (ros::publish "/footstep_marker/execute" (instance std_msgs::empty :init))
    (ros::sleep 30)
  ;;  )
  )
