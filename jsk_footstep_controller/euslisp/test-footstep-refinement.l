#!/usr/bin/env roseus

(ros::roseus-add-msgs "jsk_footstep_msgs")
(ros::roseus "test_foootstep_refinement")
(ros::advertise "footstep" jsk_footstep_msgs::FootstepArray 1)

(defun publish-footstep ()
  (let ((footstep-pos (list
                       (list (float-vector 0 0 0) 0)
                       (list (float-vector 100 200 0) 1)
                       (list (float-vector 200 0 0) 2)
                       (list (float-vector 300 200 0) 3)
                       (list (float-vector 400 0 0) 4)
                       (list (float-vector 500 200 0) 5)
                       (list (float-vector 600 0 0) 4)
                       (list (float-vector 700 200 0) 3)
                       (list (float-vector 800 0 0) 2)
                       (list (float-vector 900 200 0) 1)
                       )))
    (let ((footstep (instance jsk_footstep_msgs::FootstepArray :init)))
      (send footstep :header :frame_id "map")
      (send footstep :header :stamp (ros::time-now))
      (send footstep :footsteps
            (mapcar #'(lambda (p)
                        (let ((coords (make-coords :pos (car p))))
                          (send coords :rotate (deg2rad (cadr p)) :x)
                          (instance jsk_footstep_msgs::Footstep :init
                                    :pose (ros::coords->tf-pose coords))))
                    footstep-pos))
      (ros::publish "footstep" footstep)
      )))

(ros::rate 1)
(do-until-key
 (publish-footstep)
 (ros::sleep)
 )