#!/usr/bin/env roseus

(ros::roseus-add-msgs "jsk_footstep_msgs")
(ros::roseus "test_foootstep_refinement")
(ros::advertise "footstep" jsk_footstep_msgs::FootstepArray 1)

(defun random2 (val)
  (- (random (* 2 val)) val))

(defun publish-footstep ()
  (let ((footstep-trans (list
                         (list (float-vector 0 0 0) 0 0)
                         (list (float-vector (random 200.0) 200 0) (random2 5.0) (random2 5.0))
                         (list (float-vector (random 200.0) -200 0) (random2 5.0) (random2 5.0))
                         (list (float-vector (random 200.0) 200 0) (random2 5.0) (random2 5.0))
                         (list (float-vector (random 200.0) -200 0) (random2 5.0) (random2 5.0))
                         (list (float-vector (random 200.0) 200 0) (random2 5.0) (random2 5.0))
                         (list (float-vector (random 200.0) -200 0) (random2 5.0) (random2 5.0))
                         (list (float-vector (random 200.0) 200 0) (random2 5.0) (random2 5.0))
                         )))
    (let ((footstep (instance jsk_footstep_msgs::FootstepArray :init)))
      (send footstep :header :frame_id "map")
      (send footstep :header :stamp (ros::time-now))
      (let ((coords (make-coords))
            (footstep-coords nil))
        (dolist (trans footstep-trans)
          (let ((trans-coords (make-coords :pos (car trans))))
            (send trans-coords :rotate (deg2rad (cadr trans)) :y)
            (send trans-coords :rotate (deg2rad (caddr trans)) :x)
            (send coords :transform trans-coords)
            (print (send coords :rpy-angle))
            )
          (push (send coords :copy-worldcoords) footstep-coords))
        (send footstep :footsteps
              (mapcar #'(lambda (p)
                          (instance jsk_footstep_msgs::Footstep :init
                                    :pose (ros::coords->tf-pose p)))
                      (reverse footstep-coords)))
        (ros::publish "footstep" footstep))
      )))

(ros::rate 1)
(do-until-key
 (publish-footstep)
 (ros::sleep)
 )