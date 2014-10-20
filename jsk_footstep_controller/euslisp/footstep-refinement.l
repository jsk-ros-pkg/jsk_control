#!/usr/bin/env roseus

(ros::roseus-add-msgs "jsk_footstep_msgs")

(defmethod coordinates
  (:locate-z (zpos)
    (send self :locate
          (float-vector (elt (send self :worldpos) 0)
                        (elt (send self :worldpos) 1)
                        zpos)
          :world)))

(defun make-footstep (org-footstep coords)
  (let ((new-footstep (instance jsk_footstep_msgs::Footstep :init)))
    (send new-footstep :dimensions (send org-footstep :dimensions))
    (send new-footstep :leg (send org-footstep :leg))
    (send new-footstep :duration (send org-footstep :duration))
    (send new-footstep :pose (ros::coords->tf-pose coords))
    new-footstep))

(defun refinement (org-msg)
  (let ((new-msg (instance jsk_footstep_msgs::FootstepArray :init))
        (refined-footsteps nil)
        (prev-footstep nil)
        (prev-refined-footstep nil)
        (group 0))
    (send new-msg :header (send org-msg :header))
    (dolist (footstep (send org-msg :footsteps))
      (if prev-footstep
          (progn
            (let* ((prev-coords
                    (ros::tf-pose->coords (send prev-footstep :pose)))
                   (prev-refined-coords
                    (ros::tf-pose->coords (send prev-refined-footstep :pose)))
                   (raw-next-coords
                    (ros::tf-pose->coords (send footstep :pose)))
                   (next-coords nil)
                   (trans
                    (send prev-coords :transformation
                          (ros::tf-pose->coords (send footstep :pose)))))
              (let ((parallelp (< (acos (v. (send prev-coords :rotate-vector (float-vector 0 0 1))
                                            (send raw-next-coords :rotate-vector (float-vector 0 0 1))))
                                  *angle-threshold*))
                    (sameheightp (< (abs (- (elt (send prev-coords :worldpos) 2)
                                            (elt (send raw-next-coords :worldpos) 2)))
                                    *z-threshold*)))
                (cond
                 ((and parallelp sameheightp)
                  (send trans :rpy (car (car (send trans :rpy-angle))) 0 0)
                  (send trans :locate-z 0))
                 (parallelp
                  (send trans :rpy (car (car (send trans :rpy-angle))) 0 0))
                 (sameheightp
                  (send trans :locate-z 0)))
                (setq next-coords (send (send prev-refined-coords :copy-worldcoords) :transform trans))
                (if (not (and parallelp sameheightp))
                    (incf group))
                (let ((new-footstep (make-footstep footstep next-coords)))
                  (send new-footstep :footstep_group group)
                  (push new-footstep refined-footsteps)
                  (setq prev-refined-footstep new-footstep)))))
        (progn
          (setq prev-refined-footstep footstep)
          (push footstep refined-footsteps)))
      (setq prev-footstep footstep))
  (send new-msg :footsteps (reverse refined-footsteps))
  (ros::publish "refined_footstep" new-msg)))

(defun main()
  (ros::roseus "footstep_refinement")
  ;; read parameters
  (setq *angle-threshold* (ros::get-param "~angle_threshold" (deg2rad 5)))
  (setq *z-threshold* (ros::get-param "~z_threshold" 30))
  (ros::advertise "refined_footstep" jsk_footstep_msgs::FootstepArray 1)
  (ros::subscribe "footstep" jsk_footstep_msgs::FootstepArray #'refinement)
  (ros::spin)
  )

(main)
