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

(defun dot2rad (dot)
  (cond
   ((> dot 1.0)
    (acos 1.0))
   ((< dot -1.0)
    (acos -1.0))
   (t
    (acos dot))))

(defun pitch-angle-diff (a b)
  (abs (cadr (car (send (send a :transformation b) :rpy-angle)))))

(defun roll-angle-diff (a b)
  (abs (caddr (car (send (send a :transformation b) :rpy-angle)))))

(defun z-vec-angle-diff (a b)
  (dot2rad (v. (send a :rotate-vector (float-vector 0  0 1))
               (send b :rotate-vector (float-vector 0  0 1)))))

(defun refine-footstep (org-msg)
  (let ((new-msg (instance jsk_footstep_msgs::FootstepArray :init))
        (refined-footsteps nil)
        (prev-footstep nil)
        (prev-refined-footstep nil)
        (group 1)
        (i 0))
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
                    (send prev-coords :transformation raw-next-coords)))
              (let ((pitch-diff (pitch-angle-diff prev-coords raw-next-coords))
                    (roll-diff (roll-angle-diff prev-coords raw-next-coords))
                    (z-diff (abs (elt (send trans :worldpos) 2))))
                (let ((parallelp (and (< pitch-diff *pitch-threshold*)
                                      (< roll-diff *roll-threshold*)))
                      (sameheightp (< z-diff *z-threshold*)))
                  (ros::ros-info
                    "(~A)parallelp: ~A, pitch-diff: ~A, roll-diff: ~A "
                    i parallelp (rad2deg pitch-diff) (rad2deg roll-diff))
                  (ros::ros-info "(~A)sameheightp: ~A, z-diff: ~A"
                    i sameheightp z-diff)
                  (cond
                   ((and parallelp sameheightp)
                    (send trans :rpy (car (car (send trans :rpy-angle))) 0 0)
                    (send trans :locate-z 0))
                   (parallelp
                    (send trans :rpy (car (car (send trans :rpy-angle))) 0 0))
                   (sameheightp
                    (send trans :locate-z 0)))
                  (setq next-coords
                        (send (send prev-refined-coords :copy-worldcoords)
                              :transform trans))
                (if (not (and parallelp sameheightp))
                    (incf group))
                (let ((new-footstep (make-footstep footstep next-coords)))
                  (send new-footstep :footstep_group group)
                  (push new-footstep refined-footsteps)
                  (setq prev-refined-footstep new-footstep))))))
        (progn
          (setq prev-refined-footstep footstep)
          (send footstep :footstep_group group)
          (push footstep refined-footsteps)))
      (setq prev-footstep footstep)
      (incf i))
    (send new-msg :footsteps (reverse refined-footsteps))
    new-msg))

(defun refinement (org-msg)
  (let ((new-msg (refine-footstep org-msg)))
    (ros::publish "refined_footstep" new-msg)))

(defun refine-actionlib-result (msg)
  (let ((planning-result (send msg :result :result)))
    (send msg :result :result (refine-footstep planning-result))
    (ros::publish "refined_footstep_result" msg)))

(defun main()
  (ros::roseus "footstep_refinement")
  ;; read parameters
  (setq *pitch-threshold* (ros::get-param "~pitch_threshold" (deg2rad 5)))
  (setq *roll-threshold* (ros::get-param "~roll_threshold" (deg2rad 5)))
  (setq *z-threshold* (ros::get-param "~z_threshold" 30))
  (ros::advertise "refined_footstep" jsk_footstep_msgs::FootstepArray 1)
  (ros::advertise "refined_footstep_result" jsk_footstep_msgs::PlanFootstepsActionResult 1)
  (ros::subscribe "footstep" jsk_footstep_msgs::FootstepArray #'refinement)
  (ros::subscribe "planner_result" jsk_footstep_msgs::PlanFootstepsActionResult #'refine-actionlib-result)
  (ros::spin)
  )

(main)
