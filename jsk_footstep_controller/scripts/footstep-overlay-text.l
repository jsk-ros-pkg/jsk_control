#!/usr/bin/env roseus

(ros::roseus-add-msgs "jsk_footstep_msgs")
(ros::roseus-add-msgs "jsk_rviz_plugins")

(defun footstep-transform-text (prev-footstep next-footstep)
  (if (null prev-footstep)
      (list (format nil "    pos: (~A, ~A, ~A)" 0 0 0)
            (format nil "    rpy: (~A, ~A, ~A)" 0 0 0))
    (let ((prev-coords (ros::tf-pose->coords (send prev-footstep :pose)))
          (coords (ros::tf-pose->coords (send next-footstep :pose))))
      (let ((trans (send prev-coords :transformation coords)))
        (list (format nil "    pos: (~1,2f, ~1,2f, ~1,2f)" 
                      (* 0.001 (elt (send trans :worldpos) 0))
                      (* 0.001 (elt (send trans :worldpos) 1))
                      (* 0.001 (elt (send trans :worldpos) 2)))
              (format nil "    rpy: (~1,2f, ~1,2f, ~1,2f)" 
                      ;; ypr
                      (rad2deg (caddr (car (send trans :rpy-angle))))
                      (rad2deg (cadr (car (send trans :rpy-angle))))
                      (rad2deg (car (car (send trans :rpy-angle))))))))))
(setq *counter* 0)
(defun publish-text (footstep)
  (let ((text (instance jsk_rviz_plugins::OverlayText :init)))
    (send text :width 500)
    (send text :height 1500)
    (send text :text_size 12)
    (send text :bg_color :a 0.0)
    (send text :fg_color :a 1.0)
    (send text :left 10)
    (send text :top 10)
    (let ((texts (list (format nil "Footsteps: ~A (~A)"
                               (length (send footstep :footsteps))
                               *counter*)))
          (prev-footstep nil))
      (dotimes (i (length (send footstep :footsteps)))
        (let ((f (elt (send footstep :footsteps) i)))
          (setq texts (append texts
                              (list (format nil "  ~A(~A-~A):" 
                                            i
                                            (if (= (send f :leg) jsk_footstep_msgs::Footstep::*RIGHT*)
                                                "R" "L")
                                            (send f :footstep_group)))))
          (setq texts (append texts (footstep-transform-text prev-footstep f)))
          (setq prev-footstep f)))
      (send text :text (reduce #'(lambda (x y) (format nil "~A~%~A" x y)) texts)))
    (if (= (length (send footstep :footsteps)) 0)
        (progn
          (send text :fg_color :r 1)
          (send text :fg_color :g 0)
          (send text :fg_color :b 0))
      (progn
          (send text :fg_color :r (/ 25.0 255.0))
          (send text :fg_color :g 1)
          (send text :fg_color :b 1)))
    (incf *counter*)
    (ros::publish "footstep_text" text)))

(defun footstep-callback (msg)
  (publish-text msg))
(defun footstep-result-callback (msg)
  (publish-text (send msg :result :result)))

(ros::roseus "footstep_overlay_text")
(ros::advertise "footstep_text" 
                jsk_rviz_plugins::OverlayText 1)
(ros::subscribe "footstep" jsk_footstep_msgs::FootstepArray
                #'footstep-callback)
(ros::subscribe "footstep_planner/result"
                jsk_footstep_msgs::PlanFootstepsActionResult
                #'footstep-result-callback)
(ros::spin)
