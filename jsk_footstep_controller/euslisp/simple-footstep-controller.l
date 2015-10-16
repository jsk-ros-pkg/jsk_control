#!/usr/bin/env roseus

;; The most simplest version to execute footsteps by :set-foot-steps
;; method and have actionlib server interface
(ros::roseus "simple_footstep_controller")
(ros::roseus-add-msgs "jsk_footstep_msgs")
(load "package://jsk_footstep_controller/euslisp/util.l")
;; (load "package://jsk_robot_utils/euslisp/robot-interface-utils.l")
(setq *robot-name* (ros::get-param "/robot/type" (unix::getenv "ROBOT")))
(load (robot-interface-file *robot-name*))
(init-robot-from-name *robot-name*)

(defun footstep->coords (f)
  (let ((c (ros::tf-pose->coords (send f :pose))))
    (cond ((= (send f :leg) jsk_footstep_msgs::Footstep::*LEFT*)
           (send c :name :lleg))
          ((= (send f :leg) jsk_footstep_msgs::Footstep::*RIGHT*)
           (send c :name :rleg)))
    c))

(defun footstep-array->coords (arr)
  (mapcar #'(lambda (f)
              (footstep->coords f))
          (send arr :footsteps)))
(defun print-readable-coords (coords)
  (dolist (c coords)
    (format t "(make-coords :pos ~A :rpy ~A)~%"
            (send c :worldpos)
            (car (send c :rpy-angle)))))

(defun execute-cb (server goal)
  (ros::ros-info "execute-cb")
  (let ((footstep (send goal :goal :footstep))) 
    ;; footstep := jsk_recognition_msgs/FootstepArray
    (let ((footstep-coords (footstep-array->coords footstep)))
      (print-readable-coords footstep-coords)
      (send *ri* :set-foot-steps footstep-coords)
      (send server :set-succeeded (send server :result))
      )))

(setq *server* (instance ros::simple-action-server :init
                         (ros::get-name)
                         jsk_footstep_msgs::ExecFootstepsAction
                         :execute-cb 'execute-cb))

(ros::rate 10)
(ros::ros-info "simple footstep controller is ready as ~A" (ros::get-name))
(while (ros::ok)
  (send *server* :worker)
  (ros::spin-once)
  (ros::sleep))

