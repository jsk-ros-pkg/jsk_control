#!/usr/bin/env roseus

;; forostep-controller.l
(ros::roseus-add-msgs "jsk_footstep_msgs")

(defun execute-cb (server goal)
  (let ((footstep (send goal :goal :footstep)))
    (let ((frame-id (send footstep :header :frame_id)))
      (let ((footstep-coords
             (mapcar #'(lambda (footstep)      ;no tf transformation
                         (let ((coords (ros::tf-pose->coords
                                         (send footstep :pose))))
                           (send coords :put :l/r
                                 (if (= (send footstep :leg)
                                        jsk_footstep_msgs::Footstep::*LEFT*)
                                     :lleg
                                   :rleg))
                           coords))
                     (send footstep :footsteps))))
        (send *ri* :set-foot-steps footstep-coords))))
  (let ((result (send server :result)))
    (send server :set-succeeded result)))

(defun main()
  (ros::roseus "footstep_controller")
  ;; *ri*
  (let ((interface-file
         (ros::get-param "~interface_file"
           "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknt-interface.l"))
        (interface-class
         (ros::get-param "~interface_class" "hrp2jsknt-interface"))
        )
    (load interface-file)
    (setq *ri* (instance (eval (read-from-string interface-class)) :init)))
  ;; interface
  (setq *tf* (instance ros::transform-listener :init))
  (let ((s (instance ros::simple-action-server :init
                     "footstep_controller"
                     jsk_footstep_msgs::ExecFootstepsAction
                     :execute-cb 'execute-cb)))
    (ros::rate 10)
    (while (ros::ok)
      (send s :worker)
      (ros::spin-once)
      (ros::sleep))))

(main)
