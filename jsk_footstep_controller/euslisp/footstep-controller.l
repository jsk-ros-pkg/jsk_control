#!/usr/bin/env roseus

;; forostep-controller.l
(ros::roseus-add-msgs "jsk_footstep_msgs")

(defun execute-cb (server goal)
  (let ((footstep (send goal :goal :footstep)))
    (let ((frame-id (send footstep :header :frame_id)))
      (ros::ros-info "~A" (mapcar #'(lambda (footstep)
                                      (send (ros::tf-pose->coords
                                              (send footstep :pose)) :worldpos))
                                  (send footstep :footsteps)))
      (let ((footstep-coords
             (mapcar #'(lambda (footstep)      ;no tf transformation
                         (let ((coords (ros::tf-pose->coords
                                         (send footstep :pose))))
                           ;; force to set z=0
                           (let ((pos (send coords :worldpos)))
                             (setf (elt pos 2) 0.0)
                             (send coords :locate pos :world))
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
    (setq *ri* (instance (eval (read-from-string interface-class)) :init))
    (send *ri* :start-st)
    )
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
