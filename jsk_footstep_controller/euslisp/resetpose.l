#!/usr/bin/env roseus

(ros::roseus-add-msgs "pr2_controllers_msgs")

;; just move the robot to reset-pose
(defun main()
  (ros::roseus "resetpose")
  ;; wait controller
  (let ((interface-file
         (ros::get-param "~interface_file"
           "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknt-interface.l"))
        (interface-class
         (ros::get-param "~interface_class" "hrp2jsknt-interface"))
        (robot-class
         (ros::get-param "~robot_class" "hrp2jsknt-robot"))
        (duration
         (ros::get-param "~duration" (* 5 1000)))
        (pose-name
         (ros::get-param "~pose_name" "reset-pose")))
    (load interface-file)
    (let* ((ns "fullbody_controller/joint_trajectory_action")
           (client (instance ros::simple-action-client :init
                             ns pr2_controllers_msgs::JointTrajectoryAction)))
      (ros::ros-info "waiting ~A available" ns)
      (send client :wait-for-server)
      (ros::ros-info "done")
      )
    (setq *ri* (instance (eval (read-from-string interface-class)) :init))
    (send *ri* :start-st)
    (setq *robot* (instance (eval (read-from-string robot-class)) :init))
    ;; convert pose-name to keyword
    (send *robot* (read-from-string (format nil ":~A" pose-name)))
    (send *ri* :angle-vector (send *robot* :angle-vector) duration)
    (send *ri* :wait-interpolation)
    )
  )

(main)
(exit)
