#!/usr/bin/env roseus

;; forostep-controller.l
(ros::roseus-add-msgs "jsk_footstep_msgs")

(defun execute-cb (goal)
  )

(defun main()
  (ros::roseus "footstep_controller")

  (let ((s (instance ros::simple-action-server :init
                     "footstep_controller" jsk_footstep_msgs::ExecFootstepsAction
                     :execute-cb 'execute-cb)))
    (ros::rate 10)
    (while (ros::ok)
      (send s :worker)
      (ros::spin-once)
      (ros::sleep))))

(main)
