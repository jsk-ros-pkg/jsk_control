#! /usr/bin/env roseus

 ;; currently only for jaxon_red
(require "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon_red-interface.l")
(require "package://jsk_footstep_planner/euslisp/footstep-corrector.l")

(setq *corrector* nil)
(setq *use-correction* t)

(defun init-corrector-node ()
  (ros::roseus "footstep_modify") ;; registering node as footstep_modify
  (ros::rate (ros::get-param "~rate" 10))
  (setq *corrector* (instance footstep-corrector :init :map-frame (ros::get-param "~map_frame" "map") :odom-frame (ros::get-param "~odom_frame" "odom_init")))
  (setq *use-correction* (ros::get-param "~use_correction" t))
  (ros::subscribe (format nil "~A/footstep" (ros::get-name)) jsk_footstep_msgs::FootstepArray #'fsgetter-cb)
  (ros::advertise-service (format nil "~A/execute" (ros::get-name)) std_srvs::Empty #'execute-footsteps-cb)
  (ros::advertise-service (format nil "~A/execute_once" (ros::get-name)) std_srvs::Empty #'execute-footstep-once-cb)
  (ros::advertise-service (format nil "~A/clear_footsteps" (ros::get-name)) std_srvs::Empty #'clear-footsteps-cb)
  )

(defun fsgetter-cb (msg)
  (send *corrector* :register-footsteps-from-msg msg)
  )

(defun execute-footsteps-cb (req)
  (while (> (length (send *corrector* :get-remaining-footsteps)) 0)
    (send *corrector* :execute-footstep-once :use-correction *use-correction*)
    (send *ri* :wait-foot-steps)
    )
  (send req :response)
  )

(defun execute-footstep-once-cb (req)
  (send *corrector* :execute-footstep-once :use-correction *use-correction*)
  (send *ri* :wait-foot-steps)
  (send req :response)
  )

(defun clear-footsteps-cb (req)
  (send *corrector* :clear-footsteps)
  (send req :response)
  )

(defun corrector-node-main ()
  (init-corrector-node)
  (jaxon_red-init)
  (while (ros::ok)
   (ros::spin-once)
   (ros::sleep)
   )
  )

(corrector-node-main)
