(ros::load-ros-manifest "roseus")

(defun set-grasp-state-param
  (d)
  (if (ros::service-exists "/grasp_state_set_param")
      (ros::service-call "/grasp_state_set_param"
                         (instance roseus::StringStringRequest :init :str (format nil "~A" d))))
  )

#|
(defun test ()
  (ros::roseus "a")
  (set-grasp-state-param
   (list
    (list :larm
          (list
           (list :fail1 #f(0.117647 -5.11765 9.88235 -10.0 115.0 114.882))
           (list :fail2 #f(0.0 -5.11765 7.0 -9.52941 90.5882 89.4117))
           (list :success #f(0.0 16.9412 7.0 -9.52941 90.5882 89.4117)))
          (list :f1-2r :f2-2r :t-1p))))
  )
|#

