;; Extend robot interface 

(ros::roseus-add-msgs "jsk_footstep_msgs")
(ros::roseus-add-srvs "jsk_recognition_msgs")

(ros::advertise "/robot_footstep" jsk_footstep_msgs::FootstepArray)

(defmethod robot-model
  (:foot-size ()
    (error "Please override :foot-size this method to use"))
  (:foot-offset-transform ()
    (make-coords :pos (float-vector 20 0 -2)))
  (:footstep-to-rosmsg (foot-coords)
    (let ((frame-id (if (eq (send (car foot-coords) :name) :lleg) "lleg_end_coords" "rleg_end_coords"))
          (now (ros::time-now)))
      (let ((initial-coords (send (car foot-coords) :copy-worldcoords)))
        (let ((footsteps (mapcar #'(lambda (fc)
                                   (let ((footstep (instance jsk_footstep_msgs::Footstep :init)))
                                     (if (eq (send fc :name) :lleg)
                                         (send footstep :leg jsk_footstep_msgs::Footstep::*LEFT*)
                                       (send footstep :leg jsk_footstep_msgs::Footstep::*RIGHT*))
                                     (send footstep :pose (ros::coords->tf-pose 
                                                            (send (send initial-coords
                                                                        :transformation (send fc :copy-worldcoords) )
                                                                  :transform
                                                                  (send self :foot-offset-transform))))
                                     (multiple-value-bind
                                      (foot-x foot-y foot-z) (send self :foot-size)
                                      (send footstep :dimensions :x (* 0.001 foot-x))
                                      (send footstep :dimensions :y (* 0.001 foot-y))
                                      (send footstep :dimensions :z (* 0.001 foot-z))
                                      footstep)))
                               foot-coords)))
        (let ((footstep-array (instance jsk_footstep_msgs::FootstepArray :init)))
          (send footstep-array :header :frame_id frame-id)
          (send footstep-array :header :stamp now)
          (send footstep-array :footsteps footsteps)
          footstep-array)))))
  (:publish-footstep (foot-coords)
    (let ((msg (send self :footstep-to-rosmsg foot-coords)))
      (ros::publish "/robot_footstep" msg)
      msg))
  (:snap-footsteps (foot-coords &key
                                (service "/locomotion/snapit/align_footstep"))
    (let ((req (instance jsk_recognition_msgs::SnapFootstepRequest :init)))
      (send req :input (send self :footstep-to-rosmsg foot-coords))
      (let ((res (ros::service-call service req t)))
        (if res
            (mapcar #'(lambda (footstep)
                        (let ((coords (ros::tf-pose->coords footstep)))
                          (if (= (send footstep :leg) jsk_footstep_msgs::Footstep::*LEFT*)
                              (send coords :name :lleg)
                            (send coords :name :rleg))
                          coords))
                    (send res :output)))
        )))
  )


;; for hrp2jsknt
(when (boundp 'hrp2jsknt-robot)
  (defmethod hrp2jsknt-robot
    (:foot-size ()
      ;; x y z
      (list 240 135 5))
    )
)
;; for hrp2jsknts
(when (boundp 'hrp2jsknts-robot)
  (defmethod hrp2jsknt-robot
    (:foot-size ()
      ;; x y z
      (list 240 135 5))
    )
  )



#|
(load "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknt-interface.l")
(init-robot-from-name "HRP2JSKNT")
(load "package://jsk_footstep_planner/euslisp/robot-model-util.l")
(send *robot* :publish-footstep
      (list (make-coords :name :lleg)
            (make-coords :name :rleg :pos #f(100 -210 0))
            (make-coords :name :lleg :pos #f(200 0 0))
            (make-coords :name :rleg :pos #f(300 -210 0))
            (make-coords :name :lleg :pos #f(300 0 0))))
(send *robot* :publish-footstep
      (list (make-coords :name :lleg)
            (make-coords :name :rleg :pos #f(0 -210 0))))
|#
