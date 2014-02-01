#!/usr/bin/env roseus

(ros::load-ros-manifest "jsk_footstep_planner")

(defun main ()
  (ros::roseus "footstep_planner_client")
  (ros::advertise "/footstep_result" jsk_footstep_msgs::FootstepArray 1)
  (let ((client (instance ros::simple-action-client :init
                          "footstep_planner" jsk_footstep_msgs::PlanFootstepsAction)))
    (unix:sleep 3)
    (ros::ros-info "waiting actionlib server")
    (send client :wait-for-server)
    (ros::ros-info "waited actionlib server")
    (let ((goal (instance jsk_footstep_msgs::PlanFootstepsActionGoal :init)))
      (let ((initial-footstep (instance jsk_footstep_msgs::FootstepArray :init)))
        (send initial-footstep :header :stamp (ros::time 0))
        (send initial-footstep :header :frame_id "/odom")
        (send initial-footstep :footsteps
              (mapcar #'(lambda (param)
                          (let ((f (instance jsk_footstep_msgs::Footstep :init)))
                            (send f :pose
                                  (ros::coords->tf-pose (make-coords :pos (float-vector 0 (cadr param) 0))))
                            (send f :leg (car param))
                            f))
                      (list (list jsk_footstep_msgs::Footstep::*LEFT*
                                  -150)
                            (list jsk_footstep_msgs::Footstep::*RIGHT*
                                  150))))
        (send goal :goal :initial_footstep initial-footstep)
        )
      (let ((goal-footstep (instance jsk_footstep_msgs::FootstepArray :init)))
        (send goal-footstep :header :stamp (ros::time 0))
        (send goal-footstep :header :frame_id "/odom")
        (send goal-footstep :footsteps
              (mapcar #'(lambda (param)
                          (let ((f (instance jsk_footstep_msgs::Footstep :init)))
                            (send f :pose
                                  (ros::coords->tf-pose (make-coords :pos (float-vector 2000 (cadr param) 0)))) ;1m ahead
                            (send f :leg (car param))
                            f))
                      (list (list jsk_footstep_msgs::Footstep::*LEFT*
                                  -150)
                            (list jsk_footstep_msgs::Footstep::*RIGHT*
                                  150))))
        (send goal :goal :goal_footstep goal-footstep)
        )
      (ros::ros-info "sending goal")
      (send client :send-goal goal)
      (ros::ros-info "waiting for result")
      (send client :wait-for-result)
      (ros::ros-info "result: ~A" (send client :get-result))
      (ros::publish "/footstep_result" (send (send client :get-result) :result))
      (ros::exit)
      )
    (ros::spin-once)
    )
  )

(main)
