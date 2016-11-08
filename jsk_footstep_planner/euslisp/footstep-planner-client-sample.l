#!/usr/bin/env roseus

(ros::load-ros-manifest "jsk_footstep_planner")
(load "package://jsk_footstep_controller/euslisp/util.l")

(defun init ()
  (ros::roseus "footstep_planner_client")
  (ros::advertise "/footstep_from_marker" jsk_footstep_msgs::FootstepArray 1)
  (setq *client* (instance ros::simple-action-client :init
                           "footstep_planner" jsk_footstep_msgs::PlanFootstepsAction))
  (unix:sleep 3)
  (ros::ros-info "waiting actionlib server")
  (send *client* :wait-for-server)
  (ros::ros-info "waited actionlib server")
  )

(defun send-goal (goal-coords &key (publish-result t) (start-coords (make-coords)))
  (let ((goal (make-footstep-planning-msgs
               start-coords goal-coords :frame-id "odom")))
    (ros::ros-info "sending goal")
    (send *client* :send-goal goal)
    (ros::ros-info "waiting for result")
    (send *client* :wait-for-result)
    ;; (ros::ros-info "result: ~A" (send *client* :get-result))
    (when publish-result
      (ros::publish "/footstep_from_marker" (send (send *client* :get-result) :result)))
    (send (send *client* :get-result) :result)
    ))

(warn "
## launch simple node (not using pointcloud)
roslaunch jsk_footstep_planner optimistic_footstep_planner.launch USE_CONTROLLER:=false USE_MARKER:=false USE_PERCEPTION:=false

")

(init)
(setq result (send-goal (make-coords :pos (float-vector 3000 0 0))))
(let ((footstep-coords (footstep-array->coords result)))
  (print-readable-coords footstep-coords)
  ;; (send *ri* :set-foot-steps footstep-coords) ;; send real-robot if needed
  )
(ros::spin-once)

(warn "
(setq result (send-goal (make-coords :pos (float-vector 3000 0 0))))
(let ((footstep-coords (footstep-array->coords result)))
  (print-readable-coords footstep-coords)
  ;; (send *ri* :set-foot-steps footstep-coords) ;; send real-robot if needed
  )
")
