#!/usr/bin/env roseus

(ros::load-ros-manifest "jsk_footstep_planner")
(load "package://jsk_footstep_controller/euslisp/util.l")

(warn "
## launch simple node (not using pointcloud)
roslaunch jsk_footstep_planner optimistic_footstep_planner.launch USE_CONTROLLER:=false USE_MARKER:=false USE_PERCEPTION:=false ROBOT:=JAXON_RED

")
(initialize-eus-footstep-planning-client)
(setq result (plan-footstep-from-goal-coords 
                (list (make-coords :pos (float-vector (+ 3000 300) 300 0))
                      (make-coords :pos (float-vector (+ 3000   0)   0 0)))))
(let ((footstep-coords (footstep-array->coords result)))
  (print-readable-coords footstep-coords)
  ;; (send *ri* :set-foot-steps footstep-coords) ;; send real-robot if needed
  )
(ros::spin-once)

(warn "
(setq result (plan-footstep-from-goal-coords
               (list (make-coords :pos (float-vector (+ 3000 300) 300 0))
                     (make-coords :pos (float-vector (+ 3000   0)   0 0)))))
(let ((footstep-coords (footstep-array->coords result)))
  (print-readable-coords footstep-coords)
  ;; (send *ri* :set-foot-steps footstep-coords) ;; send real-robot if needed
  )
")
