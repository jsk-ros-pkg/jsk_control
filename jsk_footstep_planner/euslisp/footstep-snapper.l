#!/usr/bin/env roseus

;; Compute good footstep for specified PoseStamped
(ros::roseus "footstep_goal_snapper")
(load "package://jsk_footstep_planner/euslisp/simple_occupancy_grid.l")
(load "models/arrow-object.l")
(setq *tf* (instance ros::transform-listener :init))
(setq *global-frame* (ros::get-param "~global_frame" "map"))
(setq *use-gui* (ros::get-param "~use_gui" t))
(when *use-gui*
  (make-irtviewer))
(setq *grids* nil)
(setq *input-arrow* nil)
(setq *projected-arrow* nil)
(defun update-viewer ()
  (when *use-gui*
    (send *irtviewer* :objects (remove-if #'null (append *grids* (list *input-arrow* *projected-arrow*))))
    (send *irtviewer* :draw-objects))
  )

(defun grid-callback (msg)
  (setq *grids* (remove-if #'null (mapcar #'(lambda (g)
                                              (let ((trans (lookup-transform-with-duration
                                                            *tf*
                                                            *global-frame*
                                                            (send g :header :frame_id)
                                                            (send g :header :stamp)
                                                            1.0)))
                                                (if trans
                                                    (instance occupancy-grid :init g trans))))
                                          (send msg :grids))))
  (when *use-gui*
    (dotimes (i (length *grids*))
      (setf (get (elt *grids* i) :face-color) (gl::color-category20 i)))
    (update-viewer)
    ))

(defun goal-callback (msg)
  (unless *grids*
    (ros::ros-warn "Not yet grids are available")
    (return-from goal-callback nil))
  (let ((trans (lookup-transform-with-duration
                *tf* *global-frame*
                (send msg :header :frame_id)
                (send msg :header :stamp)
                1.0)))
    (unless trans
      (ros::ros-warn "Failed to lookup goal transformation")
      (return-from goal-callback nil))
    (let ((goal-coords (send trans :transform (ros::tf-pose-stamped->coords msg))))
      (send goal-coords :draw-on :flush t :color (float-vector 1 0 0))
      (setq *input-arrow* (arrow))
      (send *input-arrow* :move-to goal-coords :world)
      (let ((corresponds-grid-map (lookup-highest-corresponding-grid-map goal-coords *grids*)))
        (if corresponds-grid-map
            (progn
              (setq *projected-arrow* (arrow))
              (let ((c (send corresponds-grid-map :project-coords goal-coords)))
                (send *projected-arrow* :move-to c :world)
                (ros::publish "/move_base_footstep_simple/goal" (ros::coords->tf-pose-stamped
                                                                  c *global-frame*))))
          (ros::ros-warn "No suitable grid map found"))))
    (update-viewer)
    ))

(ros::advertise "/move_base_footstep_simple/goal" geometry_msgs::PoseStamped)
(ros::subscribe "/locomotion/env_server/output" jsk_recognition_msgs::SimpleOccupancyGridArray #'grid-callback)
(ros::subscribe "/move_base_simple/goal" geometry_msgs::PoseStamped #'goal-callback)
(ros::rate 10)
(while (ros::ok)
  (ros::spin-once)
  (when *use-gui*
    (x::window-main-one))
  (ros::sleep))

