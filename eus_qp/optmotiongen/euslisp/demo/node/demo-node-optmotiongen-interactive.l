#! /usr/bin/env roseus

(load "package://eus_qp/optmotiongen/euslisp/inverse-kinematics-wrapper.l")
(load "package://eus_qp/optmotiongen/euslisp/instant-configuration-task.l")
(load "package://eus_qp/optmotiongen/euslisp/sqp-optimization.l")
(load "package://jsk_interactive_marker/euslisp/display-robot-state.l")
(load "package://jsk_interactive_marker/euslisp/transformable-object-util.l")
(require :HRP1 "package://hrpsys_ros_bridge_tutorials/models/pa10.l")
(require "models/arrow-object.l")


(ros::roseus "optmotiongen_interactive")
(ros::roseus-add-msgs "jsk_rviz_plugins")


(defun get-coords-from-interactive-marker
    (&optional (name "object"))
  (ros::tf-pose->coords
   (send
    (send
     (ros::service-call
      "/transformable_interactive_server/get_pose"
      (instance jsk_interactive_marker::GetTransformableMarkerPoseRequest :init
                :target_name name))
     :pose_stamped)
    :pose))
  )

(defun setup-optmotiongen-for-demo-node-optmotiongen-interactive
    ()
  ;; setup robot
  (setq *robot* (instance hrp1-robot :init))
  (send (send *robot* :rarm :end-coords) :translate (float-vector 0 0 160) :local)
  (send *robot* :init-pose)
  ;; setup object
  (setq *obj* (make-cube 500 500 10))
  (send *obj* :set-color #f(0.6 0.4 0.3))
  (setq *obst* (make-cube 300 300 300))
  (send *obst* :set-color #f(0.3 0.4 0.6))
  ;; generate robot-environment
  (setq *contact-list*
        (list (instance 2d-planar-contact :init :name "object")
              ))
  (setq *robot-env*
        (instance robot-environment :init
                  :robot *robot*
                  :contact-list *contact-list*
                  :root-virtual-mode :fix
                  ))
  (send *obj* :worldcoords)
  (send *robot-env* :contact "object" :set-from-face :face (elt (send *obj* :faces) 1) :margin 0.0)
  (send (send *robot-env* :contact "object" :contact-coords) :rotate pi :x) ;; flip normal
  (send *obj* :assoc (send *robot-env* :contact "object"))
  ;; draw
  (setq *visualize?* (ros::get-param "~visualize" t))
  (when *visualize?*
    (unless (boundp '*irtviewer*)
      (make-irtviewer)
      (send *irtviewer* :change-background (float-vector 1 1 1))
      (send *irtviewer* :draw-floor 500)
      (send *irtviewer* :floor-color (float-vector 0 0 0))
      )
    (objects (list *robot* *obj* *obst*))
    )

  (setq *target-coords*
        (send *robot-env* :contact "object" :contact-coords))
  (setq *attention-coords*
        (send *robot* :rarm-end-coords))
  (setq *instant-config-task*
        (instance instant-configuration-task :init
                  :only-kinematics? t
                  :drive-joint-list nil
                  :robot-env *robot-env*
                  :kin-target-coords-list (list *target-coords*)
                  :kin-attention-coords-list (list *attention-coords*)
                  :norm-regular-scale-max 1e10
                  :norm-regular-scale-offset 1e-3
                  :delta-rotational-joint 2.0
                  :collision-pair-list
                  (mapcar #'(lambda (l) (list l *obst*)) (cdr (send *robot* :links)))
                  ))
  (setq *sqp-opt*
        (instance sqp-optimization :init
                  :config-task *instant-config-task*
                  ))
  )

(defun setup-ros-for-demo-node-optmotiongen-interactive
    ()
  (ros::advertise "/robot_state" moveit_msgs::DisplayRobotState)
  (ros::advertise "/transformable_interactive_server/set_color" std_msgs::ColorRGBA)
  (ros::advertise "/target_coords" geometry_msgs::PoseStamped)
  (ros::advertise "/attention_coords" geometry_msgs::PoseStamped)
  (ros::advertise "/collision_marker" visualization_msgs::Marker)
  (ros::advertise "/ik_error_text" jsk_rviz_plugins::OverlayText)

  (erase-all-transformable-object)
  (insert-transformable-object
   :shape-type jsk_rviz_plugins::TransformableMarkerOperate::*BOX*
   :frame-id "BASE_LINK"
   :name "object"
   :description "object"
   )
  (set-transformable-object-dimensions
   :x (x-of-cube *obj*) :y (y-of-cube *obj*) :z (z-of-cube *obj*) :name "object")
  (dotimes (i 4) ;; topic command sometimes fails
    (set-transformable-object-color
     :r 1 :g 0 :b 0 :a 0.7)
    (unix::usleep (* 100 1000))
    )
  (set-transformable-object-pose
   (make-coords :pos (float-vector 1000 0 0)) "BASE_LINK" :name "object")
  (insert-transformable-object
   :shape-type jsk_rviz_plugins::TransformableMarkerOperate::*BOX*
   :frame-id "BASE_LINK"
   :name "obstacle"
   :description "obstacle"
   )
  (set-transformable-object-dimensions
   :x (x-of-cube *obst*) :y (y-of-cube *obst*) :z (z-of-cube *obst*) :name "obstacle")
  (dotimes (i 4) ;; topic command sometimes fails
    (set-transformable-object-color
     :r 0 :g 0 :b 1 :a 0.7)
    (unix::usleep (* 100 1000))
    )
  (set-transformable-object-pose
   (make-coords :pos (float-vector -1000 0 0)) "BASE_LINK" :name "obstacle")
  )

(defun demo-node-optmotiongen-interactive
    ()
  (setup-optmotiongen-for-demo-node-optmotiongen-interactive)
  (setup-ros-for-demo-node-optmotiongen-interactive)

  (do-until-key
   (send *obj* :newcoords
         (get-coords-from-interactive-marker "object"))
   (send *obj* :worldcoords)
   (send *obst* :newcoords
         (get-coords-from-interactive-marker "obstacle"))
   (send *obst* :worldcoords)
   (send *sqp-opt* :optimize
         :loop-num 2
         :loop-num-min 2
         :update-viewer-interval (if *visualize?* 2 nil)
         :print-status-interval nil
         )
   (ros::publish
    "/robot_state"
    (angle-vector-to-display-robot-state
     *robot*
     (make-coords)
     :global-frame "BASE_LINK"))
   (ros::publish
    "/target_coords"
    (ros::coords->tf-pose-stamped
     *target-coords*
     "BASE_LINK"
     ))
   (ros::publish
    "/attention_coords"
    (ros::coords->tf-pose-stamped
     *attention-coords*
     "BASE_LINK"
     ))
   ;; Rviz becomes Segfault if you display interactive marker and line marker at the same time
   ;; https://github.com/ros-visualization/rviz/issues/1082
   (ros::publish
    "/collision_marker"
    (line-list->marker-msg
     (mapcan #'(lambda (eds) (send eds :vertices)) (send *instant-config-task* :get :collision-line-list))
     (instance std_msgs::header :init :stamp (ros::time-now) :frame_id "BASE_LINK")
     :scale 5.0
     ))
   (let* ((kin-err
           (send *instant-config-task* :kinematics-task-value))
          (pos-err ;; [mm]
           (* 1e3 (norm (subseq kin-err 0 3))))
          (normal-err ;; [deg]
           (rad2deg (norm (subseq kin-err 3 6))))
          (text-string
           (format nil "pos-err: ~7,1f [mm] normal-err: ~5,2f [deg]" pos-err normal-err))
          (text-msg
           (instance jsk_rviz_plugins::OverlayText :init :text text-string))
          )
     (ros::publish "/ik_error_text" text-msg)
     )
   )
  )

(demo-node-optmotiongen-interactive)
