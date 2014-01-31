#!/usr/bin/env roseus

(ros::load-ros-manifest "jsk_footstep_planner")
(load "package://jsk_footstep_planner/euslisp/footstep-planner.l")

(defun planes-cb (msg)
  (let ((polygons (send msg :polygons)))
    (let ((faces (mapcar #'(lambda (polygon)
                             (let ((trans (send *tf* :lookup-transform
                                                (send polygon :header :frame_id)
                                                *global-frame-id*
                                                (send polygon :header :stamp))))
                               (let ((points (mapcar #'(lambda (point)
                                                         (send (send (send trans :copy-worldcoords)
                                                                     :transform
                                                                     (make-coords :pos (ros::tf-point->pos point)))
                                                               :worldpos))
                                                     (send polygon :polygon :points))))
                                 (instance face :init :vertices points))))
                         polygons)))
      (setq *planes* (instance faceset :init :faces faces))
      ))
  (objects (list *planes*))
  )

(defclass ros-footstep-parameter
  :super footstep-parameter
  :slots (left-footsteps right-footsteps))
(defmethod ros-footstep-parameter
  (:init ()
    (send-super :init :legs-num 2))
  (:gen-footstep (&rest args)
    (send self :gen-left-footstep)
    (send self :gen-right-footstep)
    )
  (:gen-left-footstep ()
    (let ((footsteps
           (mapcar #'(lambda (x y theta)
                       (let ((c (make-coords :pos (float-vector (* 1000.0 x)
                                                                (* 1000.0 y) 0))))
                         (send c :rotate theta :z)
                         c))
                   *footstep-xs* *footstep-ys* *footstep-thetas*)))
      (dotimes (i (length footsteps))
        (let ((f (elt footsteps i)))
          (send self :gen-step :left i f)))
      ))
  (:gen-right-footstep ()
    (let ((footsteps
           (mapcar #'(lambda (x y theta)
                       (let ((c (make-coords :pos (float-vector (* 1000.0 x)
                                                                (* -1000.0 y)
                                                                0))))
                         (send c :rotate (- theta) :z)
                         c))
                   *footstep-xs* *footstep-ys* *footstep-thetas*)))
      (dotimes (i (length footsteps))
        (let ((f (elt footsteps i)))
          (send self :gen-step :right i f)))
      ))
  (:gen-step (l/r name coords)
    (let ((s (cons name coords)))
      (case l/r
        ((:left :lleg) (push s left-footsteps))
        ((:right :rleg) (push s right-footsteps))
        (t (error "unkown leg ~A" l/r)))
      ))
  (:transformation-by-leg-and-step-name (leg-name step-name)
    (case leg-name
      ((:left :lleg) (cdr (assoc step-name left-footsteps)))
      ((:right :rleg) (cdr (assoc step-name right-footsteps)))
      (t (error "unkown leg ~A" leg-name))))
  (:next-leg (l/r)
    (if (eq l/r :lleg) :rleg :lleg))
  (:next-step-coords (support-leg-coords l/r &optional (name nil))
    (let ((transformations
           (case l/r
             ((:left :lleg) left-footsteps)
             ((:right :rleg) right-footsteps)
             (t (error "unkown leg ~A" l/r)))))
      (mapcar #'(lambda (trans)
                  (cons (car trans)
                        (send (send support-leg-coords :copy-worldcoords)
                              :transform (cdr trans))))
              transformations)))
  (:move-robot-model (&rest args) nil)
  (:calc-robot-coords (a-leg b-leg &rest args) ;dummy argumentsn
    (midcoords 0.5 a-leg b-leg))
  )

(defun plan-execute-cb (server goal)
  (let* ((param (instance ros-footstep-parameter :init))
         (prblm (instance footstep-planner-graph :init
                          :scene nil :scene-grow 0
                          :footstep-parameter param
                          :obstacles nil
                          :robot nil
                          :step-max 100.0
                          :rotate-max (deg2rad 10)
                          :leg-object *leg*
                          :thr 100.0
                          :rthr (deg2rad 10)
                          :wd 0.8
                          :wp 0.0
                          :wg 1.0
                          :timeout (ros::time+ (ros::time-now) (ros::time *timeout*))
                          :collisoin-robot-model nil
                          ))
         ;; 一個目がllegだったら、llegからスタートする
         (start-leg (if (= (send (elt (send goal :goal :initial_footstep :footsteps) 0) :leg)
                           jsk_footstep_msgs::Footstep::*LEFT*)
                        :lleg :rleg))
         (support-leg (if (eq start-leg :lleg) :rleg :lleg)))
    ;; creating initial step node from goal msg
    (let ((leg-coords (mapcar #'(lambda (leg-step)
                                  (let ((pose-stamped
                                         (instance geometry_msgs::PoseStamped
                                                   :init)))
                                    (send pose-stamped :header (send goal :goal :initial_footstep :header))
                                    (send pose-stamped :pose (send leg-step :pose))
                                    (send *tf* :transform-pose *global-frame-id* pose-stamped)))
                              (send goal :goal :initial_footstep :footsteps))))
      (send prblm :start-state
            (instance step-node :init
                      :worldcoords (cadr leg-coords)
                      :prev-coords (car leg-coords)
                      :l/r start-leg
                      :robot-coords (midcoords 0.5
                                               (car leg-coords)
                                               (cadr leg-coords))
                      :angle-vector nil))
      )
    (let ((goal-foot-coords (mapcar #'(lambda (leg-step)
                                        (let ((pose-stamped
                                               (instance geometry_msgs::PoseStamped
                                                         :init)))
                                          (send pose-stamped :header (send goal :goal :goal_footstep :header))
                                          (send pose-stamped :pose (send leg-step :pose))
                                          (send *tf* :transform-pose *global-frame-id* pose-stamped)))
                                    (send goal :goal :goal_footstep :footsteps))))
      (let ((goal-coords (apply #'midcoords 0.5 goal-foot-coords)))
        (send prblm :goal-state goal-coords)        ;goal coords
        (format t "goal: ~A~%" goal-coords)
        ))
    (let ((asolver (instance a*-footstep-graph-search-solver
                             :init 1.0 :stepcost-heuristic* prblm
                             :render-leg-object *leg*)))
      (let ((solver-nodes (send asolver :solve prblm :verbose nil)))
        (let ((result (send server :result)))
          (if solver-nodes
              (let ((footsteps (instance jsk_footstep_msgs::FootstepArray :init)))
                (send footsteps :footsteps
                      (mapcar #'(lambda (n)
                                  (let ((f (instance jsk_footstep_msgs::Footstep :init)))
                                    (let ((pose (ros::coords->tf-pose (send (send n :state) :worldcoords))))
                                  (send f :pose pose)
                                  (if (eq (send (send n :state) :l/r) :lleg)
                                      (send f :leg jsk_footstep_msgs::Footstep::*LEFT*)
                                    (send f :leg jsk_footstep_msgs::Footstep::*RIGHT*))
                                  f)))
                              solver-nodes))
                (send footsteps :header (send goal :goal :initial_footstep :header))
                (send result :result footsteps)
                (send server :set-succeeded result))
              (send server :set-aborted result))
          ))
      )
    ))

(defun main ()
  (ros::roseus "jsk_footstep_planner")
  ;; read parameters
  
  ;; hrp2 footstep size is default
  (let ((leg-size-x (* 1000.0 (ros::get-param "~leg_depth" 0.247)))
        (leg-size-z (* 1000.0 (ros::get-param "~leg_height" 0.001)))
        (leg-size-y (* 1000.0 (ros::get-param "~leg_width" 0.135))))
    (setq *leg* (make-cube leg-size-x leg-size-y leg-size-z))
    (setq *global-frame-id* (ros::get-param "~global_frame_id" "/odom"))
    (setq *tf* (instance ros::transform-listener :init)))
  (setq *footstep-xs* (ros::get-param "~x"))
  (setq *footstep-ys* (ros::get-param "~y"))
  (setq *footstep-thetas* (ros::get-param "~theta"))
  (setq *timeout* (ros::get-param "~timeout" 10))
  (setq *planes* nil)
  (setq *use-planes* (ros::get-param "~use_planes" nil))
  (setq *tf* (instance ros::transform-listener :init))
  (when *use-planes*
    (ros::subscribe "planes" jsk_pcl_ros::PolygonArray #'planes-cb))
  
  (let ((s (instance ros::simple-action-server :init
                     "footstep_planner" jsk_footstep_msgs::PlanFootstepsAction
                     :execute-cb 'plan-execute-cb)))
    (while (ros::ok)
      (send s :worker)
      (ros::spin-once)
      (x::window-main-one)
      (ros::sleep))
    ))

(main)

