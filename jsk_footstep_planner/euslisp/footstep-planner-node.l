#!/usr/bin/env roseus

(ros::roseus-add-msgs "jsk_footstep_msgs")
(ros::roseus-add-msgs "jsk_pcl_ros")
(ros::roseus-add-srvs "jsk_pcl_ros")
(ros::roseus-add-srvs "std_srvs")
(ros::roseus-add-srvs "jsk_footstep_planner")
(ros::roseus-add-msgs "jsk_rviz_plugins")
(comp:compile-file-if-src-newer
  (format nil "~A/euslisp/footstep_planner.l" (ros::rospack-find "jsk_footstep_planner")))
(load "footstep_planner")
;;(load "package://jsk_footstep_planner/euslisp/footstep-planner.l")


;; for debug
(setq *debug* nil)
(if *debug* (make-irtviewer))

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
      (if faces
          (let ((fc (instance faceset :init :faces faces)))
            (setq *planes* faces)
            (when *debug*
              (if (null (send *irtviewer* :objects))
                  (progn
                    (objects (list fc)))
                (progn
                  (send *irtviewer* :objects (list fc))
                  (send *irtviewer* :draw-objects))
                )
              ))))))

(defun plan-execute-cb (server goal)
  (setq *evaled-node-num* 0)
  (let ((req (instance std_srvs::EmptyRequest :init)))
    (ros::service-call "primitive_lock" req)
    )  
  (let* ((param (instance ros-footstep-parameter :init *leg-size-x* *leg-size-y*))
         (start-time (ros::time-now))
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
                          :timeout (ros::time+ start-time (ros::time *timeout*))
                          :planes *planes*
                          :z-axis (float-vector 0 0 1)
                          :collisoin-robot-model nil))
         (goal-foot-coords (mapcar #'footstep->consed-coords
                                   (send goal :goal :goal_footstep :footsteps)))
         (initial-foot-coords (mapcar #'footstep->consed-coords
                                      (send goal :goal :initial_footstep :footsteps)))
         (goal-coords (apply #'(lambda (x y) (midcoords 0.5 (cdr x) (cdr y))) goal-foot-coords))
         ;; 一個目がllegだったら、llegからスタートする
         (start-leg (if (= (send (elt (send goal :goal :initial_footstep :footsteps) 0) :leg)
                           jsk_footstep_msgs::Footstep::*LEFT*)
                        :rleg :lleg))
         (support-leg (if (eq start-leg :lleg) :rleg :lleg)))
    ;; cheep hack
    (setq *goal-header* (send goal :goal :initial_footstep :header))
    
    ;; (if *use-env-server*
    ;;     (let ((req (instance jsk_pcl_ros::EnvironmentLockRequest :init)))
    ;;       (let ((res (ros::service-call "env_lock" req t)))
    ;;         (setq *env-id* (send res :environment_id)))))
    
    ;; creating initial step node from goal msg
    (let ((leg-coords (mapcar #'cdr initial-foot-coords)))
      (send prblm :start-state
            (instance step-node :init
                      :worldcoords (cadr leg-coords)
                      :prev-coords (car leg-coords)
                      :l/r start-leg
                      :robot-coords (midcoords 0.5
                                               (car leg-coords)
                                               (cadr leg-coords))
                      :angle-vector nil)))
    (send prblm :goal-state goal-coords) ; goal coords
    (send prblm :goal-footstep goal-foot-coords)
    ;;(send prblm :setup-goal initial-foot-coords)
    (let ((asolver (instance a*-footstep-graph-search-solver
                             :init 1.0 :stepcost-heuristic* prblm
                             ;;:init 1.0 :stepcost-heuristic prblm
                             :render-leg-object *leg*)))
      (let ((solver-nodes (send asolver :solve prblm :verbose nil)))
        (let ((result (send server :result)))
          (if solver-nodes              ;successfully solved!
              (let ((footsteps (instance jsk_footstep_msgs::FootstepArray :init)))
                (send footsteps :header (send goal :goal :initial_footstep :header))
                (send footsteps :footsteps
                      (mapcar #'(lambda (n)
                                  (let ((f (instance jsk_footstep_msgs::Footstep :init)))
                                    (let ((pose (ros::coords->tf-pose (send (send n :state) :worldcoords))))
                                      (send f :pose pose)
                                      (if (eq (send (send n :state) :l/r) :lleg)
                                          (send f :leg jsk_footstep_msgs::Footstep::*LEFT*)
                                        (send f :leg jsk_footstep_msgs::Footstep::*RIGHT*))
                                      f)))
                              ;; remove the first step, because it will be the sampe step to the initial steps
                              (cdr solver-nodes)))
                (ros::ros-info "footsteps: ~A"
                  (mapcar #'(lambda (n)
                              (cons (send (send n :state) :l/r)
                                    (send (send n :state) :worldcoords)))
                          solver-nodes))
                ;; finalize the footstep
                (let ((last-foot (send (send (car (last solver-nodes)) :state) :l/r)))
                  (let ((step-order (if (eq last-foot :lleg)
                                        (list jsk_footstep_msgs::Footstep::*RIGHT* jsk_footstep_msgs::Footstep::*LEFT*)
                                      (list jsk_footstep_msgs::Footstep::*LEFT* jsk_footstep_msgs::Footstep::*RIGHT*))))
                    (let ((final-steps (list (find-if #'(lambda (s)
                                                          (= (car step-order) (car s)))
                                                      goal-foot-coords)
                                             (find-if #'(lambda (s)
                                                          (= (cadr step-order) (car s)))
                                                      goal-foot-coords))))
                      ;; append final steps and initial footstep
                      (send footsteps :footsteps (append (send goal :goal :initial_footstep :footsteps)
                                                         (send footsteps :footsteps)
                                                         (mapcar #'(lambda (s)
                                                                     (let ((f (instance jsk_footstep_msgs::Footstep :init)))
                                                                       (let ((pose (ros::coords->tf-pose (cdr s))))
                                                                         (send f :pose pose)
                                                                         (send f :leg (car s))
                                                                         f)))
                                                                 final-steps))))))
                ;; set the size of footsteps
                (send-all (send footsteps :footsteps)
                          :dimensions (instance geometry_msgs::Vector3 :init
                                                :x (* 0.001 *leg-size-x*)
                                                :y (* 0.001 *leg-size-y*)
                                                :z (* 0.001 *leg-size-z*)))
                (send result :result footsteps)
                ;; publish the result for visualization
                (let* ((now (ros::time-now))
                       (str
                       (format nil "took ~A sec to plan~%~A node are evaled~%"
                               (send (ros::time- now start-time) :to-sec)
                               *evaled-node-num*)))
                  (ros::ros-warn str)
                  (let ((msg (instance std_msgs::Float32 :init)))
                    (send msg :data (send (ros::time- now start-time) :to-sec))
                    (ros::publish "/footstep_time" msg))
                  (let ((msg (instance jsk_rviz_plugins::OverlayText :init)))
                    (send msg :text (format nil "Planning took ~A sec" 
                                            (send (ros::time- now start-time) :to-sec)))
                    (ros::publish "/footstep_time_text" msg))
                  (let ((msg (instance std_msgs::Float32 :init)))
                    (send msg :data *evaled-node-num*)
                    (ros::publish "/footstep_count" msg)))
                (let ((msg (instance jsk_rviz_plugins::OverlayText :init)))
                    (send msg :text (format nil "~A nodes are expanded" 
                                            *evaled-node-num*))
                    (ros::publish "/footstep_count_text" msg))
                (ros::publish "/footstep" footsteps)
                (send server :set-succeeded result))
              (send server :set-aborted result))
          ))
      )
    )
  (let ((req (instance std_srvs::EmptyRequest :init)))
    (ros::service-call "primitive_unlock" req)
    )
  )

(setq *successors-count* 0)
(setq *previous-successor-published-stamp* nil)
;; for debugging
(defun publish-successors ()
  (if (and *previous-successor-published-stamp*
           (< (send (ros::time- (ros::time-now) *previous-successor-published-stamp*) :to-sec) 1.0))
      (return-from publish-successors nil))
  (let ((param (instance ros-footstep-parameter :init *leg-size-x* *leg-size-y*)))
    (let ((right-footsteps (send param :right-footsteps)))
      (if (>= *successors-count* (length right-footsteps))
          (setq *successors-count* 0))
      (when (not (= (length right-footsteps) 0))
        (let ((the-footstep (cdr (elt right-footsteps *successors-count*))))
          ;; always publish 2 footsteps
          ;; the one is at origin
          (let ((initial-footstep
                 (instance jsk_footstep_msgs::Footstep :init))
                (second-footstep
                 (instance jsk_footstep_msgs::Footstep :init)))
            ;; setup initial
            (send initial-footstep :leg jsk_footstep_msgs::Footstep::*LEFT*)
            (send initial-footstep :pose (ros::coords->tf-pose (make-coords)))
            (send second-footstep :leg jsk_footstep_msgs::Footstep::*RIGHT*)
            (send second-footstep :pose (ros::coords->tf-pose the-footstep))
            (let ((footsteps (instance jsk_footstep_msgs::FootstepArray :init)))
              (send footsteps :footsteps (list initial-footstep second-footstep))
              (send-all (send footsteps :footsteps) :dimensions
                        (instance geometry_msgs::Vector3 :init
                                  :x (* 0.001 *leg-size-x*)
                                  :y (* 0.001 *leg-size-y*)
                                  :z (* 0.001 *leg-size-z*)))
              (send footsteps :header :stamp (ros::time-now))
              (send footsteps :header :frame_id "successors_origin")
              (ros::publish "/footstep_successors" footsteps))
            (let ((text (instance jsk_rviz_plugins::OverlayText :init)))
              (let ((texts (list "Footstep parameter"
                                 (format nil "Num: ~A/~A" *successors-count* (length right-footsteps))
                                 (format nil "x: ~A" (* 0.001 (elt (send the-footstep :worldpos) 0)))
                                 (format nil "y: ~A" (* 0.001 (elt (send the-footstep :worldpos) 1)))
                                 (format nil "z: ~A" (* 0.001 (elt (send the-footstep :worldpos) 2)))
                                 (format nil "theta: ~A (~A deg)"
                                         (elt (car (send the-footstep :rpy-angle)) 2)
                                         (rad2deg (elt (car (send the-footstep :rpy-angle)) 0))))))
                (send text :text (reduce
                                  #'(lambda (x y) (format nil "~A~%~A" x y)) texts))
                (send text :left 0) (send text :top 0)
                (send text :width 500) (send text :height 500)
                (send text :text_size 36)
                (ros::publish "/footstep_successors_text" text))))))))
  (setq *previous-successor-published-stamp* (ros::time-now))
  (incf *successors-count*))

(defun change-successor-cb (req)
  (let ((m (send req :response)))
    (cond ((string= (send req :type) "small")
           (ros::ros-info "change to small footstep")
           (setq *footstep-xs* *small-footstep-xs*)
           (setq *footstep-ys* *small-footstep-ys*)
           (setq *footstep-thetas* *small-footstep-thetas*))
          ((string= (send req :type) "normal")
           (ros::ros-info "change to normal footstep")
           (setq *footstep-xs* *normal-footstep-xs*)
           (setq *footstep-ys* *normal-footstep-ys*)
           (setq *footstep-thetas* *normal-footstep-thetas*))
          )
    m))

(defun main ()
  (ros::roseus "jsk_footstep_planner")
  ;; read parameters
  
  ;; hrp2 footstep size is default
  ;;(let ((leg-size-x (* 1000.0 (ros::get-param "~leg_depth" 0.247)))
  (let ((leg-size-x (* 1000.0 (ros::get-param "~leg_depth" 0.227)))
        (leg-size-z (* 1000.0 (ros::get-param "~leg_height" 0.001)))
        (leg-size-y (* 1000.0 (ros::get-param "~leg_width" 0.135))))
    (setq *leg-size-x* leg-size-x)
    (setq *leg-size-y* leg-size-y)
    (setq *leg-size-z* leg-size-z)
    (setq *leg* (make-cube leg-size-x leg-size-y leg-size-z))
    (setq *global-frame-id* (ros::get-param "~global_frame_id" "/odom"))
    (setq *tf* (instance ros::transform-listener :init)))
  (setq *normal-footstep-xs* (ros::get-param "~x" nil))
  (setq *normal-footstep-ys* (ros::get-param "~y" nil))
  (setq *normal-footstep-thetas* (ros::get-param "~theta" nil))
  (setq *small-footstep-xs* (ros::get-param "~small_x" nil))
  (setq *small-footstep-ys* (ros::get-param "~small_y" nil))
  (setq *small-footstep-thetas* (ros::get-param "~small_theta" nil))
  (setq *footstep-xs* *normal-footstep-xs*)
  (setq *footstep-ys* *normal-footstep-ys*)
  (setq *footstep-thetas* *normal-footstep-thetas*)
  (setq *timeout* (ros::get-param "~timeout" 10))
  (setq *planes* nil)
  (setq *use-planes* (ros::get-param "~use_planes" nil))
  (setq *use-env-server* (ros::get-param "~use_env_server" nil))
  (when *use-env-server*
    (ros::ros-info "waiting for the env server")
    (ros::wait-for-service "env_lock")
    (ros::wait-for-service "polygon_on_env")
    (ros::wait-for-service "primitive_lock")
    (ros::wait-for-service "primitive_unlock")
    (ros::ros-info "found the server")
    )
  (setq *tf* (instance ros::transform-listener :init))
  (when *use-planes*
    (ros::subscribe "planes" jsk_pcl_ros::PolygonArray #'planes-cb))
  (ros::advertise "/footstep" jsk_footstep_msgs::FootstepArray)
  (ros::advertise "/footstep_time" std_msgs::Float32)
  (ros::advertise "/footstep_count" std_msgs::Float32)
  (ros::advertise "/footstep_time_text" jsk_rviz_plugins::OverlayText)
  (ros::advertise "/footstep_count_text" jsk_rviz_plugins::OverlayText)
  (ros::advertise "/footstep_successors" jsk_footstep_msgs::FootstepArray)
  (ros::advertise "/footstep_successors_text" jsk_rviz_plugins::OverlayText)
  (ros::advertise-service "/change_successor"
    jsk_footstep_planner::ChangeSuccessor
    #'change-successor-cb)
  (let ((s (instance ros::simple-action-server :init
                     "footstep_planner" jsk_footstep_msgs::PlanFootstepsAction
                     :execute-cb 'plan-execute-cb)))
    (while (ros::ok)
      (send s :worker)
      (ros::spin-once)
      (if *debug* (x::window-main-one))
      (publish-successors)
      (ros::sleep))
    ))

(main)

