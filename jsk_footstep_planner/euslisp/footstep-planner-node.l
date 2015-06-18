#!/usr/bin/env roseus

(ros::roseus-add-msgs "jsk_footstep_msgs")
(ros::roseus-add-msgs "jsk_recognition_msgs")
(ros::roseus-add-srvs "jsk_pcl_ros")
(ros::roseus-add-srvs "std_srvs")
(ros::roseus-add-srvs "jsk_footstep_planner")
(ros::roseus-add-srvs "jsk_interactive_marker")
(ros::roseus-add-msgs "jsk_interactive_marker")
(ros::roseus-add-msgs "jsk_rviz_plugins")

(comp:compile-file-if-src-newer
  (format nil "~A/euslisp/footstep_planner_util.l"
          (ros::rospack-find "jsk_footstep_planner")))
(let ((old-module (find "footstep_planner_util" *loaded-modules*
                        :key #'lisp::load-module-file-name :test #'equal)))
  (if old-module (sys::unbinload old-module)))
(load "footstep_planner_util")

;;(load "~/ros/hydro_parent/src/jskeus/irteus/irtgraph.l")
(comp:compile-file-if-src-newer
  (format nil "~A/euslisp/footstep_planner.l" (ros::rospack-find "jsk_footstep_planner")))
(load "package://jsk_footstep_planner/euslisp/footstep_planner.l")
(let ((old-module (find "footstep_planner" *loaded-modules*
                        :key #'lisp::load-module-file-name :test #'equal)))
  (if old-module (sys::unbinload old-module)))
(load "footstep_planner")
;;(load "package://jsk_footstep_planner/euslisp/footstep_planner.l")
(comp:compile-file-if-src-newer
  (format nil "~A/euslisp/simple_occupancy_grid.l" (ros::rospack-find "jsk_footstep_planner")))
(let ((old-module (find "simple_occupancy_grid" *loaded-modules*
                        :key #'lisp::load-module-file-name :test #'equal)))
  (if old-module (sys::unbinload old-module)))
(load "simple_occupancy_grid")
;;(load "package://jsk_footstep_planner/euslisp/simple_occupancy_grid.l")


(setq *grids* nil)
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
  (send-all *grids* :prepare-for-planning)
  ;; Compute g-sensor coords from global-frame-id
  (let ((gsensor-coords nil))
    ;; Lookup gsensor-coords until we found it
    (while (null gsensor-coords)
      (ros::ros-info "Waiting for gsensor coords")
      (ros::sleep)
      (setq gsensor-coords              ;gsensor -> ground
            (send *tf* :lookup-transform "imu_floor" *global-frame-id* (ros::time 0))))

    (let* ((param (instance ros-footstep-parameter :init *leg-size-x* *leg-size-y*))
           (start-time (ros::time-now))
           (prblm (instance footstep-planner-graph :init
                            :roll-paranoid *roll-paranoid*
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
                            :transition-limit *transition-limit*
                            :timeout (ros::time+ start-time (ros::time *timeout*))
                            :planes *planes*
                            :grids *grids*
                            :g-sensor-coords gsensor-coords
                            :z-axis (float-vector 0 0 1)
                            :collisoin-robot-model nil))
           (goal-foot-coords (mapcar #'(lambda (f)
                                         (footstep->consed-coords f goal))
                                     (send goal :goal :goal_footstep :footsteps)))
           (initial-foot-coords (mapcar #'(lambda (f) (footstep->consed-coords f goal))
                                        (send goal :goal :initial_footstep :footsteps)))
           (goal-coords (apply #'(lambda (x y) (midcoords 0.5 (cdr x) (cdr y))) goal-foot-coords))
           ;; If the first step of goal footstep is lleg, planning starts with lleg
           (start-leg (if (= (send (elt (send goal :goal :initial_footstep :footsteps) 0) :leg)
                             jsk_footstep_msgs::Footstep::*LEFT*)
                          :rleg :lleg))
           (support-leg (if (eq start-leg :lleg) :rleg :lleg)))
      ;; cheep hack
      (setq *goal-header* (send goal :goal :initial_footstep :header))

      ;; First of all, check *grid-maps* is available
      (when *use-env-server*
        (unless *grids*
          (ros::ros-warn "Not yet *grids* is available")
          (let ((result (send server :result)))
            (send server :set-aborted result)
            (return-from plan-execute-cb nil))))

      ;; Visualize goal footsteps
      (let ((first-step (make-footprint (car (car goal-foot-coords))))
            (second-step (make-footprint (car (cadr goal-foot-coords)))))
        ;;(print (list first-step second-step))
        (send first-step :move-to (cdr (car goal-foot-coords)) :world)
        (send second-step :move-to (cdr (cadr goal-foot-coords)) :world)
        (setq *goal-footsteps* (list first-step second-step))
        (when *use-gui*
          (send *irtviewer* :objects (append *goal-footsteps* *grids*))
          (send *irtviewer* :draw-objects)))

      ;; Check goal footstep is possible or not first
      (when *use-env-server*
        (send-all *goal-footsteps* :worldpos) ;Force to update vertices
        (when (or (not (cube-placable-p (car *goal-footsteps*) *grids*))
                  (not (cube-placable-p (cadr *goal-footsteps*) *grids*)))
          (ros::ros-warn "Goal footstep is outside of grids")
          (let ((result (send server :result)))
            (send server :set-aborted result))
          (return-from plan-execute-cb nil)))
      (ros::ros-info "Running Planning")
      
      ;; creating initial step node from goal msg
      (let ((leg-coords (mapcar #'cdr initial-foot-coords)))
        ;; Project leg-coords onto grid first
        (let ((projected-leg-coords (if *use-env-server*
                                        (mapcar
                                         #'(lambda (c)
                                             (project-coordinates-on-nearest-grid
                                              c *grids*))
                                         leg-coords)
                                      leg-coords)))
          ;; Check if the first step on grid
          (when (and *use-env-server*
                     (or (null (car projected-leg-coords))
                         (null (cadr projected-leg-coords))))
            (ros::ros-error "Failed to project first step")
            (let ((result (send server :result)))
              (send server :set-aborted result))
            (return-from plan-execute-cb nil))
          ;; Visualize initial footsteps
          (let ((first-step (make-footprint (car (car initial-foot-coords))))
                (second-step (make-footprint (car (cadr initial-foot-coords)))))
            (send first-step :move-to (car projected-leg-coords) :world)
            (send second-step :move-to (cadr projected-leg-coords) :world)
            (setq *initial-footsteps* (list first-step second-step))
            (when *use-gui*
              (send *irtviewer* :objects (append *goal-footsteps* *initial-footsteps* *grids*))
              (send *irtviewer* :draw-objects)))
          
          (send prblm :start-state
                (instance step-node :init
                          :worldcoords (cadr projected-leg-coords)
                          :prev-coords (car projected-leg-coords)
                          :l/r start-leg
                          :root t
                          :robot-coords (midcoords 0.5
                                                   (car projected-leg-coords)
                                                   (cadr projected-leg-coords))
                          :angle-vector nil))))
      (send prblm :goal-state goal-coords) ; goal coords
      (send prblm :goal-footstep goal-foot-coords)
      (let ((asolver (instance a*-footstep-graph-search-solver
                               :init 1.0 *heuristic* prblm
                               :render-leg-object *leg*)))
        (let ((solver-nodes (send asolver :solve prblm)))
          (send prblm :report)
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
                      (ros::publish (format nil "~A/footstep_time" (ros::get-name)) msg))
                    (let ((msg (instance jsk_rviz_plugins::OverlayText :init)))
                      (send msg :text (format nil "Planning took ~A sec" 
                                              (send (ros::time- now start-time) :to-sec)))
                      (ros::publish (format nil "~A/footstep_time_text" (ros::get-name)) msg))
                    (let ((msg (instance std_msgs::Float32 :init)))
                      (send msg :data *evaled-node-num*)
                      (ros::publish (format nil "~A/footstep_count" (ros::get-name)) msg)))
                  (let ((msg (instance jsk_rviz_plugins::OverlayText :init)))
                    (send msg :text (format nil "~A nodes are expanded" 
                                            *evaled-node-num*))
                    (ros::publish (format nil "~A/footstep_count_text" (ros::get-name)) msg))
                  (ros::publish (format nil "~A/footstep" (ros::get-name)) footsteps)
                  (send server :set-succeeded result))
              (send server :set-aborted result))
            ))
        )
      )
    (send-all *grids* :report)
    ))

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
              (ros::publish (format nil "~A/footstep_successors" (ros::get-name)) footsteps))
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
                (ros::publish (format nil "~A/footstep_successors_text" (ros::get-name)) text))))))))
  (setq *previous-successor-published-stamp* (ros::time-now))
  (incf *successors-count*))

(defun change-successor-cb (req)
  ;; It's deprecated!!
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

(defun grid-map-callback (msg)
  ;; first check the message is updated or not
  (when (= (length *grids*) (length (send msg :grids)))
    (let ((all-foundp t))
      (dotimes (i (length (send msg :grids)))
        (let* ((target-grid (elt (send msg :grids) i))
               (target-grid-cell-num
                (length (send target-grid :cells)))
               (foundp nil))
          (dolist (grid *grids*)
            (when (= target-grid-cell-num (send grid :cell-num))
              ;; target-grid is ok
              (setq foundp t)
              (return-from nil nil)))     ;break dolist
          (unless foundp
            (setq all-foundp nil)
            (return-from nil nil))))
      (when all-foundp
        (ros::ros-debug "Skip updating grid model")
        (return-from grid-map-callback))))
  (ros::ros-info "Updating grid model")
  (setq *grids* (remove-if #'null
                           (mapcar #'(lambda (g)
                                       (let ((trans (lookup-transform-with-duration
                                                     *tf*
                                                     *global-frame-id*
                                                     (send g :header :frame_id)
                                                     (send g :header :stamp)
                                                     1.0)))
                                         (if trans
                                             (instance occupancy-grid :init g trans))))
                                   (send msg :grids))))
  (dotimes (i (length *grids*))
    (setf (get (elt *grids* i) :face-color) (color-category20 i)))
  (when *remove-shadow-cells*
    (remove-shadow-cells *grids*))
  (when *use-gui*
    (send *irtviewer* :objects *grids*)
    (send *irtviewer* :draw-objects))
  (send-all *grids* :info)
  )

(defun make-footprint (leg)
  (let ((c (make-cube *leg-size-x* *leg-size-y* *leg-size-z*)))
    (if (= leg jsk_footstep_msgs::Footstep::*LEFT*)
        (setf (get c :face-color) :green)
      (setf (get c :face-color) :red))
    c))

(defun decompose-footstep-parameter (param)
  (let ((foot-size (cdr (assoc :foot-size param))))
    (setq *leg-size-x* (cdr (assoc :x foot-size)))
    (setq *leg-size-y* (cdr (assoc :y foot-size)))
    (setq *leg-size-z* (cdr (assoc :z foot-size))))
  (let ((foot-offset (cdr (assoc :foot-offset param))))
    (setq *leg-offset-x* (cdr (assoc :x foot-offset)))
    (setq *leg-offset-y* (cdr (assoc :y foot-offset)))
    (setq *leg-offset-z* (cdr (assoc :z foot-offset)))
    (setq *leg-offset* (float-vector *leg-offset-x* *leg-offset-y* *leg-offset-z*)))
  (setq *footstep-successors*
        (mapcar #'(lambda (successor-param)
                    (let ((x (cdr (assoc :x successor-param)))
                          (y (cdr (assoc :y successor-param)))
                          (theta (cdr (assoc :theta successor-param))))
                      (make-coords :pos (float-vector x y 0)
                                   :rpy (float-vector (deg2rad theta) 0 0))))
                (cdr (assoc :successors param))))
  (setq *footstep-transifion-first-successors*
        (mapcar #'(lambda (successor-param)
                    (let ((x (cdr (assoc :x successor-param)))
                          (y (cdr (assoc :y successor-param)))
                          (theta (cdr (assoc :theta successor-param))))
                      (make-coords :pos (float-vector x y 0)
                                   :rpy (float-vector (deg2rad theta) 0 0))))
                (cdr (assoc :transition-first-successors param))))
  (setq *footstep-transifion-second-successors*
        (mapcar #'(lambda (successor-param)
                    (let ((x (cdr (assoc :x successor-param)))
                          (y (cdr (assoc :y successor-param)))
                          (theta (cdr (assoc :theta successor-param))))
                      (make-coords :pos (float-vector x y 0)
                                   :rpy (float-vector (deg2rad theta) 0 0))))
                (cdr (assoc :transition-second-successors param))))
  (setq *transition-limit*
        (cdr (assoc :default-transition-limit param)))
  )

(defun set-heuristic-cb (req)
  (setq *heuristic* (read-from-string (send req :heuristic)))
  (send req :response))

(defun snap-footprint-cb (req)
  (let ((res (send req :response)))
    (when *grids*
      (let* ((input-pose (send req :input_pose))
             (input-coords (ros::tf-pose-stamped->coords input-pose))
             (trans (lookup-transform-with-duration
                     *tf*
                     *global-frame-id*
                     (send input-pose :header :frame_id)
                     (send input-pose :header :stamp)
                     1.0)))
        (when trans
          (let ((transformed-input-coords (send trans :transform input-coords)))
            (let ((projected-coords
                   (project-coordinates-on-nearest-grid
                    transformed-input-coords
                    *grids*)))
              (when (and projected-coords *snap-with-local-search*)
                (print (ros::tf-pose->coords (send req :lleg_pose)))
                (print (ros::tf-pose->coords (send req :rleg_pose)))
                (setq projected-coords
                      (local-search-footstep-on-grid
                       projected-coords
                       (get projected-coords :projecting-grid)
                       (instance ros-footstep-parameter
                                 :init *leg-size-x* *leg-size-y*)
                       (ros::tf-pose->coords (send req :lleg_pose))
                       (ros::tf-pose->coords (send req :rleg_pose)))))
              (when projected-coords
                (send res :success t)
                (send res :snapped_pose
                      (ros::coords->tf-pose-stamped
                        projected-coords
                        (send input-pose :header :frame_id)))
                (send (send res :snapped_pose) :header :stamp
                      (send input-pose :header :stamp))
                ))))))
    (unless (send res :success)
      (ros::ros-error "Failed to snap footprint"))
    res))

(defun snap-footprint-topic-cb (msg)
  (when *grids*
    (let* ((input-pose (send msg :input_pose))
           (input-coords (ros::tf-pose-stamped->coords input-pose))
           (trans (lookup-transform-with-duration
                   *tf*
                   *global-frame-id*
                   (send input-pose :header :frame_id)
                   (send input-pose :header :stamp)
                   1.0)))
      (when trans
        (let ((transformed-input-coords (send trans :transform input-coords)))
          (let ((projected-coords
                 (project-coordinates-on-nearest-grid
                  transformed-input-coords
                  *grids*)))
            (when (and projected-coords *snap-with-local-search*)
              (print (ros::tf-pose->coords (send msg :lleg_pose)))
              (print (ros::tf-pose->coords (send msg :rleg_pose)))
              (setq projected-coords
                    (local-search-footstep-on-grid
                     projected-coords
                     (get projected-coords :projecting-grid)
                     (instance ros-footstep-parameter
                               :init *leg-size-x* *leg-size-y*)
                     (ros::tf-pose->coords (send msg :lleg_pose))
                     (ros::tf-pose->coords (send msg :rleg_pose)))))
            (when projected-coords
              (ros::publish (format nil "~A/projected_footprint" (ros::get-name))
                            (ros::coords->tf-pose-stamped
                             projected-coords
                             (send msg :input_pose :header :frame_id))))))))))

(defun toggle-gui-debug-cb (req)
  (setq *gui-debug* (not *gui-debug*))
  (send req :response))

(defun main ()
  (ros::roseus "jsk_footstep_planner")
  ;; read parameters
  (let ((param-file (format nil "package://jsk_footstep_planner/config/~A_param.l"
                            (string-downcase (unix::getenv "ROBOT")))))
    (load param-file)
    (decompose-footstep-parameter *footstep-parameters*))
  ;; hrp2 footstep size is default
  (setq *leg* (make-cube *leg-size-x* *leg-size-y* *leg-size-z*))
  (setq *global-frame-id* (ros::get-param "~global_frame_id" "map"))
  (setq *tf* (instance ros::transform-listener :init))
  (setq *timeout* (ros::get-param "~timeout" 10))
  (setq *planes* nil)
  (setq *heuristic* (read-from-string
                     (ros::get-param "~heuristic" ":stepcost-heuristic**")))
  ;;(ros::get-param "~heuristic" ":stepcost-z-heuristic*")))
  (setq *roll-paranoid* (ros::get-param "~roll_paranoid" t))
  (setq *use-planes* (ros::get-param "~use_planes" nil))
  (setq *use-env-server* (ros::get-param "~use_env_server" t))
  (setq *use-gui* (ros::get-param "~use_gui" t))
  (setq *remove-shadow-cells* (ros::get-param "~remove_shadow_cells" nil))
  (setq *gui-debug* (ros::get-param "~gui_debug" nil))
  (setq *use-local-search* (ros::get-param "~use_local_search" t))
  (setq *snap-with-local-search* (ros::get-param "~snap_with_local_search" t))
  (if *use-gui*
      (make-irtviewer))
  (when *use-env-server*
    (ros::subscribe "grid_maps" jsk_recognition_msgs::SimpleOccupancyGridArray #'grid-map-callback))
  (setq *tf* (instance ros::transform-listener :init))
  (when *use-planes*
    (ros::subscribe "planes" jsk_recognition_msgs::PolygonArray #'planes-cb))

  (ros::advertise (format nil "~A/footstep" (ros::get-name)) jsk_footstep_msgs::FootstepArray)
  (ros::advertise (format nil "~A/footstep_time" (ros::get-name)) std_msgs::Float32)
  (ros::advertise (format nil "~A/footstep_count" (ros::get-name)) std_msgs::Float32)
  (ros::advertise (format nil "~A/footstep_time_text" (ros::get-name)) jsk_rviz_plugins::OverlayText)
  (ros::advertise (format nil "~A/footstep_count_text" (ros::get-name)) jsk_rviz_plugins::OverlayText)
  (ros::advertise (format nil "~A/footstep_successors" (ros::get-name)) jsk_footstep_msgs::FootstepArray)
  (ros::advertise (format nil "~A/footstep_successors_text" (ros::get-name)) jsk_rviz_plugins::OverlayText)
  (ros::advertise-service (format nil "~A/project_footprint" (ros::get-name)) jsk_interactive_marker::SnapFootPrint #'snap-footprint-cb)
  (ros::advertise-service (format nil "~A/set_heuristic" (ros::get-name)) jsk_interactive_marker::SetHeuristic #'set-heuristic-cb)
  (ros::advertise-service (format nil "~A/toggle_gui_debug" (ros::get-name)) std_srvs::Empty #'toggle-gui-debug-cb)
  (ros::advertise (format nil "~A/projected_footprint" (ros::get-name)) geometry_msgs::PoseStamped)
  (ros::subscribe (format nil "~A/project_footprint" (ros::get-name)) jsk_interactive_marker::SnapFootPrintInput #'snap-footprint-topic-cb)
  ;; (ros::advertise-service "/change_successor"
  ;;   jsk_footstep_planner::ChangeSuccessor
  ;;   #'change-successor-cb)
  (let ((s (instance ros::simple-action-server :init
                     (ros::get-name) jsk_footstep_msgs::PlanFootstepsAction
                     :execute-cb 'plan-execute-cb)))
    (while (ros::ok)
      (send s :worker)
      (ros::spin-once)
      (if *debug* (x::window-main-one))
      (publish-successors)
      (if *use-gui*
          (x::window-main-one))
      (ros::sleep))
    ))

(main)

