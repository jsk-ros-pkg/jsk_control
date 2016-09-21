#!/usr/bin/env roseus
(ros::roseus-add-msgs "jsk_recognition_msgs")
(ros::roseus-add-msgs "jsk_footstep_msgs")
(ros::roseus-add-msgs "jsk_footstep_controller")
(load "package://jsk_footstep_controller/euslisp/util.l")

;; The most simplest version to execute footsteps by :set-foot-steps
;; method and have actionlib server interface
(ros::roseus "go_pos_server" :anonymous nil)

;;(setq *step-refine* (ros::get-param "~/use_step_refine"))
(setq *step-refine* t)
(setq *force-replanning* (ros::get-param "~/force_replanning"))
(setq *replanning-translation-threshould* (ros::get-param "~/replanning_translation_threshold"))
(unless (numberp *replanning-translation-threshould*)
  (setq *replanning-translation-threshould* 7.0))
(setq *plane-projection* (ros::get-param "~/plane_projection" nil))
(setq *wait-for-plane* t)

;;(setq *robot-name* (ros::get-param "/robot/type" (unix::getenv "ROBOT")))
;;(load (robot-interface-file *robot-name*))
;;(init-robot-from-name *robot-name*) ;; making *ri*
;; just for calling service
(load "package://hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l")
(setq *ri* (instance rtm-ros-robot-interface :define-all-ROSBridge-srv-methods))

;; setting for jaxon
(setq *default-lleg-pos* (float-vector 0  100 0))
(setq *default-rleg-pos* (float-vector 0 -100 0))
(setq *default-lleg-rpy* (list 0 0 0))
(setq *default-rleg-rpy* (list 0 0 0))

;; setting parameters
(setq *overwrite-offset* 2)
(setq *footstep-refine-duration* 0.3)

;; global variables
(defun clear-variables ()
  (setq *current-initial-coords* nil)
  (setq *current-target-coords* nil)
  (setq *current-steps* nil)
  (setq *wait-until* nil)
  (setq *status* jsk_footstep_controller::GoPosFeedback::*WAITING*)
  (setq *previous-time* (ros::time-now))
  (setq *planes* nil)
  )

(defun publish-feedback (server &optional (steps -0))
  (let ((msg (send server :feedback)))
    (send msg :feedback :status *status*)
    (send msg :feedback :remaining_steps steps)
    (send server :publish-feedback msg)
    ))

(defun planes-cb (msg)
  (let ((polygons (send msg :polygons)))
    (let ((faces (mapcar #'(lambda (polygon)
                             (let (trans)
                               (send *tfl* :wait-for-transform (send polygon :header :frame_id) "map" (send polygon :header :stamp) 1)
                               (setq trans (send *tfl* :lookup-transform
                                                 (send polygon :header :frame_id)
                                                 "map"
                                                 (send polygon :header :stamp)))
                               (if trans
                                   (let ((points (mapcar #'(lambda (point)
                                                             (send (send (send trans :copy-worldcoords)
                                                                         :transform
                                                                         (make-coords :pos (ros::tf-point->pos point)))
                                                                   :worldpos))
                                                         (send polygon :polygon :points))))
                                     (instance face :init :vertices points))
                                 nil
                                 )
                               )
                             )
                         polygons)))
      (setq faces (remove-if #'null faces))
      (if faces
          (let ((fc (instance faceset :init :faces faces)))
            (setq *planes* faces)
            )))))

(defun execute-cb (server goal)
  (let (remaining-steps)

    (cond
     ;;; initial stete (waiting for new goal)
     ((eq *status* jsk_footstep_controller::GoPosFeedback::*WAITING*)
      ;; first time
      (unless *current-initial-coords*
        (let ((planning-goal
               (make-go-pos-planning-msgs (send goal :goal :x)
                                          (send goal :goal :y)
                                          (send goal :goal :theta)
                                          :lleg-offset (make-coords :pos *default-lleg-pos* :rpy *default-lleg-rpy*)
                                          :rleg-offset (make-coords :pos *default-rleg-pos* :rpy *default-rleg-rpy*))))
          ;;
          (let ((steps (mapcar #'(lambda (f)
                                   (footstep->coords f))
                               (send planning-goal :goal :initial_footstep :footsteps))))
            (setq *current-initial-coords* (midcoords 0.5 (car steps) (cadr steps))))
          ;;
          (let ((steps (mapcar #'(lambda (f)
                                   (footstep->coords f))
                               (send planning-goal :goal :goal_footstep :footsteps))))
            (setq *current-target-coords*  (midcoords 0.5 (car steps) (cadr steps))))
          ;;
          (ros::ros-info "sending goal")
          (send *planning-client* :send-goal planning-goal)
          )
        (setq *status* jsk_footstep_controller::GoPosFeedback::*PLANNING*)
        (publish-feedback server (length remaining-steps))
        (return-from execute-cb))
      )
     ;;; planning state (waiting for finish planning)
     ((eq *status* jsk_footstep_controller::GoPosFeedback::*PLANNING*)
      (publish-feedback server (length remaining-steps))
      (let ((ret (send *planning-client* :wait-for-result :timeout 0.001)))
        (cond
         (ret ;; planning finished successfully
          (let ((result (send (send *planning-client* :get-result) :result)))
            ;; debug
            (ros::publish *footstep-visualize-topic* result)
            ;; execute
            (let ((footstep-coords (footstep-array->coords result)))
              ;; plane projection
              (when *plane-projection*
                (when *wait-for-plane*
                  (ros::ros-warn "plane is not subscribed, wait for planes")
                  (while (not *planes*)
                    (ros::spin-once)
                    (ros::sleep)
                    )
                  )
                (setq footstep-coords (project-footsteps *planes* footstep-coords))
                )
              ;; debug
              (pprint "new steps:")
              (print-readable-coords footstep-coords)
              (send *ri* :set-foot-steps-no-wait footstep-coords);; execute ...
              (setq *current-steps* footstep-coords)
              )
            (setq *status* jsk_footstep_controller::GoPosFeedback::*WALKING*)
            (publish-feedback server (length remaining-steps))
            (ros::ros-info "execute-cb: exec new steps ~D" (length *current-steps*))
            ))
         ((= (*planning-client* . ros::simple-state) ros::*simple-goal-state-done*)
          ;; planning done but not successfully
          (clear-variables)
          (send server :set-preempted)
          (publish-feedback server (length remaining-steps))
          ))
        )
      (return-from execute-cb)
      )
     ;;; walking state (waiting for overwriting target)
     ((eq *status* jsk_footstep_controller::GoPosFeedback::*WALKING*)
      ;; loop cycle check
      (let ((now (ros::time-now)))
        (when (< (send (ros::time- now *previous-time*) :to-sec) *footstep-refine-duration*)
          (return-from execute-cb))
        (setq *previous-time* now))

      (setq remaining-steps
            (send *ri* :get-remaining-foot-step-sequence-current-index))
      (publish-feedback server (length (car remaining-steps)))

      ;; finish walking
      (unless (car remaining-steps)
        (setq *status* jsk_footstep_controller::GoPosFeedback::*FINISH*)
        (publish-feedback server 0)
        (send server :set-succeeded (send server :result))
        (clear-variables)
        (return-from execute-cb))

      ;; set new goal (over written old gol)
      (when (and (server . ros::pending-goal)
                 (= (server . ros::status)
                    actionlib_msgs::GoalStatus::*preempting*))
        (let* ((new-goal (server . ros::pending-goal))
               (new-action (send new-goal :goal :action)))
          (cond
           ((< (length (car remaining-steps)) (+ *overwrite-offset* 1 2))
            ;; do nothing, because of few remaining steps
            )
           ((= new-action jsk_footstep_controller::GoPosGoal::*OVER_WRITE*) ;; go-pos over write
            ;;
            (let ((tgt-cds (make-coords :pos (float-vector (* 1000 (send new-goal :goal :x))
                                                           (* 1000 (send new-goal :goal :y)) 0)))
                  new-target error-cds)
              (send tgt-cds :rotate (deg2rad (send new-goal :goal :theta)) :z)
              (setq new-target (send (send *current-initial-coords* :copy-worldcoords) :transform tgt-cds))
              (setq error-cds (send *current-target-coords* :transformation new-target))
              ;;
              (pprint (list 'new-target new-target))
              (pprint (list 'error-cds error-cds))
              ;;
              (cond
               ;;; just moving target pose (relying on step-refine)
               ((and (not *force-replanning*)
                     (< (/ (norm (send error-cds :pos)) (length (car remaining-steps))) ;; refined translation per step
                        *replanning-translation-threshould*))
                (let ((new-target-step (send new-target :copy-worldcoords)))
                  (cond
                   ((eq (send (car (last *current-steps*)) :name) :lleg)
                    (send new-target-step :transform (make-coords :pos *default-lleg-pos* :rpy *default-lleg-rpy*))
                    (send new-target-step :name :lleg))
                   (t
                    (send new-target-step :transform (make-coords :pos *default-rleg-pos* :rpy *default-rleg-rpy*))
                    (send new-target-step :name :rleg)))
                  ;;
                  (setq *current-steps* (append *current-steps* (list new-target-step)))
                  )
                )
               ;;; replanning
               (t ;;
                (let ((map->odom (send *tfl* :lookup-transform "map" "odom" (ros::time 0)))
                      (offset *overwrite-offset*)) ;; replan offset ?
                  (let ((steps (butlast (car remaining-steps)))
                        (idx (cadr remaining-steps))
                        istep newsteps trans start-steps start-leg)
                    (setq istep (elt steps offset)) ;;
                    (setq steps (subseq steps (+ offset 1)))
                    (setq start-steps
                          (mapcar #'(lambda (step)
                                      (let ((cds (send step :copy-worldcoords)))
                                        (send cds :transform map->odom :world) ;; convert odom(local) -> map(world)
                                        (send cds :name (send step :name))
                                        cds))
                                  (if (eq (send istep :name) :lleg)
                                      (list istep (car steps))
                                    (list (car steps) istep))))
                    (setq start-leg (if (eq (send istep :name) :lleg) :lleg :rleg))

                    (pprint (list 'start-step start-steps))
                    (pprint (list 'map->odom map->odom))

                    (let ((planning-goal
                           (make-footstep-planning-msgs
                            start-steps new-target
                            :start-leg  start-leg
                            :lleg-offset (make-coords :pos *default-lleg-pos* :rpy *default-lleg-rpy*)
                            :rleg-offset (make-coords :pos *default-rleg-pos* :rpy *default-rleg-rpy*))))

                      (ros::ros-info "sending replanning goal")
                      (send *planning-client* :send-goal planning-goal)

                      (let ((ret (send *planning-client* :wait-for-result :timeout 120.0)))
                        (ros::ros-info "replanning finished")
                        (pprint remaining-steps)
                        (cond
                         (ret
                          (let* ((result (send (send *planning-client* :get-result) :result))
                                 (odom->map (send map->odom :inverse-transformation))
                                 (step-ary (footstep-array->coords result))
                                 send-steps)
                            (ros::publish *footstep-visualize-topic* result)
                            ;; plane projection
                            (when *plane-projection*
                              (when *wait-for-plane*
                                (ros::ros-warn "plane is not subscribed, wait for planes")
                                (while (not *planes*)
                                  (ros::spin-once)
                                  (ros::sleep)
                                  )
                                )
                              (setq step-ary (project-footsteps *planes* step-ary))
                              )
                            (setq send-steps (mapcar #'(lambda (x)
                                                         (let ((cds (send x :copy-worldcoords)))
                                                           (send cds :transform odom->map :world)
                                                           (send cds :name (send x :name))
                                                           cds)) step-ary))
                            (setq *current-steps* step-ary) ;; ?
                            ;;
                            (pprint 'planned-step)
                            (print-readable-coords step-ary)
                            (pprint 'org-reamining)
                            (pprint (list 'idx idx (+ idx offset 1)))
                            (print-readable-coords steps)
                            (pprint 'send-steps)
                            (print-readable-coords send-steps)
                            ;;
                            (send *ri* :set-foot-steps-no-wait send-steps
                                  :overwrite-footstep-index (+ idx offset 1))
                            (setq *wait-until* (+ idx offset 1))
                            ))
                         (t
                          (ros::ros-warn "replanning failed")
                          ;;
                          ))
                        ))
                    ))
                ))
              )
            ;; dirty hack
            (setq (server . ros::pending-goal) nil)
            (setq (server . ros::status) actionlib_msgs::GoalStatus::*active*)
            (setq (server . ros::goal) new-goal)
            (setq (server . ros::goal-id) (send new-goal :goal_id))

            (return-from execute-cb)
            )
           #|
           ((= strategy jsk_footstep_controller::GoPosGoal::*NEW_TARGET*) ;; new target (overwrite old one)
           ;;
           (return-from execute-cb)
           )
           |#
         )))
      ;; refine steps ...

      (when *step-refine*
        ;; step may be refined based on map
        (let (map-dest cds-diff wait-until)
          (setq map-dest (copy-object (car (last *current-steps*))))

          (setq cds-diff (calc-step-error-on-map (car remaining-steps) map-dest))

          (setq remaining-steps (send *ri* :get-remaining-foot-step-sequence-current-index))
          (when (and (> (length (car remaining-steps)) (+ *overwrite-offset* 4)) ;; offset + 4 (magic number)
                     (or (not *wait-until*)
                         (>= (cadr remaining-steps) *wait-until*)))
            (pprint (list 'remaining-steps remaining-steps))
            (refine-steps remaining-steps :offset *overwrite-offset*
                          :expand-step cds-diff
                          :exec t :collision-avoid t)
            (setq *wait-until* (+ (cadr remaining-steps) *overwrite-offset* 1))
            (ros::ros-info "execute-cb: step refine at ~D" *wait-until*)
            )))
      ;;
      )
     )
    ))

(defvar *tfl* (instance ros::transform-listener :init))
(ros::subscribe (format nil "~A/planes" (ros::get-name)) jsk_recognition_msgs::PolygonArray #'planes-cb)

(setq *planning-client*
      (instance ros::simple-action-client :init
                "footstep_planner" jsk_footstep_msgs::PlanFootstepsAction))
(send *planning-client* :wait-for-server);;

(clear-variables)
(setq *server* (instance ros::simple-action-server :init
                         (ros::get-name)
                         jsk_footstep_controller::GoPosAction
                         :groupname "go_pos_action"
                         :execute-cb 'execute-cb
                         ))

;; debug
(setq *footstep-visualize-topic* "/footstep_marker/output/plan_result")
(ros::advertise *footstep-visualize-topic* jsk_footstep_msgs::FootstepArray 1)
;;

(ros::rate 20)
(ros::ros-info "GoPos Server is ready as ~A" (ros::get-name))

(while (ros::ok)
  (send *server* :worker)
  (send *server* :spin-once)
  (ros::spin-once)
  (ros::sleep)
  )
#| ;;test
(setq remaining-steps
      (let ((lstep (make-coords :pos (float-vector 150  100 0)))
            (rstep (make-coords :pos (float-vector   0 -100 0)))
            gl)
        (send lstep :name :lleg)
        (send rstep :name :rleg)
        (list (list t t rstep lstep rstep)
              2)
        ))
(setq new-target (make-coords :pos #f(3000 0 0)))
|#
