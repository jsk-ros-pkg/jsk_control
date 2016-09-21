#!/usr/bin/env roseus

;; The most simplest version to execute footsteps by :set-foot-steps
;; method and have actionlib server interface
(ros::roseus "jaxon_footstep_controller" :anonymous nil)
(ros::roseus-add-msgs "jsk_recognition_msgs")
(ros::roseus-add-msgs "jsk_footstep_msgs")
(ros::roseus-add-msgs "nav_msgs")

(setq *step-refine* (ros::get-param "~/use_step_refine"))

(load "package://jsk_footstep_controller/euslisp/util.l")
;; (load "package://jsk_robot_utils/euslisp/robot-interface-utils.l")
(setq *robot-name* (ros::get-param "/robot/type" (unix::getenv "ROBOT")))
(load (robot-interface-file *robot-name*))
(init-robot-from-name *robot-name*)

(setq *plane-projection* (ros::get-param "~/plane_projection" nil))
(setq *planes* nil)
(setq *wait-for-plane* t)

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

(defun walking-pose (&key (real t) (height 40))
  (send *robot* :reset-manip-pose)
  (send *robot* :legs :move-end-pos (float-vector 0 0 height))
  (when real
    ;; (send *ri* :set-gait-generator-param :default-orbit-type :stair)
    (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
    (send *ri* :wait-interpolation)
    )
  )
#|
(setq *footsteps* nil)
(defun execute-cb (server goal) ;; old version
  (ros::ros-info "execute-cb")
  (let ((footstep (send goal :goal :footstep)))
    ;; footstep := jsk_recognition_msgs/FootstepArray
    (let ((footstep-coords (footstep-array->coords footstep))
          ;; (original-orbit-type (send (send *ri* :get-gait-generator-param) :default_orbit_type))
          )
      (print-readable-coords footstep-coords)
      ;;(walking-pose :real t :height 40)
      (send *ri* :set-foot-steps footstep-coords)
      (setq *footsteps* footstep-coords)

      (send *ri* :set-foot-steps-no-wait footstep-coords)
      (pprint footstep-coords)

      (let (ret map-dest cds-diff wait-until)
        (setq map-dest (copy-object (car (last footstep-coords))))
        (while
            (progn
              (setq ret (send *ri* :get-remaining-foot-step-sequence-current-index))
              (car ret))
          (when *step-refine*
            ;;
            (setq cds-diff (step-error-on-map (car ret) map-dest))

            (when (and (> (length (car ret)) 6) ;; offset + 4
                       (or (not wait-until)
                           (>= (cadr ret) wait-until)))
              (setq ret (send *ri* :get-remaining-foot-step-sequence-current-index))
              (refine-steps ret :offset 2 :expand-step cds-diff :exec t)
              (setq wait-until (+ (cadr ret) 2 1))
              )
            ) ;; step-refine
          (unix::usleep (* 1000 1000)) ;; TODO: use rostime
          ))
      ;;(send *ri* :angle-vector (send *robot* :reset-manip-pose) 3000)
      ;;(send *ri* :wait-interpolation)
      ;;(send *ri* :set-gait-generator-param :default-orbit-type original-orbit-type)
      (send server :set-succeeded (send server :result))
      )))
|#

(setq *current-steps* nil)
(setq *wait-until* nil)
(setq *appending-offset* 2)
;;(setq *cntr* 0)
(defun execute-cb2 (server goal)
  (when (null *current-steps*)
    (let ((footstep-coords (footstep-array->coords (send goal :goal :footstep))))
      (when *debug*
        (pprint (list 'write *cntr*))
        (dump-structure (format nil "/tmp/footstepmsg~2,2D.l" *cntr*) goal)
        (incf *cntr*))
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
      (pprint "new steps:")
      (print-readable-coords footstep-coords)
      (send *ri* :set-foot-steps-no-wait footstep-coords)
      (setq *current-steps* footstep-coords))
    (ros::ros-info "execute-cb: receive new steps ~D" (length *current-steps*))
    (return-from execute-cb2))

  (when *debug*
    (pprint (send goal :header :stamp))
    (print-ros-msg (send goal :goal_id))
    (terpri))

  (when (and (server . ros::pending-goal)
             (= (server . ros::status) actionlib_msgs::GoalStatus::*preempting*))
    ;; TODO: check resume
    ;; get new goal
    (let* ((new-goal (server . ros::pending-goal))
           (strategy (send new-goal :goal :strategy))
           (footstep-coords (footstep-array->coords (send new-goal :goal :footstep))))
      (cond
       ((= strategy jsk_footstep_msgs::execfootstepsgoal::*RESUME*) ;; resume (append)
        (when *debug*
          (pprint (list 'write *cntr*))
          (dump-structure (format nil "/tmp/footstepmsg~2,2D.l" *cntr*) new-goal)
          (incf *cntr*))
        ;; TODO: check old-goal last = new-goal first
        (setq *current-steps* (append-steps *current-steps* footstep-coords))

        (let* ((ret (send *ri* :get-remaining-foot-step-sequence-current-index))
               (idx (cadr ret))
               (abc-steps (butlast (car ret)))
               (abc-last-foot (car (last abc-steps)))
               abc-b-list
               )
          (unless (eq (send abc-last-foot :name)
                      (send (car footstep-coords) :name))
            (setq abc-last-foot (car (last (butlast abc-steps)))))
          (unless abc-last-foot
            (ros::ros-info "execute-cb: abc-last-foot is nil")
            (return-from execute-cb2)
            )
          (setq abc-b-list
                (mapcar #'(lambda (x)
                            (let ((cds (send (send abc-last-foot :copy-worldcoords)
                                             :transform (send (car footstep-coords) :transformation x)
                                             )))
                              (send cds :name (send x :name))
                              cds))
                        footstep-coords))
          (setq abc-steps (append-steps abc-steps abc-b-list))
          (let ((sending-steps (subseq abc-steps *appending-offset*)))
            (when *plane-projection*
              (when *wait-for-plane*
                (ros::ros-warn "plane is not subscribed, wait for planes")
                (while (not *planes*)
                  (ros::spin-once)
                  (ros::sleep)
                  )
                )
              (setq sending-steps (project-footsteps *planes* sending-steps))
              )
            (send *ri* :set-foot-steps-no-wait sending-steps
                  :overwrite-footstep-index (+ idx *appending-offset* 1)))
          (setq *wait-until* (+ idx *appending-offset* 1))
          )
        ;; dirty hack
        (setq (server . ros::pending-goal) nil)
        (setq (server . ros::status) actionlib_msgs::GoalStatus::*active*)
        (setq (server . ros::goal) new-goal)
        (setq (server . ros::goal-id) (send new-goal :goal_id))
        (ros::ros-info "execute-cb: append steps ~D / total ~D steps"
                       (length footstep-coords)
                       (length *current-steps*))
        (return-from execute-cb2)
        )
       ((= strategy jsk_footstep_msgs::execfootstepsgoal::*NEW_TARGET*) ;; new target (overwrite old one)
        ;;
        (return-from execute-cb2)
        )
       )))
  ;; check done
  (let ((ret (send *ri* :get-remaining-foot-step-sequence-current-index)))
    (unless (car ret)
      (setq *current-steps* nil)
      (setq *wait-until* nil)
      (send server :set-succeeded (send server :result))
      (return-from execute-cb2)
      )
    (ros::ros-info "execute-cb: steps current ~D/~D (remaining ~D)"
                   (cadr ret) (length *current-steps*) (length (car ret)))
    )
  (when *step-refine*
    ;; step may be refined based on map
    (let (ret map-dest cds-diff wait-until)
      (setq map-dest (copy-object (car (last *current-steps*))))

      (setq ret (send *ri* :get-remaining-foot-step-sequence-current-index))
      (ros::spin-once)

      (setq cds-diff (calc-step-error-on-map (car ret) map-dest))

      (when (and (> (length (car ret)) 6) ;; offset + 4
                 (or (not *wait-until*)
                     (>= (cadr ret) *wait-until*)))
        (setq ret (send *ri* :get-remaining-foot-step-sequence-current-index))
        (pprint (list 'ret ret))
        (refine-steps ret :offset 2 :expand-step cds-diff :exec t :collision-avoid t)
        (setq *wait-until* (+ (cadr ret) 2 1))
        (ros::ros-info "execute-cb: step refine at ~D" *wait-until*)
        )
      ))
  )

(ros::subscribe (format nil "~A/planes" (ros::get-name)) jsk_recognition_msgs::PolygonArray #'planes-cb)
(setq *server* (instance ros::simple-action-server :init
                         (ros::get-name)
                         jsk_footstep_msgs::ExecFootstepsAction
                         :groupname "footstep_action"
                         :execute-cb 'execute-cb2
                         ))

(ros::rate 2)
(ros::ros-info "simple footstep controller is ready as ~A" (ros::get-name))

(while (ros::ok)
  (send *server* :worker)
  (send *server* :spin-once)
  (ros::spin-once)
  (ros::sleep)
  )
