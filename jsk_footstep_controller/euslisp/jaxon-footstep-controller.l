#!/usr/bin/env roseus

;; The most simplest version to execute footsteps by :set-foot-steps
;; method and have actionlib server interface
(ros::roseus "jaxon_footstep_controller" :anonymous nil)
(ros::roseus-add-msgs "jsk_footstep_msgs")
(ros::roseus-add-msgs "nav_msgs")

(setq *step-refine* (ros::get-param "~/use_step_refine"))
(setq *assume-horizontal-plane* (ros::get-param "~/assume_horizontal_plane"))

(load "package://jsk_footstep_controller/euslisp/util.l")
;; (load "package://jsk_robot_utils/euslisp/robot-interface-utils.l")
(setq *robot-name* (ros::get-param "/robot/type" (unix::getenv "ROBOT")))
(load (robot-interface-file *robot-name*))
(init-robot-from-name *robot-name*)

(defun walking-pose (&key (real t) (height 40))
  (send *robot* :reset-manip-pose)
  (send *robot* :legs :move-end-pos (float-vector 0 0 height))
  (when real
    ;; (send *ri* :set-gait-generator-param :default-orbit-type :stair)
    (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
    (send *ri* :wait-interpolation)
    )
  )

(defun calc-map-diff (steps map->destination odom-cds)
  (let (support-leg
        map->ft
        coords-difference
        end-coords
        )
    (when (< (length steps) 2)
      (if (eq (send map->destination :name) :lleg)
          (setq support-leg :lleg end-coords "lleg_end_coords")
        (setq support-leg :rleg end-coords "rleg_end_coords"))
      (setq map->ft (send *tfl* :lookup-transform "map" end-coords (ros::time 0)))
      (setq coords-difference
            (send map->ft :transformation map->destination))
      ;;(return-from calc-map-diff coords-difference)
      (return-from calc-map-diff)
      )

    (if (eq (send (car steps) :name) :rleg)
        (setq support-leg :lleg end-coords "lleg_end_coords")
      (setq support-leg :rleg end-coords "rleg_end_coords"))
    (let (
          body->foot
          abc->foot
          foot->laststep
          map->foot
          map->laststep
          )
      ;;(setq b->ft (send *tfl* :lookup-transform "BODY" "rleg_end_coords" (send *msg* :header :stamp)))
      (setq body->foot (send *tfl* :lookup-transform "BODY" end-coords (ros::time 0)))
      (setq abc->foot (send odom-cds :transform body->foot))
      (setq foot->laststep (send abc->foot :transformation (car (last (butlast steps)))))
      ;;
      (setq map->foot (send *tfl* :lookup-transform "map" end-coords (ros::time 0)))
      (setq map->laststep (send map->foot :transform foot->laststep))
      (pprint (list 'map->laststep map->laststep end-coords))
      (pprint (list 'map->dest map->destination))
      (setq coords-difference (send map->laststep :transformation map->destination))
      (pprint (list 'diff coords-difference))
      coords-difference
      ))
  )

(defun check-footstep-collision
  (current next)
  ;; fix next footstep to avoid collision
  ;; return original next footstep if current and next would not cause collision
  (let (current-foot next-foot)
    (setq current-foot (make-cube 235 135 20))
    (setq next-foot (make-cube 235 135 20))
    (send current-foot :move-to (send current :coords) :world)
    (send next-foot :move-to (send next :coords) :world)
    (let ((dy 0))
      (while (pqp-collision-check-objects (list current-foot) (list next-foot))
        (ros::ros-info "Collision detected between footsteps. Modify next step.")
        ;;(pprint next-foot)
        (cond
         ((equal (send next :name) :lleg)
          (setq dy (+ 1 dy))
          (send next-foot :translate #f( 0 1 0) :local))
         ((equal (send next :name) :rleg)
          (setq dy (- dy 1))
          (send next-foot :translate #f( 0 -1 0) :local))
         )
        )
      (send next :translate (float-vector 0 dy 0 ) :local)
      (pprint next)
      )
    )
  next
  )

(defun refine-steps (remaining-steps &key (offset 2) (expand-step) (exec nil) (collision-avoid nil))
  (let ((steps (butlast (car remaining-steps)))
        (idx (cadr remaining-steps))
        istep newsteps trans)
    (setq istep (elt steps offset))
    (setq steps (subseq steps (+ offset 1)))
    (when expand-step
      (let* ((last-cds (car (last steps)))
             (trans (send last-cds :transformation
                          (send (send last-cds :copy-worldcoords) :transform expand-step)
                          :world))
             (len (float (- (length steps) 2)))
             (cntr 0))
        (pprint (list 'trans trans expand-step))
        (pprint (list 'org-steps steps))
        (mapcar #'(lambda (s)
                    (let ((tr (midcoords (/ cntr len) (make-coords) trans))
                          nm)
                      ;; limit trans
                      #|
                      (setq nm (norm (send tr :worldpos)))
                      (when (> nm 15)
                        (setq tr (midcoords (/ 15 nm) (make-coords) tr)))
                      |#
                      (send s :transform tr :world)
                      (incf cntr)
                      s)) (butlast steps))
        (send last-cds :transform trans :world)
        (pprint (list 'trans-steps steps))
        ))
    (when collision-avoid
      (let ((start-time (ros::time-now))
            fslist modifiedfs) ;; instant collision check
        (push istep fslist)
        (dotimes (cnt (length steps))
          (if (eq cnt 0)
              (setq modifiedfs (check-footstep-collision istep (elt steps cnt)))
            (setq modifiedfs (check-footstep-collision (elt steps (- cnt 1)) (elt steps cnt))))
          (push modifiedfs fslist)
          )
        (setq newsteps (reverse fslist))
        (let ((now (ros::time-now)))
          (ros::ros-info "took ~A sec for collision avoid"
                  (send (ros::time- now start-time) :to-sec)))))
    (unless collision-avoid
      (setq newsteps (append (list istep) steps)))
    (pprint (list 'new newsteps))
    (when exec
      (send *ri* :set-foot-steps-no-wait newsteps
            :overwrite-footstep-index (+ idx offset 1)))
    ))

(defun convert-footstep-array-to-coords (footstep-array)
  (let (retval)
    (if *assume-horizontal-plane*
        ;; ignore z transformation and roll/pitch rotation
        (setq retval
              (mapcar #'(lambda (f)
                          (let ((ypr (car (rpy-angle (ros::tf-quaternion->rot (send f :pose :orientation))))))
                            (send f :pose :position :z 0.0)
                            (send f :pose :orientation (ros::rot->tf-quaternion (rpy-matrix (elt ypr 0) 0 0)))
                            (footstep->coords f)
                            )
                          )
                      (send footstep-array :footsteps))
              )
      (setq retval (footstep-array->coords footstep-array))
      )
    retval
    )
  )

(setq *footsteps* nil)
(defun execute-cb (server goal) ;; old version
  (ros::ros-info "execute-cb")
  (let ((footstep (send goal :goal :footstep)))
    ;; footstep := jsk_recognition_msgs/FootstepArray
    (let ((footstep-coords (convert-footstep-array-to-coords footstep))
          ;; (original-orbit-type (send (send *ri* :get-gait-generator-param) :default_orbit_type))
          )
      (print-readable-coords footstep-coords)
      ;;(walking-pose :real t :height 40)
      (send *ri* :set-foot-steps footstep-coords)
      (setq *footsteps* footstep-coords)

      (send *ri* :set-foot-steps-no-wait footstep-coords)
      (pprint footstep-coords)

      (let (ret map-dest cds-diff wait-until odom-cds)
        (setq map-dest (copy-object (car (last footstep-coords))))
        (while
            (progn
              (setq ret (send *ri* :get-remaining-foot-step-sequence-current-index))
              (car ret))
          (when *step-refine*
            ;;
            (ros::spin-once)
            (setq odom-cds (ros::tf-pose->coords (send (send *odom_msg* :pose) :pose)))

            (setq cds-diff (calc-map-diff (car ret) map-dest odom-cds))

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

(setq *current-steps* nil)
(setq *wait-until* nil)
(setq *appending-offset* 2)
;;(setq *cntr* 0)
(defun execute-cb2 (server goal)
  ;; receive new footsteps and there is no executing steps
  (when (null *current-steps*)
    (let ((footstep-coords (convert-footstep-array-to-coords (send goal :goal :footstep))))
      (when *debug*
        (pprint (list 'write *cntr*))
        (dump-structure (format nil "/tmp/footstepmsg~2,2D.l" *cntr*) goal)
        (incf *cntr*))
      (print-readable-coords footstep-coords)
      (send *ri* :set-foot-steps-no-wait footstep-coords)
      (setq *current-steps* footstep-coords))
    (ros::ros-info "execute-cb: receive new steps ~D" (length *current-steps*))
    (return-from execute-cb2))

  (when *debug*
    (pprint (send goal :header :stamp))
    (print-ros-msg (send goal :goal_id))
    (terpri))

  ;; connect new footsteps to current executing steps
  (when (and (server . ros::pending-goal)
             (= (server . ros::status) actionlib_msgs::GoalStatus::*preempting*))
    ;; TODO: check resume
    ;; get new goal
    (let* ((new-goal (server . ros::pending-goal))
           (footstep-coords (convert-footstep-array-to-coords (send new-goal :goal :footstep))))
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
      (return-from execute-cb2))
    )
  
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

  ;; refine when executing footsteps
  (when *step-refine*
    ;; step may be refined based on map
    (let (ret map-dest cds-diff wait-until)
      (setq map-dest (copy-object (car (last *current-steps*))))

      (setq ret (send *ri* :get-remaining-foot-step-sequence-current-index))
      (ros::spin-once)
      (setq odom-cds (ros::tf-pose->coords (send (send *odom_msg* :pose) :pose)))

      (setq cds-diff (calc-map-diff (car ret) map-dest odom-cds))

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

(defun abc_odom_callback (msg) (setq *odom_msg* msg))
(ros::subscribe "/odom" nav_msgs::Odometry #'abc_odom_callback)

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
