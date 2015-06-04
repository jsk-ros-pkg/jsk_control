#!/usr/bin/env roseus
(ros::roseus-add-msgs "std_msgs")
(ros::roseus-add-msgs "jsk_footstep_msgs")
(load "package://drc_task_common/euslisp/test-drc-terrain-walk.l")
;; forostep-controller.l
(ros::roseus-add-msgs "jsk_rviz_plugins")
(ros::roseus-add-srvs "jsk_footstep_controller")
(ros::roseus-add-srvs "std_srvs")
(ros::roseus-add-msgs "jsk_footstep_controller")

(defvar *all-the-results* nil)

(setq *realp* t)
;;(setq *wait-contact-state* t)
(setq *wait-contact-state* nil)
(setq *wait-convergence* t)
(setq *check-breakpoint* nil)
;;(setq *realp* nil)
;; :standing
;; :plane
;; :climing-up
(setq *walking-state* :standing)
(setq *prev-footstep* nil)
(setq *footstep-list* nil)
(setq *goal-id* nil)
(setq *footstep-list-bak* nil)
(setq *goal-id-bak* nil)
;; one of
;;   :non-interrupptible
;;   :full-interruptible
;;   :semi-interruptible
(setq *interruptible-mode* nil)
(setq *last-step* nil)
(setq *height-threshold* 100)
(setq *angle-threshold* (deg2rad 7))
;; ((footstep footstep ....) (footstep footstep ....) ...)

(defun footstep->coords (footstep frame-id)
  ;; convert jsk_footstep_msgs/Footstep to euslisp coodinates
  ;; where `frame-id' is the frame-id of `footstep'
  (let ((coords (ros::tf-pose->coords (send footstep :pose))))
    ;; in order to specify :lleg or :rleg, we need to set property list
    ;; of coordinates object, it will be read by robot-interface.
    (send coords :name
          (if (= (send footstep :leg) jsk_footstep_msgs::Footstep::*LEFT*)
              :lleg
            :rleg))
    coords))

(setq *contact-header* nil)
(setq *contact-state* jsk_footstep_controller::GroundContactState::*CONTACT_UNSTABLE*)
(setq *contact-error* 0)

(defun absmin (a b)
  (if (< (abs a) (abs b))
      a
    b))

(defun coords-z-pitch-and-roll-angle (coords)
  (let ((pitch-angle (absmin (elt (car (send coords :rpy-angle)) 1)
                             (elt (cadr (send coords :rpy-angle)) 1)))
        (roll-angle (absmin (elt (car (send coords :rpy-angle)) 2)
                            (elt (cadr (send coords :rpy-angle)) 2))))
    (list (elt (send coords :worldpos) 2) pitch-angle roll-angle)))

(defun separate-by-footsteps (footsteps)
  (let ((groups nil)
        (current-groups nil)
        (prev-footstep nil))
    (dolist (foot footsteps)
      (if (null current-groups)
          (setq current-groups (list foot))
        (progn
          ;; Compute transformation from previous step to current step
          (let ((trans (send prev-footstep :transformation foot)))
            (multiple-value-bind (zdiff pitch roll)
                (coords-z-pitch-and-roll-angle trans)
              (if (or (> (abs pitch) *angle-threshold*)
                      (> (abs roll) *angle-threshold*)
                      (> (abs zdiff) 50))
                  (progn ;the end of group because of different plane level
                    (setq groups (append groups (list current-groups)))
                    (setq current-groups (list foot)))
                (progn
                  (setq current-groups (append current-groups (list foot))))
                )))))
      (setq prev-footstep foot))
    (if current-groups
        (setq groups (append groups (list current-groups))))
    groups))


(defun separate-footstep-with-poststeps (grouped-footsteps)
  (let ((final-group nil))
    (dolist (group grouped-footsteps)
      (if (null final-group)
          (setq final-group (list group))
        (progn
          (if (> (length group) 2)
              (progn                    ;first 2 steps should be special group
                (setq final-group (append final-group (list (list (car group)
                                                                  (cadr group)))
                                          (list (cddr group)))))
            (progn                      ;no enough steps
              (setq final-group (append final-group (list group)))))
          )))
    final-group))

(defun contact-state-callback (msg)
  (setq *contact-state* (send msg :contact_state))
  (setq *contact-error* (send msg :error_pitch_angle))
  (setq *contact-header* (send msg :header))
  )

(defun contact-state->str (state)
  (cond
   ((eq state jsk_footstep_controller::GroundContactState::*CONTACT_UNSTABLE*)
    "unstable")
   ((eq state jsk_footstep_controller::GroundContactState::*CONTACT_BOTH_GROUND*)
    "ground")
   ((eq state jsk_footstep_controller::GroundContactState::*CONTACT_AIR*)
    "air")
   ((eq state jsk_footstep_controller::GroundContactState::*CONTACT_LLEG_GROUND*)
    "lleg")
   ((eq state jsk_footstep_controller::GroundContactState::*CONTACT_RLEG_GROUND*)
    "rleg")))

(defun wait-for-double-stance-phase ()
  (setq *contact-state* nil)
  (while (not (eq *contact-state* jsk_footstep_controller::GroundContactState::*CONTACT_BOTH_GROUND*))
    (ros::ros-info "contact state is ~A" (contact-state->str *contact-state*))
    (publish-breakpoint-text "Waiting for contact state to be stable")
    (ros::spin-once)
    (ros::sleep)))

(defun publish-breakpoint-text (text)
  (let ((msg (instance jsk_rviz_plugins::OverlayText :init)))
    (send msg :text text)
    (ros::publish "breakpoint_text" msg)))

(defun hrpsys-init-coords ()
  (midcoords
   0.5
   (send *ri* :get-foot-step-param :rleg-coords)
   (send *ri* :get-foot-step-param :lleg-coords)))

(defun new-goal-cb (server goal)
  ;; coordinate transformation:
  ;; 1. calc transformation from hrpsys-coords to footstep-coords[0].
  ;; 2. apply the inverse transformation to footstep-ocorods[1:]
  ;; 3. convert the footstep respected to hrpsys-mid-coords
  (cond ((eq *interruptible-mode* :non-interrupptible)
         (when *realp*
           (cond
            ((or (string= *robot-name* "JAXON")
                 (string= *robot-name* "JAXON_RED"))
             (walking-pose *robot* :root-link-height-offset -80))
            (t
             (walking-pose *robot* :root-link-height-offset -70))
            )
           (send *ri* :angle-vector (send *robot* :angle-vector) 5000)
           (send *ri* :wait-interpolation))))
  (cond ((= (send goal :goal :strategy)
            jsk_footstep_msgs::ExecFootstepsGoal::*RESUME*)
         (progn
           (print 'resuming)
           (setq *footstep-list* *footstep-list-bak*)
           (setq *footstep-list-bak* nil)
           (setq *goal-id* (send goal :goal_id :id))))
        ((= (send goal :goal :strategy)
            jsk_footstep_msgs::ExecFootstepsGoal::*NEW_TARGET*)
         (let* ((footstep (send goal :goal :footstep)) ;jsk_footstep_msgs/FootstepArray
                (frame-id (send footstep :header :frame_id)))
           (setq *footstep-transform* nil)
           (let ((counter 0))
             (while (null *footstep-transform*)
               (incf counter)
               (when (> counter 255)
                 (ros::ros-error "Failed to lookup transform of imu_floor")
                 (send server :set-preempted)
                 (return-from new-goal-cb t))
               (ros::ros-info "Waiting for footstep transform")
               ;; (setq *footstep-transform*
               ;;       (send *tf* :lookup-transform frame-id "ground" (ros::time 0)))
               ;; (send *ri* :state)
               ;; (setq *footstep-transform*
               ;;       (send (send *ri* :worldcoords) :copy-worldcoords))
               (setq *footstep-transform* (send *tf* :lookup-transform frame-id "imu_floor" (ros::time 0))) ;;, ground -> imu
               (if *footstep-transform* (return-from nil nil))
               (ros::sleep)))
           (let* ((footstep-coords            ; convert footstep to euslisp coords
                   (mapcar #'(lambda (footstep)
                               (footstep->coords footstep frame-id))
                           (send footstep :footsteps)))
                  (footstep-coords0 (car footstep-coords))
                  (footstep-coords-rest (cdr footstep-coords))
                  (hrpsys-coords (send *ri* :get-foot-step-param
                                       (if (eq (send footstep-coords0 :name) :lleg)
                                           :lleg-coords :rleg-coords)))
                  (hrpsys-mid-coords (hrpsys-init-coords)))
             (ros::ros-info "footstep-coords: ~A" footstep-coords)
             (let* ((hrpsys->ros (send hrpsys-coords :transformation footstep-coords0))
                    (ros->hrpsys (send hrpsys->ros :inverse-transformation)))
               (let ((transformed-footstep
                      (mapcar
                       #'(lambda (footstep)
                           (let ((coords (send (send ros->hrpsys :copy-worldcoords)
                                               :transform footstep)))
                             (send coords :name (send footstep :name))
                             coords))
                       footstep-coords-rest)))
                 (let ((final-footstep
                        (mapcar
                         #'(lambda (footstep)
                             (let ((c (send hrpsys-mid-coords
                                            :transformation footstep)))
                               (send c :name (send footstep :name))
                               c))
                         transformed-footstep)))
                   (push final-footstep *all-the-results*)
                   (setq *footstep-list* final-footstep)
                   (setq *goal-id* (send goal :goal_id :id))
                   ))))))))

(defun plane-condition-from-coords (coords)
  (let ((z (send coords :rotate-vector (float-vector 0 0 1))))
    (let ((yz (normalize-vector (float-vector 0 (elt z 1) (elt z 2))))
          (xz (normalize-vector (float-vector (elt z 0) 0 (elt z 2)))))
      (let ((rtheta (atan2 (elt yz 1) (elt yz 2)))
            (ptheta (atan2 (elt xz 0) (elt xz 2))))
        (ros::ros-info "rtheta: ~A" rtheta)
        (ros::ros-info "ptheta: ~A" ptheta)
        (cond ((and (> (abs rtheta) (deg2rad 5))
                    (> rtheta 0))
               :left-rolling)
              ((and (> (abs rtheta) (deg2rad 5))
                    (< rtheta 0))
               :right-rolling)
              ((and (> (abs ptheta) (deg2rad 5))
                    (> ptheta 0))
               :pitch-down)
              ((and (> (abs ptheta) (deg2rad 5))
                    (< ptheta 0))
               :pitch-up)
              (t
               :horizontal))))))

(defun rolling-plane-p (param)
  (or (eq param :left-rolling)
      (eq param :right-rolling)))

(defun next-rolling ()
  (rolling-plane-p *next-plane*))
(defun current-rolling ()
  (rolling-plane-p *current-plane*))

(defun different-height (st)
  (eq st :climing-up))

(defun quaternion-from-two-vectors (a b)
  (let* ((v0 (normalize-vector a))
         (v1 (normalize-vector b))
         (c (v. v1 v0)))
    (if (eps= c -1.0)
        (float-vector -1 0 0 0)
      (let* ((axis (v* v0 v1))
             (s (sqrt (* 2 (+ 1 c))))
             (invs (/ 1.0 s)))
        (let ((vec (scale invs axis))
              (w (* 0.5 s)))
          (normalize-vector (float-vector w (elt vec 0) (elt vec 1) (elt vec 2))))))))

(defun compute-current-origin ()
  ;; Only orientation is correct
  (if (null *last-step*)
      (send *footstep-transform* :copy-worldcoords)
    (let* ((q (quaternion-from-two-vectors
               (send *last-step* :rotate-vector (float-vector 0 0 1))
               (send *footstep-transform* :rotate-vector (float-vector 0 0 1))))
           (r (quaternion2matrix q))
           (trans (make-coords :rot r)))
      (send (send *last-step* :copy-worldcoords) :transform trans))))


;; from plane, to plane, height change, before posture, after posture, gait parameter
(setq *standing-mode-table*
      '((H   H   H0  S   S   P)
        (H   H   H+  S   S   S)
        (H   H   H-  S   S   S)
        (H   P+  H0  S   S   H)
        (H   P+  H+  S   S   S)
        (H   P+  H-  S   S   S)
        (H   P-  H0  S   S   H)
        (H   P-  H+  S   S   H)
        (H   P-  H-  S   S   S)
        (H   R   H0  S   R   H)
        (H   R   H+  S   R   S)
        (H   R   H-  S   R   H)
        (P+  H   H0  S   S   H)
        (P+  H   H+  S   S   S)
        (P+  H   H-  S   S   H)
        (P+  P+  H0  S   S   P)
        (P+  P+  H+  S   S   S)
        (P+  P+  H-  S   S   H)
        (P+  P-  H0  S   S   H)
        (P+  P-  H+  S   S   H)
        (P+  P-  H-  S   S   H)
        (P+  R   H0  S   R   H)
        (P+  R   H+  S   R   S)
        (P+  R   H-  S   R   H)
        (P-  H   H0  S   S   H)
        (P-  H   H+  S   S   S)
        (P-  H   H-  S   S   S)
        (P-  P+  H0  S   S   S)
        (P-  P+  H+  S   S   S)
        (P-  P+  H-  S   S   S)
        (P-  P-  H0  S   S   P)
        (P-  P-  H+  S   S   H)
        (P-  P-  H-  S   S   S)
        (P-  R   H0  S   R   H)
        (P-  R   H+  S   R   S)
        (P-  R   H-  S   R   S)
        (R   H   H0  S   S   H)
        (R   H   H+  R   S   S)
        (R   H   H-  R   S   S)
        (R   P+  H0  R   S   H)
        (R   P+  H+  R   S   S)
        (R   P+  H-  R   S   S)
        (R   P-  H0  R   S   H)
        (R   P-  H+  R   S   H)
        (R   P-  H-  R   S   H)
        (R   R   H0  R   R   R)
        (R   R   H+  R   R   S)
        (R   R   H-  R   R   S)))

(defun compute-plane-condition (step)
  (let* ((origin (compute-current-origin))
         (world-step (send (send origin :inverse-transformation)
                           :transform step)))
    (print world-step)
    (plane-condition-from-coords world-step)))

(defun compute-height-transition (first-step next-step)
  (cond ((and (null first-step) (null next-step))
         :standing)
        ((null *last-step*)
         (cond ((> (elt (send (send first-step :transformation next-step)
                              :worldpos) 2)
                   *height-threshold*)
                :climing-up)
               ((< (elt (send (send first-step :transformation next-step)
                              :worldpos) 2)
                   (- *height-threshold*))
                :climing-down)
               (t
                :plane)))
        ((> (elt (send (send first-step :transformation next-step)
                       :worldpos) 2)
            *height-threshold*)
         :climing-up)
        ((< (elt (send (send first-step :transformation next-step)
                       :worldpos) 2)
            (- *height-threshold*))
         :climing-down)
        (t
         :plane)))

(defun convert-to-table-symbols (from to height)
  (list (cond ((or (eq from :left-rolling)
                   (eq from :right-rolling))
               'R)
              ((eq from :pitch-up)
               'P+)
              ((eq from :pitch-down)
               'P-)
              (t
               'H))
        (cond ((or (eq to :left-rolling)
                   (eq to :right-rolling))
               'R)
              ((eq to :pitch-up)
               'P+)
              ((eq to :pitch-down)
               'P-)
              (t
               'H))
        (cond ((eq height :climing-up)
               'H+)
              ((eq height :climing-down)
               'H-)
              (t
               'H0))))

(defun set-parameter-from-table (from to height)
  (multiple-value-bind
   (from-s to-s height-s) (convert-to-table-symbols from to height)
   (dolist (candidate *standing-mode-table*)
     (when (and (eq from-s (car candidate))
                (eq to-s (cadr candidate))
                (eq height-s (caddr candidate)))
       (let ((param-s (elt candidate 5)))
         (ros::ros-info "set ri parameter for ~A" param-s)
         (cond ((eq param-s 'S)
                (if (eq from-s 'P-)
                    (set-parameter-for-rolling)
                  (set-parameter-for-stair)))
               ((eq param-s 'H)
                (set-parameter-for-rolling))
               ((eq param-s 'R)
                (set-parameter-for-planar-rolling))
               ((eq param-s 'P)
                (set-parameter-for-planar)
                )))
       (return-from set-parameter-from-table t)))))

(defun move-before-robot-from-table (from to height)
  (multiple-value-bind
      (from-s to-s height-s) (convert-to-table-symbols from to height)
    (dolist (candidate *standing-mode-table*)
      (when (and (eq from-s (car candidate))
                 (eq to-s (cadr candidate))
                 (eq height-s (caddr candidate)))
        (let ((param-s (elt candidate 3))
              (gait-param (elt candidate 5)))
          (ros::ros-info "move robot for ~A/~A" param-s gait-param)
          (if (or (string= *robot-name* "JAXON")
                  (string= *robot-name* "JAXON_RED"))
              (progn
                (cond ((or (eq gait-param 'P)
                           (eq gait-param 'R))
                       (robot-angle-straight-ease))
                      (t
                       (robot-angle-straight))))
            (progn
              (cond ((eq param-s 'S)
                     (robot-angle-straight))
                    ((eq param-s 'P)
                     (robot-angle-planar))
                    ((eq param-s 'R)
                     (robot-angle-rolling from))))))
        (return-from move-before-robot-from-table t)))))

(defun move-after-robot-from-table (from to height)
  (multiple-value-bind
   (from-s to-s height-s) (convert-to-table-symbols from to height)
   (dolist (candidate *standing-mode-table*)
     (when (and (eq from-s (car candidate))
                (eq to-s (cadr candidate))
                (eq height-s (caddr candidate)))
       (let ((param-s (elt candidate 4)))
         (ros::ros-info "move robot for ~A" param-s)
         (cond ((eq param-s 'S)
                (robot-angle-straight))
               ((eq param-s 'R)
                (robot-angle-rolling to))))
       (return-from move-after-robot-from-table t)))))


(defun change-standing-mode (first-step next-step)
  (let ((next nil))
    ;; Plane condition
    ;; set *current-plane* and *next-plane*
    (setq *current-plane* (compute-plane-condition first-step))
    (setq *next-plane* (compute-plane-condition next-step))
    ;; :standing, :climing-up or :plane
    (setq next (compute-height-transition first-step next-step))
    (ros::ros-info "first-step: ~A" first-step)
    (ros::ros-info "next-step: ~A" next-step)
    (ros::ros-info "*last-step*: ~A" *last-step*)
    (ros::ros-info "*footstep-transform* ~A" *footstep-transform*)
    (ros::ros-info "walking state: ~A ==> ~A" *walking-state* next)
    (ros::ros-info "plane condition: ~A ==> ~A" *current-plane* *next-plane*)

    (set-parameter-from-table
     *current-plane* *next-plane* next)
    (move-before-robot-from-table
     *current-plane* *next-plane* next)
    (setq *walking-state* next)))

(defun change-standing-mode-post ()
  (move-after-robot-from-table
   *current-plane* *next-plane* *walking-state*))

(defun proc-goal-cb (server goal)
  (publish-breakpoint-text " ")
  (cond ((eq *interruptible-mode* :full-interruptible)
         (ros::ros-info "remaining footsteps: ~A" (length *footstep-list*))
         (cond ((<= (length *footstep-list*) 1) ;finish
                (progn
                  ;;(change-standing-mode nil nil)
                  (publish-breakpoint-text " ")
                  (let ((result (send server :result)))
                    (send server :set-succeeded result))
                  ;; Clear last-step
                  (setq *last-step* nil)
                  (when *auto-lookground*
                    (lookaround-ground))))
               (t
                (publish-breakpoint-text " ")
                (let ((first-step (pop *footstep-list*)))
                  (setq second-step (car *footstep-list*))
                  (change-standing-mode first-step second-step)
                  ;; Send two footsteps at once for different plane transition
                  (if (or (eq *walking-state* :climing-up) ;hack
                          (eq *walking-state* :climing-down) ;hack
                          (not (eq *current-plane* *next-plane*)))
                      (progn
                        (let ((third-step (cadr *footstep-list*)))
                          (when *realp*
                            (send *ri* :state)
                            (send *ri* :adjust-foot-steps
                                  (send (send *ri* :robot) :rleg :end-coords :copy-worldcoords)
                                  (send (send *ri* :robot) :lleg :end-coords :copy-worldcoords))
                            (send *ri* :set-foot-steps
                                  (list first-step second-step third-step)))
                          (setq *last-step* second-step)
                          (pop *footstep-list*)))
                    (progn
                      (when *realp*
                        (send *ri* :state)
                        (send *ri* :adjust-foot-steps
                              (send (send *ri* :robot) :rleg :end-coords :copy-worldcoords)
                              (send (send *ri* :robot) :lleg :end-coords :copy-worldcoords))
                        (send *ri* :set-foot-steps (list first-step second-step)))
                      (setq *last-step* first-step))
                    )
                  )
                (when *wait-contact-state*
                  ;;(unix:sleep 1)
                  (wait-for-double-stance-phase))
                (when *wait-convergence*
                  (wait-for-convergence))
                (change-standing-mode-post)
                (when *check-breakpoint*
                  (publish-breakpoint-text "Breakpoint!")
                  (setq req (instance jsk_footstep_controller::RequireMonitorStatusRequest :init))
                  (send req :header *contact-header*)
                  (if(eq (send second-step :name) :rleg)
                      (send req :threshold (deg2rad 1.5)) ;lleg
                    (send req :threshold (deg2rad 1.7))) ;rleg
                  (let ((res (ros::service-call "require_foot_contact_monitor" req)))
                    (unless (send res :go)
                      (preempt-cb server nil)))))))
        ((eq *interruptible-mode* :semi-interruptible) ;only JAXON is supported!!
         (print :semi-interruptible!!)
         (let* ((prev-step (car *footstep-list*))
                (plane-separated-footsteps (separate-by-footsteps (cdr *footstep-list*)))
                (grouped-footsteps (separate-footstep-with-poststeps plane-separated-footsteps)))
           (format t "groups: ~A~%" (length grouped-footsteps))
           (format t "group nums: ~A~%" (mapcar #'length grouped-footsteps))
           (format t "group: ~A~%" grouped-footsteps)
           (dotimes (i (length grouped-footsteps))
             (format t "i: ~A~%" i)
             (let ((target-footsteps (elt grouped-footsteps i)))
               (change-standing-mode prev-step (car target-footsteps))
               ;; ;; check if next step is on the same level or not
               (when *realp*
                 (send *ri* :state)
                 (send *ri* :adjust-foot-steps
                       (send (send *ri* :robot) :rleg :end-coords :copy-worldcoords)
                       (send (send *ri* :robot) :lleg :end-coords :copy-worldcoords))
                 (send *ri* :set-foot-steps (cons prev-step target-footsteps)))
               (format t ":set-foot-steps ~A" (cons prev-step target-footsteps))
               (when *wait-convergence*
                 (wait-for-convergence))
               ;; Postprepare joint angles
               (robot-angle-straight)
               ;; Prepare for the next footstep
               (if (>= (length target-footsteps) 2)
                   (setq *last-step* (elt target-footsteps (- (length target-footsteps) 2)))
                 (setq prev-step *last-step*))
               (setq prev-step (elt target-footsteps (1- (length target-footsteps))))
               )
             )
           )
         (setq *footstep-list* nil)
         (send server :set-succeeded (send server :result))
         (setq *last-step* nil)        ;clear for next
         (when *auto-lookground*
           (lookaround-ground)))
        ((eq *interruptible-mode* :non-interrupptible)
         ;; done
         (print (send-all *footstep-list* :name))
         (print *footstep-list*)
         (when *realp*
           (send *ri* :state)
           (send *ri* :adjust-foot-steps
                 (send (send *ri* :robot) :rleg :end-coords :copy-worldcoords)
                 (send (send *ri* :robot) :lleg :end-coords :copy-worldcoords))
           (send *ri* :set-foot-steps *footstep-list*))
         (setq *footstep-list* nil)
         (when *realp*
           (walking-pose *robot* :root-link-height-offset 0)
           (send *ri* :angle-vector (send *robot* :angle-vector) 5000)
           (send *ri* :wait-interpolation))
         (send server :set-succeeded (send server :result))
         (when *auto-lookground*
           (lookaround-ground))
         )
        ))

(defun lookaround-ground ()
  (let ((req (instance std_srvs::EmptyRequest :init)))
    (ros::service-call "/lookaround_ground" req)))

(defun wait-for-convergence ()
  (let ((prev-angles (send *ri* :state :potentio-vector)))
    (while (ros::ok)
      (let ((angles (send *ri* :state :potentio-vector)))
        (print (norm (v- prev-angles angles)))
        (if (< (norm (v- prev-angles angles)) 0.1)
            (return-from wait-for-convergence t))
        (setq prev-angles angles)
        (ros::sleep)
        ))))

(defun preempt-cb (server goal)
  (print 'preempt-cb)
  (setq *footstep-list-bak* *footstep-list*)
  (setq *footstep-list* nil)
  (send server :set-preempted)
  (setq *goal-id-bak* *goal-id*)
  (setq *goal-id* nil)
  ;; cancel broadcast
  (ros::publish "/footstep_cancel_broadcast" (instance std_msgs::Empty :init))
  )

(defun execute-cb (server goal)
  (print 'execute-cb)
  (if (or (null *goal-id*)
          (not (string= *goal-id* (send goal :goal_id :id))))
      (new-goal-cb server goal)
    (proc-goal-cb server goal)))

(defun require-log-service-callback (req)
  (let ((res (send req :response)))
    (let ((*print-object* t) (*print-circle* t)
          (*print-level* nil) (*print-length* nil))
      (send res :sexp (format nil "~A" *all-the-results*))
      res)))

(defun need-to-send-angle-vector-p (r)
  (let ((av (send r :angle-vector))
        (rv (send *ri* :state :reference-vector)))
    (> (norm (v- av rv)) 5)))

(defun robot-angle-straight ()
  (if (or (string= *robot-name* "JAXON")
          (string= *robot-name* "JAXON_RED"))
      (progn
        (walking-pose *robot*
                      ;;:default-pose-method :narrow-width-pose
                      :default-pose-method :reset-manip-pose
                      ;;:root-link-height-offset -80 ; -140 for stair
                      ;; :root-link-pitch-offset 5    ;20 for stair
                      ;;:root-link-pitch-offset 20
                      ;;:root-link-height-offset -140 ; for stair
                      :root-link-height-offset -100 ; for stair
                      ;;:chest-link-pitch-offset 20 ; for stair
                      ;; :chest-link-pitch-offset 5
                      :root-link-pitch-offset 10
                      )
          ;; (walking-pose *robot*
          ;;       :default-pose-method :narrow-width-pose
          ;;       :root-link-height-offset -140
          ;;       :root-link-pitch-offset 20
          ;;       :chest-link-pitch-offset 20)
        (send *robot* :head :neck-p :joint-angle 35))
    (walking-pose *robot* :root-link-height-offset -50 :root-link-pitch-offset 5))
  ;; Check if we need to move robot or not
  (when (need-to-send-angle-vector-p *robot*)
    ;;(send *ri* :angle-vector (send *robot* :angle-vector) 5000)
    (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
    (send *ri* :wait-interpolation))
  )

(defun robot-angle-straight-ease ()
  (if (or (string= *robot-name* "JAXON")
          (string= *robot-name* "JAXON_RED"))
      (progn
        (walking-pose *robot*
                      ;;:default-pose-method :narrow-width-pose
                      :default-pose-method :reset-manip-pose
                      ;;:root-link-height-offset -80 ; -140 for stair
                      ;; :root-link-pitch-offset 5    ;20 for stair
                      ;;:root-link-pitch-offset 20
                      ;;:root-link-height-offset -140 ; for stair
                      :root-link-height-offset -40 ; for stair
                      ;;:chest-link-pitch-offset 20 ; for stair
                      ;; :chest-link-pitch-offset 5
                      :root-link-pitch-offset 10
                      )
        ;; (walking-pose *robot*
        ;;               :default-pose-method :narrow-width-pose
        ;;               :root-link-height-offset -140
        ;;               :root-link-pitch-offset 20
        ;;               :chest-link-pitch-offset 20)
        ;; (send *robot* :head :neck-p :joint-angle 35)
        )
    (walking-pose *robot* :root-link-height-offset -50 :root-link-pitch-offset 5))
  ;; Check if we need to move robot or not
  (when (need-to-send-angle-vector-p *robot*)
    ;;(send *ri* :angle-vector (send *robot* :angle-vector) 5000)
    (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
    (send *ri* :wait-interpolation))
  )

(defun robot-angle-rolling (policy)
  ;; policy := :left-rolling or :right-rolling
  (if (not (or (string= *robot-name* "JAXON")
               (string= *robot-name* "JAXON_RED")))
      (cond
       ((eq policy :left-rolling)
        (walking-pose *robot* :root-link-height-offset -50
                      :root-link-pitch-offset 5 :root-link-roll-offset 5))
       ((eq policy :right-rolling)
        (walking-pose *robot* :root-link-height-offset -50
                      :root-link-pitch-offset 5 :root-link-roll-offset -5))
       )
    (robot-angle-straight))
  (when (need-to-send-angle-vector-p *robot*)
    (send *ri* :angle-vector (send *robot* :angle-vector) 5000)
    (send *ri* :wait-interpolation))
  )


(defun set-parameter-for-rolling ()
  (high-step-stair-param-hrp2jsk)
  (send *ri* :set-gait-generator-param
        :stair-trajectory-way-point-offset #f(0.03 -0.0 0)
        :swing-trajectory-delay-time-offset 0.2
        :default-double-support-static-ratio 0.0
        ))

(defun set-parameter-for-planar-rolling ()
  (if (or (string= "JAXON" *robot-name*)
          (string= "JAXON_RED" *robot-name*))
      (set-parameter-for-planar-jaxon)
    (roll-terrain-stair-param-hrp2jsk))
  (send *ri* :set-gait-generator-param
        :stair-trajectory-way-point-offset #f(0.03 -0.0 0)
        :swing-trajectory-delay-time-offset 0.2
        :default-double-support-static-ratio 0.0
        ))

(defun set-parameter-for-stair ()
  (default-stair-param-hrp2jsk)
  (send *ri* :set-gait-generator-param
        :toe-heel-phase-ratio #f(0.02 0.28 0.2 0.0 0.2 0.25 0.05)
        ;;:stair-trajectory-way-point-offset #f(0.05 0.0 0)
        :stair-trajectory-way-point-offset #f(0.03 -0.0 0)
        :default-step-height 0.07
        ;; :stair-trajectory-way-point-offset #f(0.03 -0.0 0)
        :swing-trajectory-delay-time-offset 0.2
        :default-step-time 1.7
        :default-double-support-ratio 0.29
        :default-double-support-static-ratio 0.117
        )
  (when (or (string= "JAXON" *robot-name*)
            (string= "JAXON_RED" *robot-name*))
    (send *ri* :set-auto-balancer-param :default-zmp-offsets
          (list (float-vector -20 0 0) (float-vector -20 0 0)))
    )
  ;; (send *ri* :set-auto-balancer-param :default-zmp-offsets
  ;;       (list (float-vector 0 -10 0) (float-vector 0 10 0)))
  ;; (send *ri* :set-gait-generator-param
  ;;       ;;:toe-angle 30 :heel-angle 5
  ;;       :toe-angle 0 :heel-angle 0
  ;;       :toe-heel-phase-ratio #f(0.08 0.22 0.2 0.0 0.2 0.25 0.05)
  ;;       :stair-trajectory-way-point-offset #f(0.03 -0.0 0)
  ;;       :swing-trajectory-delay-time-offset 0.25
  ;;       :default-orbit-type :stair
  ;;       :default-step-time 1.5
  ;;       ;;:default-step-time 1.3
  ;;       :default-double-support-ratio 0.2
  ;;       ;;:default-double-support-ratio 0.16
  ;;       :default-step-height 0.05)
  )

;; (defun set-parameter-for-stair-jaxon ()
;;   (send *ri* :set-gait-generator-param
;;         :stair-trajectory-way-point-offset #f(0.03 -0.0 0)
;;         :swing-trajectory-delay-time-offset 0.25
;;         :default-orbit-type :stair
;;         :default-step-time 1.3
;;         ;; :default-double-support-ratio 0.32
;;         :default-double-support-ratio 0.16
;;         :default-step-height 0.03)
;;   (send *ri* :set-auto-balancer-param :default-zmp-offsets
;;         (list (float-vector 10 30 0) (float-vector 10 -30 0))))

(defun set-parameter-for-planar ()
  (cond
   ((or (string= *robot-name* "JAXON")
        (string= *robot-name* "JAXON_RED"))
    (set-parameter-for-planar-jaxon))
   (t
    (set-parameter-for-planar-hrp2))))

(defun set-parameter-for-planar-jaxon ()
  ;; (send *ri* :set-gait-generator-param
  ;;       ;;:default-orbit-type :cycloid
  ;;       :default-orbit-type :cycloiddelay
  ;;       :toe-angle 0 :heel-angle 0
  ;;       :default-step-time 1.2
  ;;       :default-double-support-ratio 0.32
  ;;       ;;:default-double-support-ratio 0.16
  ;;       :default-double-support-static-ratio 0.0
  ;;       :default-step-height 0.05)
  (send *ri* :set-gait-generator-param
        ;;:default-orbit-type :cycloid
        :default-orbit-type :stair
        :toe-angle 0 :heel-angle 0
        :default-step-time 1.7
        :default-double-support-ratio 0.29
        :default-double-support-static-ratio 0.117
        ;;:default-double-support-ratio 0.16
        :default-step-height 0.05)
  ;; [[0.010, 0.01, 0], [0.010, -0.01, 0]]
  ;; (send *ri* :set-auto-balancer-param :default-zmp-offsets
  ;;       (list (float-vector 10 10 0) (float-vector 10 -10 0)))
  )

(defun set-parameter-for-planar-hrp2 ()
  (send *ri* :set-gait-generator-param
        :default-orbit-type :cycloiddelay
        :default-step-time 1.1
        :toe-angle 0 :heel-angle 0
        :default-double-support-ratio 0.32
        :default-step-height 0.05)
  ;; [[0.010, 0.01, 0], [0.010, -0.01, 0]]
  ;; (send *ri* :set-auto-balancer-param :default-zmp-offsets
  ;;       (list (float-vector 10 10 0) (float-vector 10 -10 0)))
  )

(defun main()
  (ros::roseus "footstep_controller")
  ;; *ri*
  (setq *robot-name* (string-upcase (ros::get-param "~robot" (string-downcase (unix::getenv "ROBOT")))))
  (setq *interruptible-mode* (read-from-string (ros::get-param "~interruptible_mode" ":semi-interruptible")))
  (setq *auto-lookground* (ros::get-param "~auto_lookground" t))
  (load (format nil "package://hrpsys_ros_bridge_tutorials/euslisp/~A-interface.l"
                (string-downcase *robot-name*)))
  (init-robot-from-name *robot-name*)
  ;; interface
  (setq *tf* (instance ros::transform-listener :init))
  (ros::advertise-service "require_log"
    jsk_footstep_controller::RequireLog
    #'require-log-service-callback)
  (ros::advertise "breakpoint_text" jsk_rviz_plugins::OverlayText)
  (ros::subscribe "/footcoords/contact_state" 
                  jsk_footstep_controller::GroundContactState
                  #'contact-state-callback)
  (ros::advertise "/footstep_cancel_broadcast" std_msgs::Empty)
  ;; client self-client to cancel itself
  (unix:sleep 1)
  (publish-breakpoint-text " ")
  (let ((s (instance ros::simple-action-server :init
                     "footstep_controller"
                     jsk_footstep_msgs::ExecFootstepsAction
                     :execute-cb 'execute-cb
                     :preempt-cb 'preempt-cb)))
    (ros::rate 10)
    (ros::ros-info "footstep controller is ready")
    (while (ros::ok)
      (send s :worker)
      (ros::spin-once)
      (ros::sleep))))

(main)


#|
(setq aa '((#J(coordinates ((:name . :rleg)) #J(array nil #f(0.99987 0.002702 0.01588 -0.002847 0.999954 0.009123 -0.015855 -0.009167 0.999832) 2 nil 0 3 3 nil nil nil nil nil) #f(-2.5818 -134.762 -11.945)) #J(coordinates ((:name . :lleg)) #J(array nil #f(0.999716 0.000819 0.02382 -0.000794 0.999999 -0.001046 -0.023821 0.001027 0.999716) 2 nil 0 3 3 nil nil nil nil nil) #f(94.3242 52.2552 -16.8706)) #J(coordinates ((:name . :rleg)) #J(array nil #f(0.999716 0.000819 0.02382 -0.000794 0.999999 -0.001046 -0.023821 0.001027 0.999716) 2 nil 0 3 3 nil nil nil nil nil) #f(94.1522 -157.745 -17.0863)))))
(dotimes (i (1- (length footsteps)))
  (print (rad2deg (acos (v. (normalize-vector (send (elt footsteps i) :z-axis)) (normalize-vector (send (elt footsteps (1+ i)) :z-axis)))))
  )
|#


#|
(separate-footstep-with-poststeps
 (separate-by-footsteps (list (make-coords) 
                             (make-coords) (make-coords) (make-coords) (make-coords)
                             (make-coords :pos (float-vector 100 0 100))
                             (make-coords :pos (float-vector 200 0 100))
                             (make-coords :pos (float-vector 300 0 100))
                             (send (make-coords :pos (float-vector 300 0 100))
                                   :rotate (deg2rad 10) :x)
                             (send (make-coords :pos (float-vector 400 0 100))
                                   :rotate (deg2rad 10) :x)
                             (send (make-coords :pos (float-vector 500 0 100))
                                   :rotate (deg2rad 10) :x)
                             (send (make-coords :pos (float-vector 600 0 100))
                                   :rotate (deg2rad 10) :x)
                             )))
|#
