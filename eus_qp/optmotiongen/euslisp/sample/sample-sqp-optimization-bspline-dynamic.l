(load "./sample-sqp-optimization-instant.l")
(load "../bspline-dynamic-configuration-task.l")


(defun sample-sqp-optimization-bspline-dynamic
    (&key
     ;; time
     (start-time 0.0)
     (finish-time 1.0)
     (jumping-time 0.4)
     (landing-time 0.6)
     ;; motion
     (landing-pos (float-vector 200 -200 0))
     ;; task-time-divide-num
     (kin-task-time-divide-num 20)
     (eom-task-time-divide-num 50)
     (centroid-task-time-divide-num 20)
     (posture-task-time-divide-num 5)
     ;; bspline setting
     (joint-num-control-point 10)
     (centroid-num-control-point 20)
     (bspline-order 4)
     (cog-bspline-order 4) ;; 5
     ;; other setting
     (fix-contact? nil)
     (optimize-torque? nil)
     (avoid-collision? nil)
     (pre-process-func)
     (visualize-callback-func)
     (visualize-loop? t)
     &allow-other-keys
     )
  ;; setup wall and arrow
  (setq *wall* (make-cube 500 500 20))
  (send *wall* :newcoords
        (make-coords :pos (float-vector 300 500 800) :rpy (list 0 (deg2rad -20) (deg2rad 20))))
  (send *wall* :set-color (float-vector 0.5 0.5 0.25))
  (send *wall* :worldcoords)
  (setq *arrow-rleg-start* (instance arrow-object :init))
  (send *arrow-rleg-start* :newcoords
        (make-coords :pos (float-vector 0 -105 0) :rpy (list 0 0 0)))
  (setq *arrow-lleg-start* (instance arrow-object :init))
  (send *arrow-lleg-start* :newcoords
        (make-coords :pos (float-vector 0 105 0) :rpy (list 0 0 0)))
  (setq *arrow-rleg-finish* (instance arrow-object :init))
  (send *arrow-rleg-finish* :newcoords
        (send (send *arrow-rleg-start* :copy-worldcoords) :translate landing-pos))
  (setq *arrow-lleg-finish* (instance arrow-object :init))
  (send *arrow-lleg-finish* :newcoords
        (send (send *arrow-lleg-start* :copy-worldcoords) :translate landing-pos))
  (setq *arrow-list*
        (list *arrow-rleg-start* *arrow-lleg-start* *arrow-rleg-finish* *arrow-lleg-finish*))
  (when avoid-collision?
    (setq *obst* (make-cube 200 200 500))
    (send *obst* :translate (float-vector 500 0 250))
    (send *obst* :set-color (float-vector 1 0 0) 0.5)
    )
  ;; setup robot
  (setup-hrp2jsknts-for-sample-sqp-optimization)
  (cond (fix-contact?
         (setq *robot-env*
               (instance robot-environment :init :robot *robot*))
         )
        (t
         (setq *contact-list*
               (list (instance 2d-planar-contact :init :name "larm")
                     ))
         (setq *robot-env*
               (instance robot-environment :init :robot *robot* :contact-list *contact-list*))
         (send *robot-env* :contact "larm" :set-from-face :face (elt (send *wall* :faces) 1))
         ))
  (send *robot-env* :put :wall *wall*)
  (send *robot-env* :put :arrow-rleg-start *arrow-rleg-start*)
  (send *robot-env* :put :arrow-lleg-start *arrow-lleg-start*)
  (send *robot-env* :put :arrow-rleg-finish *arrow-rleg-finish*)
  (send *robot-env* :put :arrow-lleg-finish *arrow-lleg-finish*)
  (send *robot-env* :put :jumping-time jumping-time)
  (send *robot-env* :put :landing-time landing-time)
  (send *robot-env* :put :rleg-kin-scale-mat (diagonal (float-vector 10 10 10 1 1 1)))
  (send *robot-env* :put :lleg-kin-scale-mat (diagonal (float-vector 10 10 10 1 1 1)))
  (send *robot-env* :put :larm-kin-scale-mat (diagonal (float-vector 10 10 10 1 1 1)))
  ;; setup contact constraint
  (send *robot-env* :put :rleg-contact-constraint
        (instance default-contact-constraint
                  :init-from-support-polygon
                  :mu-trans 0.5
                  :mu-rot 0.001
                  :max-fz 1000
                  :support-polygon (send *robot-env* :robot :get :rleg_link5-support-polygon)
                  :support-polygon-coords (send (send *robot-env* :robot :get :rleg-contact-coords) :worldcoords)
                  :support-polygon-margin 40
                  ))
  (send *robot-env* :put :lleg-contact-constraint
        (instance default-contact-constraint
                  :init-from-support-polygon
                  :mu-trans 0.5
                  :mu-rot 0.001
                  :max-fz 1000
                  :support-polygon (send *robot-env* :robot :get :lleg_link5-support-polygon)
                  :support-polygon-coords (send (send *robot-env* :robot :get :lleg-contact-coords) :worldcoords)
                  :support-polygon-margin 40
                  ))
  (send *robot-env* :put :larm-contact-constraint
        (instance default-contact-constraint
                  :init-from-support-polygon
                  :mu-trans 1.0
                  :mu-rot 0.001
                  :max-fz 1000
                  :support-polygon (send *robot-env* :robot :get :larm_link6-support-polygon)
                  :support-polygon-coords (send (send *robot-env* :robot :get :larm-contact-coords) :worldcoords)
                  :support-polygon-margin 0
                  ))
  (send *robot-env* :robot :reset-manip-pose)
  (send *robot-env* :robot :legs :move-end-pos (float-vector 0 0 100) :world)
  (send *robot-env* :robot :fix-leg-to-coords (make-coords))
  ;; draw
  (unless (boundp '*irtviewer*)
    (make-irtviewer)
    (send *irtviewer* :change-background (float-vector 1 1 1))
    (send *irtviewer* :set-val 'x::draw-floor 500)
    (when (assoc 'x::floor-color (send *irtviewer* :slots))
      (send *irtviewer* :set-val 'x::floor-color (float-vector 0 0 0)))
    )
  (objects (append (list *robot-env* *wall*) *arrow-list* (if avoid-collision? (list *obst*))))

  (setq *bspline-dynamic-config-task*
        (instance bspline-dynamic-configuration-task :init
                  :robot-env *robot-env*
                  :drive-joint-list
                  (if optimize-torque?
                      (send *robot-env* :drive-joint-list) nil)
                  :posture-joint-list
                  (send *robot-env* :variant-joint-list)
                  :posture-joint-angle-list
                  (send-all (send *robot-env* :variant-joint-list) :joint-angle)
                  :target-posture-scale 1e-1
                  :kin-task-time-list
                  (mapcar
                   #'(lambda (i)
                       (+ (* (/ (float i) (1- kin-task-time-divide-num)) (- finish-time start-time)) start-time))
                   (range kin-task-time-divide-num))
                  :eom-task-time-list
                  (mapcar
                   #'(lambda (i)
                       (+ (* (/ (float i) (1- eom-task-time-divide-num)) (- finish-time start-time)) start-time))
                   (range eom-task-time-divide-num))
                  :centroid-task-time-list
                  (mapcar
                   #'(lambda (i)
                       (+ (* (/ (float i) (1- centroid-task-time-divide-num)) (- finish-time start-time)) start-time))
                   (range centroid-task-time-divide-num))
                  :posture-task-time-list
                  (mapcar
                   #'(lambda (i)
                       (+ (* (/ (float i) (1- posture-task-time-divide-num)) (- finish-time start-time)) start-time))
                   (range posture-task-time-divide-num))
                  :theta-bst
                  (instance bspline-trajectory
                            :init
                            :start-time start-time
                            :finish-time finish-time
                            :num-control-point joint-num-control-point
                            :bspline-order bspline-order
                            :dim-instant-config (length (send *robot-env* :variant-joint-list))
                            :stationery-start-finish-regular-scale 1e2
                            :diff-square-integration-regular-scale 1e-2
                            )
                  :cog-bst
                  (instance bspline-trajectory
                            :init
                            :start-time start-time
                            :finish-time finish-time
                            :num-control-point centroid-num-control-point
                            :bspline-order cog-bspline-order
                            :dim-instant-config 3
                            :stationery-start-finish-regular-scale 1e6
                            :diff-square-integration-regular-scale 10.0
                            )
                  :ang-moment-bst
                  (instance bspline-trajectory
                            :init
                            :start-time start-time
                            :finish-time finish-time
                            :num-control-point centroid-num-control-point
                            :bspline-order bspline-order
                            :dim-instant-config 3
                            )
                  :wrench-bst
                  (instance bspline-trajectory
                            :init
                            :start-time start-time
                            :finish-time finish-time
                            :num-control-point centroid-num-control-point
                            :bspline-order bspline-order
                            :dim-instant-config (* 3 6)
                            :stationery-start-finish-regular-scale 1e-5
                            :diff-square-integration-regular-scale 1e-5
                            )
                  :torque-bst
                  (instance bspline-trajectory
                            :init
                            :start-time start-time
                            :finish-time finish-time
                            :num-control-point joint-num-control-point
                            :bspline-order bspline-order
                            :dim-instant-config 0
                            )
                  :kin-target-coords-list-func
                  (cond (fix-contact?
                         #'(lambda (robot-env tm)
                             (cond ((< tm (send robot-env :get :jumping-time))
                                    (list (send (send robot-env :get :arrow-rleg-start) :copy-worldcoords)
                                          (send (send robot-env :get :arrow-lleg-start) :copy-worldcoords)
                                          (send (send robot-env :get :wall) :copy-worldcoords)
                                          ))
                                   ((< tm (send robot-env :get :landing-time))
                                    (list (send (send robot-env :get :wall) :copy-worldcoords)
                                          ))
                                   (t
                                    (list (send (send robot-env :get :arrow-rleg-finish) :copy-worldcoords)
                                          (send (send robot-env :get :arrow-lleg-finish) :copy-worldcoords)
                                          (send (send robot-env :get :wall) :copy-worldcoords)
                                          )))
                             ))
                        (t
                         #'(lambda (robot-env tm)
                             (cond ((< tm (send robot-env :get :jumping-time))
                                    (list (send (send robot-env :get :arrow-rleg-start) :copy-worldcoords)
                                          (send (send robot-env :get :arrow-lleg-start) :copy-worldcoords)
                                          (send robot-env :contact "larm" :contact-coords)
                                          ))
                                   ((< tm (send robot-env :get :landing-time))
                                    (list (send robot-env :contact "larm" :contact-coords)
                                          ))
                                   (t
                                    (list (send (send robot-env :get :arrow-rleg-finish) :copy-worldcoords)
                                          (send (send robot-env :get :arrow-lleg-finish) :copy-worldcoords)
                                          (send robot-env :contact "larm" :contact-coords)
                                          )))
                             )))
                  :kin-attention-coords-list-func
                  #'(lambda (robot-env tm)
                      (cond ((< tm (send robot-env :get :jumping-time))
                             (list (send robot-env :robot :get :rleg-contact-coords)
                                   (send robot-env :robot :get :lleg-contact-coords)
                                   (send robot-env :robot :get :larm-contact-coords)
                                   ))
                            ((< tm (send robot-env :get :landing-time))
                             (list (send robot-env :robot :get :larm-contact-coords)
                                   ))
                            (t
                             (list (send robot-env :robot :get :rleg-contact-coords)
                                   (send robot-env :robot :get :lleg-contact-coords)
                                   (send robot-env :robot :get :larm-contact-coords)
                                   )))
                      )
                  :contact-target-coords-list-func
                  (cond (fix-contact?
                         #'(lambda (robot-env tm)
                             (cond ((< tm (send robot-env :get :jumping-time))
                                    (list (send (send robot-env :get :arrow-rleg-start) :copy-worldcoords)
                                          (send (send robot-env :get :arrow-lleg-start) :copy-worldcoords)
                                          (send (send robot-env :get :wall) :copy-worldcoords)
                                          ))
                                   ((< tm (send robot-env :get :landing-time))
                                    (list nil
                                          nil
                                          (send (send robot-env :get :wall) :copy-worldcoords)
                                          ))
                                   (t
                                    (list (send (send robot-env :get :arrow-rleg-finish) :copy-worldcoords)
                                          (send (send robot-env :get :arrow-lleg-finish) :copy-worldcoords)
                                          (send (send robot-env :get :wall) :copy-worldcoords)
                                          )))
                             ))
                        (t
                         #'(lambda (robot-env tm)
                             (cond ((< tm (send robot-env :get :jumping-time))
                                    (list (send (send robot-env :get :arrow-rleg-start) :copy-worldcoords)
                                          (send (send robot-env :get :arrow-lleg-start) :copy-worldcoords)
                                          (send robot-env :contact "larm" :contact-coords)
                                          ))
                                   ((< tm (send robot-env :get :landing-time))
                                    (list nil
                                          nil
                                          (send robot-env :contact "larm" :contact-coords)
                                          ))
                                   (t
                                    (list (send (send robot-env :get :arrow-rleg-finish) :copy-worldcoords)
                                          (send (send robot-env :get :arrow-lleg-finish) :copy-worldcoords)
                                          (send robot-env :contact "larm" :contact-coords)
                                          )))
                             )))
                  :contact-attention-coords-list-func
                  #'(lambda (robot-env tm)
                      (cond ((< tm (send robot-env :get :jumping-time))
                             (list (send robot-env :robot :get :rleg-contact-coords)
                                   (send robot-env :robot :get :lleg-contact-coords)
                                   (send robot-env :robot :get :larm-contact-coords)
                                   ))
                            ((< tm (send robot-env :get :landing-time))
                             (list nil
                                   nil
                                   (send robot-env :robot :get :larm-contact-coords)
                                   ))
                            (t
                             (list (send robot-env :robot :get :rleg-contact-coords)
                                   (send robot-env :robot :get :lleg-contact-coords)
                                   (send robot-env :robot :get :larm-contact-coords)
                                   )))
                      )
                  :contact-constraint-list-func
                  #'(lambda (robot-env tm)
                      (list (send robot-env :get :rleg-contact-constraint)
                            (send robot-env :get :lleg-contact-constraint)
                            (send robot-env :get :larm-contact-constraint)
                            ))
                  :collision-pair-list
                  (if avoid-collision?
                      (mapcar #'(lambda (l) (list l *obst*)) (flatten (send *robot-env* :robot :legs :links))))
                  :kin-scale-mat-list-func
                  #'(lambda (robot-env tm)
                      (cond ((< tm (send robot-env :get :jumping-time))
                             (list (send robot-env :get :rleg-kin-scale-mat)
                                   (send robot-env :get :lleg-kin-scale-mat)
                                   (send robot-env :get :larm-kin-scale-mat)
                                   ))
                            ((< tm (send robot-env :get :landing-time))
                             (list (send robot-env :get :larm-kin-scale-mat)
                                   ))
                            (t
                             (list (send robot-env :get :rleg-kin-scale-mat)
                                   (send robot-env :get :lleg-kin-scale-mat)
                                   (send robot-env :get :larm-kin-scale-mat)
                                   )))
                      )
                  :norm-regular-scale-max 1e-4
                  :norm-regular-scale-offset 1e-7
                  ))

  (setq *sqp-opt*
        (instance sqp-optimization :init
                  :config-task *bspline-dynamic-config-task*
                  :pre-process-func
                  pre-process-func
                  :post-process-func
                  #'(lambda (sqp)
                      (when (= (mod (send sqp :iteration) 5) 0)
                        ;; (validate-element-jacobian-bspline-dynamic :config-task (send sqp :config-task))
                        ;; (validate-jacobian :config-task (send sqp :config-task))
                        ))
                  ))
  (send *sqp-opt* :optimize
        :loop-num 10
        :print-status-interval 5
        )

  (send *bspline-dynamic-config-task* :play-animation
        :robot-env *robot-env*
        :loop? visualize-loop?
        :visualize-callback-func visualize-callback-func
        )
  )
(warn "(sample-sqp-optimization-bspline-dynamic)~%")
