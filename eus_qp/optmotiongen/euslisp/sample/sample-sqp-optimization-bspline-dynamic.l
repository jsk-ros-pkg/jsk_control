(load "./sample-sqp-optimization-instant.l")
(load "../bspline-dynamic-configuration-task.l")


(defun sample-sqp-optimization-bspline-dynamic
    (&key
     ;; time
     (start-time 0.0)
     (finish-time 1.0)
     (jumping-time 0.45)
     (landing-time 0.55)
     ;; motion
     (landing-pos (float-vector 200 0 0))
     ;; task-time-divide-num
     (kin-task-time-divide-num 10)
     (eom-task-time-divide-num 50)
     (centroid-task-time-divide-num 20)
     (posture-task-time-divide-num 5)
     ;; bspline setting
     (joint-num-control-point 10)
     (centroid-num-control-point 20)
     (bspline-order 4)
     (cog-bspline-order 5)
     ;; other setting
     (optimize-torque? nil)
     (avoid-collision? nil)
     (graph-filename
      (ros::resolve-ros-path "package://eus_qp/optmotiongen/logs/sample-sqp-optimization-bspline-dynamic/graph.pdf"))
     (output-filename
      (ros::resolve-ros-path "package://eus_qp/optmotiongen/logs/sample-sqp-optimization-bspline-dynamic/tmp"))
     (pre-process-func)
     (visualize-callback-func)
     (visualize-loop? t)
     &allow-other-keys
     )
  ;; setup arrow
  (setq *arrow-rleg* (instance arrow-object :init))
  (send *arrow-rleg* :newcoords
        (make-coords :pos (float-vector 0 -105 0) :rpy (list 0 0 0)))
  (setq *arrow-lleg* (instance arrow-object :init))
  (send *arrow-lleg* :newcoords
        (make-coords :pos (float-vector 0 105 0) :rpy (list 0 0 0)))
  (setq *arrow-rleg-landing* (instance arrow-object :init))
  (send *arrow-rleg-landing* :newcoords
        (send (send *arrow-rleg* :copy-worldcoords) :translate landing-pos))
  (setq *arrow-lleg-landing* (instance arrow-object :init))
  (send *arrow-lleg-landing* :newcoords
        (send (send *arrow-lleg* :copy-worldcoords) :translate landing-pos))
  (setq *arrow-list*
        (list *arrow-rleg* *arrow-lleg* *arrow-rleg-landing* *arrow-lleg-landing*))
  ;; setup robot
  (setup-hrp2jsknts-for-sample-sqp-optimization)
  (setq *robot-env*
        (instance robot-environment :init :robot *robot*))
  (send *robot-env* :put :max-angular-moment (float-vector 1.0 1.0 1.0))
  (send *robot-env* :put :arrow-rleg *arrow-rleg*)
  (send *robot-env* :put :arrow-lleg *arrow-lleg*)
  (send *robot-env* :put :arrow-rleg-landing *arrow-rleg-landing*)
  (send *robot-env* :put :arrow-lleg-landing *arrow-lleg-landing*)
  (send *robot-env* :put :jumping-time jumping-time)
  (send *robot-env* :put :landing-time landing-time)
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
  (send *robot-env* :robot :reset-pose)
  (send *robot-env* :robot :legs :move-end-pos (float-vector 0 0 200) :world)
  ;; draw
  (unless (boundp '*irtviewer*)
    (make-irtviewer)
    (send *irtviewer* :change-background (float-vector 1 1 1))
    (send *irtviewer* :draw-floor 500)
    (send *irtviewer* :floor-color (float-vector 0 0 0))
    )
  (objects (append (list *robot-env*) *arrow-list* (if avoid-collision? (list *obst*))))

  (setq *bspline-dynamic-config-task*
        (instance bspline-dynamic-configuration-task :init
                  :robot-env *robot-env*
                  :drive-joint-list
                  (if optimize-torque?
                      (send *robot-env* :drive-joint-list) nil)
                  :posture-joint-list
                  (subseq (send *robot-env* :variant-joint-list) 6)
                  :posture-joint-angle-list
                  (subseq (send-all (send *robot-env* :variant-joint-list) :joint-angle) 6)
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
                            :stationery-start-finish-regular-scale 1e6
                            :zero-diff-stationery-start-finish-regular-scale 1e6
                            :diff-square-integration-regular-scale 1.0
                            )
                  :wrench-bst
                  (instance bspline-trajectory
                            :init
                            :start-time start-time
                            :finish-time finish-time
                            :num-control-point centroid-num-control-point
                            :bspline-order bspline-order
                            :dim-instant-config (* 2 6)
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
                  #'(lambda (robot-env tm)
                      (cond ((< tm (send robot-env :get :jumping-time))
                             (list (send (send robot-env :get :arrow-rleg) :copy-worldcoords)
                                   (send (send robot-env :get :arrow-lleg) :copy-worldcoords)
                                   ))
                            ((< tm (send robot-env :get :landing-time))
                             (list ))
                            (t
                             (list (send (send robot-env :get :arrow-rleg-landing) :copy-worldcoords)
                                   (send (send robot-env :get :arrow-lleg-landing) :copy-worldcoords)
                                   )))
                      )
                  :kin-attention-coords-list-func
                  #'(lambda (robot-env tm)
                      (cond ((< tm (send robot-env :get :jumping-time))
                             (list (send robot-env :robot :get :rleg-contact-coords)
                                   (send robot-env :robot :get :lleg-contact-coords)
                                   ))
                            ((< tm (send robot-env :get :landing-time))
                             (list ))
                            (t
                             (list (send robot-env :robot :get :rleg-contact-coords)
                                   (send robot-env :robot :get :lleg-contact-coords)
                                   )))
                      )
                  :contact-target-coords-list-func
                  #'(lambda (robot-env tm)
                      (cond ((< tm (send robot-env :get :jumping-time))
                             (list (send (send robot-env :get :arrow-rleg) :copy-worldcoords)
                                   (send (send robot-env :get :arrow-lleg) :copy-worldcoords)
                                   ))
                            ((< tm (send robot-env :get :landing-time))
                             (list nil
                                   nil
                                   ))
                            (t
                             (list (send (send robot-env :get :arrow-rleg-landing) :copy-worldcoords)
                                   (send (send robot-env :get :arrow-lleg-landing) :copy-worldcoords)
                                   )))
                      )
                  :contact-attention-coords-list-func
                  #'(lambda (robot-env tm)
                      (cond ((< tm (send robot-env :get :jumping-time))
                             (list (send robot-env :robot :get :rleg-contact-coords)
                                   (send robot-env :robot :get :lleg-contact-coords)
                                   ))
                            ((< tm (send robot-env :get :landing-time))
                             (list nil
                                   nil
                                   ))
                            (t
                             (list (send robot-env :robot :get :rleg-contact-coords)
                                   (send robot-env :robot :get :lleg-contact-coords)
                                   ))))
                  :contact-constraint-list-func
                  #'(lambda (robot-env tm)
                      (list (send robot-env :get :rleg-contact-constraint)
                            (send robot-env :get :lleg-contact-constraint)
                            ))
                  :collision-pair-list
                  (if avoid-collision?
                      (mapcar #'(lambda (l) (list l *obst*)) (flatten (send *robot-env* :robot :legs :links))))
                  :kin-task-scale 1e1
                  :cog-task-scale 1e3
                  :ang-moment-task-scale 0.0
                  :posture-task-scale 5e-1
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

  (when graph-filename
    (send *bspline-dynamic-config-task* :generate-graph :graph-filename graph-filename))

  (when output-filename
    (unix:system
     (format nil "DIRNAME=`dirname ~a`; if [ ! -d $DIRNAME ]; then mkdir -p $DIRNAME; fi" output-filename))
    (setq *rs-list*
          (send *bspline-dynamic-config-task* :generate-robot-state-list
                :robot-env *robot-env*
                :joint-name-list (send-all (send *robot-env* :robot :joint-list-orig) :name)
                :root-link-name (send (car (send *robot-env* :robot :links)) :name)
                :step-time 0.004
                ))
    (play-robot-state-list
     :rs-list *rs-list*
     :robot-env *robot-env*
     :joint-name-list (send-all (send *robot-env* :robot :joint-list-orig) :name)
     :root-link-name (send (car (send *robot-env* :robot :links)) :name)
     )
    (dump-loadable-structure (format nil "~a.l" output-filename) *rs-list*)
    (dump-seq-pattern-file
     *rs-list*
     output-filename
     :robot (send *robot-env* :robot)
     ))

  (send *bspline-dynamic-config-task* :play-animation
        :robot-env *robot-env*
        :loop? visualize-loop?
        :visualize-callback-func visualize-callback-func
        )
  )
(warn "(sample-sqp-optimization-bspline-dynamic)~%")
