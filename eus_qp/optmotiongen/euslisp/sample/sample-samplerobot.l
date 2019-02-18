(load "../bspline-configuration-task.l")
(load "../instant-manipulation-configuration-task.l")
(load "../sqp-optimization.l")
(load "../bspline-dynamic-configuration-task.l")
(require "irteus/demo/sample-arm-model.l")
(require "irteus/demo/sample-robot-model.l")
(require "models/arrow-object.l")
(require "models/room610-chair-object.l")


(defun sample-robot-sqp-instant-manip-config-task
    (&key
     (optimize-torque? t)
     (avoid-collision? t)
     (fix-obj? nil)
     (loop-num 50)
     (pre-process-func)
     &allow-other-keys
     )
  ;; setup robot
  (setq *robot* (instance sample-robot :init))
  (send *robot* :init-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  (create-support-polygon *robot* (send (send *robot* :rleg :end-coords) :parent))
  (create-support-polygon *robot* (send (send *robot* :lleg :end-coords) :parent))
  ;; setup object
  (setq *obj* (instance room610-chair-object :init))
  (send (car (send *obj* :links)) :weight (* 1 1e3)) ;; 1 kg
  (send-all (send *obj* :bodies) :set-color (float-vector 0.8 0.4 0.2))
  (send *obj* :put :object-coords
        (make-cascoords :parent (car (send *obj* :links)) :coords (send *obj* :copy-worldcoords)))
  (send (send *obj* :handle-side-larm)
        :transform (make-coords :pos (float-vector -60 0 0) :rpy (list pi/2 0 0)))
  (send (send *obj* :handle-side-rarm)
        :transform (make-coords :pos (float-vector -60 0 0) :rpy (list -pi/2 0 0)))
  (send *obj* :put :convex-hull-body
        (null-output (convex-hull-3d (send (car (send *obj* :bodies)) :vertices))))
  (send (car (send *obj* :links)) :assoc (send *obj* :get :convex-hull-body))
  (send *obj* :newcoords
        (make-coords :pos (float-vector 1000 0 0)))
  ;; setup environment
  (setq *ground-rleg* (make-cube 300 300 20))
  (send *ground-rleg* :set-color (float-vector 0.5 0.5 0.25))
  (send *ground-rleg* :newcoords
        (make-coords :pos (float-vector 0 -200 200) :rpy (list 0 0 (deg2rad -20))))
  (setq *arrow-lleg* (instance arrow-object :init))
  (send *arrow-lleg* :newcoords
        (make-coords :pos (float-vector 0 100 0) :rpy (list 0 0 0)))
  (when fix-obj?
    (setq *arrow-obj* (instance arrow-object :init))
    (send *arrow-obj* :newcoords
          (make-coords :pos (float-vector 600 0 100)))
    )
  (setq *env-list* (remove nil (list *ground-rleg* *arrow-lleg* (if fix-obj? *arrow-obj*))))
  (send-all *env-list* :worldcoords)
  ;; setup contact
  (setq *contact-list*
        (list (instance 2d-planar-contact :init :name "rleg")))
  (setq *robot-obj-env*
        (instance robot-object-environment :init :robot *robot* :object *obj* :contact-list *contact-list*))
  (send *robot-obj-env* :contact "rleg" :set-from-face :face (elt (send *ground-rleg* :faces) 1))
  (send *robot-obj-env* :contact "rleg" :rotate pi :z :local)
  ;; draw
  (unless (boundp '*irtviewer*)
    (make-irtviewer)
    (send *irtviewer* :change-background (float-vector 1 1 1))
    (send *irtviewer* :draw-floor 500)
    (send *irtviewer* :floor-color (float-vector 0 0 0))
    )
  (objects (append (list *robot* *obj*) *env-list*))

  (setq *instant-manip-config-task*
        (instance* instant-manipulation-configuration-task :init
                   :robot-env *robot-obj-env*
                   :drive-joint-list
                   (if optimize-torque? (send *robot-obj-env* :drive-joint-list) nil)
                   :posture-joint-list
                   (send *robot-obj-env* :robot :joint-list)
                   :posture-joint-angle-list
                   (send-all (send *robot-obj-env* :robot :joint-list) :joint-angle)
                   :kin-target-coords-list
                   (remove
                    nil
                    (list (send *robot-obj-env* :contact "rleg" :contact-coords)
                          (send *arrow-lleg* :copy-worldcoords)
                          (send *robot-obj-env* :object :handle-side-rarm)
                          (send *robot-obj-env* :object :handle-side-larm)
                          (if fix-obj? (send *arrow-obj* :copy-worldcoords))
                          ))
                   :kin-attention-coords-list
                   (remove
                    nil
                    (list (send *robot-obj-env* :robot :rleg-end-coords)
                          (send *robot-obj-env* :robot :lleg-end-coords)
                          (send *robot-obj-env* :robot :rarm-end-coords)
                          (send *robot-obj-env* :robot :larm-end-coords)
                          (if fix-obj? (send *robot-obj-env* :object :get :object-coords))
                          ))
                   :contact-target-coords-list
                   (list (send *robot-obj-env* :contact "rleg" :contact-coords)
                         (send *arrow-lleg* :copy-worldcoords)
                         (send *robot-obj-env* :object :handle-side-rarm)
                         (send *robot-obj-env* :object :handle-side-larm)
                         )
                   :contact-attention-coords-list
                   (list (send *robot-obj-env* :robot :rleg-end-coords)
                         (send *robot-obj-env* :robot :lleg-end-coords)
                         (send *robot-obj-env* :robot :rarm-end-coords)
                         (send *robot-obj-env* :robot :larm-end-coords)
                         )
                   :contact-constraint-list
                   (list
                    (instance default-contact-constraint
                              :init-from-support-polygon
                              :support-polygon (send (send (send *robot* :rleg :end-coords) :parent) :get :support-polygon)
                              :support-polygon-coords (send (send *robot* :rleg :end-coords) :worldcoords)
                              )
                    (instance default-contact-constraint
                              :init-from-support-polygon
                              :support-polygon (send (send (send *robot* :lleg :end-coords) :parent) :get :support-polygon)
                              :support-polygon-coords (send (send *robot* :lleg :end-coords) :worldcoords)
                              )
                    (instance 6d-min-max-contact-constraint
                              :init (float-vector 1000 1000 1000 10 10 10))
                    (instance 6d-min-max-contact-constraint
                              :init (float-vector 1000 1000 1000 10 10 10))
                    )
                   :contact-target-coords-obj-list
                   (list (send *robot-obj-env* :object :handle-side-rarm)
                         (send *robot-obj-env* :object :handle-side-larm)
                         )
                   :contact-constraint-obj-list
                   (list
                    (instance 6d-min-max-contact-constraint
                              :init (float-vector 1000 1000 1000 10 10 10))
                    (instance 6d-min-max-contact-constraint
                              :init (float-vector 1000 1000 1000 10 10 10))
                    )
                   :act-react-pair-list
                   (list
                    (list (send *robot-obj-env* :object :handle-side-rarm) (send *robot-obj-env* :object :handle-side-rarm))
                    (list (send *robot-obj-env* :object :handle-side-larm) (send *robot-obj-env* :object :handle-side-larm))
                    )
                   :collision-pair-list
                   (if avoid-collision?
                       (mapcar
                        #'(lambda (l)
                            (list l (send *robot-obj-env* :object :get :convex-hull-body)))
                        (flatten
                         (mapcar #'(lambda (limb) (send *robot-obj-env* :robot limb :links))
                                 (list :torso :rleg :lleg))
                         )))
                   :collision-distance-margin 0.05
                   :delta-linear-joint 100 ;; [mm]
                   :delta-rotational-joint 30 ;; [deg]
                   (if optimize-torque?
                       (list
                        :norm-regular-scale-max 1e-6
                        ))
                   ))

  (setq *sqp-opt*
        (instance sqp-optimization :init
                  :config-task *instant-manip-config-task*
                  :pre-process-func
                  pre-process-func
                  :post-process-func
                  #'(lambda (sqp)
                      (when (= (mod (send sqp :iteration) 20) 0)
                        ;; (validate-element-jacobian-manipulation :config-task (send sqp :config-task))
                        ;; (validate-jacobian :config-task (send sqp :config-task))
                        ))
                  ))
  (send *sqp-opt* :optimize :loop-num loop-num)
  )
(warn "(sample-robot-sqp-instant-manip-config-task)~%")


(defun sample-arm-sqp-bspline-config-task
    (&key
     (kin-task-list
      (list
       ;; list of timing, variable-timing?, target-coords, pos-tolerance, rot-tolerance
       (list 0.0 t (make-coords :pos (float-vector 400 -300 100)) 0.0 0.0)
       (list 1.0 t (make-coords :pos (float-vector 400 -300 250)) 0.0 0.0)
       (list 2.0 t (make-coords :pos (float-vector 400 -300 400)) 0.05 0.0)
       (list 2.25 t (make-coords :pos (float-vector 400 -100 400)) 0.0 0.0)
       (list 2.75 t (make-coords :pos (float-vector 400 100 400)) 0.0 0.0)
       (list 3.0 t (make-coords :pos (float-vector 400 300 400)) 0.05 0.0)
       (list 4.0 t (make-coords :pos (float-vector 400 300 250)) 0.0 0.0)
       (list 5.0 t (make-coords :pos (float-vector 400 300 100)) 0.0 0.0)
       )
      )
     (loop-num 50)
     (graph-filename
      (ros::resolve-ros-path "package://eus_qp/optmotiongen/logs/sample-arm-sqp-bspline-config-task-graph.pdf"))
     (pre-process-func)
     (visualize-callback-func)
     (visualize-loop? nil)
     &allow-other-keys
     )
  ;; setup environment
  (setq *arrow-list* (mapcar #'(lambda (x) (instance arrow-object :init)) (range (length kin-task-list))))
  (dotimes (i (length kin-task-list))
    (send (elt *arrow-list* i) :newcoords (elt (elt kin-task-list i) 2))
    )
  ;; setup robot
  (setq *robot* (instance sarmclass :init))
  (send-all (send *robot* :joint-list) :max-joint-velocity 1.0)
  (send-all (send *robot* :joint-list) :put :max-joint-acceleration 2.0)
  ;; draw
  (unless (boundp '*irtviewer*)
    (make-irtviewer)
    (send *irtviewer* :change-background (float-vector 1 1 1))
    (send *irtviewer* :draw-floor 500)
    (send *irtviewer* :floor-color (float-vector 0 0 0))
    )
  (objects (append (list *robot*) *arrow-list*))

  (send-all (send *robot* :joint-list) :joint-angle (rad2deg 1.0))
  (setq *bspline-config-task*
        (instance bspline-configuration-task :init
                  :name :sample-sqp-optimization-bspline
                  :robot *robot*
                  :start-time 0.0
                  :finish-time 5.0
                  :num-control-point 20
                  :kin-time-list
                  (mapcar #'(lambda (x) (elt x 0)) kin-task-list)
                  :kin-variable-timing-list
                  (mapcar #'(lambda (x) (elt x 1)) kin-task-list)
                  :kin-target-coords-list
                  (mapcar #'(lambda (x) (elt x 2)) kin-task-list)
                  :kin-attention-coords-list
                  (make-list (length kin-task-list) :initial-element (send *robot* :end-coords))
                  :kin-pos-tolerance-list
                  (mapcar #'(lambda (x) (elt x 3)) kin-task-list)
                  :kin-rot-tolerance-list
                  (mapcar #'(lambda (x) (elt x 4)) kin-task-list)
                  :keep-timing-order? t
                  :stationery-start-finish-task-scale 1e-2
                  :first-diff-square-integration-regular-scale 1e-8
                  :second-diff-square-integration-regular-scale 1e-8
                  :third-diff-square-integration-regular-scale 1e-8
                  :motion-duration-regular-scale 1e-6
                  ))

  (setq *sqp-opt*
        (instance sqp-optimization :init
                  :config-task *bspline-config-task*
                  :pre-process-func
                  pre-process-func
                  :post-process-func
                  #'(lambda (sqp)
                      (when (= (mod (send sqp :iteration) 20) 0)
                        ;; (validate-element-jacobian-bspline :config-task (send sqp :config-task))
                        ;; (validate-jacobian :config-task (send sqp :config-task))
                        ))
                  ))
  (send *sqp-opt* :optimize :loop-num loop-num)
  (when graph-filename
    (send *bspline-config-task* :plot-theta-graph :dump-pdf? t :dump-filename graph-filename))
  (send *bspline-config-task* :play-animation
        :robot *robot*
        :loop? visualize-loop?
        :visualize-callback-func visualize-callback-func
        )
  )
(warn "(sample-arm-sqp-bspline-config-task)~%")


(defun sample-robot-sqp-bspline-dynamic-config-task
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
     (loop-num 10)
     (optimize-torque? nil)
     (avoid-collision? nil)
     (graph-filename
      (ros::resolve-ros-path "package://eus_qp/optmotiongen/logs/sample-robot-sqp-bspline-dynamic-config-task/graph.pdf"))
     (pre-process-func)
     (visualize-callback-func)
     (visualize-loop? nil)
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
  (setq *robot* (instance sample-robot :init))
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
                  :support-polygon-coords (send (send *robot-env* :robot :rleg-end-coords) :worldcoords)
                  :support-polygon-margin 40
                  ))
  (send *robot-env* :put :lleg-contact-constraint
        (instance default-contact-constraint
                  :init-from-support-polygon
                  :mu-trans 0.5
                  :mu-rot 0.001
                  :max-fz 1000
                  :support-polygon (send *robot-env* :robot :get :lleg_link5-support-polygon)
                  :support-polygon-coords (send (send *robot-env* :robot :lleg-end-coords) :worldcoords)
                  :support-polygon-margin 40
                  ))
  (send *robot-env* :robot :reset-pose)
  (send *robot-env* :robot :legs :move-end-pos (float-vector 0 0 100) :world)
  (send *robot-env* :robot :fix-leg-to-coords (make-coords))
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
                             (list (send robot-env :robot :rleg-end-coords)
                                   (send robot-env :robot :lleg-end-coords)
                                   ))
                            ((< tm (send robot-env :get :landing-time))
                             (list ))
                            (t
                             (list (send robot-env :robot :rleg-end-coords)
                                   (send robot-env :robot :lleg-end-coords)
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
                             (list (send robot-env :robot :rleg-end-coords)
                                   (send robot-env :robot :lleg-end-coords)
                                   ))
                            ((< tm (send robot-env :get :landing-time))
                             (list nil
                                   nil
                                   ))
                            (t
                             (list (send robot-env :robot :rleg-end-coords)
                                   (send robot-env :robot :lleg-end-coords)
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
        :loop-num loop-num
        :print-status-interval 5
        )

  (when graph-filename
    (send *bspline-dynamic-config-task* :generate-graph :graph-filename graph-filename))

  (send *bspline-dynamic-config-task* :play-animation
        :robot-env *robot-env*
        :loop? visualize-loop?
        :visualize-callback-func visualize-callback-func
        )
  )
(warn "(sample-robot-sqp-bspline-dynamic-config-task)~%")
