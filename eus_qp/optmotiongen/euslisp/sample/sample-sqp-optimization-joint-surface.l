(load "../joint-surface-configuration-task.l")
(load "../sqp-msc-optimization.l")
(load "../inverse-kinematics-wrapper.l")
(require "irteus/demo/sample-robot-model.l")
(require "models/arrow-object.l")


(defun sample-sqp-optimization-joint-surface
    (&key
     (use-msc? nil)
     (target-coords (make-coords :pos (float-vector 1300 -800 600) :rpy (list 0 0 0)))
     (only-kinematics? t)
     (loop-num 50)
     (rotation-normal? t) ;; rotation-normal? nil is not supported
     (pre-process-func)
     &allow-other-keys
     )
  ;; setup robot
  (setq *robot* (instance sample-robot :init))
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  (create-support-polygon *robot* (send (send *robot* :rleg :end-coords) :parent))
  (create-support-polygon *robot* (send (send *robot* :lleg :end-coords) :parent))
  (send *robot* :put :body
        (make-prism (list (float-vector 100 -100 0) (float-vector -100 -100 0) (float-vector 0 100 0)) 300))
  (send (send *robot* :get :body) :set-color #f(1 1 0))
  (send (send *robot* :get :body) :newcoords (send (send *robot* :rarm :end-coords) :copy-worldcoords))
  (send (send (send *robot* :rarm :end-coords) :parent) :assoc (send *robot* :get :body))
  (send *robot* :set-val 'bodies
        (append (send *robot* :get-val 'bodies) (list (send *robot* :get :body))))
  (send (send *robot* :rarm :end-coords) :name :rarm-end-coords) ;; necessary for joint-surface-configuration-task
  ;; set up environment
  (setq *arrow* (instance arrow-object :init))
  (send *arrow* :newcoords target-coords)
  ;; setup contact
  (setq *robot-env*
        (instance robot-environment :init
                  :robot *robot*
                  :contact-list
                  (if (not rotation-normal?)
                      (list
                       (instance contact-ik-arg :init
                                 :name "rarm"
                                 :target-coords (send *arrow* :copy-worldcoords)
                                 :translation-axis t
                                 :rotation-axis :z
                                 )))
                  :root-virtual-mode :6dof
                  ))
  ;; draw
  (unless (boundp '*irtviewer*)
    (make-irtviewer)
    (send *irtviewer* :change-background (float-vector 1 1 1))
    (send *irtviewer* :draw-floor 500)
    (send *irtviewer* :floor-color (float-vector 0 0 0))
    )
  (objects (list *robot* *arrow*))

  (setq *instant-config-task*
        (instance instant-configuration-task :init
                  :robot-env *robot-env*
                  :drive-joint-list nil
                  :only-kinematics? only-kinematics?
                  :kin-target-coords-list
                  (list (send (send *robot* :rleg :end-coords) :copy-worldcoords)
                        (send (send *robot* :lleg :end-coords) :copy-worldcoords)
                        (cond (rotation-normal?
                               (send *arrow* :copy-worldcoords))
                              (t
                               (send *robot-env* :contact "rarm" :contact-coords)))
                        )
                  :kin-attention-coords-list
                  (list (send *robot* :rleg :end-coords)
                        (send *robot* :lleg :end-coords)
                        (send *robot* :rarm :end-coords)
                        )
                  :kin-rotation-type-list
                  (if rotation-normal? (list :axis-angle :axis-angle :normal))
                  :contact-target-coords-list
                  (if (not only-kinematics?)
                      (list (send (send *robot* :rleg :end-coords) :copy-worldcoords)
                            (send (send *robot* :lleg :end-coords) :copy-worldcoords)
                            (cond (rotation-normal?
                                   (send *arrow* :copy-worldcoords))
                                  (t
                                   (send *robot-env* :contact "rarm" :contact-coords)))
                            ))
                  :contact-attention-coords-list
                  (if (not only-kinematics?)
                      (list (send *robot* :rleg :end-coords)
                            (send *robot* :lleg :end-coords)
                            (send *robot* :rarm :end-coords)
                            ))
                  :contact-constraint-list
                  (if (not only-kinematics?)
                      (list
                       (instance default-contact-constraint
                                 :init-from-support-polygon
                                 :support-polygon (send (send (send *robot-env* :robot :rleg :end-coords) :parent) :get :support-polygon)
                                 :support-polygon-coords (send (send *robot-env* :robot :rleg :end-coords) :worldcoords)
                                 )
                       (instance default-contact-constraint
                                 :init-from-support-polygon
                                 :support-polygon (send (send (send *robot-env* :robot :lleg :end-coords) :parent) :get :support-polygon)
                                 :support-polygon-coords (send (send *robot-env* :robot :lleg :end-coords) :worldcoords)
                                 )
                       (instance 6d-min-max-contact-constraint
                                 :init (float-vector 20 20 100 0 0 0))
                       ))
                  :norm-regular-scale-max 1e-2
                  :norm-regular-scale-offset 1e-6
                  ))

  (setq *surface-config-task*
        (instance surface-configuration-task :init
                  :body (send *robot* :get :body)
                  :initial-surface-pos (send (send *robot-env* :robot :rarm-end-coords) :worldpos)
                  :kin-target-coords
                  (cond (rotation-normal?
                         (send *arrow* :copy-worldcoords))
                        (t
                         (send *robot-env* :contact "rarm" :contact-coords)))
                  ))

  (setq *joint-surface-config-task*
        (instance joint-surface-configuration-task :init
                  :instant-config-task *instant-config-task*
                  :surface-kin-contact-list
                  (list
                   (list *surface-config-task* :rarm-end-coords (if (not only-kinematics?) :rarm-end-coords)))
                  :instant-config-distance-scale 0.01
                  ))

  (cond (use-msc?
         (setq *sqp-opt*
               (instance sqp-msc-optimization :init
                         :config-task *joint-surface-config-task*
                         :dispersion-scale 1e1
                         :dispersion-scale-decrease-ratio 0.8
                         :pre-process-func
                         pre-process-func
                         ;; :post-process-func
                         ;; #'(lambda (sqp) (read-line))
                         ))
         (dolist (config-task (send *sqp-opt* :config-task-list))
           (send config-task :set-config
                 (gaussian-random (send *joint-surface-config-task* :dim-config) 0 1e-2)
                 :relative? t
                 )
           )
         (objects (append
                   (send-all (send-all (send-all (send *sqp-opt* :config-task-list)
                                                 :instant-config-task) :robot-env) :robot)
                   (list *arrow*)
                   ))
         )
        (t
         (setq *sqp-opt*
               (instance sqp-optimization :init
                         :config-task *joint-surface-config-task*
                         :pre-process-func
                         pre-process-func
                         ;; :post-process-func
                         ;; #'(lambda (sqp) (read-line))
                         ))
         ))

  (send *sqp-opt* :optimize :loop-num loop-num :print-status-interval 1)

  (when use-msc?
    (send *sqp-opt* :draw-each-configuration :additional-objects (list *arrow*)))
  )
(warn "(sample-sqp-optimization-joint-surface)~%")
