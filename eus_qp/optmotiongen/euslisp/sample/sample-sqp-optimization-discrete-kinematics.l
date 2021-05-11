(load "../discrete-kinematics-configuration-task.l")
(load "../sqp-msc-optimization.l")
(load "../validate-jacobian.l")
(require "irteus/demo/sample-arm-model.l")
(require "models/arrow-object.l")


(defun sample-sqp-optimization-discrete-kinematics
    (&key
     (use-sqp-msc? nil)
     (pre-process-func)
     &allow-other-keys
     )
  (setq *robot* (instance sarmclass :init))
  (setq *robot-env*
        (instance robot-environment :init :robot *robot* :root-virtual-mode :fix))
  (setq *arrow-list* (mapcar #'(lambda (x) (instance arrow-object :init)) (make-list 3)))
  (unless (boundp '*irtviewer*)
    (make-irtviewer)
    (send *irtviewer* :change-background (float-vector 1 1 1))
    (send *irtviewer* :draw-floor 500)
    (send *irtviewer* :floor-color (float-vector 0 0 0))
    )
  (objects (append (list *robot*) *arrow-list*))

  (setq *time* 0.0)

  (do-until-key
   (setq *time* (+ *time* 0.1))
   (send (elt *arrow-list* 0) :newcoords
         (make-coords :pos (float-vector (+ (* 500 (cos (+ *time* pi/2))) 750) (* -200 (sin (+ *time* pi/2))) 300)
                      :rpy (list (deg2rad 20) 0 0)))
   (send (elt *arrow-list* 1) :newcoords
         (make-coords :pos (float-vector (+ (* 600 (cos *time*)) 200) -500 200)
                      :rpy (list (deg2rad 20) 0 0)))
   (send (elt *arrow-list* 2) :newcoords
         (make-coords :pos (float-vector 0 500 (* 500 (sin *time*)))
                      :rpy (list (deg2rad 20) 0 0)))

   (send *robot* :init-pose)
   (setq *instant-config-task*
         (instance discrete-kinematics-configuration-task :init
                   :robot-env *robot-env*
                   :drive-joint-list nil
                   :only-kinematics? t
                   :kin-target-coords-list
                   (list (send-all *arrow-list* :copy-worldcoords))
                   :kin-attention-coords-list
                   (list (make-list (length *arrow-list*) :initial-element (send *robot-env* :robot :end-coords)))
                   ))
   (setq *sqp-opt*
         (instance* (if use-sqp-msc? sqp-msc-optimization sqp-optimization) :init
                    (append
                     (list
                      :config-task *instant-config-task*
                      :pre-process-func pre-process-func
                      :post-process-func
                      #'(lambda (sqp)
                          (when (= (mod (send sqp :iteration) 20) 0)
                            ;; (validate-element-jacobian :config-task (send sqp :config-task))
                            ;; (validate-jacobian :config-task (send sqp :config-task))
                            ))
                      :no-print? t
                      :no-visualize? t
                      )
                     (if use-sqp-msc?
                         (list
                          :num-msc 2
                          :dispersion-scale 1e-1
                          :dispersion-scale-min 1e-10
                          :dispersion-scale-decrease-ratio 0.8
                          :config-vector-dist2-min 1e-15
                          )))
                    ))
   (when use-sqp-msc?
     (send *irtviewer* :objects (append *arrow-list* (send-all (send *sqp-opt* :config-task-list) :robot-env))))
   (send *sqp-opt* :optimize :loop-num 30)
   (send *irtviewer* :draw-objects)
   (unix::usleep (* 50 1000))
   (x::window-main-one)
   )
  t)
(warn "(sample-sqp-optimization-discrete-kinematics)~%")
