(load "../surface-configuration-task.l")
(load "../sqp-msc-optimization.l")
(require "models/arrow-object.l")


(defun sample-sqp-optimization-surface
    (&key
     (use-msc? nil)
     (initial-surface-pos (float-vector 0 0 0))
     (target-coords (make-coords :pos (float-vector 300 -200 300)))
     ((:body body0))
     (body-type :cube)
     (delta-u 0.02)
     (normal-task-scale 1)
     (norm-regular-scale-max 1e10)
     (norm-regular-scale-offset 1e0)
     (loop-num 50)
     (pre-process-func)
     (convergence-check-func)
     &allow-other-keys
     )
  ;; setup environment
  (cond (body0
         (setq *body* body0)
         )
        ((equal body-type :cube)
         (setq *body* (make-cube 200 400 100))
         )
        ((equal body-type :cone)
         (setq *body* (make-cone (float-vector 0 100 200) (list (float-vector 200 -100 0) (float-vector -200 -100 0) (float-vector 0 200 0))))
         )
        ((equal body-type :prism)
         (setq *body* (make-prism (list (float-vector 100 -100 0) (float-vector -100 -100 0) (float-vector 0 100 0)) 200))
         )
        ((equal body-type :ellipsoid)
         (setq *body* (make-ellipsoid))
         )
        ((equal body-type :link)
         (require :rhp3 "package://khi_ros_bridge_tutorials/euslisp/rhp3-interface.l")
         (setq *robot* (instance rhp3-robot :init))
         (setq *body* (elt (send (elt (send *robot* :links) 10) :bodies) 0))
         (setq *body* (convex-hull-3d (send (elt (send (elt (send *robot* :links) 10) :bodies) 0) :vertices)))
         (send *body* :translate (scale -1 (vector-mean (send *body* :vertices))))
         ))
  (setq *arrow* (instance arrow-object :init))
  (send *arrow* :newcoords target-coords)
  ;; draw
  (unless (boundp '*irtviewer*)
    (make-irtviewer)
    (send *irtviewer* :change-background (float-vector 1 1 1))
    (send *irtviewer* :draw-floor 500)
    (send *irtviewer* :floor-color (float-vector 0 0 0))
    )
  (objects (list *body* *arrow*))

  (setq *surface-config-task*
        (instance surface-configuration-task :init
                  :body *body*
                  :initial-surface-pos initial-surface-pos
                  :kin-target-coords (send *arrow* :copy-worldcoords)
                  :delta-u delta-u
                  :normal-task-scale normal-task-scale
                  :norm-regular-scale-max norm-regular-scale-max
                  :norm-regular-scale-offset norm-regular-scale-offset
                  ))

  (cond (use-msc?
         (setq *sqp-opt*
               (instance sqp-msc-optimization :init
                         :config-task *surface-config-task*
                         :dispersion-scale 1e2
                         :dispersion-scale-decrease-ratio 0.8
                         :pre-process-func
                         pre-process-func
                         ;; :post-process-func
                         ;; #'(lambda (sqp) (read-line))
                         ))
         (dolist (config-task (send *sqp-opt* :config-task-list))
           (send config-task :set-config
                 (gaussian-random 2 0 1e-2)
                 :relative? t
                 )
           )
         )
        (t
         (setq *sqp-opt*
               (instance sqp-optimization :init
                         :config-task *surface-config-task*
                         :pre-process-func
                         pre-process-func
                         ;; :post-process-func
                         ;; #'(lambda (sqp) (read-line))
                         :convergence-check-func
                         convergence-check-func
                         ))
         ))

  (send *sqp-opt* :optimize :loop-num loop-num :print-status-interval 1)
  )
(warn "(sample-sqp-optimization-surface)~%")
