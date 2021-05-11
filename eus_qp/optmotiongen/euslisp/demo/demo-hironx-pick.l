(load "package://eus_qp/optmotiongen/euslisp/bspline-configuration-task.l")
(load "package://eus_qp/optmotiongen/euslisp/sqp-optimization.l")
(load "package://eus_qp/optmotiongen/euslisp/validate-jacobian.l")
(require :hironxjsk "package://hrpsys_ros_bridge_tutorials/models/hironxjsk.l")
(require "models/arrow-object.l")


(defun setup-robot-for-demo-hironx-pick
    ()
  (setq *robot* (instance hironxjsk-robot :init))

  (send (send *robot* :joint "CHEST_JOINT0") :max-joint-velocity 5.0)
  (send-all (send *robot* :joint-list) :put :max-joint-acceleration 100.0)
  (send (send *robot* :joint "CHEST_JOINT0") :put :max-joint-acceleration 30.0)
  )

(defun demo-hironx-pick
    (&key
     (avoid-collision? nil)
     (kin-task-list
      (list
       (list (make-coords :pos (float-vector 500 -250 0) :rpy (list 0 0 0)) 0.0 0.0)
       (list (make-coords :pos (float-vector 500 -250 100) :rpy (list 0 0 0)) 0.0 0.0)
       (list (make-coords :pos (float-vector 500 -250 200) :rpy (list 0 0 0)) 0.1 0.0)
       (list (make-coords :pos (float-vector 500 -100 200) :rpy (list 0 0 0)) 0.0 0.0)
       (list (make-coords :pos (float-vector 500 100 200) :rpy (list 0 0 0)) 0.0 0.0)
       (list (make-coords :pos (float-vector 500 250 200) :rpy (list 0 0 0)) 0.1 0.0)
       (list (make-coords :pos (float-vector 500 250 100) :rpy (list 0 0 0)) 0.0 0.0)
       (list (make-coords :pos (float-vector 500 250 0) :rpy (list 0 0 0)) 0.0 0.0)
       )
      ;; (list
      ;;  (list (make-coords :pos (float-vector 500 -300 0) :rpy (list 0 0 0)) 0.0 0.0)
      ;;  (list (make-coords :pos (float-vector 500 -300 150) :rpy (list 0 0 0)) 0.0 0.0)
      ;;  (list (make-coords :pos (float-vector 500 -300 300) :rpy (list 0 0 0)) 0.1 0.0)
      ;;  (list (make-coords :pos (float-vector 500 -100 300) :rpy (list 0 0 0)) 0.0 0.0)
      ;;  (list (make-coords :pos (float-vector 500 100 300) :rpy (list 0 0 0)) 0.0 0.0)
      ;;  (list (make-coords :pos (float-vector 500 300 300) :rpy (list 0 0 0)) 0.1 0.0)
      ;;  (list (make-coords :pos (float-vector 500 300 150) :rpy (list 0 0 0)) 0.0 0.0)
      ;;  (list (make-coords :pos (float-vector 500 300 0) :rpy (list 0 0 0)) 0.0 0.0)
      ;;  )
      )
     (max-motion-duration 1.5) ;; [sec]
     )
  ;; setup environment
  (setq *arrow-list* (mapcar #'(lambda (x) (instance arrow-object :init)) (range (length kin-task-list))))
  (dotimes (i (length kin-task-list))
    (send (elt *arrow-list* i) :newcoords (elt (elt kin-task-list i) 0))
    )
  ;; setup robot
  (setup-robot-for-demo-hironx-pick)
  ;; draw
  (unless (boundp '*irtviewer*)
    (make-irtviewer)
    (send *irtviewer* :change-background (float-vector 1 1 1))
    (send *irtviewer* :draw-floor 500)
    (send *irtviewer* :floor-color (float-vector 0 0 0))
    )
  (objects (append (list *robot*) *arrow-list*))

  (send *robot* :larm :elbow-p :joint-angle -150)
  (setq *bspline-config-task*
        (instance bspline-configuration-task :init
                  :name :demo-hironx-pick
                  :robot *robot*
                  :joint-list
                  (send-all (cdr (send *robot* :link-list (send *robot* :rarm-end-coords))) :joint)
                  :bspline-order 4
                  :start-time 0.0
                  :finish-time max-motion-duration
                  :num-control-point 40
                  :kin-time-list
                  (mapcar #'(lambda (i) (* (/ (float i) (1- (length kin-task-list))) max-motion-duration)) (range (length kin-task-list)))
                  :kin-variable-timing-list
                  (make-list (length kin-task-list) :initial-element t)
                  :kin-target-coords-list
                  (mapcar #'(lambda (x) (elt x 0)) kin-task-list)
                  :kin-attention-coords-list
                  (make-list (length kin-task-list) :initial-element (send *robot* :rarm :end-coords))
                  :kin-pos-tolerance-list
                  (mapcar #'(lambda (x) (elt x 1)) kin-task-list)
                  :kin-rot-tolerance-list
                  (mapcar #'(lambda (x) (elt x 2)) kin-task-list)
                  :joint-angle-margin 1.0
                  :keep-timing-order? t
                  :stationery-start-finish-task-scale 1e-1
                  :first-diff-square-integration-regular-scale 1e-8
                  :second-diff-square-integration-regular-scale 1e-8
                  :third-diff-square-integration-regular-scale 1e-8
                  :motion-duration-regular-scale 1e-2
                  :norm-regular-scale-max 1e-3
                  :norm-regular-scale-offset 1e-5
                  :timing-norm-regular-scale-max 1e-3
                  :timing-norm-regular-scale-offset 1e-5
                  ))

  (setq *sqp-opt*
        (instance sqp-optimization :init
                  :config-task *bspline-config-task*
                  :post-process-func
                  #'(lambda (sqp)
                      (when (= (mod (send sqp :iteration) 20) 0)
                        ;; (validate-element-jacobian-bspline :config-task (send sqp :config-task))
                        ;; (validate-jacobian :config-task (send sqp :config-task))
                        ))
                  ))
  (send *sqp-opt* :optimize :loop-num 30)
  (send *bspline-config-task* :plot-theta-graph :dump-pdf? t)
  (send *bspline-config-task* :play-animation :robot *robot*)
  )
(warn "(demo-hironx-pick)~%")
