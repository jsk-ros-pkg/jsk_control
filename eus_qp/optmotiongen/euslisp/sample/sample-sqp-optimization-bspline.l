(load "../bspline-configuration-task.l")
(load "../sqp-optimization.l")
(load "../validate-jacobian.l")
(require :HRP1 "package://hrpsys_ros_bridge_tutorials/models/pa10.l")
(require "models/arrow-object.l")


(defun setup-robot-for-sample-sqp-optimization-bspline
    ()
  (setq *robot* (instance hrp1-robot :init))
  (let* ((tip-body
          (make-cone (float-vector 0 0 50)
                     (mapcar
                      #'(lambda (i)
                          (scale 16 (float-vector (cos (* 2pi (/ i 10.0))) (sin (* 2pi (/ i 10.0))) 0)))
                      (range 16))))
         )
    (send tip-body :set-color (float-vector 0.4 0.4 0.1))
    (send tip-body :newcoords
          (send (send (send *robot* :rarm :end-coords) :copy-worldcoords) :translate (float-vector 0 0 160) :local))
    (send (send (send *robot* :rarm :end-coords) :parent) :assoc tip-body)
    (send (send *robot* :rarm :end-coords) :translate (float-vector 0 0 210) :local)
    (send *robot* :put :tip-body tip-body)
    (send *robot* :set-val 'bodies
          (append (send *robot* :get-val 'bodies) (list (send *robot* :get :tip-body))))
    )
  (send-all (send *robot* :joint-list) :max-joint-velocity 1.0)
  (send-all (send *robot* :joint-list) :put :max-joint-acceleration 2.0)
  )

(defun sample-sqp-optimization-bspline
    (&key
     (avoid-collision? nil)
     (kin-task-list
      (list
       (list 0.0 t (make-coords :pos (float-vector 600 -300 200) :rpy (list 0 0 pi)) 0.0 0.0)
       (list 1.0 t (make-coords :pos (float-vector 600 -300 350) :rpy (list 0 0 pi)) 0.0 0.0)
       (list 2.0 t (make-coords :pos (float-vector 600 -300 500) :rpy (list 0 0 pi)) 0.05 0.0)
       (list 2.25 t (make-coords :pos (float-vector 600 -100 500) :rpy (list 0 0 pi)) 0.0 0.0)
       (list 2.75 t (make-coords :pos (float-vector 600 100 500) :rpy (list 0 0 pi)) 0.0 0.0)
       (list 3.0 t (make-coords :pos (float-vector 600 300 500) :rpy (list 0 0 pi)) 0.05 0.0)
       (list 4.0 t (make-coords :pos (float-vector 600 300 350) :rpy (list 0 0 pi)) 0.0 0.0)
       (list 5.0 t (make-coords :pos (float-vector 600 300 200) :rpy (list 0 0 pi)) 0.0 0.0)
       ))
     (graph-filename
      (ros::resolve-ros-path "package://eus_qp/optmotiongen/logs/sample-sqp-optimization-bspline-graph.pdf"))
     (pre-process-func)
     (visualize-callback-func)
     (visualize-loop? t)
     &allow-other-keys
     )
  ;; setup environment
  (setq *arrow-list* (mapcar #'(lambda (x) (instance arrow-object :init)) (range (length kin-task-list))))
  (dotimes (i (length kin-task-list))
    (send (elt *arrow-list* i) :newcoords (elt (elt kin-task-list i) 2))
    )
  ;; setup robot
  (setup-robot-for-sample-sqp-optimization-bspline)
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
                  (make-list (length kin-task-list) :initial-element (send *robot* :rarm :end-coords))
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
  (send *sqp-opt* :optimize :loop-num 50)
  (when graph-filename
    (send *bspline-config-task* :plot-theta-graph :dump-pdf? t :dump-filename graph-filename))
  (send *bspline-config-task* :play-animation
        :robot *robot*
        :loop? visualize-loop?
        :visualize-callback-func visualize-callback-func
        )
  )
(warn "(sample-sqp-optimization-bspline)~%")
