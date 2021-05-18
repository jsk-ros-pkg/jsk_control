(load "package://eus_qp/optmotiongen/euslisp/joint-surface-configuration-task.l")
(load "package://eus_qp/optmotiongen/euslisp/inverse-kinematics-wrapper.l")
(load "package://eus_qp/optmotiongen/euslisp/instant-configuration-task.l")
(load "package://eus_qp/optmotiongen/euslisp/sqp-msc-optimization.l")
(load "package://eus_qp/optmotiongen/euslisp/validate-jacobian.l")
(require :HRP1 "package://hrpsys_ros_bridge_tutorials/models/pa10.l")
(require "models/arrow-object.l")


(defun setup-robot-for-demo-pa10-surface-contact
    ()
  (setq *robot* (instance hrp1-robot :init))
  (send *robot* :init-pose)
  (let* ((tip-body (make-ellipsoid 150 150 250))
         )
    (send tip-body :set-color (float-vector 0.4 0.4 0.1))
    (send tip-body :newcoords
          (send (send (send *robot* :link "J7_LINK") :copy-worldcoords) :translate (float-vector 0 0 350) :local))
    (send (send (send *robot* :rarm :end-coords) :parent) :assoc tip-body)
    (send *robot* :put :tip-body tip-body)
    (send *robot* :set-val 'bodies
          (append (send *robot* :get-val 'bodies) (list (send *robot* :get :tip-body))))
    )
  )

(defun demo-pa10-surface-contact
    (&key
     (use-msc? nil)
     (cube-surface? t)
     (loop-num 50)
     (pre-process-func)
     &allow-other-keys
     )
  ;; setup robot
  (setup-robot-for-demo-pa10-surface-contact)
  ;; setup object
  (cond (cube-surface?
         (setq *obj* (make-cube 500 500 500))
         )
        (t
         (setq *obj* (make-cube 500 500 10))
         ))
  (send *obj* :newcoords
        (make-coords :pos (float-vector 750 750 250) :rpy (list 0 (deg2rad -30) 0)))
  (send *obj* :set-color #f(0.6 0.4 0.3))
  ;; generate robot-environment
  (cond (cube-surface?
         (setq *contact-list* nil)
         )
        (t
         (setq *contact-list*
               (list (instance 2d-planar-contact :init :name "object" :fix-yaw? t)))
         ))
  (setq *robot-env*
        (instance robot-environment :init
                  :robot *robot*
                  :contact-list *contact-list*
                  :root-virtual-mode :fix
                  ))
  (cond (cube-surface?
         (send *obj* :put :obj-coords
               (make-cascoords :name :obj-coords
                               :parent *obj*
                               :coords (send *obj* :copy-worldcoords)
                               ))
         )
        (t
         (send *obj* :worldcoords)
         (send *robot-env* :contact "object" :set-from-face :face (elt (send *obj* :faces) 1) :margin 0.0)
         (send (send *robot-env* :contact "object" :contact-coords) :rotate pi :x) ;; flip normal
         (send *obj* :assoc (send *robot-env* :contact "object"))
         ))
  ;; draw
  (unless (boundp '*irtviewer*)
    (make-irtviewer)
    (send *irtviewer* :change-background (float-vector 1 1 1))
    (send *irtviewer* :draw-floor 500)
    (send *irtviewer* :floor-color (float-vector 0 0 0))
    )
  (objects (list *robot* *obj*))

  (cond (cube-surface?
         (setq *target-coords* (send *obj* :get :obj-coords))
         )
        (t
         (setq *target-coords* (send *robot-env* :contact "object" :contact-coords))
         ))
  (setq *attention-coords*
        (send *robot* :rarm-end-coords))
  (setq *instant-config-task*
        (instance instant-configuration-task :init
                  :only-kinematics? t
                  :drive-joint-list nil
                  :robot-env *robot-env*
                  :kin-target-coords-list (list *target-coords*)
                  :kin-attention-coords-list (list *attention-coords*)
                  :kin-scale-list (list (list 1 1 1 2))
                  :kin-rotation-type-list (list :normal)
                  :norm-regular-scale-max 1e20
                  :norm-regular-scale-offset 1e-3
                  :delta-rotational-joint 2.0
                  ))

  (setq *tip-surface-config-task*
        (instance surface-configuration-task :init
                  :body (send *robot-env* :robot :get :tip-body)
                  :initial-surface-pos
                  (send *target-coords* :worldpos)
                  :kin-target-coords
                  '(lambda () (car (send *instant-config-task* :kin-target-coords-list)))
                  :delta-u 1.0
                  :flip-normal? nil
                  :normal-task-scale 20
                  :norm-regular-scale-max 1e20
                  :norm-regular-scale-offset 1e-2
                  ))

  (when cube-surface?
    (setq *cube-surface-config-task*
          (instance surface-configuration-task :init
                    :body *obj*
                    :initial-surface-pos
                    (send *attention-coords* :worldpos)
                    :kin-target-coords
                    '(lambda () (car (send *instant-config-task* :kin-attention-coords-list)))
                    :delta-u 1.0
                    :flip-normal? t
                    :normal-task-scale 4
                    :norm-regular-scale-max 1e20
                    :norm-regular-scale-offset 1e-3
                    ))
    )

  (setq *joint-surface-config-task*
        (instance joint-surface-configuration-task :init
                  :instant-config-task *instant-config-task*
                  :surface-kin-contact-list
                  (remove
                   nil
                   (list
                    (list *tip-surface-config-task* :rarm-end-coords nil :attention)
                    (if cube-surface?
                        (list *cube-surface-config-task* :obj-coords nil :target))
                    ))
                  :instant-config-distance-scale 10
                  ))

  (cond (use-msc?
         (setq *sqp-opt*
               (instance sqp-msc-optimization :init
                         :num-msc 3
                         :dispersion-scale 1e5
                         :dispersion-scale-decrease-ratio 0.5
                         :config-task *joint-surface-config-task* ;; *instant-config-task*
                         :pre-process-func
                         pre-process-func
                         :post-process-func
                         #'(lambda (sqp)
                             ;; (read-line)
                             (when (= (mod (send sqp :iteration) 10) 0)
                               (warning-message 3 "iteration: ~a~%" (send sqp :iteration))
                               (dotimes (i (length (send sqp :config-task-list)))
                                 (warning-message 3 " config-task [~a/~a]~%" (1+ i) (length (send sqp :config-task-list)))
                                 (output-task-error (elt (send sqp :config-task-list) i)))
                               )
                             )
                         ))
         (objects
          (append (send-all (send-all (send *sqp-opt* :config-task-list) :instant-config-task) :robot-env)
                  (list *obj*)))
         )
        (t
         (setq *sqp-opt*
               (instance sqp-optimization :init
                         :config-task *joint-surface-config-task* ;; *instant-config-task*
                         :pre-process-func
                         pre-process-func
                         :post-process-func
                         #'(lambda (sqp)
                             ;; (read-line)
                             (when (= (mod (send sqp :iteration) 10) 0)
                               (warning-message 3 "iteration: ~a~%" (send sqp :iteration))
                               (output-task-error (send sqp :config-task)))
                             )
                         ))
         ))
  (send *sqp-opt* :optimize :loop-num loop-num :print-status-interval 2)

  (when use-msc?
    (send *sqp-opt* :draw-each-configuration :additional-objects (list *obj*)))
  )
(warn "~%(demo-pa10-surface-contact)~%")

(defun output-task-error
    (config-task)
  (let* ((instant-config-task
          (send config-task :instant-config-task))
         (kin-task-value
          (send instant-config-task :kinematics-task-value)
          )
         (pos-error ;; [mm]
          (scale 1e3 (subseq kin-task-value 0 3)))
         (rot-error ;; [deg]
          (rad2deg (acos (- (- (elt kin-task-value 3) 1.0))))
          )
         )
    (warning-message 3 "  pos: ~6,3f ~6,3f ~6,3f [mm]~%" (elt pos-error 0) (elt pos-error 1) (elt pos-error 2))
    (warning-message 3 "  rot: ~6,3f [deg]~%" rot-error)
    ))
