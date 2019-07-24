(load "package://eus_qp/optmotiongen/euslisp/trajectory-configuration-task.l")
(load "package://eus_qp/optmotiongen/euslisp/inverse-kinematics-statics-wrapper.l")
(load "package://eus_qp/optmotiongen/euslisp/sqp-optimization.l")
(load "package://eus_qp/optmotiongen/euslisp/validate-jacobian.l")
(require :jaxon_red "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon_red-interface.l")
(require "models/arrow-object.l")


(defun setup-robot-for-demo-jaxon-hand-force
    ()
  ;; robot
  (setq *robot* (instance jaxon_red-robot :init))
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  (send *robot* :put :reset-pose-joint-angle-list (coerce (send *robot* :angle-vector) cons))
  ;; contact constraint
  (create-support-polygon *robot* (send (send *robot* :lleg-end-coords) :parent))
  (create-support-polygon *robot* (send (send *robot* :rleg-end-coords) :parent))
  (send *robot* :put :lleg-foot-contact-constraint
        (instance default-contact-constraint
                  :init-from-support-polygon
                  :mu-trans 0.2
                  :support-polygon (send (send (send *robot* :lleg-end-coords) :parent) :get :support-polygon)
                  :support-polygon-coords (send (send *robot* :lleg-end-coords) :worldcoords)
                  :support-polygon-margin 10
                  ))
  (send *robot* :put :rleg-foot-contact-constraint
        (instance default-contact-constraint
                  :init-from-support-polygon
                  :mu-trans 0.2
                  :support-polygon (send (send (send *robot* :rleg-end-coords) :parent) :get :support-polygon)
                  :support-polygon-coords (send (send *robot* :rleg-end-coords) :worldcoords)
                  :support-polygon-margin 10
                  ))
  )

(defun demo-jaxon-hand-force-maximize
    (&key
     (hand-force-dir (float-vector 0 0 1))
     (pre-process-func)
     &allow-other-keys
     )
  ;; setup
  (setup-robot-for-demo-jaxon-hand-force)
  (setq *robot-env*
        (instance robot-environment :init :robot *robot*))
  (setq *arrow* (instance arrow-object :init))
  (send *arrow* :newcoords (make-coords :pos (float-vector 700 300 1000)))
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
                  :name :knee-stand
                  :robot-env *robot-env*
                  :drive-joint-list nil
                  :kin-target-coords-list
                  (list
                   (send (send *robot-env* :robot :lleg-end-coords) :copy-worldcoords)
                   (send (send *robot-env* :robot :rleg-end-coords) :copy-worldcoords)
                   (send *arrow* :copy-worldcoords)
                   )
                  :kin-attention-coords-list
                  (list
                   (send *robot-env* :robot :lleg-end-coords)
                   (send *robot-env* :robot :rleg-end-coords)
                   (send *robot-env* :robot :larm-end-coords)
                   )
                  :contact-target-coords-list
                  (list
                   (send (send *robot-env* :robot :lleg-end-coords) :copy-worldcoords)
                   (send (send *robot-env* :robot :rleg-end-coords) :copy-worldcoords)
                   (orient-coords-to-axis (send *arrow* :copy-worldcoords) hand-force-dir)
                   )
                  :contact-attention-coords-list
                  (list
                   (send *robot-env* :robot :lleg-end-coords)
                   (send *robot-env* :robot :rleg-end-coords)
                   (send *robot-env* :robot :larm-end-coords)
                   )
                  :contact-constraint-list
                  (list
                   (send *robot-env* :robot :get :lleg-foot-contact-constraint)
                   (send *robot-env* :robot :get :rleg-foot-contact-constraint)
                   (instance 6d-min-max-contact-constraint :init (float-vector 0 0 1e10 0 0 0))
                   )
                  :wrench-maximize-scale 1e-4
                  :norm-regular-scale-max 1e-3
                  :norm-regular-scale-offset 1e-6
                  :wrench-maximize-direction-list
                  (list
                   (float-vector 0 0 0 0 0 0)
                   (float-vector 0 0 0 0 0 0)
                   (concatenate float-vector hand-force-dir (float-vector 0 0 0))
                   )
                  :collision-pair-list
                  (apply
                   #'append
                   (mapcar
                    #'(lambda (l1)
                        (mapcar
                         #'(lambda (l2) (list l1 l2))
                         (subseq (send *robot* :larm) 4)))
                    (list (send *robot* :link "CHEST_LINK1") (send *robot* :link "CHEST_LINK2"))))
                  :collision-distance-margin
                  0.01
                  :posture-joint-list
                  (send *robot-env* :robot :joint-list)
                  :posture-joint-angle-list
                  (send *robot-env* :robot :get :reset-pose-joint-angle-list)
                  :target-posture-scale 1e-3
                  ))
  (send *instant-config-task* :put :thre (make-list 3 :initial-element 1)) ;; [mm]
  (send *instant-config-task* :put :rthre (make-list 3 :initial-element (deg2rad 1))) ;; [rad]
  (send *instant-config-task* :put :eom-trans-thre 1e-2)
  (send *instant-config-task* :put :eom-rot-thre 1e-2)
  (send *instant-config-task* :put :torque-thre 1e-2)

  (setq *sqp-opt*
        (instance sqp-optimization :init
                  :config-task *instant-config-task*
                  :convergence-check-func #'sqp-convergence-check
                  :failure-callback-func #'sqp-failure-callback
                  :pre-process-func
                  pre-process-func
                  ;; :post-process-func
                  ;; #'(lambda (sqp)
                  ;;     (when (> (send sqp :iteration) 200)
                  ;;       (setf (slot *instant-config-task* instant-configuration-task '_wrench-maximize-scale) 0)
                  ;;       ))
                  ))
  (send *sqp-opt* :optimize :loop-num 300 :loop-num-min 200)
  (warning-message 2 "hand-force = ~a [N] (direction is ~a)~%"
                   (v. (subseq (elt (send *instant-config-task* :wrench-list) 2) 0 3) (normalize-vector hand-force-dir))
                   (normalize-vector hand-force-dir))
  )
(warn ";; 1. maximize hand-force example~%")
(warn "(demo-jaxon-hand-force-maximize)~%")
(warn "(demo-jaxon-hand-force-maximize :hand-force-dir (float-vector 0 0 -1))~%")

(defun demo-jaxon-hand-force-constant
    (&key
     (larm-hand-force (float-vector 0 0 300))
     (pre-process-func)
     &allow-other-keys
     )
  ;; setup
  (setup-robot-for-demo-jaxon-hand-force)
  (setq *robot-env*
        (instance robot-environment :init :robot *robot*))
  (setq *arrow* (instance arrow-object :init))
  (send *arrow* :newcoords (make-coords :pos (float-vector 700 300 1000)))
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
                  :name :knee-stand
                  :robot-env *robot-env*
                  :drive-joint-list nil
                  :kin-target-coords-list
                  (list
                   (send (send *robot-env* :robot :lleg-end-coords) :copy-worldcoords)
                   (send (send *robot-env* :robot :rleg-end-coords) :copy-worldcoords)
                   (send *arrow* :copy-worldcoords)
                   )
                  :kin-attention-coords-list
                  (list
                   (send *robot-env* :robot :lleg-end-coords)
                   (send *robot-env* :robot :rleg-end-coords)
                   (send *robot-env* :robot :larm-end-coords)
                   )
                  :contact-target-coords-list
                  (list
                   (send (send *robot-env* :robot :lleg-end-coords) :copy-worldcoords)
                   (send (send *robot-env* :robot :rleg-end-coords) :copy-worldcoords)
                   )
                  :contact-attention-coords-list
                  (list
                   (send *robot-env* :robot :lleg-end-coords)
                   (send *robot-env* :robot :rleg-end-coords)
                   )
                  :contact-constraint-list
                  (list
                   (send *robot-env* :robot :get :lleg-foot-contact-constraint)
                   (send *robot-env* :robot :get :rleg-foot-contact-constraint)
                   )
                  :external-wrench-list
                  (list (concatenate float-vector larm-hand-force (float-vector 0 0 0)))
                  :external-coords-list
                  (list (send *robot-env* :robot :larm-end-coords))
                  :collision-pair-list
                  (apply
                   #'append
                   (mapcar
                    #'(lambda (l1)
                        (mapcar
                         #'(lambda (l2) (list l1 l2))
                         (subseq (send *robot* :larm) 4)))
                    (list (send *robot* :link "CHEST_LINK1") (send *robot* :link "CHEST_LINK2"))))
                  :collision-distance-margin
                  0.01
                  :posture-joint-list
                  (send *robot-env* :robot :joint-list)
                  :posture-joint-angle-list
                  (send *robot-env* :robot :get :reset-pose-joint-angle-list)
                  :target-posture-scale 1e-3
                  ))
  (send *instant-config-task* :put :thre (make-list 3 :initial-element 1)) ;; [mm]
  (send *instant-config-task* :put :rthre (make-list 3 :initial-element (deg2rad 1))) ;; [rad]
  (send *instant-config-task* :put :eom-trans-thre 1e-2)
  (send *instant-config-task* :put :eom-rot-thre 1e-2)
  (send *instant-config-task* :put :torque-thre 1e-2)

  (setq *sqp-opt*
        (instance sqp-optimization :init
                  :config-task *instant-config-task*
                  :convergence-check-func #'sqp-convergence-check
                  :failure-callback-func #'sqp-failure-callback
                  :pre-process-func
                  pre-process-func
                  ;; :post-process-func
                  ;; #'(lambda (sqp))
                  ))
  (send *sqp-opt* :optimize :loop-num 50)
  )
(warn ";; 2. constant hand-force example~%")
(warn "(demo-jaxon-hand-force-constant)~%")
(warn "(demo-jaxon-hand-force-constant :larm-hand-force (float-vector 0 0 -200))~%")

(defun loop-demo-jaxon-hand-force-constant
    ()
  (demo-jaxon-hand-force-constant)
  (let* ((i 0)
         (fz)
         )
    (do-until-key
     (incf i)
     (setq fz (* 400 (sin (* 0.01 (float i)))))
     (send *instant-config-task* :external-wrench-list (list (float-vector 0 0 fz 0 0 0)))
     (send *sqp-opt* :optimize
           :update-viewer-interval nil
           :print-status-interval nil
           :loop-num 50
           :loop-num-min 2
           )
     ))
  )
(warn "(loop-demo-jaxon-hand-force-constant)~%")
