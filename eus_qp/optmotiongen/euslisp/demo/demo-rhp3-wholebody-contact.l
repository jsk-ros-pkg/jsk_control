(load "package://eus_qp/optmotiongen/euslisp/joint-surface-configuration-task.l")
(load "package://eus_qp/optmotiongen/euslisp/inverse-kinematics-statics-wrapper.l")
(load "package://eus_qp/optmotiongen/euslisp/sqp-optimization.l")
(load "package://eus_qp/optmotiongen/euslisp/validate-jacobian.l")
(require :rhp3 "package://khi_ros_bridge_tutorials/euslisp/rhp3-interface.l")
(require "models/arrow-object.l")


(defun setup-rhp3-for-demo-rhp3-wholebody-contact
    ()
  (setq *robot* (instance rhp3-robot :init))
  ;; assoc contact coords
  ;;;; hand
  (send *robot* :put :rarm-hand-contact-coords
        (make-cascoords
         :name :rarm-hand-contact-coords
         :parent (send (send *robot* :rarm :end-coords) :parent)
         :coords
         (send (send (send *robot* :rarm :end-coords) :copy-worldcoords)
               :transform
               (make-coords :pos #f(30 0 0) :rpy (list -pi/2 0 pi/2)))))
  (send *robot* :put :larm-hand-contact-coords
        (make-cascoords
         :name :larm-hand-contact-coords
         :parent (send (send *robot* :larm :end-coords) :parent)
         :coords
         (send (send (send *robot* :larm :end-coords) :copy-worldcoords)
               :transform
               (make-coords :pos #f(30 0 0) :rpy (list pi/2 0 -pi/2)))))
  ;;;; foot
  (send *robot* :put :rleg-foot-contact-coords
        (make-cascoords
         :name :rleg-foot-contact-coords
         :parent (send (send *robot* :rleg :end-coords) :parent)
         :coords
         (send (send *robot* :rleg :end-coords) :copy-worldcoords)))
  (send *robot* :put :lleg-foot-contact-coords
        (make-cascoords
         :name :lleg-foot-contact-coords
         :parent (send (send *robot* :lleg :end-coords) :parent)
         :coords
         (send (send *robot* :lleg :end-coords) :copy-worldcoords)))
  ;;;; knee
  (send *robot* :put :lleg-knee-contact-coords
        (make-cascoords
         :name :lleg-knee-contact-coords
         :parent (send (send *robot* :lleg :knee-p) :child-link)
         :coords
         (send (send (send (send *robot* :lleg :knee-p) :child-link) :copy-worldcoords)
               :transform (make-coords :pos #f(110 0 0) :rpy (list 0 -pi/2 0)))))
  (send *robot* :put :rleg-knee-contact-coords
        (make-cascoords
         :name :rleg-knee-contact-coords
         :parent (send (send *robot* :rleg :knee-p) :child-link)
         :coords
         (send (send (send (send *robot* :rleg :knee-p) :child-link) :copy-worldcoords)
               :transform (make-coords :pos #f(110 0 0) :rpy (list 0 -pi/2 0)))))
  ;;;; elbow
  (send *robot* :put :rarm-elbow-contact-coords
        (make-cascoords
         :name :rarm-elbow-contact-coords
         :parent (send (send *robot* :rarm :elbow-p) :child-link)
         :coords
         (send
          (send (send (send *robot* :rarm :elbow-p) :child-link) :copy-worldcoords)
          :transform (make-coords :pos #f(-85 -0 0) :rpy (list 0 pi/2 0)))))
  (send *robot* :put :larm-elbow-contact-coords
        (make-cascoords
         :name :larm-elbow-contact-coords
         :parent (send (send *robot* :larm :elbow-p) :child-link)
         :coords
         (send
          (send (send (send *robot* :larm :elbow-p) :child-link) :copy-worldcoords)
          :transform (make-coords :pos #f(-85 0 0) :rpy (list 0 pi/2 0)))))
  ;; save pose
  (send *robot* :reset-pose)
  (send *robot* :put :reset-pose-joint-angle-list (coerce (send *robot* :angle-vector) cons))
  (send *robot* :move-coords
        (make-coords)
        (midcoords 0.5
                   (send *robot* :get :rleg-foot-contact-coords)
                   (send *robot* :get :lleg-foot-contact-coords)))
  )

(defun setup-contact-constraint-for-demo-rhp3-wholebody-contact
    ()
  (send *robot* :put :lleg-foot-contact-constraint
        (instance default-contact-constraint :init
                  :mu-trans 0.1 :mu-rot 0.01 :max-fz 1000
                  :l-min-x -10.0 :l-max-x 10.0 :l-min-y -10.0 :l-max-y 10.0
                  ))
  (send *robot* :put :rleg-foot-contact-constraint
        (instance default-contact-constraint :init
                  :mu-trans 0.1 :mu-rot 0.01 :max-fz 1000
                  :l-min-x -10.0 :l-max-x 10.0 :l-min-y -10.0 :l-max-y 10.0
                  ))
  (send *robot* :put :lleg-knee-contact-constraint
        (instance default-contact-constraint :init
                  :mu-trans 0.1 :mu-rot 0.01 :max-fz 1000
                  :l-min-x -1.0 :l-max-x 1.0 :l-min-y -10.0 :l-max-y 10.0
                  ))
  (send *robot* :put :rleg-knee-contact-constraint
        (instance default-contact-constraint :init
                  :mu-trans 0.1 :mu-rot 0.01 :max-fz 1000
                  :l-min-x -1.0 :l-max-x 1.0 :l-min-y -10.0 :l-max-y 10.0
                  ))
  (send *robot* :put :larm-elbow-contact-constraint
        (instance default-contact-constraint :init
                  :mu-trans 0.1 :mu-rot 0.01 :max-fz 1000
                  :l-min-x -1.0 :l-max-x 1.0 :l-min-y -1.0 :l-max-y 1.0
                  ))
  (send *robot* :put :rarm-elbow-contact-constraint
        (instance default-contact-constraint :init
                  :mu-trans 0.1 :mu-rot 0.01 :max-fz 1000
                  :l-min-x -1.0 :l-max-x 1.0 :l-min-y -1.0 :l-max-y 1.0
                  ))
  )

(defun demo-rhp3-wholebody-contact-body-surface
    (&key
     (optimize-torque? nil)
     (pre-process-func)
     &allow-other-keys
     )
  ;; setup environment
  (setq *ground-lleg* (make-cube 400 400 20))
  (setq *ground-rarm* (make-cube 400 400 20))
  (send *ground-lleg* :newcoords
        (make-coords :pos (float-vector 200 300 100) :rpy (list 0 (deg2rad -10) (deg2rad 45))))
  (send *ground-rarm* :newcoords
        (make-coords :pos (float-vector 200 -600 500) :rpy (list 0 (deg2rad 10) (deg2rad 80))))
  (setq *ground-list* (list *ground-lleg* *ground-rarm*))
  (send-all *ground-list* :worldcoords)
  (send-all *ground-list* :set-color (float-vector 0.5 0.5 0.25))
  (setq *arrow* (instance arrow-object :init))
  (send *arrow* :newcoords
        (make-coords :pos (float-vector 900 200 500)))
  ;; setup robot
  (setup-rhp3-for-demo-rhp3-wholebody-contact)
  (setq *contact-list*
        (list (instance 2d-planar-contact :init :name :lleg-knee :fix-yaw? t)
              (instance 2d-planar-contact :init :name :rarm-elbow :fix-yaw? t)
              (instance contact-ik-arg :init :name :larm-hand
                        :target-coords (send *arrow* :copy-worldcoords)
                        :translation-axis t
                        :rotation-axis nil
                        )
              ))
  (setq *robot-env*
        (instance robot-environment :init :robot *robot* :contact-list *contact-list*))
  (send *robot-env* :contact :lleg-knee :set-from-face :face (elt (send *ground-lleg* :faces) 1))
  (send *robot-env* :contact :rarm-elbow :set-from-face :face (elt (send *ground-rarm* :faces) 0))
  (setup-contact-constraint-for-demo-rhp3-wholebody-contact)
  ;; draw
  (unless (boundp '*irtviewer*)
    (make-irtviewer)
    (send *irtviewer* :change-background (float-vector 1 1 1))
    (send *irtviewer* :draw-floor 500)
    (send *irtviewer* :floor-color (float-vector 0 0 0))
    )
  (objects (append (list *robot* *arrow*) *ground-list*))

  (setq *instant-config-task*
        (instance instant-configuration-task :init
                  :robot-env *robot-env*
                  :drive-joint-list nil
                  :kin-target-coords-list
                  (list
                   (send (send *robot-env* :robot :get :rleg-foot-contact-coords) :copy-worldcoords)
                   (send (send *robot-env* :contact :lleg-knee) :contact-coords)
                   (send (send *robot-env* :contact :rarm-elbow) :contact-coords)
                   (send (send *robot-env* :contact :larm-hand) :contact-coords)
                   )
                  :kin-attention-coords-list
                  (list
                   (send *robot-env* :robot :get :rleg-foot-contact-coords)
                   (send *robot-env* :robot :get :lleg-knee-contact-coords)
                   (send *robot-env* :robot :get :rarm-elbow-contact-coords)
                   (send *robot-env* :robot :get :larm-hand-contact-coords)
                   )
                  :kin-rotation-type-list
                  (list :axis-angle :normal :normal :axis-angle)
                  :contact-target-coords-list
                  (list
                   (send (send *robot-env* :robot :get :rleg-foot-contact-coords) :copy-worldcoords)
                   (send (send *robot-env* :contact :lleg-knee) :contact-coords)
                   (send (send *robot-env* :contact :rarm-elbow) :contact-coords)
                   )
                  :contact-attention-coords-list
                  (list
                   (send *robot-env* :robot :get :rleg-foot-contact-coords)
                   (send *robot-env* :robot :get :lleg-knee-contact-coords)
                   (send *robot-env* :robot :get :rarm-elbow-contact-coords)
                   )
                  :contact-constraint-list
                  (list
                   (send *robot-env* :robot :get :rleg-foot-contact-constraint)
                   (send *robot-env* :robot :get :lleg-knee-contact-constraint)
                   (send *robot-env* :robot :get :rarm-elbow-contact-constraint)
                   )
                  :norm-regular-scale-max 1e-3
                  :norm-regular-scale-offset 1e-6
                  :collision-pair-list
                  (list
                   (list *ground-rarm* (send *robot-env* :robot :link "R_SHOULDER_Y_LINK"))
                   (list *ground-rarm* (send *robot-env* :robot :link "R_WRIST_Y_LINK"))
                   )
                  :collision-distance-margin 0.002
                  ))

  (let* ((lleg-knee-link
          (send (send *robot-env* :robot :lleg :knee-p) :child-link))
         (lleg-knee-body
          (convex-hull-3d (flatten (send-all (send lleg-knee-link :bodies) :vertices))))
         (rarm-elbow-link
          (send (send *robot-env* :robot :rarm :elbow-p) :child-link))
         (rarm-elbow-body
          (convex-hull-3d (flatten (send-all (send rarm-elbow-link :bodies) :vertices))))
         )
    (send lleg-knee-link :assoc lleg-knee-body)
    (setq *lleg-knee-surface-config-task*
          (instance surface-configuration-task :init
                    :body lleg-knee-body
                    :initial-surface-pos
                    (send (send *robot-env* :robot :get :lleg-knee-contact-coords) :worldpos)
                    :kin-target-coords
                    (send (send *robot-env* :contact :lleg-knee) :contact-coords)
                    :flip-normal? t
                    ))
    (send rarm-elbow-link :assoc rarm-elbow-body)
    (setq *rarm-elbow-surface-config-task*
          (instance surface-configuration-task :init
                    :body rarm-elbow-body
                    :initial-surface-pos
                    (send (send *robot-env* :robot :get :rarm-elbow-contact-coords) :worldpos)
                    :kin-target-coords
                    (send (send *robot-env* :contact :rarm-elbow) :contact-coords)
                    :flip-normal? t
                    ))
    )

  (setq *joint-surface-config-task*
        (instance joint-surface-configuration-task :init
                  :instant-config-task *instant-config-task*
                  :surface-kin-contact-list
                  (list
                   (list *lleg-knee-surface-config-task* :lleg-knee-contact-coords :lleg-knee-contact-coords)
                   (list *rarm-elbow-surface-config-task* :rarm-elbow-contact-coords :rarm-elbow-contact-coords)
                   )
                  ))

  (setq *sqp-opt*
        (instance sqp-optimization :init
                  :config-task *joint-surface-config-task*
                  :pre-process-func
                  pre-process-func
                  :post-process-func
                  #'(lambda (sqp)
                      ;; (read-line)
                      (when (= (mod (send sqp :iteration) 30) 0)
                        ;; (validate-jacobian :config-task (send sqp :config-task))
                        )))
        )
  (send *sqp-opt* :optimize :loop-num 50 :print-status-interval 1)
  )
(warn "(demo-rhp3-wholebody-contact-body-surface)~%")
