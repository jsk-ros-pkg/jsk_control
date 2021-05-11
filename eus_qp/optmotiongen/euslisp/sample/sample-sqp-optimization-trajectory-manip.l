(load "./sample-sqp-optimization-instant-manip.l")
(load "../trajectory-configuration-task.l")


(defun setup-contact-constraint-for-sample-sqp-optimization-manip
    ()
  (setq *rleg-contact-constraint*
        (instance* default-contact-constraint
                   :init
                   :mu-trans 0.2
                   :mu-rot 0.01
                   :max-fz 400
                   (let* ((poly
                           (send *robot-obj-env* :robot :get :rleg_link5-support-polygon))
                          (vs
                           (mapcar #'(lambda (v)
                                       (send (send (send *robot-obj-env* :robot :get :rleg-contact-coords) :worldcoords)
                                             :inverse-transform-vector v))
                                   (send poly :vertices)
                                   ))
                          )
                     (list :l-min-x (+ (elt (find-extream vs #'(lambda (v) (elt v 0)) #'<) 0) 20)
                           :l-max-x (- (elt (find-extream vs #'(lambda (v) (elt v 0)) #'>) 0) 20)
                           :l-min-y (+ (elt (find-extream vs #'(lambda (v) (elt v 1)) #'<) 1) 20)
                           :l-max-y (- (elt (find-extream vs #'(lambda (v) (elt v 1)) #'>) 1) 20)
                           ))
                   ))
  (setq *lleg-contact-constraint*
        (instance* default-contact-constraint
                   :init
                   :mu-trans 0.2
                   :mu-rot 0.01
                   :max-fz 400
                   (let* ((poly
                           (send *robot-obj-env* :robot :get :lleg_link5-support-polygon))
                          (vs
                           (mapcar #'(lambda (v)
                                       (send (send (send *robot-obj-env* :robot :get :lleg-contact-coords) :worldcoords)
                                             :inverse-transform-vector v))
                                   (send poly :vertices)
                                   ))
                          )
                     (list :l-min-x (+ (elt (find-extream vs #'(lambda (v) (elt v 0)) #'<) 0) 20)
                           :l-max-x (- (elt (find-extream vs #'(lambda (v) (elt v 0)) #'>) 0) 20)
                           :l-min-y (+ (elt (find-extream vs #'(lambda (v) (elt v 1)) #'<) 1) 20)
                           :l-max-y (- (elt (find-extream vs #'(lambda (v) (elt v 1)) #'>) 1) 20)
                           ))
                   ))
  (setq *rarm-grasp-constraint*
        (instance 6d-min-max-contact-constraint
                  :init (float-vector 1000 1000 1000 100 100 100)))
  (setq *larm-grasp-constraint*
        (instance 6d-min-max-contact-constraint
                  :init (float-vector 1000 1000 1000 100 100 100)))
  (setq *rarm-grasp-constraint-obj*
        (instance 6d-min-max-contact-constraint
                  :init (float-vector 1000 1000 1000 100 100 100)))
  (setq *larm-grasp-constraint-obj*
        (instance 6d-min-max-contact-constraint
                  :init (float-vector 1000 1000 1000 100 100 100)))
  (setq *ground-contact-constraint-obj*
        (instance default-contact-constraint
                  :init
                  :mu-trans 0.2
                  :mu-rot 0.01
                  :max-fz 1000
                  :l-min-x -200.0
                  :l-max-x 200.0
                  :l-min-y -200.0
                  :l-max-y 200.0
                  ))
  )

(defun sample-sqp-optimization-trajectory-manip
    (&key
     (optimize-torque? nil)
     (optimize-start-end-torque? t)
     (avoid-collision? t)
     (fix-grasp? t)
     (pre-process-func)
     (use-adjacent-regular-scale-list? t)
     (visualize-callback-func)
     (visualize-loop? t)
     &allow-other-keys
     )
  ;; setup environment
  (setq *ground-rleg* (make-cube 400 400 20))
  (send *ground-rleg* :set-color (float-vector 0.5 0.5 0.25))
  (send *ground-rleg* :newcoords
        (make-coords :pos (float-vector 0 -500 100) :rpy (list 0 0 (deg2rad -20))))
  (setq *arrow-lleg* (instance arrow-object :init))
  (send *arrow-lleg* :newcoords
        (make-coords :pos (float-vector 0 100 0) :rpy (list 0 0 0)))
  (setq *arrow-obj-start* (instance arrow-object :init))
  (send *arrow-obj-start* :newcoords
        (make-coords :pos (float-vector 700 100 0)))
  (setq *arrow-obj-finish* (instance arrow-object :init))
  (send *arrow-obj-finish* :newcoords
        (make-coords :pos (float-vector 400 0 100) :rpy (list (deg2rad 10) (deg2rad 20) (deg2rad 5))))
  (setq *env-list* (list *ground-rleg* *arrow-lleg* *arrow-obj-start* *arrow-obj-start* *arrow-obj-finish*))
  (send-all *env-list* :worldcoords)
  ;; setup robot
  (setup-hrp2jsknts-for-sample-sqp-optimization)
  (setup-for-sample-sqp-optimization-manip)
  (setq *contact-list*
        (list (instance 2d-planar-contact :init :name "rleg")))
  (setq *robot-obj-env*
        (instance robot-object-environment :init :robot *robot* :object *obj* :contact-list *contact-list*))
  (send *robot-obj-env* :contact "rleg" :set-from-face :face (elt (send *ground-rleg* :faces) 1))
  (send *robot-obj-env* :contact "rleg" :rotate pi :z :local)
  (setq *robot-obj-env-list* nil)
  (setq *instant-manip-config-task-list* nil)
  (setup-contact-constraint-for-sample-sqp-optimization-manip)
  ;; draw
  (unless (boundp '*irtviewer*)
    (make-irtviewer)
    (send *irtviewer* :change-background (float-vector 1 1 1))
    (send *irtviewer* :draw-floor 500)
    (send *irtviewer* :floor-color (float-vector 0 0 0))
    )

  (push (copy-object *robot-obj-env*) *robot-obj-env-list*)
  (push
   (instance instant-manipulation-configuration-task :init
             :name :obj-start-no-reach
             :robot-env (car *robot-obj-env-list*)
             :drive-joint-list
             (if (or optimize-torque? optimize-start-end-torque?) (send (car *robot-obj-env-list*) :drive-joint-list) nil)
             :posture-joint-list
             (send (car *robot-obj-env-list*) :robot :joint-list)
             :posture-joint-angle-list
             (send-all (send (car *robot-obj-env-list*) :robot :joint-list) :joint-angle)
             :kin-target-coords-list
             (list (send (car *robot-obj-env-list*) :contact "rleg" :contact-coords)
                   (send *arrow-lleg* :copy-worldcoords)
                   (send *arrow-obj-start* :copy-worldcoords)
                   )
             :kin-attention-coords-list
             (list (send (car *robot-obj-env-list*) :robot :get :rleg-contact-coords)
                   (send (car *robot-obj-env-list*) :robot :get :lleg-contact-coords)
                   (send (car *robot-obj-env-list*) :object :get :object-coords)
                   )
             :contact-target-coords-list
             (cond (fix-grasp?
                    (list (send (car *robot-obj-env-list*) :contact "rleg" :contact-coords)
                          (send *arrow-lleg* :copy-worldcoords)
                          )
                    )
                   (t
                    ;; todo
                    ))
             :contact-attention-coords-list
             (list (send (car *robot-obj-env-list*) :robot :get :rleg-contact-coords)
                   (send (car *robot-obj-env-list*) :robot :get :lleg-contact-coords)
                   )
             :contact-constraint-list
             (list *rleg-contact-constraint*
                   *lleg-contact-constraint*
                   )
             :contact-target-coords-obj-list
             (list (send *arrow-obj-start* :copy-worldcoords))
             :contact-constraint-obj-list
             (list *ground-contact-constraint-obj*)
             :collision-pair-list
             (if avoid-collision?
                 (mapcar
                  #'(lambda (l)
                      (list l (send (car *robot-obj-env-list*) :object :get :convex-hull-body)))
                  (flatten
                   (mapcar #'(lambda (limb) (send (car *robot-obj-env-list*) :robot limb :links))
                           (list :torso :rleg :lleg :rarm :larm))
                   )))
             :collision-distance-margin 0.05
             :delta-linear-joint 100 ;; [mm]
             :delta-rotational-joint 30 ;; [deg]
             :target-posture-scale 1e-2
             )
   *instant-manip-config-task-list*
   )

  (push (copy-object *robot-obj-env*) *robot-obj-env-list*)
  (push
   (instance instant-manipulation-configuration-task :init
             :name :obj-start-no-contact
             :robot-env (car *robot-obj-env-list*)
             :drive-joint-list
             (if optimize-torque? (send (car *robot-obj-env-list*) :drive-joint-list) nil)
             :posture-joint-list
             (send (car *robot-obj-env-list*) :robot :joint-list)
             :posture-joint-angle-list
             (send-all (send (car *robot-obj-env-list*) :robot :joint-list) :joint-angle)
             :kin-target-coords-list
             (cond (fix-grasp?
                    (list (send (car *robot-obj-env-list*) :contact "rleg" :contact-coords)
                          (send *arrow-lleg* :copy-worldcoords)
                          (send (car *robot-obj-env-list*) :object :handle-side-rarm)
                          (send (car *robot-obj-env-list*) :object :handle-side-larm)
                          (send *arrow-obj-start* :copy-worldcoords)
                          )
                    )
                   (t
                    ;; todo
                    ))
             :kin-attention-coords-list
             (list (send (car *robot-obj-env-list*) :robot :get :rleg-contact-coords)
                   (send (car *robot-obj-env-list*) :robot :get :lleg-contact-coords)
                   (send (car *robot-obj-env-list*) :robot :get :rarm-grasp-coords)
                   (send (car *robot-obj-env-list*) :robot :get :larm-grasp-coords)
                   (send (car *robot-obj-env-list*) :object :get :object-coords)
                   )
             :contact-target-coords-list
             (cond (fix-grasp?
                    (list (send (car *robot-obj-env-list*) :contact "rleg" :contact-coords)
                          (send *arrow-lleg* :copy-worldcoords)
                          )
                    )
                   (t
                    ;; todo
                    ))
             :contact-attention-coords-list
             (list (send (car *robot-obj-env-list*) :robot :get :rleg-contact-coords)
                   (send (car *robot-obj-env-list*) :robot :get :lleg-contact-coords)
                   )
             :contact-constraint-list
             (list *rleg-contact-constraint*
                   *lleg-contact-constraint*
                   )
             :contact-target-coords-obj-list
             (list (send *arrow-obj-start* :copy-worldcoords))
             :contact-constraint-obj-list
             (list *ground-contact-constraint-obj*)
             :collision-pair-list
             (if avoid-collision?
                 (mapcar
                  #'(lambda (l)
                      (list l (send (car *robot-obj-env-list*) :object :get :convex-hull-body)))
                  (flatten
                   (mapcar #'(lambda (limb) (send (car *robot-obj-env-list*) :robot limb :links))
                           (list :torso :rleg :lleg))
                   )))
             :collision-distance-margin 0.05
             :delta-linear-joint 100 ;; [mm]
             :delta-rotational-joint 30 ;; [deg]
             )
   *instant-manip-config-task-list*
   )

  (push (copy-object *robot-obj-env*) *robot-obj-env-list*)
  (push
   (instance instant-manipulation-configuration-task :init
             :name :obj-start-contact
             :robot-env (car *robot-obj-env-list*)
             :drive-joint-list
             (if optimize-torque? (send (car *robot-obj-env-list*) :drive-joint-list) nil)
             :posture-joint-list
             (send (car *robot-obj-env-list*) :robot :joint-list)
             :posture-joint-angle-list
             (send-all (send (car *robot-obj-env-list*) :robot :joint-list) :joint-angle)
             :kin-target-coords-list
             (cond (fix-grasp?
                    (list (send (car *robot-obj-env-list*) :contact "rleg" :contact-coords)
                          (send *arrow-lleg* :copy-worldcoords)
                          (send (car *robot-obj-env-list*) :object :handle-side-rarm)
                          (send (car *robot-obj-env-list*) :object :handle-side-larm)
                          (send *arrow-obj-start* :copy-worldcoords)
                          )
                    )
                   (t
                    ;; todo
                    ))
             :kin-attention-coords-list
             (list (send (car *robot-obj-env-list*) :robot :get :rleg-contact-coords)
                   (send (car *robot-obj-env-list*) :robot :get :lleg-contact-coords)
                   (send (car *robot-obj-env-list*) :robot :get :rarm-grasp-coords)
                   (send (car *robot-obj-env-list*) :robot :get :larm-grasp-coords)
                   (send (car *robot-obj-env-list*) :object :get :object-coords)
                   )
             :contact-target-coords-list
             (cond (fix-grasp?
                    (list (send (car *robot-obj-env-list*) :contact "rleg" :contact-coords)
                          (send *arrow-lleg* :copy-worldcoords)
                          (send (car *robot-obj-env-list*) :object :handle-side-rarm)
                          (send (car *robot-obj-env-list*) :object :handle-side-larm)
                          )
                    )
                   (t
                    ;; todo
                    ))
             :contact-attention-coords-list
             (list (send (car *robot-obj-env-list*) :robot :get :rleg-contact-coords)
                   (send (car *robot-obj-env-list*) :robot :get :lleg-contact-coords)
                   (send (car *robot-obj-env-list*) :robot :get :rarm-grasp-coords)
                   (send (car *robot-obj-env-list*) :robot :get :larm-grasp-coords)
                   )
             :contact-constraint-list
             (list *rleg-contact-constraint*
                   *lleg-contact-constraint*
                   *rarm-grasp-constraint*
                   *larm-grasp-constraint*
                   )
             :contact-target-coords-obj-list
             (cond (fix-grasp?
                    (list (send *arrow-obj-start* :copy-worldcoords)
                          (send (car *robot-obj-env-list*) :object :handle-side-rarm)
                          (send (car *robot-obj-env-list*) :object :handle-side-larm)
                          )
                    )
                   (t
                    ;; todo
                    ))
             :contact-constraint-obj-list
             (list *ground-contact-constraint-obj*
                   *rarm-grasp-constraint-obj*
                   *larm-grasp-constraint-obj*
                   )
             :act-react-pair-list
             (list
              (list (send (car *robot-obj-env-list*) :object :handle-side-rarm) (send (car *robot-obj-env-list*) :object :handle-side-rarm))
              (list (send (car *robot-obj-env-list*) :object :handle-side-larm) (send (car *robot-obj-env-list*) :object :handle-side-larm))
              )
             :collision-pair-list
             (if avoid-collision?
                 (mapcar
                  #'(lambda (l)
                      (list l (send (car *robot-obj-env-list*) :object :get :convex-hull-body)))
                  (flatten
                   (mapcar #'(lambda (limb) (send (car *robot-obj-env-list*) :robot limb :links))
                           (list :torso :rleg :lleg))
                   )))
             :collision-distance-margin 0.05
             :delta-linear-joint 100 ;; [mm]
             :delta-rotational-joint 30 ;; [deg]
             )
   *instant-manip-config-task-list*
   )

  (push (copy-object *robot-obj-env*) *robot-obj-env-list*)
  (push
   (instance instant-manipulation-configuration-task :init
             :name :obj-start-float
             :robot-env (car *robot-obj-env-list*)
             :drive-joint-list
             (if optimize-torque? (send (car *robot-obj-env-list*) :drive-joint-list) nil)
             :posture-joint-list
             (send (car *robot-obj-env-list*) :robot :joint-list)
             :posture-joint-angle-list
             (send-all (send (car *robot-obj-env-list*) :robot :joint-list) :joint-angle)
             :kin-target-coords-list
             (cond (fix-grasp?
                    (list (send (car *robot-obj-env-list*) :contact "rleg" :contact-coords)
                          (send *arrow-lleg* :copy-worldcoords)
                          (send (car *robot-obj-env-list*) :object :handle-side-rarm)
                          (send (car *robot-obj-env-list*) :object :handle-side-larm)
                          (send *arrow-obj-start* :copy-worldcoords)
                          )
                    )
                   (t
                    ;; todo
                    ))
             :kin-attention-coords-list
             (list (send (car *robot-obj-env-list*) :robot :get :rleg-contact-coords)
                   (send (car *robot-obj-env-list*) :robot :get :lleg-contact-coords)
                   (send (car *robot-obj-env-list*) :robot :get :rarm-grasp-coords)
                   (send (car *robot-obj-env-list*) :robot :get :larm-grasp-coords)
                   (send (car *robot-obj-env-list*) :object :get :object-coords)
                   )
             :contact-target-coords-list
             (cond (fix-grasp?
                    (list (send (car *robot-obj-env-list*) :contact "rleg" :contact-coords)
                          (send *arrow-lleg* :copy-worldcoords)
                          (send (car *robot-obj-env-list*) :object :handle-side-rarm)
                          (send (car *robot-obj-env-list*) :object :handle-side-larm)
                          )
                    )
                   (t
                    ;; todo
                    ))
             :contact-attention-coords-list
             (list (send (car *robot-obj-env-list*) :robot :get :rleg-contact-coords)
                   (send (car *robot-obj-env-list*) :robot :get :lleg-contact-coords)
                   (send (car *robot-obj-env-list*) :robot :get :rarm-grasp-coords)
                   (send (car *robot-obj-env-list*) :robot :get :larm-grasp-coords)
                   )
             :contact-constraint-list
             (list *rleg-contact-constraint*
                   *lleg-contact-constraint*
                   *rarm-grasp-constraint*
                   *larm-grasp-constraint*
                   )
             :contact-target-coords-obj-list
             (cond (fix-grasp?
                    (list (send (car *robot-obj-env-list*) :object :handle-side-rarm)
                          (send (car *robot-obj-env-list*) :object :handle-side-larm)
                          )
                    )
                   (t
                    ;; todo
                    ))
             :contact-constraint-obj-list
             (list *rarm-grasp-constraint-obj*
                   *larm-grasp-constraint-obj*
                   )
             :act-react-pair-list
             (list
              (list (send (car *robot-obj-env-list*) :object :handle-side-rarm) (send (car *robot-obj-env-list*) :object :handle-side-rarm))
              (list (send (car *robot-obj-env-list*) :object :handle-side-larm) (send (car *robot-obj-env-list*) :object :handle-side-larm))
              )
             :collision-pair-list
             (if avoid-collision?
                 (mapcar
                  #'(lambda (l)
                      (list l (send (car *robot-obj-env-list*) :object :get :convex-hull-body)))
                  (flatten
                   (mapcar #'(lambda (limb) (send (car *robot-obj-env-list*) :robot limb :links))
                           (list :torso :rleg :lleg))
                   )))
             :collision-distance-margin 0.05
             :delta-linear-joint 100 ;; [mm]
             :delta-rotational-joint 30 ;; [deg]
             )
   *instant-manip-config-task-list*
   )

  (push (copy-object *robot-obj-env*) *robot-obj-env-list*)
  (push
   (instance instant-manipulation-configuration-task :init
             :name :obj-start-above
             :robot-env (car *robot-obj-env-list*)
             :drive-joint-list
             (if optimize-torque? (send (car *robot-obj-env-list*) :drive-joint-list) nil)
             :posture-joint-list
             (send (car *robot-obj-env-list*) :robot :joint-list)
             :posture-joint-angle-list
             (send-all (send (car *robot-obj-env-list*) :robot :joint-list) :joint-angle)
             :kin-target-coords-list
             (cond (fix-grasp?
                    (list (send (car *robot-obj-env-list*) :contact "rleg" :contact-coords)
                          (send *arrow-lleg* :copy-worldcoords)
                          (send (car *robot-obj-env-list*) :object :handle-side-rarm)
                          (send (car *robot-obj-env-list*) :object :handle-side-larm)
                          (send (send *arrow-obj-start* :copy-worldcoords) :translate (float-vector 0 0 50))
                          )
                    )
                   (t
                    ;; todo
                    ))
             :kin-attention-coords-list
             (list (send (car *robot-obj-env-list*) :robot :get :rleg-contact-coords)
                   (send (car *robot-obj-env-list*) :robot :get :lleg-contact-coords)
                   (send (car *robot-obj-env-list*) :robot :get :rarm-grasp-coords)
                   (send (car *robot-obj-env-list*) :robot :get :larm-grasp-coords)
                   (send (car *robot-obj-env-list*) :object :get :object-coords)
                   )
             :contact-target-coords-list
             (cond (fix-grasp?
                    (list (send (car *robot-obj-env-list*) :contact "rleg" :contact-coords)
                          (send *arrow-lleg* :copy-worldcoords)
                          (send (car *robot-obj-env-list*) :object :handle-side-rarm)
                          (send (car *robot-obj-env-list*) :object :handle-side-larm)
                          )
                    )
                   (t
                    ;; todo
                    ))
             :contact-attention-coords-list
             (list (send (car *robot-obj-env-list*) :robot :get :rleg-contact-coords)
                   (send (car *robot-obj-env-list*) :robot :get :lleg-contact-coords)
                   (send (car *robot-obj-env-list*) :robot :get :rarm-grasp-coords)
                   (send (car *robot-obj-env-list*) :robot :get :larm-grasp-coords)
                   )
             :contact-constraint-list
             (list *rleg-contact-constraint*
                   *lleg-contact-constraint*
                   *rarm-grasp-constraint*
                   *larm-grasp-constraint*
                   )
             :contact-target-coords-obj-list
             (cond (fix-grasp?
                    (list (send (car *robot-obj-env-list*) :object :handle-side-rarm)
                          (send (car *robot-obj-env-list*) :object :handle-side-larm)
                          )
                    )
                   (t
                    ;; todo
                    ))
             :contact-constraint-obj-list
             (list *rarm-grasp-constraint-obj*
                   *larm-grasp-constraint-obj*
                   )
             :act-react-pair-list
             (list
              (list (send (car *robot-obj-env-list*) :object :handle-side-rarm) (send (car *robot-obj-env-list*) :object :handle-side-rarm))
              (list (send (car *robot-obj-env-list*) :object :handle-side-larm) (send (car *robot-obj-env-list*) :object :handle-side-larm))
              )
             :collision-pair-list
             (if avoid-collision?
                 (mapcar
                  #'(lambda (l)
                      (list l (send (car *robot-obj-env-list*) :object :get :convex-hull-body)))
                  (flatten
                   (mapcar #'(lambda (limb) (send (car *robot-obj-env-list*) :robot limb :links))
                           (list :torso :rleg :lleg))
                   )))
             :collision-distance-margin 0.05
             :delta-linear-joint 100 ;; [mm]
             :delta-rotational-joint 30 ;; [deg]
             )
   *instant-manip-config-task-list*
   )

  (dotimes (i 2)
    (push (copy-object *robot-obj-env*) *robot-obj-env-list*)
    (push
     (instance instant-manipulation-configuration-task :init
               :name :obj-lifting
               :robot-env (car *robot-obj-env-list*)
               :drive-joint-list
               (if optimize-torque? (send (car *robot-obj-env-list*) :drive-joint-list) nil)
               :posture-joint-list
               (send (car *robot-obj-env-list*) :robot :joint-list)
               :posture-joint-angle-list
               (send-all (send (car *robot-obj-env-list*) :robot :joint-list) :joint-angle)
               :kin-target-coords-list
               (cond (fix-grasp?
                      (list (send (car *robot-obj-env-list*) :contact "rleg" :contact-coords)
                            (send *arrow-lleg* :copy-worldcoords)
                            (send (car *robot-obj-env-list*) :object :handle-side-rarm)
                            (send (car *robot-obj-env-list*) :object :handle-side-larm)
                            )
                      )
                     (t
                      ;; todo
                      ))
               :kin-attention-coords-list
               (list (send (car *robot-obj-env-list*) :robot :get :rleg-contact-coords)
                     (send (car *robot-obj-env-list*) :robot :get :lleg-contact-coords)
                     (send (car *robot-obj-env-list*) :robot :get :rarm-grasp-coords)
                     (send (car *robot-obj-env-list*) :robot :get :larm-grasp-coords)
                     )
               :contact-target-coords-list
               (cond (fix-grasp?
                      (list (send (car *robot-obj-env-list*) :contact "rleg" :contact-coords)
                            (send *arrow-lleg* :copy-worldcoords)
                            (send (car *robot-obj-env-list*) :object :handle-side-rarm)
                            (send (car *robot-obj-env-list*) :object :handle-side-larm)
                            )
                      )
                     (t
                      ;; todo
                      ))
               :contact-attention-coords-list
               (list (send (car *robot-obj-env-list*) :robot :get :rleg-contact-coords)
                     (send (car *robot-obj-env-list*) :robot :get :lleg-contact-coords)
                     (send (car *robot-obj-env-list*) :robot :get :rarm-grasp-coords)
                     (send (car *robot-obj-env-list*) :robot :get :larm-grasp-coords)
                     )
               :contact-constraint-list
               (list *rleg-contact-constraint*
                     *lleg-contact-constraint*
                     *rarm-grasp-constraint*
                     *larm-grasp-constraint*
                     )
               :contact-target-coords-obj-list
               (cond (fix-grasp?
                      (list (send (car *robot-obj-env-list*) :object :handle-side-rarm)
                            (send (car *robot-obj-env-list*) :object :handle-side-larm)
                            )
                      )
                     (t
                      ;; todo
                      ))
               :contact-constraint-obj-list
               (list *rarm-grasp-constraint-obj*
                     *larm-grasp-constraint-obj*
                     )
               :act-react-pair-list
               (list
                (list (send (car *robot-obj-env-list*) :object :handle-side-rarm) (send (car *robot-obj-env-list*) :object :handle-side-rarm))
                (list (send (car *robot-obj-env-list*) :object :handle-side-larm) (send (car *robot-obj-env-list*) :object :handle-side-larm))
                )
               :collision-pair-list
               (if avoid-collision?
                   (mapcar
                    #'(lambda (l)
                        (list l (send (car *robot-obj-env-list*) :object :get :convex-hull-body)))
                    (flatten
                     (mapcar #'(lambda (limb) (send (car *robot-obj-env-list*) :robot limb :links))
                             (list :torso :rleg :lleg))
                     )))
               :collision-distance-margin 0.05
               :delta-linear-joint 100 ;; [mm]
               :delta-rotational-joint 30 ;; [deg]
               )
     *instant-manip-config-task-list*
     )
    )

  (push (copy-object *robot-obj-env*) *robot-obj-env-list*)
  (push
   (instance instant-manipulation-configuration-task :init
             :name :obj-finish
             :robot-env (car *robot-obj-env-list*)
             :drive-joint-list
             (if (or optimize-torque? optimize-start-end-torque?) (send (car *robot-obj-env-list*) :drive-joint-list) nil)
             :posture-joint-list
             (send (car *robot-obj-env-list*) :robot :joint-list)
             :posture-joint-angle-list
             (send-all (send (car *robot-obj-env-list*) :robot :joint-list) :joint-angle)
             :kin-target-coords-list
             (cond (fix-grasp?
                    (list (send (car *robot-obj-env-list*) :contact "rleg" :contact-coords)
                          (send *arrow-lleg* :copy-worldcoords)
                          (send (car *robot-obj-env-list*) :object :handle-side-rarm)
                          (send (car *robot-obj-env-list*) :object :handle-side-larm)
                          (send *arrow-obj-finish* :copy-worldcoords)
                          )
                    )
                   (t
                    ;; todo
                    ))
             :kin-attention-coords-list
             (list (send (car *robot-obj-env-list*) :robot :get :rleg-contact-coords)
                   (send (car *robot-obj-env-list*) :robot :get :lleg-contact-coords)
                   (send (car *robot-obj-env-list*) :robot :get :rarm-grasp-coords)
                   (send (car *robot-obj-env-list*) :robot :get :larm-grasp-coords)
                   (send (car *robot-obj-env-list*) :object :get :object-coords)
                   )
             :contact-target-coords-list
             (cond (fix-grasp?
                    (list (send (car *robot-obj-env-list*) :contact "rleg" :contact-coords)
                          (send *arrow-lleg* :copy-worldcoords)
                          (send (car *robot-obj-env-list*) :object :handle-side-rarm)
                          (send (car *robot-obj-env-list*) :object :handle-side-larm)
                          )
                    )
                   (t
                    ;; todo
                    ))
             :contact-attention-coords-list
             (list (send (car *robot-obj-env-list*) :robot :get :rleg-contact-coords)
                   (send (car *robot-obj-env-list*) :robot :get :lleg-contact-coords)
                   (send (car *robot-obj-env-list*) :robot :get :rarm-grasp-coords)
                   (send (car *robot-obj-env-list*) :robot :get :larm-grasp-coords)
                   )
             :contact-constraint-list
             (list *rleg-contact-constraint*
                   *lleg-contact-constraint*
                   *rarm-grasp-constraint*
                   *larm-grasp-constraint*
                   )
             :contact-target-coords-obj-list
             (cond (fix-grasp?
                    (list (send (car *robot-obj-env-list*) :object :handle-side-rarm)
                          (send (car *robot-obj-env-list*) :object :handle-side-larm)
                          )
                    )
                   (t
                    ;; todo
                    ))
             :contact-constraint-obj-list
             (list *rarm-grasp-constraint-obj*
                   *larm-grasp-constraint-obj*
                   )
             :act-react-pair-list
             (list
              (list (send (car *robot-obj-env-list*) :object :handle-side-rarm) (send (car *robot-obj-env-list*) :object :handle-side-rarm))
              (list (send (car *robot-obj-env-list*) :object :handle-side-larm) (send (car *robot-obj-env-list*) :object :handle-side-larm))
              )
             :collision-pair-list
             (if avoid-collision?
                 (mapcar
                  #'(lambda (l)
                      (list l (send (car *robot-obj-env-list*) :object :get :convex-hull-body)))
                  (flatten
                   (mapcar #'(lambda (limb) (send (car *robot-obj-env-list*) :robot limb :links))
                           (list :torso :rleg :lleg))
                   )))
             :collision-distance-margin 0.05
             :delta-linear-joint 100 ;; [mm]
             :delta-rotational-joint 30 ;; [deg]
             )
   *instant-manip-config-task-list*
   )

  (objects (append *robot-obj-env-list* *env-list*))

  (setq *trajectory-manip-config-task*
        (instance* trajectory-configuration-task :init
                   (append
                    (list
                     :instant-config-task-list (reverse *instant-manip-config-task-list*)
                     :norm-regular-scale-max 5e-4
                     :norm-regular-scale-offset 1e-7
                     :torque-regular-scale 1e-4
                     )
                    (cond (use-adjacent-regular-scale-list?
                           (list :adjacent-regular-scale-list (list 5e-4 5e-4 5e-4 5e-4 5e-4 5e-4 5e-4)))
                           (t
                            (list :adjacent-regular-scale 5e-4))
                           ))
                    )
                   )

  (setq *sqp-opt*
        (instance sqp-optimization :init
                  :config-task *trajectory-manip-config-task*
                  :pre-process-func
                  pre-process-func
                  :post-process-func
                  #'(lambda (sqp)
                      (when (= (mod (send sqp :iteration) 20) 0)
                        ;; (validate-element-jacobian-manipulation :config-task (send sqp :config-task))
                        ;; (validate-jacobian :config-task (send sqp :config-task))
                        ))
                  ))
  (send *sqp-opt* :optimize :loop-num 50)

  (objects (append (list *robot-obj-env*) *env-list*))
  (send *trajectory-manip-config-task* :play-animation
        :robot-env *robot-obj-env*
        :loop? visualize-loop?
        :visualize-callback-func visualize-callback-func
        )
  )
(warn "(sample-sqp-optimization-trajectory-manip)~%")
