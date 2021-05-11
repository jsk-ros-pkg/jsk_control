(load "./sample-sqp-optimization-instant.l")
(load "../sqp-msc-optimization.l")
(load "../inverse-kinematics-wrapper.l")


(defun sample-sqp-msc-optimization-instant
    (&key
     (num-of-solution-candidates 4)
     (optimize-torque? nil)
     (only-kinematics? nil)
     (pre-process-func)
     (visualize-callback-func)
     (visualize-loop? t)
     &allow-other-keys
     )
  ;; setup environment
  (setq *ground-rleg* (make-cube 400 400 20))
  (setq *ground-lleg* (make-cube 400 400 20))
  (setq *ground-larm* (make-cube 500 500 20))
  (send *ground-rleg* :newcoords
        (make-coords :pos (float-vector 0 -250 300) :rpy (list 0 0 (deg2rad -30))))
  (send *ground-lleg* :newcoords
        (make-coords :pos (float-vector -100 200 200) :rpy (list 0 (deg2rad 20) 0)))
  (send *ground-larm* :newcoords
        (make-coords :pos (float-vector 700 500 1000) :rpy (list 0 (deg2rad -45) (deg2rad 45))))
  (setq *env-list* (list *ground-rleg* *ground-lleg* *ground-larm*))
  (send-all *env-list* :worldcoords)
  (send-all *env-list* :set-color (float-vector 0.5 0.5 0.25))
  (setq *arrow* (instance arrow-object :init))
  (send *arrow* :newcoords
        (make-coords :pos (float-vector 600 -300 1200)))
  (setq *obst* (make-cube 100 100 800))
  (send *obst* :newcoords (make-coords :pos (float-vector 400 -250 1000) :rpy (list (deg2rad 45) 0 0)))
  (send *obst* :set-color (float-vector 1 0 0) 0.5)
  ;; setup robot
  (setup-hrp2jsknts-for-sample-sqp-optimization :only-kinematics? only-kinematics?)
  (send *robot* :init-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  (setq *contact-list*
        (list (instance 2d-planar-contact :init :name "rleg")
              (instance 2d-planar-contact :init :name "lleg")
              (instance 2d-planar-contact :init :name "larm")
              (instance contact-ik-arg :init
                        :name "target-coords"
                        :target-coords (send *arrow* :copy-worldcoords)
                        :translation-axis t
                        :rotation-axis nil
                        )
              ))
  (setq *robot-env*
        (instance robot-environment :init :robot *robot* :contact-list *contact-list*))
  (send *robot-env* :contact "rleg" :set-from-face :face (elt (send *ground-rleg* :faces) 1))
  (send *robot-env* :contact "lleg" :set-from-face :face (elt (send *ground-lleg* :faces) 1))
  (send *robot-env* :contact "larm" :set-from-face :face (elt (send *ground-larm* :faces) 1))
  (send *robot-env* :contact "rleg" :rotate pi :z :local)
  (send *robot-env* :contact "lleg" :rotate pi :z :local)
  ;; draw
  (unless (boundp '*irtviewer*)
    (make-irtviewer)
    (send *irtviewer* :change-background (float-vector 1 1 1))
    (send *irtviewer* :draw-floor 500)
    (send *irtviewer* :floor-color (float-vector 0 0 0))
    )
  (setq *instant-config-task*
        (instance instant-configuration-task :init
                  :robot-env *robot-env*
                  :drive-joint-list
                  (if (and (not only-kinematics?) optimize-torque?)
                      (send *robot-env* :drive-joint-list) nil)
                  :only-kinematics? only-kinematics?
                  :posture-joint-list
                  (send *robot-env* :robot :joint-list)
                  :posture-joint-angle-list
                  (send-all (send *robot-env* :robot :joint-list) :joint-angle)
                  :kin-target-coords-list
                  (list (send *robot-env* :contact "rleg" :contact-coords)
                        (send *robot-env* :contact "lleg" :contact-coords)
                        (send *robot-env* :contact "target-coords" :contact-coords)
                        (send *robot-env* :contact "larm" :contact-coords)
                        )
                  :kin-attention-coords-list
                  (list (send *robot-env* :robot :get :rleg-contact-coords)
                        (send *robot-env* :robot :get :lleg-contact-coords)
                        (send *robot-env* :robot :get :rarm-contact-coords)
                        (send *robot-env* :robot :get :larm-contact-coords)
                        )
                  :contact-target-coords-list
                  (if (not only-kinematics?)
                      (list (send *robot-env* :contact "rleg" :contact-coords)
                            (send *robot-env* :contact "lleg" :contact-coords)
                            (send *robot-env* :contact "larm" :contact-coords)
                            ))
                  :contact-attention-coords-list
                  (if (not only-kinematics?)
                      (list (send *robot-env* :robot :get :rleg-contact-coords)
                            (send *robot-env* :robot :get :lleg-contact-coords)
                            (send *robot-env* :robot :get :larm-contact-coords)
                            ))
                  :contact-constraint-list
                  (if (not only-kinematics?)
                      (list
                       (instance* default-contact-constraint
                                  :init
                                  :mu-trans 0.2
                                  :mu-rot 0.01
                                  :max-fz 400
                                  (let* ((poly
                                          (send *robot-env* :robot :get :rleg_link5-support-polygon))
                                         (vs
                                          (mapcar #'(lambda (v)
                                                      (send (send (send *robot-env* :robot :get :rleg-contact-coords) :worldcoords)
                                                            :inverse-transform-vector v))
                                                  (send poly :vertices)
                                                  ))
                                         )
                                    (list :l-min-x (+ (elt (find-extream vs #'(lambda (v) (elt v 0)) #'<) 0) 20)
                                          :l-max-x (- (elt (find-extream vs #'(lambda (v) (elt v 0)) #'>) 0) 20)
                                          :l-min-y (+ (elt (find-extream vs #'(lambda (v) (elt v 1)) #'<) 1) 20)
                                          :l-max-y (- (elt (find-extream vs #'(lambda (v) (elt v 1)) #'>) 1) 20)
                                          ))
                                  )
                       (instance* default-contact-constraint
                                  :init
                                  :mu-trans 0.2
                                  :mu-rot 0.01
                                  :max-fz 400
                                  (let* ((poly
                                          (send *robot-env* :robot :get :lleg_link5-support-polygon))
                                         (vs
                                          (mapcar #'(lambda (v)
                                                      (send (send (send *robot-env* :robot :get :lleg-contact-coords) :worldcoords)
                                                            :inverse-transform-vector v))
                                                  (send poly :vertices)
                                                  ))
                                         )
                                    (list :l-min-x (+ (elt (find-extream vs #'(lambda (v) (elt v 0)) #'<) 0) 20)
                                          :l-max-x (- (elt (find-extream vs #'(lambda (v) (elt v 0)) #'>) 0) 20)
                                          :l-min-y (+ (elt (find-extream vs #'(lambda (v) (elt v 1)) #'<) 1) 20)
                                          :l-max-y (- (elt (find-extream vs #'(lambda (v) (elt v 1)) #'>) 1) 20)
                                          ))
                                  )
                       (instance default-contact-constraint
                                 :init
                                 :mu-trans 0.2
                                 :mu-rot 0.01
                                 :max-fz 200
                                 :contact-face
                                 (send *robot-env* :robot :get :larm_link6-support-polygon)
                                 )
                       ))
                  :collision-pair-list
                  (mapcar #'(lambda (l) (list l *obst*)) (send *robot-env* :robot :links))
                  :collision-distance-margin 0.1
                  :delta-linear-joint 100 ;; [mm]
                  :delta-rotational-joint 30 ;; [deg]
                  ))

  (setq *sqp-opt*
        (instance sqp-msc-optimization :init
                  :config-task *instant-config-task*
                  :pre-process-func
                  pre-process-func
                  :post-process-func
                  #'(lambda (sqp)
                      (when (= (mod (send sqp :iteration) 20) 0)
                        ;; (validate-element-jacobian :config-task (send sqp :config-task))
                        ;; (validate-jacobian :config-task (send sqp :config-task))
                        ))
                  :num-msc num-of-solution-candidates
                  :dispersion-scale 1e-1
                  :dispersion-scale-min 1e-10
                  :dispersion-scale-decrease-ratio 0.8
                  :config-vector-dist2-min 1e-15
                  ))
  (objects
   (append (send-all (send *sqp-opt* :config-task-list) :robot-env) *env-list* (list *arrow* *obst*)))
  (send *sqp-opt* :optimize :loop-num 80)

  (do-until-key
   (dolist (config-task (send *sqp-opt* :config-task-list))
     (objects (append *env-list* (list (send config-task :robot-env) *arrow* *obst*)))
     (unix::usleep (* 1000 1000))
     (when visualize-callback-func
       (dotimes (i 20) (funcall visualize-callback-func)))
     )
   (unless visualize-loop?
     (return-from nil nil))
   )
  )
(warn "(sample-sqp-msc-optimization-instant)~%")
