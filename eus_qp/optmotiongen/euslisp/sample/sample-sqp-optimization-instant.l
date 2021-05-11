(load "../instant-configuration-task.l")
(load "../sqp-optimization.l")
(load "../validate-jacobian.l")
(require :hrp2jsknts "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknts-interface.l")
(require "models/arrow-object.l")


(defmethod hrp2jsknts-robot
  (:joint-list
   ()
   ;; remove toe joint
   (set-difference joint-list
                   (list (send self :rleg :toe-p) (send self :lleg :toe-p)))
   )
  (:joint-list-orig
   ()
   joint-list
   )
  )

(defun setup-hrp2jsknts-for-sample-sqp-optimization
    (&key (only-kinematics? nil))
  (setq *robot* (instance hrp2jsknts-robot :init))
  ;; assoc contact coords
  (send *robot* :put :rleg-contact-coords
        (make-cascoords :name :rleg-contact-coords
                        :parent (send *robot* :link "RLEG_LINK5")
                        :coords (send (send *robot* :rleg-end-coords) :copy-worldcoords)
                        ))
  (send *robot* :put :lleg-contact-coords
        (make-cascoords :name :lleg-contact-coords
                        :parent (send *robot* :link "LLEG_LINK5")
                        :coords (send (send *robot* :lleg-end-coords) :copy-worldcoords)
                        ))
  (send *robot* :put :rarm-contact-coords
        (make-cascoords :name :rarm-contact-coords
                        :parent (send *robot* :link "RARM_LINK6")
                        :coords (send (send (send *robot* :link "RARM_LINK6") :copy-worldcoords)
                                      :translate (scale 1e3 (float-vector -0.0042 -0.0233 -0.2345)))
                        ))
  (send *robot* :put :larm-contact-coords
        (make-cascoords :name :larm-contact-coords
                        :parent (send *robot* :link "LARM_LINK6")
                        :coords (send (send (send *robot* :link "LARM_LINK6") :copy-worldcoords)
                                      :translate (scale 1e3 (float-vector -0.0042 0.0233 -0.2345)))
                        ))
  (send *robot* :init-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  (unless only-kinematics?
    ;; create support polygon
    (create-support-polygon *robot* (send *robot* :hand :rarm :link "RARM_LINK6"))
    (create-support-polygon *robot* (send *robot* :hand :larm :link "LARM_LINK6"))
    (create-support-polygon *robot* (send *robot* :link "RLEG_LINK5"))
    (create-support-polygon *robot* (send *robot* :link "LLEG_LINK5"))
    )
  ;; disable min-max table
  (dolist (jnt (send *robot* :joint-list))
    (send jnt :set-val 'joint-min-max-table nil)
    (send jnt :set-val 'joint-min-max-target nil)
    )
  ;; set narrow limit of min/max angle instead of min-max table
  (send (send *robot* :joint "RLEG_JOINT0") :min-angle -35)
  (send (send *robot* :joint "LLEG_JOINT0") :max-angle 35)
  (send (send *robot* :joint "RLEG_JOINT2") :min-angle -70)
  (send (send *robot* :joint "LLEG_JOINT2") :min-angle -70)
  (send (send *robot* :joint "CHEST_JOINT1") :min-angle -1)
  (send (send *robot* :joint "CHEST_JOINT1") :max-angle 42)
  (send (send *robot* :joint "RARM_JOINT5") :min-angle -50)
  (send (send *robot* :joint "RARM_JOINT5") :max-angle 50)
  (send (send *robot* :joint "RARM_JOINT6") :min-angle -50)
  (send (send *robot* :joint "RARM_JOINT6") :max-angle 50)
  (send (send *robot* :joint "LARM_JOINT5") :min-angle -50)
  (send (send *robot* :joint "LARM_JOINT5") :max-angle 50)
  (send (send *robot* :joint "LARM_JOINT6") :min-angle -50)
  (send (send *robot* :joint "LARM_JOINT6") :max-angle 50)
  )

(defun sample-sqp-optimization-instant
    (&key
     (optimize-torque? t)
     (avoid-collision? t)
     (apply-external-force? nil)
     (fix-contact? nil)
     (set-kin-scale? nil)
     (only-kinematics? nil)
     (pre-process-func)
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
        (make-coords :pos (float-vector 900 500 1000) :rpy (list 0 (deg2rad -45) (deg2rad 45))))
  (setq *env-list* (list *ground-rleg* *ground-lleg* *ground-larm*))
  (send-all *env-list* :worldcoords)
  (send-all *env-list* :set-color (float-vector 0.5 0.5 0.25))
  (setq *arrow* (instance arrow-object :init))
  (send *arrow* :newcoords
        (make-coords :pos (float-vector 1050 -500 1100) :rpy (list (deg2rad -45) -pi/2 0)))
  (when avoid-collision?
    (setq *obst* (make-cube 200 200 500))
    (send *obst* :translate (float-vector 500 0 250))
    (send *obst* :set-color (float-vector 1 0 0) 0.5)
    )
  ;; setup robot
  (setup-hrp2jsknts-for-sample-sqp-optimization :only-kinematics? only-kinematics?)
  (cond (fix-contact?
         (setq *robot-env*
               (instance robot-environment :init :robot *robot*))
         )
        (t
         (setq *contact-list*
               (list (instance 2d-planar-contact :init :name "rleg")
                     (instance 2d-planar-contact :init :name "lleg")
                     (instance 2d-planar-contact :init :name "larm")
                     ))
         (setq *robot-env*
               (instance robot-environment :init :robot *robot* :contact-list *contact-list*))
         (send *robot-env* :contact "rleg" :set-from-face :face (elt (send *ground-rleg* :faces) 1))
         (send *robot-env* :contact "lleg" :set-from-face :face (elt (send *ground-lleg* :faces) 1))
         (send *robot-env* :contact "larm" :set-from-face :face (elt (send *ground-larm* :faces) 1))
         (send *robot-env* :contact "rleg" :rotate pi :z :local)
         (send *robot-env* :contact "lleg" :rotate pi :z :local)
         ))
  ;; draw
  (unless (boundp '*irtviewer*)
    (make-irtviewer)
    (send *irtviewer* :change-background (float-vector 1 1 1))
    (send *irtviewer* :draw-floor 500)
    (send *irtviewer* :floor-color (float-vector 0 0 0))
    )
  (objects (append (list *robot* *arrow*) *env-list* (if avoid-collision? (list *obst*))))

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
                  (cond (fix-contact?
                         (list (send *ground-rleg* :copy-worldcoords)
                               (send *ground-lleg* :copy-worldcoords)
                               (send *arrow* :copy-worldcoords)
                               (send *ground-larm* :copy-worldcoords)
                               )
                         )
                        (t
                         (list (send *robot-env* :contact "rleg" :contact-coords)
                               (send *robot-env* :contact "lleg" :contact-coords)
                               (send *arrow* :copy-worldcoords)
                               (send *robot-env* :contact "larm" :contact-coords)
                               )
                         ))
                  :kin-attention-coords-list
                  (list (send *robot-env* :robot :get :rleg-contact-coords)
                        (send *robot-env* :robot :get :lleg-contact-coords)
                        (send *robot-env* :robot :get :rarm-contact-coords)
                        (send *robot-env* :robot :get :larm-contact-coords)
                        )
                  :contact-target-coords-list
                  (if (not only-kinematics?)
                      (cond (fix-contact?
                             (list (send *ground-rleg* :copy-worldcoords)
                                   (send *ground-lleg* :copy-worldcoords)
                                   (send *ground-larm* :copy-worldcoords)
                                   )
                             )
                            (t
                             (list (send *robot-env* :contact "rleg" :contact-coords)
                                   (send *robot-env* :contact "lleg" :contact-coords)
                                   (send *robot-env* :contact "larm" :contact-coords)
                                   )
                             )))
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
                  :kin-scale-list
                  (if set-kin-scale?
                      (list 1.0
                            10.0
                            (list 1 2 3 4 5 6)
                            (list 1 2 3 4 5 6)
                            ))
                  :collision-pair-list
                  (if avoid-collision?
                      (mapcar #'(lambda (l) (list l *obst*)) (flatten (send *robot-env* :robot :legs :links))))
                  :external-wrench-list
                  (if (and (not only-kinematics?) apply-external-force?)
                      (list (float-vector 0 100 0 0 0 0)))
                  :external-coords-list
                  (if (and (not only-kinematics?) apply-external-force?)
                      (list (send *robot-env* :robot :get :rarm-contact-coords)))
                  ))

  (setq *sqp-opt*
        (instance sqp-optimization :init
                  :config-task *instant-config-task*
                  :pre-process-func
                  pre-process-func
                  :post-process-func
                  #'(lambda (sqp)
                      (when (= (mod (send sqp :iteration) 20) 0)
                        (validate-element-jacobian :config-task (send sqp :config-task))
                        ;; (validate-jacobian :config-task (send sqp :config-task))
                        ))
                  ))
  (send *sqp-opt* :optimize :loop-num 50)
  )
(warn "(sample-sqp-optimization-instant)~%")
