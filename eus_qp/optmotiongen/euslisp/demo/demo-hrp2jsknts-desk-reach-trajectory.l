(load "./demo-hrp2jsknts-desk-reach-instant.l")
(load "package://eus_qp/optmotiongen/euslisp/trajectory-configuration-task.l")


(defun setup-contact-constraint-for-demo-hrp2jsknts-desk-reach
    ()
  (send (send *robot-env* :robot :get :rleg-contact-coords)
        :put :contact-constraint
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
        )
  (send (send *robot-env* :robot :get :lleg-contact-coords)
        :put :contact-constraint
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
        )
  (send (send *robot-env* :robot :get :larm-thumb-contact-coords)
        :put :contact-constraint
        (instance default-contact-constraint
                  :init
                  :mu-trans 0.2
                  :mu-rot 0.01
                  :max-fz 400
                  :l-min-x -1.0
                  :l-max-x 1.0
                  :l-min-y -1.0
                  :l-max-y 1.0
                  ;; :contact-face
                  ;; (send *robot-env* :robot :get :larm_link6-support-polygon)
                  )
        )
  (send (send *robot-env* :robot :get :larm-contact-coords)
        :put :contact-constraint
        (instance default-contact-constraint
                  :init
                  :mu-trans 0.2
                  :mu-rot 0.01
                  :max-fz 400
                  :l-min-x -1.0
                  :l-max-x 1.0
                  :l-min-y -1.0
                  :l-max-y 1.0
                  ;; :contact-face
                  ;; (send *robot-env* :robot :get :larm_link6-support-polygon)
                  )
        )
  )

(defun get-collision-pair-list-from-robot-env
    (robot-env)
  (append
   (mapcar #'(lambda (l) (list l *desk*))
           (remove-if
            #'(lambda (l) (or (substringp "LARM" (send l :name))
                              (substringp "HEAD" (send l :name))))
            (send robot-env :robot :links)))
   (mapcar #'(lambda (l) (list l *obst-on-desk*))
           (remove-if
            #'(lambda (l) (or (substringp "LARM" (send l :name))
                              (substringp "HEAD" (send l :name))))
            (send robot-env :robot :links)))
   (list
    (list (send robot-env :robot :link "LLEG_LINK2")
          (car (send (send robot-env :robot :hand :larm) :links)))
    (list (send robot-env :robot :link "LLEG_LINK3")
          (car (send (send robot-env :robot :hand :larm) :links)))
    )
   (list
    (list (send robot-env :robot :link "RLEG_LINK2")
          (car (send (send robot-env :robot :hand :rarm) :links)))
    (list (send robot-env :robot :link "RLEG_LINK3")
          (car (send (send robot-env :robot :hand :rarm) :links)))
    (list (send robot-env :robot :link "CHEST_LINK1")
          (send robot-env :robot :link "LARM_LINK6"))
    (list (send robot-env :robot :link "BODY")
          (send robot-env :robot :link "LARM_LINK6"))
    )
   (list
    (list (send robot-env :robot :link "RLEG_LINK3")
          (send robot-env :robot :link "LLEG_LINK3"))
    )
   )
  )

(defun demo-hrp2jsknts-desk-reach-trajectory
    (&key
     (use-thumb-contact? nil)
     (optimize-torque? nil)
     (optimize-torque-only-key-pose? t)
     (avoid-collision? t)
     (only-kinematics? nil)
     (output-filename "/tmp/demo-hrp2jsknts-desk-reach-trajectory")
     (keypose-output-file "/tmp/demo-hrp2jsknts-desk-reach-trajectory-keypose.l")
     (pre-process-func)
     (visualize-callback-func)
     (visualize-loop? t)
     &allow-other-keys
     )
  ;; setup target
  (setq *arrow* (instance arrow-object :init))
  ;; (send *arrow* :newcoords
  ;;       (make-coords :pos (float-vector 1300 -200 50) :rpy (list 0 (deg2rad -30) 0)))
  (send *arrow* :newcoords
        (make-coords :pos (float-vector 1300 -300 50) :rpy (list 0 (deg2rad -30) 0)))
  ;; setup environment
  (setq *ground* (make-cube 1000 2000 10))
  (send *ground* :newcoords (make-coords :pos (float-vector 500 0 -5)))
  ;; (setq *ground* (make-cube 2000 2000 10))
  ;; (send *ground* :newcoords (make-coords :pos (float-vector 1250 0 -5)))
  (setq *desk* (instance room73b2-askul-1200x700-desk-0-object :init))
  (send *desk* :newcoords
        (make-coords :pos (float-vector 1500 0 0) :rpy (list 0 0 0)))
  (setq *obst-on-desk* (make-cube 700 1200 1000))
  (send *obst-on-desk* :newcoords
        (send (send (send *desk* :worldcoords) :copy-worldcoords)
              :translate (float-vector 0 0 1200)))
  (send *obst-on-desk* :set-color #f(1 0 0) 0.1)
  (setq *env-list* (list *ground* *desk* *obst-on-desk*))
  (send-all *env-list* :worldcoords)
  (send-all (send *desk* :bodies) :worldcoords) ;; need this for some reason
  ;; setup robot
  (setup-hrp2jsknts-for-demo-hrp2jsknts-desk-reach :only-kinematics? only-kinematics?)
  ;; setup contact
  (setq *contact-list*
        (list (instance 2d-planar-contact :init :name "rleg")
              (instance 2d-planar-contact :init :name "lleg")
              (instance 2d-planar-contact :init :name "larm" :contact-pre-offset 200)
              ))
  (setq *robot-env*
        (instance robot-environment :init :robot *robot* :contact-list *contact-list*))
  (send *robot-env* :contact "rleg" :set-from-face :face
        (elt (send *ground* :faces) 1))
  (send *robot-env* :contact "lleg" :set-from-face :face
        (elt (send *ground* :faces) 1))
  (send *robot-env* :contact "larm" :set-from-face :face
        (elt (send (elt (send *desk* :bodies) 0) :faces) 3))
  (send *robot-env* :contact "rleg" :rotate pi :z :local)
  (send *robot-env* :contact "lleg" :rotate pi :z :local)
  (send *robot-env* :contact "larm" :rotate pi :z :local)
  (send (send *robot-env* :contact "larm" :joint :larm-root-virtual-yaw)
        :max-angle 0)
  (send (send *robot-env* :contact "larm" :joint :larm-root-virtual-yaw)
        :min-angle 0)
  (send (send *robot-env* :contact "rleg" :joint :rleg-root-virtual-yaw)
        :max-angle 60)
  (send (send *robot-env* :contact "rleg" :joint :rleg-root-virtual-yaw)
        :min-angle -60)
  (send (send *robot-env* :contact "lleg" :joint :lleg-root-virtual-yaw)
        :max-angle 60)
  (send (send *robot-env* :contact "lleg" :joint :lleg-root-virtual-yaw)
        :min-angle -60)
  (when use-thumb-contact?
    (send (send *robot-env* :contact "larm" :contact-pre-coords) :translate (float-vector 0 0 100) :world))
  ;; setup contact constraint
  (setup-contact-constraint-for-demo-hrp2jsknts-desk-reach)
  ;; set initial posture
  (send *robot-env* :robot :reset-manip-pose)
  (cond (use-thumb-contact?
         (send *robot-env* :robot :hand :larm :angle-vector #f(90.0 90.0 -10.0 10.0 -38.0 -38.0))
         )
        (t
         (send *robot-env* :robot :hand :larm :grasp-pose)
         ))
  ;; draw
  (unless (boundp '*irtviewer*)
    (make-irtviewer)
    (send *irtviewer* :change-background (float-vector 1 1 1))
    (send *irtviewer* :draw-floor 500)
    (send *irtviewer* :floor-color (float-vector 0 0 0))
    )

  (setq *robot-env-list* nil)
  (setq *instant-config-task-list* nil)

  (push (copy-object *robot-env*) *robot-env-list*)
  (push
   (instance instant-configuration-task :init
             :name :larm-no-touch
             :robot-env (car *robot-env-list*)
             :drive-joint-list
             (if (and (not only-kinematics?) optimize-torque?)
                 (send (car *robot-env-list*) :drive-joint-list) nil)
             :only-kinematics? only-kinematics?
             :posture-joint-list
             (send (car *robot-env-list*) :robot :joint-list)
             :posture-joint-angle-list
             (send-all (send (car *robot-env-list*) :robot :joint-list) :joint-angle)
             :kin-target-coords-list
             (list (send (car *robot-env-list*) :contact "rleg" :contact-coords)
                   (send (car *robot-env-list*) :contact "lleg" :contact-coords)
                   (send (car *robot-env-list*) :contact "larm" :contact-pre-coords)
                   )
             :kin-attention-coords-list
             (list (send (car *robot-env-list*) :robot :get :rleg-contact-coords)
                   (send (car *robot-env-list*) :robot :get :lleg-contact-coords)
                   (if use-thumb-contact?
                       (send (car *robot-env-list*) :robot :get :larm-thumb-contact-coords)
                     (send (car *robot-env-list*) :robot :get :larm-contact-coords))
                   )
             :contact-target-coords-list
             (if (not only-kinematics?)
                 (list (send (car *robot-env-list*) :contact "rleg" :contact-coords)
                       (send (car *robot-env-list*) :contact "lleg" :contact-coords)
                       ))
             :contact-attention-coords-list
             (if (not only-kinematics?)
                 (list (send (car *robot-env-list*) :robot :get :rleg-contact-coords)
                       (send (car *robot-env-list*) :robot :get :lleg-contact-coords)
                       ))
             :collision-pair-list
             (if avoid-collision?
                 (get-collision-pair-list-from-robot-env (car *robot-env-list*)))
             :collision-distance-margin 0.05
             )
   *instant-config-task-list*
   )

  (push (copy-object *robot-env*) *robot-env-list*)
  (push
   (instance instant-configuration-task :init
             :name :larm-no-contact-wrench
             :robot-env (car *robot-env-list*)
             :drive-joint-list
             (if (and (not only-kinematics?) optimize-torque?)
                 (send (car *robot-env-list*) :drive-joint-list) nil)
             :only-kinematics? only-kinematics?
             :posture-joint-list
             (send (car *robot-env-list*) :robot :joint-list)
             :posture-joint-angle-list
             (send-all (send (car *robot-env-list*) :robot :joint-list) :joint-angle)
             :kin-target-coords-list
             (list (send (car *robot-env-list*) :contact "rleg" :contact-coords)
                   (send (car *robot-env-list*) :contact "lleg" :contact-coords)
                   (send (car *robot-env-list*) :contact "larm" :contact-coords)
                   )
             :kin-attention-coords-list
             (list (send (car *robot-env-list*) :robot :get :rleg-contact-coords)
                   (send (car *robot-env-list*) :robot :get :lleg-contact-coords)
                   (if use-thumb-contact?
                       (send (car *robot-env-list*) :robot :get :larm-thumb-contact-coords)
                     (send (car *robot-env-list*) :robot :get :larm-contact-coords))
                   )
             :contact-target-coords-list
             (if (not only-kinematics?)
                 (list (send (car *robot-env-list*) :contact "rleg" :contact-coords)
                       (send (car *robot-env-list*) :contact "lleg" :contact-coords)
                       ))
             :contact-attention-coords-list
             (if (not only-kinematics?)
                 (list (send (car *robot-env-list*) :robot :get :rleg-contact-coords)
                       (send (car *robot-env-list*) :robot :get :lleg-contact-coords)
                       ))
             :collision-pair-list
             (if avoid-collision?
                 (get-collision-pair-list-from-robot-env (car *robot-env-list*)))
             :collision-distance-margin 0.05
             )
   *instant-config-task-list*
   )

  (push (copy-object *robot-env*) *robot-env-list*)
  (push
   (instance instant-configuration-task :init
             :name :larm-with-contact-wrench
             :robot-env (car *robot-env-list*)
             :drive-joint-list
             (if (and (not only-kinematics?) optimize-torque?)
                 (send (car *robot-env-list*) :drive-joint-list) nil)
             :only-kinematics? only-kinematics?
             :posture-joint-list
             (send (car *robot-env-list*) :robot :joint-list)
             :posture-joint-angle-list
             (send-all (send (car *robot-env-list*) :robot :joint-list) :joint-angle)
             :kin-target-coords-list
             (list (send (car *robot-env-list*) :contact "rleg" :contact-coords)
                   (send (car *robot-env-list*) :contact "lleg" :contact-coords)
                   (send (car *robot-env-list*) :contact "larm" :contact-coords)
                   )
             :kin-attention-coords-list
             (list (send (car *robot-env-list*) :robot :get :rleg-contact-coords)
                   (send (car *robot-env-list*) :robot :get :lleg-contact-coords)
                   (if use-thumb-contact?
                       (send (car *robot-env-list*) :robot :get :larm-thumb-contact-coords)
                     (send (car *robot-env-list*) :robot :get :larm-contact-coords))
                   )
             :contact-target-coords-list
             (if (not only-kinematics?)
                 (list (send (car *robot-env-list*) :contact "rleg" :contact-coords)
                       (send (car *robot-env-list*) :contact "lleg" :contact-coords)
                       (send (car *robot-env-list*) :contact "larm" :contact-coords)
                       ))
             :contact-attention-coords-list
             (if (not only-kinematics?)
                 (list (send (car *robot-env-list*) :robot :get :rleg-contact-coords)
                       (send (car *robot-env-list*) :robot :get :lleg-contact-coords)
                       (if use-thumb-contact?
                           (send (car *robot-env-list*) :robot :get :larm-thumb-contact-coords)
                         (send (car *robot-env-list*) :robot :get :larm-contact-coords))
                       ))
             :collision-pair-list
             (if avoid-collision?
                 (get-collision-pair-list-from-robot-env (car *robot-env-list*)))
             :collision-distance-margin 0.05
             )
   *instant-config-task-list*
   )

  (push (copy-object *robot-env*) *robot-env-list*)
  (push
   (instance instant-configuration-task :init
             :name :rarm-reaching
             :robot-env (car *robot-env-list*)
             :drive-joint-list
             (if (and (not only-kinematics?) (or optimize-torque? optimize-torque-only-key-pose?))
                 (send (car *robot-env-list*) :drive-joint-list) nil)
             :only-kinematics? only-kinematics?
             :posture-joint-list
             (send (car *robot-env-list*) :robot :joint-list)
             :posture-joint-angle-list
             (send-all (send (car *robot-env-list*) :robot :joint-list) :joint-angle)
             :kin-target-coords-list
             (list (send (car *robot-env-list*) :contact "rleg" :contact-coords)
                   (send (car *robot-env-list*) :contact "lleg" :contact-coords)
                   (send *arrow* :copy-worldcoords)
                   (send (car *robot-env-list*) :contact "larm" :contact-coords)
                   )
             :kin-attention-coords-list
             (list (send (car *robot-env-list*) :robot :get :rleg-contact-coords)
                   (send (car *robot-env-list*) :robot :get :lleg-contact-coords)
                   (send (car *robot-env-list*) :robot :get :rarm-contact-coords)
                   (if use-thumb-contact?
                       (send (car *robot-env-list*) :robot :get :larm-thumb-contact-coords)
                     (send (car *robot-env-list*) :robot :get :larm-contact-coords))
                   )
             :contact-target-coords-list
             (if (not only-kinematics?)
                 (list (send (car *robot-env-list*) :contact "rleg" :contact-coords)
                       (send (car *robot-env-list*) :contact "lleg" :contact-coords)
                       (send (car *robot-env-list*) :contact "larm" :contact-coords)
                       ))
             :contact-attention-coords-list
             (if (not only-kinematics?)
                 (list (send (car *robot-env-list*) :robot :get :rleg-contact-coords)
                       (send (car *robot-env-list*) :robot :get :lleg-contact-coords)
                       (if use-thumb-contact?
                           (send (car *robot-env-list*) :robot :get :larm-thumb-contact-coords)
                         (send (car *robot-env-list*) :robot :get :larm-contact-coords))
                       ))
             :collision-pair-list
             (if avoid-collision?
                 (get-collision-pair-list-from-robot-env (car *robot-env-list*)))
             :collision-distance-margin 0.05
             )
   *instant-config-task-list*
   )

  (objects (append *robot-env-list* *env-list*))

  (setq *trajectory-config-task*
        (instance* trajectory-configuration-task :init
                   :instant-config-task-list (reverse *instant-config-task-list*)
                   (if optimize-torque?
                       (list
                        :norm-regular-scale-max 1e-4
                        :norm-regular-scale-offset 1e-7
                        :adjacent-regular-scale 2e-3
                        :torque-regular-scale 1e-4
                        ))
                   ))

  (setq *sqp-opt*
        (instance sqp-optimization :init
                  :config-task *trajectory-config-task*
                  :pre-process-func
                  pre-process-func
                  :post-process-func
                  #'(lambda (sqp)
                      (when (= (mod (send sqp :iteration) 20) 0)
                        ;; (mapcar
                        ;;  #'(lambda (config-task) (validate-element-jacobian :config-task config-task))
                        ;;  ;; #'(lambda (config-task) (validate-jacobian :config-task config-task))
                        ;;  (send (send sqp :config-task) :instant-config-task-list))
                        ))
                  ))
  (send *sqp-opt* :optimize :loop-num 50)

  (when keypose-output-file
    (setq *av-list*
          (mapcar #'(lambda (config-task)
                      (coerce
                       (send-all (send (send config-task :robot-env) :robot :joint-list-orig) :joint-angle)
                       float-vector))
                  (send *trajectory-config-task* :instant-config-task-list)))
    (setq *rc-list*
          (mapcar #'(lambda (config-task)
                      (send (send (send config-task :robot-env) :robot :worldcoords) :copy-worldcoords))
                  (send *trajectory-config-task* :instant-config-task-list)))
    (dump-loadable-structure keypose-output-file *av-list* *rc-list*)
    )

  (when output-filename
    (setq *rs-list*
          (send *trajectory-config-task* :generate-robot-state-list
                :robot-env *robot-env*
                :joint-name-list (send-all (send *robot-env* :robot :joint-list) :name)
                :root-link-name (send (car (send *robot-env* :robot :links)) :name)
                :divide-num 200
                ))
    (objects (append (list *robot-env*) *env-list*))
    (play-robot-state-list
     :rs-list *rs-list*
     :robot-env *robot-env*
     :joint-name-list (send-all (send *robot-env* :robot :joint-list) :name)
     :root-link-name (send (car (send *robot-env* :robot :links)) :name)
     )
    (dump-loadable-structure (format nil "~a.l" output-filename) *rs-list*)
    (dump-seq-pattern-file
     *rs-list*
     output-filename
     :robot (send *robot-env* :robot)
     ))

  (objects (append (list *robot-env*) *env-list*))
  (send *trajectory-config-task* :play-animation
        :robot-env *robot-env*
        :loop? visualize-loop?
        :visualize-callback-func visualize-callback-func
        )
  )
(warn "(demo-hrp2jsknts-desk-reach-trajectory)~%")
