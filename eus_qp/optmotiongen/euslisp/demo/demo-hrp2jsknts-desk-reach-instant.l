(load "package://eus_qp/optmotiongen/euslisp/instant-configuration-task.l")
(load "package://eus_qp/optmotiongen/euslisp/sqp-optimization.l")
(load "package://eus_qp/optmotiongen/euslisp/validate-jacobian.l")
(require :hrp2jsknts "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknts-interface.l")
(require "models/arrow-object.l")
(require "models/room73b2-askul-1200x700-desk-0-object.l")


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

(defun setup-hrp2jsknts-for-demo-hrp2jsknts-desk-reach
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
  (send *robot* :put :larm-thumb-contact-coords
        (make-cascoords :name :larm-contact-coords
                        :parent (send *robot* :link "LARM_LINK6")
                        :coords (send (send (send (send *robot* :link "LARM_LINK6") :copy-worldcoords)
                                            :translate (scale 1e3 (float-vector -0.005 -0.045 -0.14055)))
                                      :rotate pi/2 :z :local)
                        ))
  (unless only-kinematics?
    ;; create support polygon
    (send *robot* :init-pose)
    (send *robot* :fix-leg-to-coords (make-coords))
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

(defun demo-hrp2jsknts-desk-reach-instant
    (&key
     (use-thumb-contact? nil)
     (optimize-torque? nil)
     (avoid-collision? t)
     (apply-external-force? nil)
     (fix-contact? nil)
     (only-kinematics? nil)
     (pre-process-func)
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
         (send *robot-env* :contact "rleg" :set-from-face :face
               (elt (send *ground* :faces) 1))
         (send *robot-env* :contact "lleg" :set-from-face :face
               (elt (send *ground* :faces) 1))
         (send *robot-env* :contact "larm" :set-from-face :face
               (elt (send (elt (send *desk* :bodies) 0) :faces) 3))
         (send *robot-env* :contact "rleg" :rotate pi :z :local)
         (send *robot-env* :contact "lleg" :rotate pi :z :local)
         (send *robot-env* :contact "larm" :rotate pi :z :local)
         ;; (send *robot-env* :contact "larm" :translate (float-vector 0 300 0) :local)
         ;; (send (send *robot-env* :contact "larm" :joint :larm-root-virtual-y)
         ;;       :max-angle 100)
         ;; (send (send *robot-env* :contact "larm" :joint :larm-root-virtual-y)
         ;;       :min-angle -100)
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
         ))
  ;; draw
  (unless (boundp '*irtviewer*)
    (make-irtviewer)
    (send *irtviewer* :change-background (float-vector 1 1 1))
    (send *irtviewer* :draw-floor 500)
    (send *irtviewer* :floor-color (float-vector 0 0 0))
    )
  (objects (append (list *robot* *arrow*) *env-list*))

  (send *robot-env* :robot :reset-manip-pose)
  (when use-thumb-contact?
    (send *robot* :hand :larm :angle-vector #f(90.0 90.0 -10.0 10.0 -38.0 -38.0)))
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
                         (list (send *ground* :copy-worldcoords)
                               (send *ground* :copy-worldcoords)
                               (send *arrow* :copy-worldcoords)
                               (send *ground* :copy-worldcoords)
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
                        (if use-thumb-contact?
                            (send *robot-env* :robot :get :larm-thumb-contact-coords)
                          (send *robot-env* :robot :get :larm-contact-coords))
                        )
                  :contact-target-coords-list
                  (if (not only-kinematics?)
                      (cond (fix-contact?
                             (list (send *ground* :copy-worldcoords)
                                   (send *ground* :copy-worldcoords)
                                   (send *ground* :copy-worldcoords)
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
                            (if use-thumb-contact?
                                (send *robot-env* :robot :get :larm-thumb-contact-coords)
                              (send *robot-env* :robot :get :larm-contact-coords))
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
                                 :max-fz 400
                                 :l-min-x -1.0
                                 :l-max-x 1.0
                                 :l-min-y -1.0
                                 :l-max-y 1.0
                                 ;; :contact-face
                                 ;; (send *robot-env* :robot :get :larm_link6-support-polygon)
                                 )
                       ))
                  :collision-pair-list
                  ;; (if avoid-collision?
                  ;;     (mapcar #'(lambda (l) (list l *desk*)) (flatten (send *robot-env* :robot :legs :links))))
                  (if avoid-collision?
                      (append
                       (mapcar #'(lambda (l) (list l *desk*))
                               (remove-if
                                #'(lambda (l) (or (substringp "LARM" (send l :name))
                                                  (substringp "HEAD" (send l :name))))
                                (send *robot-env* :robot :links)))
                       (mapcar #'(lambda (l) (list l *obst-on-desk*))
                               (remove-if
                                #'(lambda (l) (or (substringp "LARM" (send l :name))
                                                  (substringp "HEAD" (send l :name))))
                                (send *robot-env* :robot :links)))
                       (list
                        (list (send *robot-env* :robot :link "LLEG_LINK2")
                              (car (send (send *robot-env* :robot :hand :larm) :links)))
                        (list (send *robot-env* :robot :link "LLEG_LINK3")
                              (car (send (send *robot-env* :robot :hand :larm) :links)))
                        )
                       (list
                        (list (send *robot-env* :robot :link "RLEG_LINK2")
                              (car (send (send *robot-env* :robot :hand :rarm) :links)))
                        (list (send *robot-env* :robot :link "RLEG_LINK3")
                              (car (send (send *robot-env* :robot :hand :rarm) :links)))
                        (list (send *robot-env* :robot :link "CHEST_LINK1")
                              (send *robot-env* :robot :link "LARM_LINK6"))
                        (list (send *robot-env* :robot :link "BODY")
                              (send *robot-env* :robot :link "LARM_LINK6"))
                        )
                       (list
                        (list (send *robot-env* :robot :link "RLEG_LINK3")
                              (send *robot-env* :robot :link "LLEG_LINK3"))
                        )
                       ))
                  :collision-distance-margin 0.05
                  :external-wrench-list
                  (if (and (not only-kinematics?) apply-external-force?)
                      (list (float-vector 0 100 0 0 0 0)))
                  :external-coords-list
                  (if (and (not only-kinematics?) apply-external-force?)
                      (list (send *robot-env* :robot :get :rarm-contact-coords)))
                  :norm-regular-scale-max
                  (cond (only-kinematics? 1e-4)
                        (optimize-torque? 1e-4)
                        (t 5e-3))
                  ))

  (setq *sqp-opt*
        (instance sqp-optimization :init
                  :config-task *instant-config-task*
                  :pre-process-func
                  pre-process-func
                  :post-process-func
                  #'(lambda (sqp)
                      (when (= (mod (send sqp :iteration) 20) 0)
                        ;; (validate-element-jacobian :config-task (send sqp :config-task))
                        ;; (validate-jacobian :config-task (send sqp :config-task))
                        ))
                  ))
  (send *sqp-opt* :optimize :loop-num 50)
  )
(warn "(demo-hrp2jsknts-desk-reach-instant)~%")
