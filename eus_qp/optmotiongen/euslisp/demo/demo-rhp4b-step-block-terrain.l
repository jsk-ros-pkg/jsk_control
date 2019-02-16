(load "../instant-configuration-task.l")
(load "../sqp-optimization.l")
(load "../validate-jacobian.l")
(require :rhp4b "package://khi_ros_bridge_tutorials/euslisp/rhp4b-interface.l")
(require "models/arrow-object.l")
(load "../trajectory-configuration-task.l")


(defun setup-rhp4b-for-demo-rhp4b-step-block-terrain
    ()
  (setq *robot* (instance rhp4b-robot :init))
  ;; assoc contact coords
  (send *robot* :put :rleg-contact-coords
        (make-cascoords :name :rleg-contact-coords
                        :parent (send *robot* :link "R_ANKLE_P_LINK")
                        :coords (send (send (send *robot* :rleg-end-coords) :copy-worldcoords)
                                      :translate (float-vector 0 0 30))
                        ))
  (send *robot* :put :lleg-contact-coords
        (make-cascoords :name :lleg-contact-coords
                        :parent (send *robot* :link "L_ANKLE_P_LINK")
                        :coords (send (send (send *robot* :lleg-end-coords) :copy-worldcoords)
                                      :translate (float-vector 0 0 30))
                        ))
  (send *robot* :put :rarm-contact-coords
        (make-cascoords :name :rarm-contact-coords
                        :parent (send *robot* :link "R_WRIST_Y_LINK")
                        :coords (send (send (send (send *robot* :rarm-end-coords) :copy-worldcoords)
                                            :translate (float-vector -45 0 0)) :rotate -pi/2 :y)

                        ))
  (send *robot* :put :larm-contact-coords
        (make-cascoords :name :larm-contact-coords
                        :parent (send *robot* :link "L_WRIST_Y_LINK")
                        :coords (send (send (send (send *robot* :larm-end-coords) :copy-worldcoords)
                                            :translate (float-vector -45 0 0)) :rotate -pi/2 :y)
                        ))
  ;; create support polygon
  (send *robot* :init-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  (create-support-polygon *robot* (send *robot* :link "L_WRIST_Y_LINK") :thre 10.0)
  (create-support-polygon *robot* (send *robot* :link "R_WRIST_Y_LINK") :thre 10.0)
  (create-support-polygon *robot* (send *robot* :link "L_ANKLE_P_LINK") :thre 10.0)
  (create-support-polygon *robot* (send *robot* :link "R_ANKLE_P_LINK") :thre 10.0)
  ;; disable min-max table
  (dolist (jnt (send *robot* :joint-list))
    (send jnt :set-val 'joint-min-max-table nil)
    (send jnt :set-val 'joint-min-max-target nil)
    )
  ;; overwrite max/min-angle
  (send (send *robot* :joint "L_CROTCH_Y") :max-angle 60)
  (send (send *robot* :joint "L_CROTCH_Y") :min-angle -60)
  (send (send *robot* :joint "R_CROTCH_Y") :max-angle 60)
  (send (send *robot* :joint "R_CROTCH_Y") :min-angle -60)
  (send (send *robot* :joint "L_SHOULDER_P") :max-angle 60)
  (send (send *robot* :joint "L_SHOULDER_P") :min-angle -120)
  (send (send *robot* :joint "R_SHOULDER_P") :max-angle 60)
  (send (send *robot* :joint "R_SHOULDER_P") :min-angle -120)
  ;; set torque limit
  (send (send *robot* :joint "L_CROTCH_Y") :max-joint-torque 114.37)
  (send (send *robot* :joint "L_CROTCH_P") :max-joint-torque 173.78)
  (send (send *robot* :joint "L_CROTCH_R") :max-joint-torque 173.78)
  (send (send *robot* :joint "L_KNEE_P") :max-joint-torque 434.45)
  (send (send *robot* :joint "L_ANKLE_R") :max-joint-torque 173.78)
  (send (send *robot* :joint "L_ANKLE_P") :max-joint-torque 173.78)
  (send (send *robot* :joint "CHEST_Y") :max-joint-torque 173.78)
  (send (send *robot* :joint "CHEST_P") :max-joint-torque 173.78)
  (send (send *robot* :joint "R_CROTCH_Y") :max-joint-torque 114.37)
  (send (send *robot* :joint "R_CROTCH_P") :max-joint-torque 173.78)
  (send (send *robot* :joint "R_CROTCH_R") :max-joint-torque 173.78)
  (send (send *robot* :joint "R_KNEE_P") :max-joint-torque 434.45)
  (send (send *robot* :joint "R_ANKLE_R") :max-joint-torque 173.78)
  (send (send *robot* :joint "R_ANKLE_P") :max-joint-torque 173.78)
  (send (send *robot* :joint "HEAD_Y") :max-joint-torque 14.70)
  (send (send *robot* :joint "HEAD_P") :max-joint-torque 14.70)
  (send (send *robot* :joint "L_SHOULDER_P") :max-joint-torque 92.93)
  (send (send *robot* :joint "L_SHOULDER_R") :max-joint-torque 92.93)
  (send (send *robot* :joint "L_SHOULDER_Y") :max-joint-torque 101.37)
  (send (send *robot* :joint "L_ELBOW_P") :max-joint-torque 247.16)
  (send (send *robot* :joint "L_ELBOW_Y") :max-joint-torque 58.41)
  (send (send *robot* :joint "L_WRIST_P") :max-joint-torque 79.65)
  (send (send *robot* :joint "L_WRIST_Y") :max-joint-torque 79.65)
  (send (send *robot* :joint "L_HAND") :max-joint-torque 94.40)
  (send (send *robot* :joint "R_SHOULDER_P") :max-joint-torque 92.93)
  (send (send *robot* :joint "R_SHOULDER_R") :max-joint-torque 92.93)
  (send (send *robot* :joint "R_SHOULDER_Y") :max-joint-torque 101.37)
  (send (send *robot* :joint "R_ELBOW_P") :max-joint-torque 247.16)
  (send (send *robot* :joint "R_ELBOW_Y") :max-joint-torque 58.41)
  (send (send *robot* :joint "R_WRIST_P") :max-joint-torque 79.65)
  (send (send *robot* :joint "R_WRIST_Y") :max-joint-torque 79.65)
  (send (send *robot* :joint "R_HAND") :max-joint-torque 94.40)
  )

(defun setup-contact-constraint-for-demo-rhp4b-step-block-terrain
    (&key
     (mu-trans 0.3)
     (mu-rot 0.01)
     (max-fz 500)
     (support-polygon-margin 20)
     )
  (send (send *robot-env* :robot :get :rleg-contact-coords)
        :put :contact-constraint
        (instance* default-contact-constraint
                   :init
                   :mu-trans mu-trans
                   :mu-rot mu-rot
                   :max-fz max-fz
                   (let* ((poly
                           (send *robot-env* :robot :get :r_ankle_p_link-support-polygon))
                          (vs
                           (mapcar #'(lambda (v)
                                       (send (send (send *robot-env* :robot :get :rleg-contact-coords) :worldcoords)
                                             :inverse-transform-vector v))
                                   (send poly :vertices)
                                   ))
                          )
                     (list :l-min-x (+ (elt (find-extream vs #'(lambda (v) (elt v 0)) #'<) 0) support-polygon-margin)
                           :l-max-x (- (elt (find-extream vs #'(lambda (v) (elt v 0)) #'>) 0) support-polygon-margin)
                           :l-min-y (+ (elt (find-extream vs #'(lambda (v) (elt v 1)) #'<) 1) support-polygon-margin)
                           :l-max-y (- (elt (find-extream vs #'(lambda (v) (elt v 1)) #'>) 1) support-polygon-margin)
                           ))
                   ))
  (send (send *robot-env* :robot :get :lleg-contact-coords)
        :put :contact-constraint
        (instance* default-contact-constraint
                   :init
                   :mu-trans mu-trans
                   :mu-rot mu-rot
                   :max-fz max-fz
                   (let* ((poly
                           (send *robot-env* :robot :get :l_ankle_p_link-support-polygon))
                          (vs
                           (mapcar #'(lambda (v)
                                       (send (send (send *robot-env* :robot :get :lleg-contact-coords) :worldcoords)
                                             :inverse-transform-vector v))
                                   (send poly :vertices)
                                   ))
                          )
                     (list :l-min-x (+ (elt (find-extream vs #'(lambda (v) (elt v 0)) #'<) 0) support-polygon-margin)
                           :l-max-x (- (elt (find-extream vs #'(lambda (v) (elt v 0)) #'>) 0) support-polygon-margin)
                           :l-min-y (+ (elt (find-extream vs #'(lambda (v) (elt v 1)) #'<) 1) support-polygon-margin)
                           :l-max-y (- (elt (find-extream vs #'(lambda (v) (elt v 1)) #'>) 1) support-polygon-margin)
                           ))
                   ))
  (send (send *robot-env* :robot :get :rarm-contact-coords)
        :put :contact-constraint
        (instance default-contact-constraint
                  :init
                  :mu-trans mu-trans
                  :mu-rot mu-rot
                  :max-fz max-fz
                  :contact-face (send *robot-env* :robot :get :r_wrist_y_link-support-polygon)
                  ;; :l-min-x -1 :l-max-x -1 :l-min-y 1 :l-max-y 1
                  ))
  (send (send *robot-env* :robot :get :larm-contact-coords)
        :put :contact-constraint
        (instance default-contact-constraint
                  :init
                  :mu-trans mu-trans
                  :mu-rot mu-rot
                  :max-fz max-fz
                  :contact-face (send *robot-env* :robot :get :l_wrist_y_link-support-polygon)
                  ;; :l-min-x -1 :l-max-x -1 :l-min-y 1 :l-max-y 1
                  ))
  )

(defun demo-rhp4b-step-block-terrain-trajectory
    (&key
     (optimize-torque? nil)
     (optimize-start-end-torque? t)
     (output-filename
      (ros::resolve-ros-path "package://eus_qp/optmotiongen/logs/demo-rhp4b-step-block-terrain-trajectory/tmp"))
     (pre-process-func)
     (visualize-callback-func)
     (visualize-loop? t)
     &allow-other-keys
     )
  ;; setup environment
  (setq *ground-rleg* (make-cube 400 400 20))
  (setq *ground-lleg* (make-cube 400 400 20))
  (setq *ground-rarm* (make-cube 500 500 20))
  (setq *ground-larm* (make-cube 500 500 20))
  (send *ground-rleg* :newcoords
        (make-coords :pos (float-vector 0 -250 300) :rpy (list 0 0 (deg2rad -30))))
  (send *ground-lleg* :newcoords
        (make-coords :pos (float-vector -100 200 200) :rpy (list 0 (deg2rad 20) 0)))
  (send *ground-rarm* :newcoords
        (make-coords :pos (float-vector 900 -500 1000) :rpy (list 0 (deg2rad -60) (deg2rad -45))))
  (send *ground-larm* :newcoords
        (make-coords :pos (float-vector 900 500 1000) :rpy (list 0 (deg2rad -45) (deg2rad 45))))
  (setq *env-list* (list *ground-rleg* *ground-lleg* *ground-rarm* *ground-larm*))
  (send-all *env-list* :worldcoords)
  (send-all *env-list* :set-color (float-vector 0.5 0.5 0.25))
  ;; setup robot
  (setup-rhp4b-for-demo-rhp4b-step-block-terrain)
  (setq *contact-list*
        (list (instance 2d-planar-contact :init :name "rleg" :contact-pre-offset 150)
              (instance 2d-planar-contact :init :name "lleg" :contact-pre-offset 150)
              (instance 2d-planar-contact :init :name "rarm")
              (instance 2d-planar-contact :init :name "larm")
              ))
  (setq *robot-env*
        (instance robot-environment :init :robot *robot* :contact-list *contact-list*))
  (send *robot-env* :contact "rleg" :set-from-face :face (elt (send *ground-rleg* :faces) 1))
  (send *robot-env* :contact "lleg" :set-from-face :face (elt (send *ground-lleg* :faces) 1))
  (send *robot-env* :contact "rarm" :set-from-face :face (elt (send *ground-rarm* :faces) 1))
  (send *robot-env* :contact "larm" :set-from-face :face (elt (send *ground-larm* :faces) 1))
  (send *robot-env* :contact "rleg" :rotate pi :z :local)
  (send *robot-env* :contact "lleg" :rotate pi :z :local)
  (setup-contact-constraint-for-demo-rhp4b-step-block-terrain)
  (setq *robot-env-list* nil)
  (setq *instant-config-task-list* nil)
  ;; draw
  (unless (boundp '*irtviewer*)
    (make-irtviewer)
    (send *irtviewer* :change-background (float-vector 1 1 1))
    (send *irtviewer* :draw-floor 500)
    (send *irtviewer* :floor-color (float-vector 0 0 0))
    )

  (push (copy-object *robot-env*) *robot-env-list*)
  (push
   (instance instant-configuration-task :init
             :name :lleg-ground
             :robot-env (car *robot-env-list*)
             :drive-joint-list (if (or optimize-torque? optimize-start-end-torque?) (send (car *robot-env-list*) :drive-joint-list) nil)
             :kin-target-coords-list
             (list (send (car *robot-env-list*) :contact "rleg" :contact-coords)
                   (make-coords :pos (float-vector 300 200 0))
                   (send (car *robot-env-list*) :contact "rarm" :contact-coords)
                   (send (car *robot-env-list*) :contact "larm" :contact-coords)
                   )
             :kin-attention-coords-list
             (list (send (car *robot-env-list*) :robot :get :rleg-contact-coords)
                   (send (car *robot-env-list*) :robot :get :lleg-contact-coords)
                   (send (car *robot-env-list*) :robot :get :rarm-contact-coords)
                   (send (car *robot-env-list*) :robot :get :larm-contact-coords)
                   )
             :contact-target-coords-list
             (list (send (car *robot-env-list*) :contact "rleg" :contact-coords)
                   (make-coords :pos (float-vector 300 200 0))
                   (send (car *robot-env-list*) :contact "rarm" :contact-coords)
                   (send (car *robot-env-list*) :contact "larm" :contact-coords)
                   )
             :contact-attention-coords-list
             (list (send (car *robot-env-list*) :robot :get :rleg-contact-coords)
                   (send (car *robot-env-list*) :robot :get :lleg-contact-coords)
                   (send (car *robot-env-list*) :robot :get :rarm-contact-coords)
                   (send (car *robot-env-list*) :robot :get :larm-contact-coords)
                   )
             :posture-joint-list
             (send (car *robot-env-list*) :robot :joint-list)
             :posture-joint-angle-list
             (send-all (send (car *robot-env-list*) :robot :joint-list) :joint-angle)
             )
   *instant-config-task-list*
   )

  (push (copy-object *robot-env*) *robot-env-list*)
  (push
   (instance instant-configuration-task :init
             :name :lleg-ground-without-force
             :robot-env (car *robot-env-list*)
             :drive-joint-list (if optimize-torque? (send (car *robot-env-list*) :drive-joint-list) nil)
             :kin-target-coords-list
             (list (send (car *robot-env-list*) :contact "rleg" :contact-coords)
                   (make-coords :pos (float-vector 300 200 0))
                   (send (car *robot-env-list*) :contact "rarm" :contact-coords)
                   (send (car *robot-env-list*) :contact "larm" :contact-coords)
                   )
             :kin-attention-coords-list
             (list (send (car *robot-env-list*) :robot :get :rleg-contact-coords)
                   (send (car *robot-env-list*) :robot :get :lleg-contact-coords)
                   (send (car *robot-env-list*) :robot :get :rarm-contact-coords)
                   (send (car *robot-env-list*) :robot :get :larm-contact-coords)
                   )
             :contact-target-coords-list
             (list (send (car *robot-env-list*) :contact "rleg" :contact-coords)
                   (send (car *robot-env-list*) :contact "rarm" :contact-coords)
                   (send (car *robot-env-list*) :contact "larm" :contact-coords)
                   )
             :contact-attention-coords-list
             (list (send (car *robot-env-list*) :robot :get :rleg-contact-coords)
                   (send (car *robot-env-list*) :robot :get :rarm-contact-coords)
                   (send (car *robot-env-list*) :robot :get :larm-contact-coords)
                   )
             )
   *instant-config-task-list*
   )

  (push (copy-object *robot-env*) *robot-env-list*)
  (push
   (instance instant-configuration-task :init
             :name :lleg-ground-above
             :robot-env (car *robot-env-list*)
             :drive-joint-list (if optimize-torque? (send (car *robot-env-list*) :drive-joint-list) nil)
             :kin-target-coords-list
             (list (send (car *robot-env-list*) :contact "rleg" :contact-coords)
                   (make-coords :pos (float-vector 300 200 200))
                   (send (car *robot-env-list*) :contact "rarm" :contact-coords)
                   (send (car *robot-env-list*) :contact "larm" :contact-coords)
                   )
             :kin-attention-coords-list
             (list (send (car *robot-env-list*) :robot :get :rleg-contact-coords)
                   (send (car *robot-env-list*) :robot :get :lleg-contact-coords)
                   (send (car *robot-env-list*) :robot :get :rarm-contact-coords)
                   (send (car *robot-env-list*) :robot :get :larm-contact-coords)
                   )
             :contact-target-coords-list
             (list (send (car *robot-env-list*) :contact "rleg" :contact-coords)
                   (send (car *robot-env-list*) :contact "rarm" :contact-coords)
                   (send (car *robot-env-list*) :contact "larm" :contact-coords)
                   )
             :contact-attention-coords-list
             (list (send (car *robot-env-list*) :robot :get :rleg-contact-coords)
                   (send (car *robot-env-list*) :robot :get :rarm-contact-coords)
                   (send (car *robot-env-list*) :robot :get :larm-contact-coords)
                   )
             )
   *instant-config-task-list*
   )

  (dotimes (i 2)
    (push (copy-object *robot-env*) *robot-env-list*)
    (push
     (instance instant-configuration-task :init
               :name :lleg-swing
               :robot-env (car *robot-env-list*)
               :drive-joint-list (if optimize-torque? (send (car *robot-env-list*) :drive-joint-list) nil)
               :kin-target-coords-list
               (list (send (car *robot-env-list*) :contact "rleg" :contact-coords)
                     (send (car *robot-env-list*) :contact "rarm" :contact-coords)
                     (send (car *robot-env-list*) :contact "larm" :contact-coords)
                     )
               :kin-attention-coords-list
               (list (send (car *robot-env-list*) :robot :get :rleg-contact-coords)
                     (send (car *robot-env-list*) :robot :get :rarm-contact-coords)
                     (send (car *robot-env-list*) :robot :get :larm-contact-coords)
                     )
               :contact-target-coords-list
               (list (send (car *robot-env-list*) :contact "rleg" :contact-coords)
                     (send (car *robot-env-list*) :contact "rarm" :contact-coords)
                     (send (car *robot-env-list*) :contact "larm" :contact-coords)
                     )
               :contact-attention-coords-list
               (list (send (car *robot-env-list*) :robot :get :rleg-contact-coords)
                     (send (car *robot-env-list*) :robot :get :rarm-contact-coords)
                     (send (car *robot-env-list*) :robot :get :larm-contact-coords)
                     )
               :collision-pair-list
               (mapcar #'(lambda (l) (list l *ground-lleg*)) (send (car *robot-env-list*) :robot :lleg :links))
               )
     *instant-config-task-list*
     )
    )

  (push (copy-object *robot-env*) *robot-env-list*)
  (push
   (instance instant-configuration-task :init
             :name :lleg-step-above
             :robot-env (car *robot-env-list*)
             :drive-joint-list (if optimize-torque? (send (car *robot-env-list*) :drive-joint-list) nil)
             :kin-target-coords-list
             (list (send (car *robot-env-list*) :contact "rleg" :contact-coords)
                   (send (car *robot-env-list*) :contact "lleg" :contact-pre-coords)
                   (send (car *robot-env-list*) :contact "rarm" :contact-coords)
                   (send (car *robot-env-list*) :contact "larm" :contact-coords)
                   )
             :kin-attention-coords-list
             (list (send (car *robot-env-list*) :robot :get :rleg-contact-coords)
                   (send (car *robot-env-list*) :robot :get :lleg-contact-coords)
                   (send (car *robot-env-list*) :robot :get :rarm-contact-coords)
                   (send (car *robot-env-list*) :robot :get :larm-contact-coords)
                   )
             :contact-target-coords-list
             (list (send (car *robot-env-list*) :contact "rleg" :contact-coords)
                   (send (car *robot-env-list*) :contact "rarm" :contact-coords)
                   (send (car *robot-env-list*) :contact "larm" :contact-coords)
                   )
             :contact-attention-coords-list
             (list (send (car *robot-env-list*) :robot :get :rleg-contact-coords)
                   (send (car *robot-env-list*) :robot :get :rarm-contact-coords)
                   (send (car *robot-env-list*) :robot :get :larm-contact-coords)
                   )
             :collision-pair-list
             (list (list (send (car *robot-env-list*) :robot :link "R_KNEE_P_LINK")
                         (send (car *robot-env-list*) :robot :link "L_KNEE_P_LINK")))
             )
   *instant-config-task-list*
   )

  (push (copy-object *robot-env*) *robot-env-list*)
  (push
   (instance instant-configuration-task :init
             :name :lleg-step-without-force
             :robot-env (car *robot-env-list*)
             :drive-joint-list (if optimize-torque? (send (car *robot-env-list*) :drive-joint-list) nil)
             :kin-target-coords-list
             (list (send (car *robot-env-list*) :contact "rleg" :contact-coords)
                   (send (car *robot-env-list*) :contact "lleg" :contact-coords)
                   (send (car *robot-env-list*) :contact "rarm" :contact-coords)
                   (send (car *robot-env-list*) :contact "larm" :contact-coords)
                   )
             :kin-attention-coords-list
             (list (send (car *robot-env-list*) :robot :get :rleg-contact-coords)
                   (send (car *robot-env-list*) :robot :get :lleg-contact-coords)
                   (send (car *robot-env-list*) :robot :get :rarm-contact-coords)
                   (send (car *robot-env-list*) :robot :get :larm-contact-coords)
                   )
             :contact-target-coords-list
             (list (send (car *robot-env-list*) :contact "rleg" :contact-coords)
                   (send (car *robot-env-list*) :contact "rarm" :contact-coords)
                   (send (car *robot-env-list*) :contact "larm" :contact-coords)
                   )
             :contact-attention-coords-list
             (list (send (car *robot-env-list*) :robot :get :rleg-contact-coords)
                   (send (car *robot-env-list*) :robot :get :rarm-contact-coords)
                   (send (car *robot-env-list*) :robot :get :larm-contact-coords)
                   )
             :collision-pair-list
             (list (list (send (car *robot-env-list*) :robot :link "R_KNEE_P_LINK")
                         (send (car *robot-env-list*) :robot :link "L_KNEE_P_LINK")))
             )
   *instant-config-task-list*
   )

  (push (copy-object *robot-env*) *robot-env-list*)
  (push
   (instance instant-configuration-task :init
             :name :lleg-step
             :robot-env (car *robot-env-list*)
             :drive-joint-list (if optimize-torque? (send (car *robot-env-list*) :drive-joint-list) nil)
             :kin-target-coords-list
             (list (send (car *robot-env-list*) :contact "rleg" :contact-coords)
                   (send (car *robot-env-list*) :contact "lleg" :contact-coords)
                   (send (car *robot-env-list*) :contact "rarm" :contact-coords)
                   (send (car *robot-env-list*) :contact "larm" :contact-coords)
                   )
             :kin-attention-coords-list
             (list (send (car *robot-env-list*) :robot :get :rleg-contact-coords)
                   (send (car *robot-env-list*) :robot :get :lleg-contact-coords)
                   (send (car *robot-env-list*) :robot :get :rarm-contact-coords)
                   (send (car *robot-env-list*) :robot :get :larm-contact-coords)
                   )
             :contact-target-coords-list
             (list (send (car *robot-env-list*) :contact "rleg" :contact-coords)
                   (send (car *robot-env-list*) :contact "lleg" :contact-coords)
                   (send (car *robot-env-list*) :contact "rarm" :contact-coords)
                   (send (car *robot-env-list*) :contact "larm" :contact-coords)
                   )
             :contact-attention-coords-list
             (list (send (car *robot-env-list*) :robot :get :rleg-contact-coords)
                   (send (car *robot-env-list*) :robot :get :lleg-contact-coords)
                   (send (car *robot-env-list*) :robot :get :rarm-contact-coords)
                   (send (car *robot-env-list*) :robot :get :larm-contact-coords)
                   )
             :collision-pair-list
             (list (list (send (car *robot-env-list*) :robot :link "R_KNEE_P_LINK")
                         (send (car *robot-env-list*) :robot :link "L_KNEE_P_LINK")))
             )
   *instant-config-task-list*
   )

  (push (copy-object *robot-env*) *robot-env-list*)
  (push
   (instance instant-configuration-task :init
             :name :rleg-step-without-force
             :robot-env (car *robot-env-list*)
             :drive-joint-list (if optimize-torque? (send (car *robot-env-list*) :drive-joint-list) nil)
             :kin-target-coords-list
             (list (send (car *robot-env-list*) :contact "rleg" :contact-coords)
                   (send (car *robot-env-list*) :contact "lleg" :contact-coords)
                   (send (car *robot-env-list*) :contact "rarm" :contact-coords)
                   (send (car *robot-env-list*) :contact "larm" :contact-coords)
                   )
             :kin-attention-coords-list
             (list (send (car *robot-env-list*) :robot :get :rleg-contact-coords)
                   (send (car *robot-env-list*) :robot :get :lleg-contact-coords)
                   (send (car *robot-env-list*) :robot :get :rarm-contact-coords)
                   (send (car *robot-env-list*) :robot :get :larm-contact-coords)
                   )
             :contact-target-coords-list
             (list (send (car *robot-env-list*) :contact "lleg" :contact-coords)
                   (send (car *robot-env-list*) :contact "rarm" :contact-coords)
                   (send (car *robot-env-list*) :contact "larm" :contact-coords)
                   )
             :contact-attention-coords-list
             (list (send (car *robot-env-list*) :robot :get :lleg-contact-coords)
                   (send (car *robot-env-list*) :robot :get :rarm-contact-coords)
                   (send (car *robot-env-list*) :robot :get :larm-contact-coords)
                   )
             :collision-pair-list
             (list (list (send (car *robot-env-list*) :robot :link "R_KNEE_P_LINK")
                         (send (car *robot-env-list*) :robot :link "L_KNEE_P_LINK")))
             )
   *instant-config-task-list*
   )

  (push (copy-object *robot-env*) *robot-env-list*)
  (push
   (instance instant-configuration-task :init
             :name :rleg-step-above
             :robot-env (car *robot-env-list*)
             :drive-joint-list (if (or optimize-torque? optimize-start-end-torque?) (send (car *robot-env-list*) :drive-joint-list) nil)
             :kin-target-coords-list
             (list (send (car *robot-env-list*) :contact "rleg" :contact-pre-coords)
                   (send (car *robot-env-list*) :contact "lleg" :contact-coords)
                   (send (car *robot-env-list*) :contact "rarm" :contact-coords)
                   (send (car *robot-env-list*) :contact "larm" :contact-coords)
                   )
             :kin-attention-coords-list
             (list (send (car *robot-env-list*) :robot :get :rleg-contact-coords)
                   (send (car *robot-env-list*) :robot :get :lleg-contact-coords)
                   (send (car *robot-env-list*) :robot :get :rarm-contact-coords)
                   (send (car *robot-env-list*) :robot :get :larm-contact-coords)
                   )
             :contact-target-coords-list
             (list (send (car *robot-env-list*) :contact "lleg" :contact-coords)
                   (send (car *robot-env-list*) :contact "rarm" :contact-coords)
                   (send (car *robot-env-list*) :contact "larm" :contact-coords)
                   )
             :contact-attention-coords-list
             (list (send (car *robot-env-list*) :robot :get :lleg-contact-coords)
                   (send (car *robot-env-list*) :robot :get :rarm-contact-coords)
                   (send (car *robot-env-list*) :robot :get :larm-contact-coords)
                   )
             :collision-pair-list
             (list (list (send (car *robot-env-list*) :robot :link "R_KNEE_P_LINK")
                         (send (car *robot-env-list*) :robot :link "L_KNEE_P_LINK")))
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
(warn "(demo-rhp4b-step-block-terrain-trajectory)~%")
