(load "../instant-manipulation-configuration-task.l")
(load "../sqp-optimization.l")
(load "../validate-jacobian.l")
(require :rhp4b "package://khi_ros_bridge_tutorials/models/rhp4b.l")
(require "models/arrow-object.l")

(defun setup-rhp4b-for-test-sqp-optimization
  ()
  (setq *robot* (instance rhp4b-robot :init))
  ;; assoc contact coords
  (send *robot* :put :lleg-contact-coords
        (make-cascoords :name :lleg-contact-coords
                        :parent (send *robot* :link "L_ANKLE_P_LINK")
                        :coords (send (send (send *robot* :lleg-end-coords) :copy-worldcoords)
                                      :translate (float-vector 0 0 30))
                        ))
  (send *robot* :put :rleg-contact-coords
        (make-cascoords :name :rleg-contact-coords
                        :parent (send *robot* :link "R_ANKLE_P_LINK")
                        :coords (send (send (send *robot* :rleg-end-coords) :copy-worldcoords)
                                      :translate (float-vector 0 0 30))
                        ))
  (send *robot* :put :larm-contact-coords
        (make-cascoords :name :larm-contact-coords
                        :parent (send *robot* :link "L_WRIST_Y_LINK")
                        :coords (send (send *robot* :larm-end-coords) :copy-worldcoords)
                        ))
  (send *robot* :put :rarm-contact-coords
        (make-cascoords :name :rarm-contact-coords
                        :parent (send *robot* :link "R_WRIST_Y_LINK")
                        :coords (send (send *robot* :rarm-end-coords) :copy-worldcoords)
                        ))
  ;; assoc grasp coords
  (send *robot* :put :rarm-grasp-coords
        (make-cascoords :name :rarm-grasp-coords
                        :parent (send *robot* :link "R_WRIST_Y_LINK")
                        :coords (send (send *robot* :rarm-end-coords) :copy-worldcoords)
                        ))
  (send *robot* :put :larm-grasp-coords
        (make-cascoords :name :larm-grasp-coords
                        :parent (send *robot* :link "L_WRIST_Y_LINK")
                        :coords (send (send *robot* :larm-end-coords) :copy-worldcoords)
                        ))
  ;; create support polygon
  (send *robot* :init-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  (create-support-polygon *robot* (send *robot* :link "L_WRIST_Y_LINK"))
  (create-support-polygon *robot* (send *robot* :link "R_WRIST_Y_LINK"))
  (create-support-polygon *robot* (send *robot* :link "L_ANKLE_P_LINK"))
  (create-support-polygon *robot* (send *robot* :link "R_ANKLE_P_LINK"))
  ;; disable min-max table
  (dolist (jnt (send *robot* :joint-list))
    (send jnt :set-val 'joint-min-max-table nil)
    (send jnt :set-val 'joint-min-max-target nil)
    )
  ;; overwrite max/min-angle
  ;; 足が逆を向くのを防ぐため
  (send (send *robot* :joint "L_CROTCH_Y") :max-angle 60)
  (send (send *robot* :joint "L_CROTCH_Y") :min-angle -60)
  (send (send *robot* :joint "R_CROTCH_Y") :max-angle 60)
  (send (send *robot* :joint "R_CROTCH_Y") :min-angle -60)
  ;; set dummy torque limit
  (send (send *robot* :joint "L_CROTCH_Y") :max-joint-torque 100)
  (send (send *robot* :joint "L_CROTCH_P") :max-joint-torque 200)
  (send (send *robot* :joint "L_CROTCH_R") :max-joint-torque 200)
  (send (send *robot* :joint "L_KNEE_P") :max-joint-torque 400)
  (send (send *robot* :joint "L_ANKLE_R") :max-joint-torque 200)
  (send (send *robot* :joint "L_ANKLE_P") :max-joint-torque 200)
  (send (send *robot* :joint "CHEST_Y") :max-joint-torque 200)
  (send (send *robot* :joint "CHEST_P") :max-joint-torque 200)
  (send (send *robot* :joint "R_CROTCH_Y") :max-joint-torque 100)
  (send (send *robot* :joint "R_CROTCH_P") :max-joint-torque 200)
  (send (send *robot* :joint "R_CROTCH_R") :max-joint-torque 200)
  (send (send *robot* :joint "R_KNEE_P") :max-joint-torque 400)
  (send (send *robot* :joint "R_ANKLE_R") :max-joint-torque 200)
  (send (send *robot* :joint "R_ANKLE_P") :max-joint-torque 200)
  (send (send *robot* :joint "HEAD_Y") :max-joint-torque 10)
  (send (send *robot* :joint "HEAD_P") :max-joint-torque 10)
  (send (send *robot* :joint "L_SHOULDER_P") :max-joint-torque 100)
  (send (send *robot* :joint "L_SHOULDER_R") :max-joint-torque 100)
  (send (send *robot* :joint "L_SHOULDER_Y") :max-joint-torque 100)
  (send (send *robot* :joint "L_ELBOW_P") :max-joint-torque 200)
  (send (send *robot* :joint "L_ELBOW_Y") :max-joint-torque 50)
  (send (send *robot* :joint "L_WRIST_P") :max-joint-torque 100)
  (send (send *robot* :joint "L_WRIST_Y") :max-joint-torque 100)
  (send (send *robot* :joint "L_HAND") :max-joint-torque 100)
  (send (send *robot* :joint "R_SHOULDER_P") :max-joint-torque 100)
  (send (send *robot* :joint "R_SHOULDER_R") :max-joint-torque 100)
  (send (send *robot* :joint "R_SHOULDER_Y") :max-joint-torque 100)
  (send (send *robot* :joint "R_ELBOW_P") :max-joint-torque 200)
  (send (send *robot* :joint "R_ELBOW_Y") :max-joint-torque 50)
  (send (send *robot* :joint "R_WRIST_P") :max-joint-torque 100)
  (send (send *robot* :joint "R_WRIST_Y") :max-joint-torque 100)
  (send (send *robot* :joint "R_HAND") :max-joint-torque 100)
  )

(defun create-support-polygon
  (robot
   link
   &key
   (thre 10.0)
   )
  (send link :worldcoords)
  (let* ((target-nm (read-from-string (format nil ":~A-sole-body" (send link :name))))
         (vs (remove-duplicates
              (flatten (send-all (send link :bodies) :vertices))
              :test #'(lambda (x y) (eps-v= x y *epsilon*))))
         (min-vs (find-extream vs #'(lambda (x) (elt x 2)) #'<))
         (b (send
             (make-bounding-box
              (remove-if
               #'(lambda (p)
                   (< thre (- (elt p 2) (elt min-vs 2)))) vs))
             :body)))
    (send link :assoc b)
    (send robot :put target-nm b)
    (send b :worldcoords)
    (let ((f (find-if #'(lambda (x) (memq :bottom (send x :id))) (send b :faces))))
      (send f :name (send link :name))
      (send robot :put (read-from-string (format nil ":~A-support-polygon" (send link :name))) f)
      f)
    ))

(defun demo-rhp4b-reach-suitcase-instant
  (&key
   (optimize-torque? t)
   (apply-external-force? nil)
   (fix-grasp? t)
   )
  ;; setup environment
  (setq *ground* (make-cube 2000 2000 50))
  (send *ground* :set-color (float-vector 0.8 0.5 0.5))
  (send *ground* :translate (float-vector 0 0 -25))
  (send *ground* :worldcoords)

  ;; setup robot
  (setup-rhp4b-for-test-sqp-optimization)
  (setq *contact-list*
        (list (instance 2d-planar-contact :init :name "rleg")
              (instance 2d-planar-contact :init :name "lleg")
              ))

  ;; setup suitcase
  (let* ((height 770)
         (body1 (make-cube 380 560 height))
         (body-h1 (make-cube 25 240 35))
         (body-h2 (make-cube 25 35 250))
         (link1)
         (clink (instance cascaded-link :init))
         )
    (send body1 :assoc body-h1)
    (send body1 :assoc body-h2)
    (send body-h1 :translate (float-vector -32.5 0 385))
    (send body-h1 :set-color :red)
    (send body-h2 :translate (float-vector -32.5 285 0))
    (send body-h2 :set-color :yellow)
    (setq link1 (instance bodyset-link :init (make-cascoords) :bodies (list body1 body-h1 body-h2) :name :suitcase-link))
    (send clink :assoc link1)
    (setf (slot clink cascaded-link 'links) (list link1))
    (setf (slot clink cascaded-link 'joint-list) nil)
    (send clink :init-ending)
    (setq *obj* clink)
    (send (car (send *obj* :links)) :weight (* 20 1e3))
    ;; assoc grasp coords for obj
    (send *obj* :put :handle-side-rarm
          (make-cascoords :parent (car (send *obj* :links))
                          :pos(send body-h1 :worldpos)
                          :rpy (list pi/2 pi/2 0)
                          ))
    (send *obj* :put :handle-side-larm
          (make-cascoords :parent (car (send *obj* :links))
                          :pos (send body-h2 :worldpos)
                          :rpy (list -pi/2 0 0)
                          ))
    (send *obj* :put :bottom-coords
          (make-cascoords :parent (car (send *obj* :links))
                          :pos (float-vector 0 0 (* -0.5 height))
                          ))
    )
  (send *obj* :move-coords
        (make-coords :pos (float-vector 500 0 0))
        (send (send *obj* :get :bottom-coords) :copy-worldcoords)
        )
  (send *obj* :worldcoords)

  (setq *arrow-obj* (instance arrow-object :init))
  (send *arrow-obj* :newcoords (send *obj* :copy-worldcoords))

  ;; assoc object coords
  (send *obj* :put :object-coords
        (make-cascoords :name :object-coords
                        :parent (car (send *obj* :links))
                        :coords (send *obj* :copy-worldcoords)
                        ))

  (setq *robot-obj-env*
        (instance robot-object-environment :init :robot *robot* :object *obj* :contact-list *contact-list*))
  (send *robot-obj-env* :contact "rleg" :set-from-face :face (elt (send *ground* :faces) 1))
  (send *robot-obj-env* :contact "lleg" :set-from-face :face (elt (send *ground* :faces) 1))
  (send (send *robot-obj-env* :contact "rleg" :contact-coords) :rotate pi :z)
  (send (send *robot-obj-env* :contact "lleg" :contact-coords) :rotate pi :z)

  ;; draw
  (unless (boundp '*irtviewer*)
    (make-irtviewer)
    (send *irtviewer* :change-background (float-vector 1 1 1))
    (send *irtviewer* :set-val 'x::draw-floor 500)
    (when (assoc 'x::floor-color (send *irtviewer* :slots))
      (send *irtviewer* :set-val 'x::floor-color (float-vector 0 0 0)))
    )
  (setq *env-list* (list *ground* *arrow-obj*))
  (objects (append (list *robot* *obj*) *env-list*))

  (setq *instant-manip-config-task*
        (instance* instant-manipulation-configuration-task :init
                   :robot-env *robot-obj-env*
                   :drive-joint-list (if optimize-torque? (send *robot-obj-env* :drive-joint-list) nil)
                   :posture-joint-list
                   (send *robot-obj-env* :robot :joint-list)
                   :posture-joint-angle-list
                   (send-all (send *robot-obj-env* :robot :joint-list) :joint-angle)
                   :kin-target-coords-list
                   (cond (fix-grasp?
                          (remove
                           nil
                           (list (send *robot-obj-env* :contact "rleg" :contact-coords)
                                 (send *robot-obj-env* :contact "lleg" :contact-coords)
                                 (send *robot-obj-env* :object :get :handle-side-rarm)
                                 (send *robot-obj-env* :object :get :handle-side-larm)
                                 (send *arrow-obj* :copy-worldcoords)
                                 ))
                          )
                         (t
                          ))
                   :kin-attention-coords-list
                   (remove
                    nil
                    (list (send *robot-obj-env* :robot :get :rleg-contact-coords)
                          (send *robot-obj-env* :robot :get :lleg-contact-coords)
                          (send *robot-obj-env* :robot :get :rarm-grasp-coords)
                          (send *robot-obj-env* :robot :get :larm-grasp-coords)
                          (send *robot-obj-env* :object :get :object-coords)
                          ))
                   :contact-target-coords-list
                   (cond (fix-grasp?
                          (list (send *robot-obj-env* :contact "rleg" :contact-coords)
                                (send *robot-obj-env* :contact "lleg" :contact-coords)
                                (send *robot-obj-env* :object :get :handle-side-rarm)
                                (send *robot-obj-env* :object :get :handle-side-larm)
                                )
                          )
                         (t
                          ))
                   :contact-attention-coords-list
                   (list (send *robot-obj-env* :robot :get :rleg-contact-coords)
                         (send *robot-obj-env* :robot :get :lleg-contact-coords)
                         (send *robot-obj-env* :robot :get :rarm-grasp-coords)
                         (send *robot-obj-env* :robot :get :larm-grasp-coords)
                         )
                   :contact-constraint-list
                   (list
                    (instance* default-contact-constraint
                               :init
                               :mu-trans 0.2 :mu-rot 0.01 :max-fz 1000
                               (let* ((poly
                                       (send *robot-obj-env* :robot :get :r_ankle_p_link-support-polygon))
                                      (vs
                                       (mapcar #'(lambda (v)
                                                   (send (send (send *robot-obj-env* :robot :get :rleg-contact-coords) :worldcoords)
                                                         :inverse-transform-vector v))
                                               (send poly :vertices)
                                               ))
                                      )
                                 ;; find-extreamは列中のある値でのソート
                                 (list :l-min-x (+ (elt (find-extream vs #'(lambda (v) (elt v 0)) #'<) 0) 20)
                                       :l-max-x (- (elt (find-extream vs #'(lambda (v) (elt v 0)) #'>) 0) 20)
                                       :l-min-y (+ (elt (find-extream vs #'(lambda (v) (elt v 1)) #'<) 1) 20)
                                       :l-max-y (- (elt (find-extream vs #'(lambda (v) (elt v 1)) #'>) 1) 20)
                                       ))
                               )
                    (instance* default-contact-constraint
                               :init
                               :mu-trans 0.2 :mu-rot 0.01 :max-fz 1000
                               (let* ((poly
                                       (send *robot-obj-env* :robot :get :l_ankle_p_link-support-polygon))
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
                               )
                    (instance 6d-min-max-contact-constraint
                              :init (float-vector 1000 1000 1000 100 100 100))
                    (instance 6d-min-max-contact-constraint
                              :init (float-vector 1000 1000 1000 100 100 100))
                    )
                   :contact-target-coords-obj-list
                   (cond (fix-grasp?
                          (list (send *robot-obj-env* :object :get :handle-side-rarm)
                                (send *robot-obj-env* :object :get :handle-side-larm)
                                (send (send *robot-obj-env* :object :get :bottom-coords) :copy-worldcoords)
                                )
                          )
                         (t
                          ))
                   :contact-constraint-obj-list
                   (list
                    (instance 6d-min-max-contact-constraint
                              :init (float-vector 1000 1000 1000 100 100 100))
                    (instance 6d-min-max-contact-constraint
                              :init (float-vector 1000 1000 1000 100 100 100))
                    (instance* default-contact-constraint
                               :init
                               :mu-trans 0.2 :mu-rot 0.01 :max-fz 1000
                               (let* ((poly
                                       (elt (send *robot-obj-env* :object :faces) 0)
                                       )
                                      (vs
                                       (mapcar #'(lambda (v)
                                                   (send (send (send *robot-obj-env* :object :get :bottom-coords) :copy-worldcoords)
                                                         :inverse-transform-vector v))
                                               (send poly :vertices)
                                               )))
                                 (list :l-min-x (+ (elt (find-extream vs #'(lambda (v) (elt v 0)) #'<) 0) 20)
                                       :l-max-x (- (elt (find-extream vs #'(lambda (v) (elt v 0)) #'>) 0) 20)
                                       :l-min-y (+ (elt (find-extream vs #'(lambda (v) (elt v 1)) #'<) 1) 20)
                                       :l-max-y (- (elt (find-extream vs #'(lambda (v) (elt v 1)) #'>) 1) 20)
                                       ))
                               )
                    )
                   :act-react-pair-list
                   (list
                    (list (send *robot-obj-env* :object :get :handle-side-rarm)
                          (send *robot-obj-env* :object :get :handle-side-rarm))
                    (list (send *robot-obj-env* :object :get :handle-side-larm)
                          (send *robot-obj-env* :object :get :handle-side-larm))
                    )
                   :collision-pair-list
                   (append
                    (mapcar #'(lambda (l1 l2) (list l1 l2))
                            (send *robot-obj-env* :robot :lleg :links)
                            (send *robot-obj-env* :robot :rleg :links))
                    (mapcar #'(lambda (l) (list l (send *robot-obj-env* :object)))
                            (flatten
                             (list (send *robot-obj-env* :robot :legs :links)
                                   (send *robot-obj-env* :robot :torso :links))))
                    )
                   :collision-distance-margin-list
                   (append
                    (make-list
                     (length (send *robot-obj-env* :robot :lleg :links))
                     :initial-element 0.01) ;; [m]
                    (make-list
                     (length (flatten
                              (list (send *robot-obj-env* :robot :legs :links)
                                    (send *robot-obj-env* :robot :torso :links))))
                     :initial-element 0.1) ;; [m]
                    )
                   (cond (optimize-torque?
                          (list :norm-regular-scale-max 5e-5
                                :target-posture-scale 5e-4
                                )
                          )
                         (t
                          (list :norm-regular-scale-max 1e-2
                                :target-posture-scale 1e-3
                                )
                          ))
                   ))
  (setq *sqp-opt*
        (instance sqp-optimization :init
                  :config-task *instant-manip-config-task*
                  ;; :post-process-func
                  ;; #'(lambda (sqp)
                  ;;     (when (= (mod (send sqp :iteration) 20) 0)
                  ;;       (validate-element-jacobian-manipulation :config-task (send sqp :config-task))
                  ;;       (validate-jacobian :config-task (send sqp :config-task))
                  ;;       ))
                  ))
  (send *sqp-opt* :optimize :loop-num 50)
  )
(warn "~%(demo-rhp4b-reach-suitcase-instant)~%")
