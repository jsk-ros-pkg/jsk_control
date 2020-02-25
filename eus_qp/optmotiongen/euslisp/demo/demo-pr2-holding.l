(load "package://eus_qp/optmotiongen/euslisp/joint-surface-configuration-task.l")
(load "package://eus_qp/optmotiongen/euslisp/inverse-kinematics-statics-wrapper.l")
(load "package://eus_qp/optmotiongen/euslisp/sqp-msc-optimization.l")
(load "package://eus_qp/optmotiongen/euslisp/validate-jacobian.l")
(require :pr2 "package://pr2eus/pr2.l")


(defun setup-robot-for-demo-pr2-holding
    ()
  (setq *robot* (instance pr2-robot :init))
  ;; assoc coords
  (send *robot* :put :rarm-shoulder-contact-coords
        (make-cascoords
         :name :rarm-shoulder-contact-coords
         :parent (car (send *robot* :rarm :links))
         ))
  (send *robot* :put :rarm-upperarm-contact-coords
        (make-cascoords
         :name :rarm-upperarm-contact-coords
         :parent (find-if #'(lambda (l) (equal (send l :name) "r_upper_arm_link")) (all-child-links (car (send *robot* :rarm :links))))
         ))
  (send *robot* :put :rarm-forearm-contact-coords
        (make-cascoords
         :name :rarm-forearm-contact-coords
         :parent (find-if #'(lambda (l) (equal (send l :name) "r_forearm_link")) (all-child-links (car (send *robot* :rarm :links))))
         ))
  (send *robot* :put :rarm-gripper-contact-coords
        (make-cascoords
         :name :rarm-gripper-contact-coords
         :parent (find-if #'(lambda (l) (equal (send l :name) "r_gripper_palm_link")) (all-child-links (car (send *robot* :rarm :links))))
         ))
  (send *robot* :put :larm-shoulder-contact-coords
        (make-cascoords
         :name :larm-shoulder-contact-coords
         :parent (car (send *robot* :larm :links))
         ))
  (send *robot* :put :larm-upperarm-contact-coords
        (make-cascoords
         :name :larm-upperarm-contact-coords
         :parent (find-if #'(lambda (l) (equal (send l :name) "l_upper_arm_link")) (all-child-links (car (send *robot* :larm :links))))
         ))
  (send *robot* :put :larm-forearm-contact-coords
        (make-cascoords
         :name :larm-forearm-contact-coords
         :parent (find-if #'(lambda (l) (equal (send l :name) "l_forearm_link")) (all-child-links (car (send *robot* :larm :links))))
         ))
  (send *robot* :put :larm-gripper-contact-coords
        (make-cascoords
         :name :larm-gripper-contact-coords
         :parent (find-if #'(lambda (l) (equal (send l :name) "l_gripper_palm_link")) (all-child-links (car (send *robot* :larm :links))))
         ))
  ;; save pose
  (send *robot* :reset-pose)
  (send *robot* :put :reset-pose-joint-angle-list (coerce (send *robot* :angle-vector) cons))
  (send *robot* :newcoords (make-coords))
  )

(defun setup-object-for-demo-pr2-holding
    (&key
     (obj-dimensions (list 350 350 260))
     )
  (setq *obj* (apply #'make-cube obj-dimensions))
  ;; upper / lower
  (setq *upper-obj* (make-cube (* 1.0 (x-of-cube *obj*)) (* 1.0 (y-of-cube *obj*)) (* 0.5 (z-of-cube *obj*))))
  (send *upper-obj* :translate (float-vector 0 0 (* 0.25 (z-of-cube *obj*))))
  (send *obj* :assoc *upper-obj*)
  (setq *lower-obj* (make-cube (x-of-cube *obj*) (y-of-cube *obj*) (* 0.5 (z-of-cube *obj*))))
  (send *lower-obj* :translate (float-vector 0 0 (* -0.25 (z-of-cube *obj*))))
  (send *obj* :assoc *lower-obj*)
  (setq *lower-obj-faceset*
        (instance faceset :init :faces (remove (elt (send *lower-obj* :faces) 1) (send *lower-obj* :faces))))
  (send *obj* :assoc *lower-obj-faceset*)
  (setq *bottom-obj-faceset*
        (instance faceset :init :faces (list (elt (send *lower-obj* :faces) 0))))
  (send *obj* :assoc *bottom-obj-faceset*)
  (send *obj* :newcoords (make-coords :pos (float-vector 1000 0 800)))
  (send *obj* :set-color #f(0.6 0.4 0.3))
  ;; assoc coords
  (send *obj* :put :rarm-shoulder-contact-coords
        (make-cascoords :name :rarm-shoulder-contact-coords :parent *obj*))
  (send *obj* :put :rarm-upperarm-contact-coords
        (make-cascoords :name :rarm-upperarm-contact-coords :parent *obj*))
  (send *obj* :put :rarm-forearm-contact-coords
        (make-cascoords :name :rarm-forearm-contact-coords :parent *obj*))
  (send *obj* :put :rarm-gripper-contact-coords
        (make-cascoords :name :rarm-gripper-contact-coords :parent *obj*))
  (send *obj* :put :larm-shoulder-contact-coords
        (make-cascoords :name :larm-shoulder-contact-coords :parent *obj*))
  (send *obj* :put :larm-upperarm-contact-coords
        (make-cascoords :name :larm-upperarm-contact-coords :parent *obj*))
  (send *obj* :put :larm-forearm-contact-coords
        (make-cascoords :name :larm-forearm-contact-coords :parent *obj*))
  (send *obj* :put :larm-gripper-contact-coords
        (make-cascoords :name :larm-gripper-contact-coords :parent *obj*))
  )

(defun demo-pr2-holding-body-surface
    (&key
     (use-msc? nil)
     (setup-robot? t)
     (fix-base? nil)
     (obj-dimensions (list 350 350 260))
     (posture-joint-list)
     (posture-joint-angle-list)
     (target-posture-scale)
     (loop-num 200)
     (pre-process-func)
     (post-process-func)
     &allow-other-keys
     )
  ;; setup robot
  (when setup-robot?
    (setup-robot-for-demo-pr2-holding)
    (setq *robot-env*
          (instance robot-environment :init :robot *robot* :root-virtual-mode :planar))
    )
  ;; setup object
  (setup-object-for-demo-pr2-holding :obj-dimensions obj-dimensions)
  ;; draw
  (unless (boundp '*irtviewer*)
    (make-irtviewer)
    (send *irtviewer* :change-background (float-vector 1 1 1))
    (send *irtviewer* :draw-floor 500)
    (send *irtviewer* :floor-color (float-vector 0 0 0))
    )
  (objects (list *robot* *obj*))

  (setq *instant-config-task*
        (instance instant-configuration-task :init
                  :only-kinematics? t
                  :robot-env *robot-env*
                  :variant-joint-list
                  (cond (fix-base?
                         (set-difference
                          (send *robot-env* :joint-list)
                          (append (send *robot-env* :root-virtual-joint-list)
                                  (list (send *robot-env* :robot :joint "torso_lift_joint")))
                          )
                         )
                        (t
                         (send *robot-env* :joint-list)
                         ))
                  :invariant-joint-list
                  nil
                  :drive-joint-list nil
                  :kin-target-coords-list
                  (list
                   ;; (send *obj* :get :rarm-shoulder-contact-coords)
                   ;; (send *obj* :get :rarm-upperarm-contact-coords)
                   (send *obj* :get :rarm-forearm-contact-coords)
                   (send *obj* :get :rarm-gripper-contact-coords)
                   (send *obj* :get :larm-shoulder-contact-coords)
                   (send *obj* :get :larm-upperarm-contact-coords)
                   (send *obj* :get :larm-forearm-contact-coords)
                   (send *obj* :get :larm-gripper-contact-coords)
                   )
                  :kin-attention-coords-list
                  (list
                   ;; (send *robot-env* :robot :get :rarm-shoulder-contact-coords)
                   ;; (send *robot-env* :robot :get :rarm-upperarm-contact-coords)
                   (send *robot-env* :robot :get :rarm-forearm-contact-coords)
                   (send *robot-env* :robot :get :rarm-gripper-contact-coords)
                   (send *robot-env* :robot :get :larm-shoulder-contact-coords)
                   (send *robot-env* :robot :get :larm-upperarm-contact-coords)
                   (send *robot-env* :robot :get :larm-forearm-contact-coords)
                   (send *robot-env* :robot :get :larm-gripper-contact-coords)
                   )
                  :kin-rotation-type :normal
                  :norm-regular-scale-max 1e20
                  :norm-regular-scale-offset 1e-6
                  :delta-rotational-joint 2.0
                  :posture-joint-list
                  (or posture-joint-list
                      (send *robot-env* :robot :joint-list))
                  :posture-joint-angle-list
                  (or posture-joint-angle-list
                      (send *robot-env* :robot :get :reset-pose-joint-angle-list))
                  :target-posture-scale
                  (or 1e-2
                      target-posture-scale)
                  :collision-pair-list
                  (mapcar
                   #'(lambda (l) (list l *upper-obj*))
                   (list
                    (car (send *robot* :rarm :links))
                    (find-if #'(lambda (l) (equal (send l :name) "r_upper_arm_link")) (all-child-links (car (send *robot* :rarm :links))))
                    (find-if #'(lambda (l) (equal (send l :name) "r_forearm_link")) (all-child-links (car (send *robot* :rarm :links))))
                    (find-if #'(lambda (l) (equal (send l :name) "r_gripper_l_finger_link")) (all-child-links (car (send *robot* :rarm :links))))
                    (find-if #'(lambda (l) (equal (send l :name) "r_gripper_r_finger_link")) (all-child-links (car (send *robot* :rarm :links))))
                    (car (send *robot* :larm :links))
                    (find-if #'(lambda (l) (equal (send l :name) "l_upper_arm_link")) (all-child-links (car (send *robot* :larm :links))))
                    (find-if #'(lambda (l) (equal (send l :name) "l_forearm_link")) (all-child-links (car (send *robot* :larm :links))))
                    (find-if #'(lambda (l) (equal (send l :name) "l_gripper_l_finger_link")) (all-child-links (car (send *robot* :larm :links))))
                    (find-if #'(lambda (l) (equal (send l :name) "l_gripper_r_finger_link")) (all-child-links (car (send *robot* :larm :links))))
                    )
                   )
                  :collision-distance-margin 0.001
                  ))

  ;; generate robot surface
  ;; (setq *rarm-shoulder-surface-config-task*
  ;;       (generate-surface-configuration-task
  ;;        :link (car (send *robot* :rarm :links))
  ;;        :kin-target-coords
  ;;        '(lambda () (send *instant-config-task* :kin-target-coords :name :rarm-shoulder-contact-coords))
  ;;        ))
  ;; (setq *rarm-upperarm-surface-config-task*
  ;;       (generate-surface-configuration-task
  ;;        :link (find-if #'(lambda (l) (equal (send l :name) "r_upper_arm_link")) (all-child-links (car (send *robot* :rarm :links))))
  ;;        :kin-target-coords
  ;;        '(lambda () (send *instant-config-task* :kin-target-coords :name :larm-upperarm-contact-coords))
  ;;        ))
  (setq *rarm-forearm-surface-config-task*
        (generate-surface-configuration-task
         :link (find-if #'(lambda (l) (equal (send l :name) "r_forearm_link")) (all-child-links (car (send *robot* :rarm :links))))
         :kin-target-coords
         '(lambda () (send *instant-config-task* :kin-target-coords :name :rarm-forearm-contact-coords))
         ))
  (setq *rarm-gripper-surface-config-task*
        (generate-surface-configuration-task
         :link (find-if #'(lambda (l) (equal (send l :name) "r_gripper_palm_link")) (all-child-links (car (send *robot* :rarm :links))))
         :body
         (convex-hull-3d
          (remove-duplicates
           (flatten
            (send-all
             (flatten
              (send-all
               (remove-if-not
                #'(lambda (l)
                    (or
                     (equal (send l :name) "r_gripper_palm_link")
                     (equal (send l :name) "r_gripper_l_finger_link")
                     (equal (send l :name) "r_gripper_r_finger_link")
                     (equal (send l :name) "r_gripper_l_finger_tip_link")
                     (equal (send l :name) "r_gripper_r_finger_tip_link")
                     ))
                (all-child-links (car (send *robot* :rarm :links))))
               :bodies))
             :vertices))
           :test #'(lambda (x y) (<= (norm (v- x y)) 1e-3))
           ))
         :kin-target-coords
         '(lambda () (send *instant-config-task* :kin-target-coords :name :rarm-gripper-contact-coords))
         ))
  (setq *larm-shoulder-surface-config-task*
        (generate-surface-configuration-task
         :link (car (send *robot* :larm :links))
         :kin-target-coords
         '(lambda () (send *instant-config-task* :kin-target-coords :name :larm-shoulder-contact-coords))
         ))
  (setq *larm-upperarm-surface-config-task*
        (generate-surface-configuration-task
         :link (find-if #'(lambda (l) (equal (send l :name) "l_upper_arm_link")) (all-child-links (car (send *robot* :larm :links))))
         :kin-target-coords
         '(lambda () (send *instant-config-task* :kin-target-coords :name :larm-upperarm-contact-coords))
         ))
  (setq *larm-forearm-surface-config-task*
        (generate-surface-configuration-task
         :link (find-if #'(lambda (l) (equal (send l :name) "l_forearm_link")) (all-child-links (car (send *robot* :larm :links))))
         :kin-target-coords
         '(lambda () (send *instant-config-task* :kin-target-coords :name :larm-forearm-contact-coords))
         ))
  (setq *larm-gripper-surface-config-task*
        (generate-surface-configuration-task
         :link (find-if #'(lambda (l) (equal (send l :name) "l_gripper_palm_link")) (all-child-links (car (send *robot* :larm :links))))
         :body
         (convex-hull-3d
          (remove-duplicates
           (flatten
            (send-all
             (flatten
              (send-all
               (remove-if-not
                #'(lambda (l)
                    (or
                     (equal (send l :name) "l_gripper_palm_link")
                     (equal (send l :name) "l_gripper_l_finger_link")
                     (equal (send l :name) "l_gripper_r_finger_link")
                     (equal (send l :name) "l_gripper_l_finger_tip_link")
                     (equal (send l :name) "l_gripper_r_finger_tip_link")
                     ))
                (all-child-links (car (send *robot* :larm :links))))
               :bodies))
             :vertices))
           :test #'(lambda (x y) (<= (norm (v- x y)) 1e-3))
           ))
         :kin-target-coords
         '(lambda () (send *instant-config-task* :kin-target-coords :name :larm-gripper-contact-coords))
         ))

  ;; generate object surface
  ;; (setq *obj-rarm-shoulder-surface-config-task*
  ;;       (generate-surface-configuration-task
  ;;        :body *lower-obj-faceset*
  ;;        :kin-target-coords
  ;;        '(lambda () (send *instant-config-task* :kin-attention-coords :name :rarm-shoulder-contact-coords))
  ;;        :flip-normal? t
  ;;        ))
  ;; (setq *obj-rarm-upperarm-surface-config-task*
  ;;       (generate-surface-configuration-task
  ;;        :body *lower-obj-faceset*
  ;;        :kin-target-coords
  ;;        '(lambda () (send *instant-config-task* :kin-attention-coords :name :rarm-upperarm-contact-coords))
  ;;        :flip-normal? t
  ;;        ))
  (setq *obj-rarm-forearm-surface-config-task*
        (generate-surface-configuration-task
         :body *lower-obj-faceset*
         :kin-target-coords
         '(lambda () (send *instant-config-task* :kin-attention-coords :name :rarm-forearm-contact-coords))
         :flip-normal? t
         ))
  (setq *obj-rarm-gripper-surface-config-task*
        (generate-surface-configuration-task
         :body *lower-obj-faceset*
         :kin-target-coords
         '(lambda () (send *instant-config-task* :kin-attention-coords :name :rarm-gripper-contact-coords))
         :flip-normal? t
         ))
  (setq *obj-larm-shoulder-surface-config-task*
        (generate-surface-configuration-task
         :body *lower-obj-faceset*
         :kin-target-coords
         '(lambda () (send *instant-config-task* :kin-attention-coords :name :larm-shoulder-contact-coords))
         :flip-normal? t
         ))
  (setq *obj-larm-upperarm-surface-config-task*
        (generate-surface-configuration-task
         :body *lower-obj-faceset*
         :kin-target-coords
         '(lambda () (send *instant-config-task* :kin-attention-coords :name :larm-upperarm-contact-coords))
         :flip-normal? t
         ))
  (setq *obj-larm-forearm-surface-config-task*
        (generate-surface-configuration-task
         :body *lower-obj-faceset*
         :kin-target-coords
         '(lambda () (send *instant-config-task* :kin-attention-coords :name :larm-forearm-contact-coords))
         :flip-normal? t
         ))
  (setq *obj-larm-gripper-surface-config-task*
        (generate-surface-configuration-task
         :body *bottom-obj-faceset* ;; *lower-obj-faceset*
         :kin-target-coords
         '(lambda () (send *instant-config-task* :kin-attention-coords :name :larm-gripper-contact-coords))
         :flip-normal? t
         ))

  (setq *joint-surface-config-task*
        (instance joint-surface-configuration-task :init
                  :instant-config-task *instant-config-task*
                  :surface-kin-contact-list
                  (list
                   ;; robot
                   ;; (list *rarm-shoulder-surface-config-task* :rarm-shoulder-contact-coords nil)
                   ;; (list *rarm-upperarm-surface-config-task* :rarm-upperarm-contact-coords nil)
                   (list *rarm-forearm-surface-config-task* :rarm-forearm-contact-coords nil)
                   (list *rarm-gripper-surface-config-task* :rarm-gripper-contact-coords nil)
                   (list *larm-shoulder-surface-config-task* :larm-shoulder-contact-coords nil)
                   (list *larm-upperarm-surface-config-task* :larm-upperarm-contact-coords nil)
                   (list *larm-forearm-surface-config-task* :larm-forearm-contact-coords nil)
                   (list *larm-gripper-surface-config-task* :larm-gripper-contact-coords nil)
                   ;; object
                   ;; (list *obj-rarm-shoulder-surface-config-task* :rarm-shoulder-contact-coords nil :target)
                   ;; (list *obj-rarm-upperarm-surface-config-task* :rarm-upperarm-contact-coords nil :target)
                   (list *obj-rarm-forearm-surface-config-task* :rarm-forearm-contact-coords nil :target)
                   (list *obj-rarm-gripper-surface-config-task* :rarm-gripper-contact-coords nil :target)
                   (list *obj-larm-shoulder-surface-config-task* :larm-shoulder-contact-coords nil :target)
                   (list *obj-larm-upperarm-surface-config-task* :larm-upperarm-contact-coords nil :target)
                   (list *obj-larm-forearm-surface-config-task* :larm-forearm-contact-coords nil :target)
                   (list *obj-larm-gripper-surface-config-task* :larm-gripper-contact-coords nil :target)
                   )
                  ))
  ;; (send *instant-config-task* :set-val '_collision-pair-list
  ;;       (mapcar
  ;;        #'(lambda (l) (list l *upper-obj*))
  ;;        (remove
  ;;         *lower-obj-faceset*
  ;;         (mapcar
  ;;          #'(lambda (x) (send (elt x 0) :body))
  ;;          (send *joint-surface-config-task* :surface-kin-contact-list))
  ;;         )))
  ;; (send *instant-config-task* :set-val '_collision-distance-margin-list
  ;;       (make-list (length (send *instant-config-task* :get-val '_collision-pair-list)) :initial-element 0.001))
  ;; (send *instant-config-task* :set-val '_num-collision
  ;;       (length (send *instant-config-task* :get-val '_collision-pair-list)))

  (cond (use-msc?
         (setq *sqp-opt*
               (instance sqp-msc-optimization :init
                         :num-msc 4
                         :dispersion-scale-decrease-ratio 0.9
                         :config-task *joint-surface-config-task* ;; *instant-config-task*
                         :pre-process-func
                         pre-process-func
                         :post-process-func
                         post-process-func
                         )
               )
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
                         post-process-func
                         )
               )
         ))
  (send *sqp-opt* :optimize :loop-num loop-num :print-status-interval 10)

  (when use-msc?
    (send *sqp-opt* :draw-each-configuration :additional-objects (list *obj*)))
  )
(warn "(demo-pr2-holding-body-surface)~%")

(defun demo-pr2-holding-receive-generate
    (&key
     (obj-dimensions (list 350 350 260))
     (pre-padding-scale 1.1)
     (padding-scale 1.0)
     (output-filename
      (ros::resolve-ros-path "package://eus_qp/optmotiongen/logs/demo-pr2-holding/demo-pr2-holding-receive-log.l"))
     )
  (setq *av-list* nil)
  ;; holding posture
  (warning-message 2 "=== generate holding posture ===~%")
  (demo-pr2-holding-body-surface
   :setup-robot? t
   :fix-base? nil
   :obj-dimensions (mapcar #'(lambda (x) (* padding-scale x)) obj-dimensions)
   )
  (push (send *robot-env* :robot :angle-vector) *av-list*)
  ;; pre-holding posture
  (warning-message 2 "=== generate pre-holding posture ===~%")
  (demo-pr2-holding-body-surface
   :setup-robot? nil
   :fix-base? t
   :obj-dimensions (mapcar #'(lambda (x) (* pre-padding-scale x)) obj-dimensions)
   :posture-joint-list
   (remove
    (send *robot-env* :robot :joint "torso_lift_joint")
    (send *robot-env* :robot :joint-list))
   :posture-joint-angle-list
   (mapcar
    #'(lambda (j) (send j :joint-angle))
    (remove
     (send *robot-env* :robot :joint "torso_lift_joint")
     (send *robot-env* :robot :joint-list))
    )
   :target-posture-scale 1e1
   :loop-num 50
   )
  (push (send *robot-env* :robot :angle-vector) *av-list*)
  ;; save posture list
  (setq *root-coords* (send (car (send (send *robot-env* :robot) :links)) :copy-worldcoords))
  (unix:system
   (format nil "DIRNAME=`dirname ~a`; if [ ! -d $DIRNAME ]; then mkdir -p $DIRNAME; fi" output-filename))
  (dump-loadable-structure output-filename *av-list* *root-coords*)
  )
(warn "(demo-pr2-holding-receive-generate)~%")

(defun demo-pr2-holding-receive-send
    (&key
     (output-filename
      (ros::resolve-ros-path "package://eus_qp/optmotiongen/logs/demo-pr2-holding/demo-pr2-holding-receive-log.l"))
     )
  (require "package://pr2eus/pr2-interface.l")
  (pr2-init)
  (setq *robot* *pr2*)
  (load output-filename)
  (send *robot* :reset-pose)
  (send *robot* :move-coords
        *root-coords*
        (car (send *robot* :links)))
  ;;(send *ri* :angle-vector (send *robot* :angle-vector) 5000)
  ;;(send *ri* :wait-interpolation)
  (send *ri* :start-grasp)
  (warning-message 2 "=== press Enter to send posture motion ===~%")
  (read-line)
  (unix::usleep (* 5000 1000))
  ;; pre-holding posture
  (send *robot* :angle-vector (elt *av-list* 0))
  (send *irtviewer* :draw-objects)
  (send *ri* :angle-vector (send *robot* :angle-vector) 5000)
  (send *ri* :wait-interpolation)
  ;; holding posture
  (unix::usleep (* 5000 1000))
  (send *robot* :angle-vector (elt *av-list* 1))
  (send *irtviewer* :draw-objects)
  (send *ri* :angle-vector (send *robot* :angle-vector) 5000)
  (send *ri* :wait-interpolation)
  )
(warn "(demo-pr2-holding-receive-send)~%")
