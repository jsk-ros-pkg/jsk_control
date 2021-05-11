(load "package://eus_qp/optmotiongen/euslisp/inverse-kinematics-wrapper.l")
(load "package://eus_qp/optmotiongen/euslisp/instant-configuration-task.l")
(load "package://eus_qp/optmotiongen/euslisp/sqp-optimization.l")
(load "package://eus_qp/optmotiongen/euslisp/validate-jacobian.l")
(load "package://eus_qp/optmotiongen/euslisp/robot-object-environment.l")
(require :pr2 "package://pr2eus/pr2-interface.l")
(require "models/room73b2-askul-1200x700-desk-0-object.l")


(defun setup-robot-for-demo-pr2-regrasp-object
    ()
  (setq *robot* (instance pr2-robot :init))
  ;; assoc contact coords
  (send *robot* :put :rarm-contact-coords
        (make-cascoords :name :rarm-contact-coords
                        :parent (send (send *robot* :rarm :end-coords) :parent)
                        :coords (send (send (send (send *robot* :rarm :end-coords) :parent) :copy-worldcoords)
                                      :transform (make-coords :pos (float-vector 0 0 0) :rpy (list 0 0 0)))
                        ))
  (send *robot* :put :larm-contact-coords
        (make-cascoords :name :larm-contact-coords
                        :parent (send (send *robot* :larm :end-coords) :parent)
                        :coords (send (send (send (send *robot* :larm :end-coords) :parent) :copy-worldcoords)
                                      :transform (make-coords :pos (float-vector 0 0 0) :rpy (list 0 0 0)))
                        ))
  (send *robot* :reset-pose)
  )

(defun setup-object-for-demo-pr2-regrasp-object
    (&key
     (radius 20)
     (height 500)
     (initial-object-coords
      (make-coords :pos (float-vector 900 0 720) :rpy (list (deg2rad 30) 0 0)))
     (target-grasp-coords-relative-to-object
      (make-coords :pos (float-vector 0 0 (* 0.5 height)) :rpy (list pi/2 0 0)))
     )
  (let* ((body1 (make-cylinder radius height))
         (clink (instance cascaded-link :init))
         )
    (send body1 :set-color (float-vector 1 0 1))
    (setq link1 (instance bodyset-link :init (make-cascoords) :bodies (list body1) :name :suitcase-link))
    (send clink :assoc link1)
    (setf (slot clink cascaded-link 'links) (list link1))
    (setf (slot clink cascaded-link 'joint-list) nil)
    (send clink :init-ending)
    (setq *obj* clink)
    (send *obj* :put :object-coords
          (make-cascoords
           :parent (car (send *obj* :links))
           :pos (float-vector 0 0 (* 0.5 height))
           :rpy (list 0 -pi/2 0)
           ))
    (send *obj* :put :target-grasp-coords
          (make-cascoords
           :parent (car (send *obj* :links))
           :coords target-grasp-coords-relative-to-object
           ))
    (send *obj* :put :target-grasp-pre-coords
          (make-cascoords
           :parent (car (send *obj* :links))
           :coords
           (send (send target-grasp-coords-relative-to-object :copy-worldcoords) :translate (float-vector -100 0 0))
           ))
    (setq *initial-object-coords* initial-object-coords)
    (setq *obj-edge*
          (make-line (elt (send (elt (send *obj* :faces) 0) :centroid) 1)
                     (elt (send (elt (send *obj* :faces) 1) :centroid) 1)))
    (send *obj-edge* :put :direction-move-target-axis :z)
    (setq *obj-edge-contact*
          (generate-contact-ik-arg-from-line-segment
           :name "object"
           :line-seg *obj-edge*
           :margin 20
           ))
    (send *obj* :assoc *obj-edge-contact*)
    (send *obj-edge-contact* :put :contact-pre-coords
          (make-cascoords :parent
                          (send *obj-edge-contact* :contact-coords)
                          :coords
                          (send (send (send *obj-edge-contact* :contact-coords) :copy-worldcoords) :translate (float-vector -100 0 0))
                          ))
    ))

(defun demo-pr2-regrasp-object
    (&key
     (pre-process-func)
     (visualize-callback-func)
     (visualize-loop? t)
     &allow-other-keys
     )
  ;; setup robot and object
  (setup-robot-for-demo-pr2-regrasp-object)
  (setup-object-for-demo-pr2-regrasp-object)
  (setq *robot-env*
        (instance robot-object-environment :init
                  :robot *robot*
                  :object *obj*
                  :contact-list (list *obj-edge-contact*)
                  :root-virtual-mode :planar
                  ))
  ;; setup environment
  (setq *desk* (instance room73b2-askul-1200x700-desk-0-object :init))
  (send *desk* :newcoords
        (make-coords :pos (float-vector 1000 0 0) :rpy (list 0 0 0)))
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
             :name :pregrasp-larm
             :robot-env (car *robot-env-list*)
             :only-kinematics? t
             :posture-joint-list
             (send (car *robot-env-list*) :robot :joint-list)
             :posture-joint-angle-list
             (send-all (send (car *robot-env-list*) :robot :joint-list) :joint-angle)
             :kin-target-coords-list
             (list *initial-object-coords*
                   (send (car *robot-env-list*) :contact "object" :get :contact-pre-coords)
                   )
             :kin-attention-coords-list
             (list (send (car *robot-env-list*) :object :get :object-coords)
                   (send (car *robot-env-list*) :robot :get :larm-contact-coords)
                   )
             :collision-pair-list
             (mapcar #'(lambda (l) (list l *desk*)) (send (car *robot-env-list*) :robot :links))
             :collision-distance-margin 0.01
             )
   *instant-config-task-list*
   )

  (push (copy-object *robot-env*) *robot-env-list*)
  (push
   (instance instant-configuration-task :init
             :name :grasp-larm
             :robot-env (car *robot-env-list*)
             :only-kinematics? t
             :posture-joint-list
             (send (car *robot-env-list*) :robot :joint-list)
             :posture-joint-angle-list
             (send-all (send (car *robot-env-list*) :robot :joint-list) :joint-angle)
             :kin-target-coords-list
             (list *initial-object-coords*
                   (send (car *robot-env-list*) :contact "object" :contact-coords)
                   )
             :kin-attention-coords-list
             (list (send (car *robot-env-list*) :object :get :object-coords)
                   (send (car *robot-env-list*) :robot :get :larm-contact-coords)
                   )
             :collision-pair-list
             (mapcar #'(lambda (l) (list l *desk*)) (send (car *robot-env-list*) :robot :links))
             :collision-distance-margin 0.01
             )
   *instant-config-task-list*
   )

  (push (copy-object *robot-env*) *robot-env-list*)
  (push
   (instance instant-configuration-task :init
             :name :liftup-object
             :robot-env (car *robot-env-list*)
             :only-kinematics? t
             :posture-joint-list
             (send (car *robot-env-list*) :robot :joint-list)
             :posture-joint-angle-list
             (send-all (send (car *robot-env-list*) :robot :joint-list) :joint-angle)
             :kin-target-coords-list
             (list (send (send *initial-object-coords* :copy-worldcoords) :translate (float-vector 0 0 150) :world)
                   (send (car *robot-env-list*) :contact "object" :contact-coords)
                   )
             :kin-attention-coords-list
             (list (send (car *robot-env-list*) :object :get :object-coords)
                   (send (car *robot-env-list*) :robot :get :larm-contact-coords)
                   )
             :collision-pair-list
             (mapcar #'(lambda (l) (list l *desk*)) (send (car *robot-env-list*) :robot :links))
             :collision-distance-margin 0.01
             )
   *instant-config-task-list*
   )

  (push (copy-object *robot-env*) *robot-env-list*)
  (push
   (instance instant-configuration-task :init
             :name :pregrasp-rarm
             :robot-env (car *robot-env-list*)
             :only-kinematics? t
             :posture-joint-list
             (send (car *robot-env-list*) :robot :joint-list)
             :posture-joint-angle-list
             (send-all (send (car *robot-env-list*) :robot :joint-list) :joint-angle)
             :kin-target-coords-list
             (list (send (car *robot-env-list*) :contact "object" :contact-coords)
                   (send (car *robot-env-list*) :object :get :target-grasp-pre-coords)
                   )
             :kin-attention-coords-list
             (list (send (car *robot-env-list*) :robot :get :larm-contact-coords)
                   (send (car *robot-env-list*) :robot :get :rarm-contact-coords)
                   )
             :collision-pair-list
             (mapcar #'(lambda (l) (list l *desk*)) (send (car *robot-env-list*) :robot :links))
             :collision-distance-margin 0.01
             )
   *instant-config-task-list*
   )

  (push (copy-object *robot-env*) *robot-env-list*)
  (push
   (instance instant-configuration-task :init
             :name :grasp-rarm
             :robot-env (car *robot-env-list*)
             :only-kinematics? t
             :posture-joint-list
             (send (car *robot-env-list*) :robot :joint-list)
             :posture-joint-angle-list
             (send-all (send (car *robot-env-list*) :robot :joint-list) :joint-angle)
             :kin-target-coords-list
             (list (send (car *robot-env-list*) :contact "object" :contact-coords)
                   (send (car *robot-env-list*) :object :get :target-grasp-coords)
                   )
             :kin-attention-coords-list
             (list (send (car *robot-env-list*) :robot :get :larm-contact-coords)
                   (send (car *robot-env-list*) :robot :get :rarm-contact-coords)
                   )
             :collision-pair-list
             (mapcar #'(lambda (l) (list l *desk*)) (send (car *robot-env-list*) :robot :links))
             :collision-distance-margin 0.01
             )
   *instant-config-task-list*
   )

  (push (copy-object *robot-env*) *robot-env-list*)
  (push
   (instance instant-configuration-task :init
             :name :release-larm
             :robot-env (car *robot-env-list*)
             :only-kinematics? t
             :posture-joint-list
             (send (car *robot-env-list*) :robot :joint-list)
             :posture-joint-angle-list
             (send-all (send (car *robot-env-list*) :robot :joint-list) :joint-angle)
             :kin-target-coords-list
             (list (send (car *robot-env-list*) :contact "object" :get :contact-pre-coords)
                   (send (car *robot-env-list*) :object :get :target-grasp-coords)
                   )
             :kin-attention-coords-list
             (list (send (car *robot-env-list*) :robot :get :larm-contact-coords)
                   (send (car *robot-env-list*) :robot :get :rarm-contact-coords)
                   )
             :collision-pair-list
             (mapcar #'(lambda (l) (list l *desk*)) (send (car *robot-env-list*) :robot :links))
             :collision-distance-margin 0.01
             )
   *instant-config-task-list*
   )

  (objects (append (list *desk*) *robot-env-list*))

  (setq *trajectory-config-task*
        (instance trajectory-configuration-task :init
                  :instant-config-task-list (reverse *instant-config-task-list*)
                  :norm-regular-scale-max 1e-2
                  :norm-regular-scale-offset 1e-6
                  :adjacent-regular-scale 1e-3
                  ))

  (setq *sqp-opt*
        (instance sqp-optimization :init
                  :config-task *trajectory-config-task*
                  :pre-process-func
                  pre-process-func
                  :post-process-func
                  #'(lambda (sqp)
                      (when (= (mod (send sqp :iteration) 20) 0)
                        ;; (validate-element-jacobian :config-task (send sqp :config-task))
                        ;; (validate-jacobian :config-task (send sqp :config-task))
                        ))
                  ))
  (send *sqp-opt* :optimize :loop-num 100)

  (objects (list *desk* *robot-env*))
  (send *trajectory-config-task* :play-animation
        :robot-env *robot-env*
        :loop? visualize-loop?
        :visualize-callback-func visualize-callback-func
        )
  )
(warn "(demo-pr2-regrasp-object)~%")
