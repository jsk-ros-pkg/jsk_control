(load "package://eus_qp/optmotiongen/euslisp/sqp-optimization.l")
(load "package://eus_qp/optmotiongen/euslisp/validate-jacobian.l")
(load "package://eus_qp/optmotiongen/euslisp/trajectory-configuration-task.l")
(require :robotis_op2 "package://jsk_darwin/euslisp/robotis_op2-interface.l")


(defun setup-robot-for-demo-darwin-toy-terrain
    ()
  (setq *robot* (instance robotis_op2-robot :init))
  ;; set min max joint angle
  (set-min-max-joint-angle)
  ;; overwrite min max joint angle
  (send (send *robot* :joint "r_hip_yaw") :max-angle 0) ;; inside
  (send (send *robot* :joint "r_hip_yaw") :min-angle -20) ;; outside
  (send (send *robot* :joint "l_hip_yaw") :max-angle 20) ;; outside
  (send (send *robot* :joint "l_hip_yaw") :min-angle 0) ;; inside
  (send (send *robot* :joint "r_hip_roll") :max-angle 45) ;; outside
  (send (send *robot* :joint "r_hip_roll") :min-angle -45) ;; inside
  (send (send *robot* :joint "l_hip_roll") :max-angle 45) ;; inside
  (send (send *robot* :joint "l_hip_roll") :min-angle -45) ;; outside
  ;; set max joint torque
  (send-all (send *robot* :joint-list) :max-joint-torque 0.5)
  ;; create support polygon and assoc contact coords
  (create-support-polygon *robot*
                          (send *robot* :link "r_ank_roll_link")
                          :normal (scale -1 (send (send *robot* :link "r_ank_roll_link") :z-axis)))
  (send *robot* :put :rleg-contact-coords
        (make-cascoords :name :rleg-contact-coords
                        :parent (send *robot* :link "r_ank_roll_link")
                        :coords (generate-coords-from-face (send *robot* :get :r_ank_roll_link-support-polygon) :flip? t)
                        ))
  (create-support-polygon *robot*
                          (send *robot* :link "l_ank_roll_link")
                          :normal (scale -1 (send (send *robot* :link "l_ank_roll_link") :z-axis)))
  (send *robot* :put :lleg-contact-coords
        (make-cascoords :name :lleg-contact-coords
                        :parent (send *robot* :link "l_ank_roll_link")
                        :coords (generate-coords-from-face (send *robot* :get :l_ank_roll_link-support-polygon) :flip? t)
                        ))
  (create-support-polygon *robot*
                          (send *robot* :link "r_el_link")
                          :normal (send (send *robot* :link "r_el_link") :z-axis))
  (send *robot* :put :rarm-contact-coords
        (make-cascoords :name :rarm-contact-coords
                        :parent (send *robot* :link "r_el_link")
                        :coords (generate-coords-from-face (send *robot* :get :r_el_link-support-polygon) :flip? t)
                        ))
  (create-support-polygon *robot*
                          (send *robot* :link "l_el_link")
                          :normal (send (send *robot* :link "l_el_link") :z-axis))
  (send *robot* :put :larm-contact-coords
        (make-cascoords :name :larm-contact-coords
                        :parent (send *robot* :link "l_el_link")
                        :coords (generate-coords-from-face (send *robot* :get :l_el_link-support-polygon) :flip? t)
                        ))
  )

(defun setup-contact-constraint-for-demo-darwin-toy-terrain
    (&key
     (foot-mu-trans 0.3)
     (hand-mu-trans 0.1)
     (mu-rot 0.01)
     (max-fz 100)
     (support-polygon-margin 20)
     )
  (send (send *robot-env* :robot :get :rleg-contact-coords)
        :put :contact-constraint
        (instance* default-contact-constraint
                   :init
                   :mu-trans foot-mu-trans
                   :mu-rot mu-rot
                   :max-fz max-fz
                   (let* ((poly
                           (send *robot* :get :r_ank_roll_link-support-polygon))
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
                   :mu-trans foot-mu-trans
                   :mu-rot mu-rot
                   :max-fz max-fz
                   (let* ((poly
                           (send *robot* :get :l_ank_roll_link-support-polygon))
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
                  :mu-trans hand-mu-trans
                  :mu-rot mu-rot
                  :max-fz max-fz
                  :l-min-x -1 :l-max-x -1 :l-min-y 1 :l-max-y 1
                  ))
  (send (send *robot-env* :robot :get :larm-contact-coords)
        :put :contact-constraint
        (instance default-contact-constraint
                  :init
                  :mu-trans hand-mu-trans
                  :mu-rot mu-rot
                  :max-fz max-fz
                  :l-min-x -1 :l-max-x -1 :l-min-y 1 :l-max-y 1
                  ))
  )

(defun demo-darwin-toy-terrain
    (&key
     (optimize-torque? nil)
     (optimize-key-frame-torque? t)
     (output-filename
      nil ;; (ros::resolve-ros-path "package://eus_qp/optmotiongen/logs/demo-darwin-toy-terrain/tmp")
      )
     )
  ;; setup environment
  (setq *ground-rleg* (make-cube 300 300 20))
  (setq *ground-lleg* (make-cube 300 300 20))
  (setq *ground-rarm* (make-cube 300 300 20))
  (setq *ground-larm* (make-cube 300 300 20))
  (setq *obst1* (make-cube 300 300 20))
  (send *ground-rleg* :move-coords
        (make-coords :pos (float-vector 0 0 0) :rpy (list 0 0 (deg2rad -32)))
        (make-coords :pos (float-vector 0 150 0)))
  (send *ground-lleg* :move-coords
        (make-coords :pos (float-vector 0 0 0) :rpy (list 0 0 (deg2rad 18)))
        (make-coords :pos (float-vector 0 -150 0)))
  (send *ground-rarm* :move-coords
        (make-coords :pos (float-vector 200 0 230) :rpy (list 0 (deg2rad -53) 0))
        (make-coords :pos (float-vector -150 0 0)))
  (send *ground-larm* :move-coords
        (make-coords :pos (float-vector 200 0 230) :rpy (list 0 (deg2rad -53) 0))
        (make-coords :pos (float-vector -150 0 0)))
  (send *obst1* :move-coords
        (send (send *ground-lleg* :copy-worldcoords)
              :transform (make-coords :pos (float-vector 0 150 0) :rpy (list 0 0 pi/2)))
        (make-coords :pos (float-vector 0 -50 0)))
  (setq *env-list* (list *ground-rleg* *ground-lleg* *ground-rarm* *ground-larm*))
  (setq *obst-list* (list *obst1*))
  (send-all *env-list* :worldcoords)
  (send-all *env-list* :set-color (float-vector 0.5 0.5 0.25))
  (send-all *obst-list* :set-color (float-vector 1 0 0) 0.5)
  ;; setup robot
  (setup-robot-for-demo-darwin-toy-terrain)
  (setq *contact-list*
        (list (instance 2d-planar-contact :init :name "rleg" :contact-pre-offset 50)
              (instance 2d-planar-contact :init :name "lleg")
              (instance 2d-planar-contact :init :name "rarm")
              (instance 2d-planar-contact :init :name "larm")
              ))
  (setq *robot-env*
        (instance robot-environment :init :robot *robot* :contact-list *contact-list*))
  (send *robot-env* :contact "rleg" :set-from-face :face (elt (send *ground-rleg* :faces) 1) :margin 100)
  (send *robot-env* :contact "lleg" :set-from-face :face (elt (send *ground-lleg* :faces) 1) :margin 100)
  (send *robot-env* :contact "rarm" :set-from-face :face (elt (send *ground-rarm* :faces) 1) :margin 100)
  (send *robot-env* :contact "larm" :set-from-face :face (elt (send *ground-larm* :faces) 1) :margin 100)
  (send *robot-env* :contact "rleg" :rotate pi :z :local)
  (send *robot-env* :contact "lleg" :rotate pi :z :local)
  (setup-contact-constraint-for-demo-darwin-toy-terrain)
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
             :name :rarm-lift
             :robot-env (car *robot-env-list*)
             :drive-joint-list
             (if optimize-torque? (send (car *robot-env-list*) :drive-joint-list) nil)
             :kin-target-coords-list
             (list
              (send (car *robot-env-list*) :contact "rleg" :contact-coords)
              (send (car *robot-env-list*) :contact "lleg" :contact-coords)
              )
             :kin-attention-coords-list
             (list
              (send (car *robot-env-list*) :robot :get :rleg-contact-coords)
              (send (car *robot-env-list*) :robot :get :lleg-contact-coords)
              )
             :contact-target-coords-list
             (list
              (send (car *robot-env-list*) :contact "rleg" :contact-coords)
              (send (car *robot-env-list*) :contact "lleg" :contact-coords)
              )
             :contact-attention-coords-list
             (list
              (send (car *robot-env-list*) :robot :get :rleg-contact-coords)
              (send (car *robot-env-list*) :robot :get :lleg-contact-coords)
              )
             :posture-joint-list
             (send (car *robot-env-list*) :robot :joint-list)
             :posture-joint-angle-list
             (send-all (send (car *robot-env-list*) :robot :joint-list) :joint-angle)
             :collision-pair-list
             (append
              (mapcar #'(lambda (l) (list *obst1* l)) (send (car *robot-env-list*) :robot :larm :links))
              (list (list (send (car *robot-env-list*) :link "r_el_link") *ground-rarm*)))
             :collision-distance-margin 0.02
             )
   *instant-config-task-list*
   )

  (push (copy-object *robot-env*) *robot-env-list*)
  (push
   (instance instant-configuration-task :init
             :name :rarm-contact-without-force
             :robot-env (car *robot-env-list*)
             :drive-joint-list
             (if (or optimize-torque? optimize-key-frame-torque?) (send (car *robot-env-list*) :drive-joint-list) nil)
             :kin-target-coords-list
             (list
              (send (car *robot-env-list*) :contact "rleg" :contact-coords)
              (send (car *robot-env-list*) :contact "lleg" :contact-coords)
              (send (car *robot-env-list*) :contact "rarm" :contact-coords)
              )
             :kin-attention-coords-list
             (list
              (send (car *robot-env-list*) :robot :get :rleg-contact-coords)
              (send (car *robot-env-list*) :robot :get :lleg-contact-coords)
              (send (car *robot-env-list*) :robot :get :rarm-contact-coords)
              )
             :contact-target-coords-list
             (list
              (send (car *robot-env-list*) :contact "rleg" :contact-coords)
              (send (car *robot-env-list*) :contact "lleg" :contact-coords)
              )
             :contact-attention-coords-list
             (list
              (send (car *robot-env-list*) :robot :get :rleg-contact-coords)
              (send (car *robot-env-list*) :robot :get :lleg-contact-coords)
              )
             :posture-joint-list
             (send (car *robot-env-list*) :robot :joint-list)
             :posture-joint-angle-list
             (send-all (send (car *robot-env-list*) :robot :joint-list) :joint-angle)
             :collision-pair-list
             (mapcar #'(lambda (l) (list *obst1* l)) (send (car *robot-env-list*) :robot :larm :links))
             :collision-distance-margin 0.02
             )
   *instant-config-task-list*
   )

  (push (copy-object *robot-env*) *robot-env-list*)
  (push
   (instance instant-configuration-task :init
             :name :rleg-contact-without-force
             :robot-env (car *robot-env-list*)
             :drive-joint-list
             (if optimize-torque? (send (car *robot-env-list*) :drive-joint-list) nil)
             :kin-target-coords-list
             (list
              (send (car *robot-env-list*) :contact "rleg" :contact-coords)
              (send (car *robot-env-list*) :contact "lleg" :contact-coords)
              (send (car *robot-env-list*) :contact "rarm" :contact-coords)
              )
             :kin-attention-coords-list
             (list
              (send (car *robot-env-list*) :robot :get :rleg-contact-coords)
              (send (car *robot-env-list*) :robot :get :lleg-contact-coords)
              (send (car *robot-env-list*) :robot :get :rarm-contact-coords)
              )
             :contact-target-coords-list
             (list
              (send (car *robot-env-list*) :contact "lleg" :contact-coords)
              (send (car *robot-env-list*) :contact "rarm" :contact-coords)
              )
             :contact-attention-coords-list
             (list
              (send (car *robot-env-list*) :robot :get :lleg-contact-coords)
              (send (car *robot-env-list*) :robot :get :rarm-contact-coords)
              )
             :posture-joint-list
             (send (car *robot-env-list*) :robot :joint-list)
             :posture-joint-angle-list
             (send-all (send (car *robot-env-list*) :robot :joint-list) :joint-angle)
             :collision-pair-list
             (mapcar #'(lambda (l) (list *obst1* l)) (send (car *robot-env-list*) :robot :larm :links))
             :collision-distance-margin 0.02
             )
   *instant-config-task-list*
   )

  (push (copy-object *robot-env*) *robot-env-list*)
  (push
   (instance instant-configuration-task :init
             :name :rleg-lift
             :robot-env (car *robot-env-list*)
             :drive-joint-list
             (if optimize-torque? (send (car *robot-env-list*) :drive-joint-list) nil)
             :kin-target-coords-list
             (list
              (send (car *robot-env-list*) :contact "rleg" :contact-pre-coords)
              (send (car *robot-env-list*) :contact "lleg" :contact-coords)
              (send (car *robot-env-list*) :contact "rarm" :contact-coords)
              )
             :kin-attention-coords-list
             (list
              (send (car *robot-env-list*) :robot :get :rleg-contact-coords)
              (send (car *robot-env-list*) :robot :get :lleg-contact-coords)
              (send (car *robot-env-list*) :robot :get :rarm-contact-coords)
              )
             :contact-target-coords-list
             (list
              (send (car *robot-env-list*) :contact "lleg" :contact-coords)
              (send (car *robot-env-list*) :contact "rarm" :contact-coords)
              )
             :contact-attention-coords-list
             (list
              (send (car *robot-env-list*) :robot :get :lleg-contact-coords)
              (send (car *robot-env-list*) :robot :get :rarm-contact-coords)
              )
             :posture-joint-list
             (send (car *robot-env-list*) :robot :joint-list)
             :posture-joint-angle-list
             (send-all (send (car *robot-env-list*) :robot :joint-list) :joint-angle)
             :collision-pair-list
             (mapcar #'(lambda (l) (list *obst1* l)) (send (car *robot-env-list*) :robot :larm :links))
             :collision-distance-margin 0.02
             )
   *instant-config-task-list*
   )

  (push (copy-object *robot-env*) *robot-env-list*)
  (push
   (instance instant-configuration-task :init
             :name :rleg-lift-2
             :robot-env (car *robot-env-list*)
             :drive-joint-list
             (if (or optimize-torque? optimize-key-frame-torque?) (send (car *robot-env-list*) :drive-joint-list) nil)
             :kin-target-coords-list
             (list
              (send (car *robot-env-list*) :contact "lleg" :contact-coords)
              (send (car *robot-env-list*) :contact "rarm" :contact-coords)
              )
             :kin-attention-coords-list
             (list
              (send (car *robot-env-list*) :robot :get :lleg-contact-coords)
              (send (car *robot-env-list*) :robot :get :rarm-contact-coords)
              )
             :contact-target-coords-list
             (list
              (send (car *robot-env-list*) :contact "lleg" :contact-coords)
              (send (car *robot-env-list*) :contact "rarm" :contact-coords)
              )
             :contact-attention-coords-list
             (list
              (send (car *robot-env-list*) :robot :get :lleg-contact-coords)
              (send (car *robot-env-list*) :robot :get :rarm-contact-coords)
              )
             :posture-joint-list
             (send (car *robot-env-list*) :robot :joint-list)
             :posture-joint-angle-list
             (send-all (send (car *robot-env-list*) :robot :joint-list) :joint-angle)
             :collision-pair-list
             (append
              (mapcar #'(lambda (l) (list *obst1* l)) (send (car *robot-env-list*) :robot :larm :links))
              (list (list (send (car *robot-env-list*) :link "r_ank_roll_link") *ground-rleg*)))
             :collision-distance-margin 0.02
             )
   *instant-config-task-list*
   )

  (objects (append *robot-env-list* *env-list* *obst-list*))

  (setq *trajectory-config-task*
        (instance* trajectory-configuration-task :init
                   :instant-config-task-list (reverse *instant-config-task-list*)
                   (if optimize-torque?
                       (list
                        :norm-regular-scale-max 1e-4
                        :norm-regular-scale-offset 1e-7
                        :adjacent-regular-scale 5e-4
                        :torque-regular-scale 1e-4
                        ))
                   ))

  (setq *sqp-opt*
        (instance sqp-optimization :init
                  :config-task *trajectory-config-task*
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
    (dump-loadable-structure (format nil "~a.l" output-filename) *rs-list*)
    (dump-seq-pattern-file
     *rs-list*
     output-filename
     :robot (send *robot-env* :robot)
     ))

  (objects (append (list *robot-env*) *env-list* *obst-list*))
  (send *trajectory-config-task* :play-animation :robot-env *robot-env*)
  )
(warn "(demo-darwin-toy-terrain)~%")



(defun move-real-robot-for-demo-darwin-toy-terrain
    (&key
     (send-reverse? t)
     )
  (setq *av-seq*
        (send-all (send-all (send-all
                             (send *trajectory-config-task* :instant-config-task-list)
                             :robot-env) :robot) :angle-vector))
  (send *robot* :angle-vector (car *av-seq*))
  (send *robot* :rarm :angle-vector #f(0.0 0.0 0.0))
  (send *ri* :angle-vector (send *robot* :angle-vector) 5000)
  (send *ri* :wait-interpolation)
  (dolist (av (if send-reverse? (append *av-seq* (cdr (reverse *av-seq*))) *av-seq*))
    (send *robot* :angle-vector av)
    (send *irtviewer* :draw-objects)
    (read-line)
    (send *ri* :angle-vector (send *robot* :angle-vector) 5000)
    (send *ri* :wait-interpolation)
    )
  (send *robot* :angle-vector (car *av-seq*))
  (send *robot* :rarm :angle-vector #f(0.0 0.0 0.0))
  (send *ri* :angle-vector (send *robot* :angle-vector) 5000)
  (send *ri* :wait-interpolation)
  )
