(load "package://eus_qp/optmotiongen/euslisp/trajectory-configuration-task.l")
(load "package://eus_qp/optmotiongen/euslisp/sqp-msc-optimization.l")
(load "package://eus_qp/optmotiongen/euslisp/validate-jacobian.l")
(require "models/arrow-object.l")


(defun demo-2d-body-trajectory
    (&key
     (num-of-solution-candidates 4)
     (pre-process-func)
     (visualize-callback-func)
     (visualize-loop? t)
     &allow-other-keys
     )
  ;; setup target
  (setq *arrow-start* (instance arrow-object :init))
  (send *arrow-start* :newcoords (make-coords :pos (float-vector 0 0 0)))
  (setq *arrow-finish* (instance arrow-object :init))
  (send *arrow-finish* :newcoords (make-coords :pos (float-vector 1500 0 0)))
  ;; setup environment
  (setq *obst1* (make-cylinder 250 10 :segments 5))
  (send *obst1* :set-color (float-vector 0 1 1))
  (send *obst1* :newcoords (make-coords :pos (float-vector 550 50 0)))
  (setq *obst2* (make-cube 200 500 10))
  (send *obst2* :set-color (float-vector 0 1 1))
  (send *obst2* :newcoords (make-coords :pos (float-vector 1000 -500 0) :rpy (list (deg2rad -10) 0 0)))
  (setq *obst3* (make-cube 100 400 10))
  (send *obst3* :set-color (float-vector 0 1 1))
  (send *obst3* :newcoords (make-coords :pos (float-vector 1100 300 0) :rpy (list (deg2rad 30) 0 0)))
  (setq *obst-list* (list *obst1* *obst2* *obst3*))
  (setq *env-list* (append (list *arrow-start* *arrow-finish*) *obst-list*))
  ;; setup robot
  (let* ((body1 (make-cylinder 30 50))
         (clink (instance cascaded-link :init))
         )
    (send body1 :set-color (float-vector 1 0 1))
    (setq link1 (instance bodyset-link :init (make-cascoords) :bodies (list body1) :name :suitcase-link))
    (send clink :assoc link1)
    (setf (slot clink cascaded-link 'links) (list link1))
    (setf (slot clink cascaded-link 'joint-list) nil)
    (send clink :init-ending)
    (setq *robot* clink)
    (send *robot* :put :robot-coords
          (make-cascoords :parent (car (send *robot* :links))))
    )
  (setq *robot-env*
        (instance robot-environment :init :robot *robot* :root-virtual-mode :planar))
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
             :name :start
             :robot-env (car *robot-env-list*)
             :only-kinematics? t
             :kin-target-coords-list
             (list (send *arrow-start* :copy-worldcoords))
             :kin-attention-coords-list
             (list (send (car *robot-env-list*) :robot :get :robot-coords))
             :collision-pair-list
             (mapcar #'(lambda (x) (list (car (send (car *robot-env-list*) :robot :links)) x)) *obst-list*)
             :collision-distance-margin 0.1
             )
   *instant-config-task-list*
   )

  (dotimes (i 8)
    (push (copy-object *robot-env*) *robot-env-list*)
    (push
     (instance instant-configuration-task :init
               :name :mid
               :robot-env (car *robot-env-list*)
               :only-kinematics? t
               :kin-target-coords-list nil
               :kin-attention-coords-list nil
               :collision-pair-list
               (mapcar #'(lambda (x) (list (car (send (car *robot-env-list*) :robot :links)) x)) *obst-list*)
               :collision-distance-margin 0.1
               )
     *instant-config-task-list*
     )
    )

  (push (copy-object *robot-env*) *robot-env-list*)
  (push
   (instance instant-configuration-task :init
             :name :finish
             :robot-env (car *robot-env-list*)
             :only-kinematics? t
             :kin-target-coords-list
             (list (send *arrow-finish* :copy-worldcoords))
             :kin-attention-coords-list
             (list (send (car *robot-env-list*) :robot :get :robot-coords))
             :collision-pair-list
             (mapcar #'(lambda (x) (list (car (send (car *robot-env-list*) :robot :links)) x)) *obst-list*)
             :collision-distance-margin 0.1
             )
   *instant-config-task-list*
   )

  (setq *trajectory-config-task*
        (instance trajectory-configuration-task :init
                  :instant-config-task-list (reverse *instant-config-task-list*)
                  :norm-regular-scale-max 1e-4
                  :norm-regular-scale-offset 1e-7
                  :adjacent-regular-scale 1e-2
                  ))

  (setq *sqp-opt*
        (instance sqp-msc-optimization :init
                  :config-task *trajectory-config-task*
                  :pre-process-func
                  pre-process-func
                  :post-process-func
                  #'(lambda (sqp)
                      (send *irtviewer* :viewer :viewing :look #f(800 0 6000) #f(800 0 0) #f(1 0 0))
                      (when (= (mod (send sqp :iteration) 20) 0)
                        ;; (mapcar
                        ;;  #'(lambda (config-task) (validate-element-jacobian :config-task config-task))
                        ;;  ;; #'(lambda (config-task) (validate-jacobian :config-task config-task))
                        ;;  (send (send sqp :config-task) :instant-config-task-list))
                        ))
                  :num-msc num-of-solution-candidates
                  :dispersion-scale 1e-1
                  :dispersion-scale-min 1e-10
                  :dispersion-scale-decrease-ratio 0.8
                  :config-vector-dist2-min 1e-15
                  ))

  ;; set different color for each solution candidate
  (dotimes (i (length (send *sqp-opt* :config-task-list)))
    (let* ((config-task
            (elt (send *sqp-opt* :config-task-list) i))
           (robot-env-list
            (send-all (send config-task :instant-config-task-list) :robot-env))
           (bodies
            (flatten (send-all robot-env-list :robot :bodies)))
           (color
            (scale (/ 1.0  255) (coerce (color-category10 (1+ i)) float-vector)))
           )
      (send-all bodies :set-color color)
      ))
  (objects
   (append (send-all (flatten (send-all (send *sqp-opt* :config-task-list) :instant-config-task-list)) :robot-env)
           *env-list*))

  (send *sqp-opt* :optimize :loop-num 50)

  ;; play motion of all solution candidates
  (do-until-key
   (dolist (config-task (send *sqp-opt* :config-task-list))
     (let* ((robot-env
             (send (car (send config-task :instant-config-task-list)) :robot-env))
            (av-orig
             (send robot-env :angle-vector))
            )
       (objects (append (list robot-env) *env-list*))
       (send *irtviewer* :viewer :viewing :look #f(800 0 6000) #f(800 0 0) #f(1 0 0))
       (send *irtviewer* :draw-objects)
       (send config-task :play-animation
             :robot-env robot-env
             :loop? nil
             :visualize-callback-func visualize-callback-func
             )
       (send robot-env :angle-vector av-orig)
       ))
   (unless visualize-loop?
     (return-from nil nil))
   )
  )
(warn "(demo-2d-body-trajectory)~%")
