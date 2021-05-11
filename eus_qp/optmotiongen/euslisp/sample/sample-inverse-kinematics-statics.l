(load "../inverse-kinematics-statics-wrapper.l")
(require "irteus/demo/sample-robot-model.l")
(require "models/arrow-object.l")


(defun sample-robot-reach-iks-raw
    (&key
     (optimize-torque? t)
     (pre-process-func)
     (post-process-func)
     (loop-num 50)
     &allow-other-keys
     )
  (setq *robot* (instance sample-robot :init))
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  (create-support-polygon *robot* (send (send *robot* :rleg :end-coords) :parent))
  (create-support-polygon *robot* (send (send *robot* :lleg :end-coords) :parent))
  (setq *robot-env*
        (instance robot-environment :init :robot *robot* :root-virtual-mode :6dof))
  (setq *arrow* (instance arrow-object :init))
  (send *arrow* :newcoords
        (make-coords :pos (float-vector 400 -200 600) :rpy (list 0 0 0)))
  (objects (list *robot* *arrow*))

  (setq *instant-config-task*
        (instance instant-configuration-task :init
                  :robot-env *robot-env*
                  :drive-joint-list (if optimize-torque? (send *robot-env* :drive-joint-list))
                  :kin-target-coords-list
                  (list (send (send *robot* :rleg :end-coords) :copy-worldcoords)
                        (send (send *robot* :lleg :end-coords) :copy-worldcoords)
                        (send *arrow* :copy-worldcoords)
                        )
                  :kin-attention-coords-list
                  (list (send *robot* :rleg :end-coords)
                        (send *robot* :lleg :end-coords)
                        (send *robot* :rarm :end-coords)
                        )
                  :contact-target-coords-list
                  (list (send (send *robot* :rleg :end-coords) :copy-worldcoords)
                        (send (send *robot* :lleg :end-coords) :copy-worldcoords)
                        (send *arrow* :copy-worldcoords)
                        )
                  :contact-attention-coords-list
                  (list (send *robot* :rleg :end-coords)
                        (send *robot* :lleg :end-coords)
                        (send *robot* :rarm :end-coords)
                        )
                  :contact-constraint-list
                  (list
                   (instance default-contact-constraint
                             :init-from-support-polygon
                             :support-polygon (send (send (send *robot-env* :robot :rleg :end-coords) :parent) :get :support-polygon)
                             :support-polygon-coords (send (send *robot-env* :robot :rleg :end-coords) :worldcoords)
                             )
                   (instance default-contact-constraint
                             :init-from-support-polygon
                             :support-polygon (send (send (send *robot-env* :robot :lleg :end-coords) :parent) :get :support-polygon)
                             :support-polygon-coords (send (send *robot-env* :robot :lleg :end-coords) :worldcoords)
                             )
                   (instance 6d-min-max-contact-constraint
                             :init (float-vector 20 20 20 0 0 0))
                   )
                  :norm-regular-scale-max 1e-5
                  :norm-regular-scale-offset 1e-6
                  ))

  (setq *sqp-opt*
        (instance sqp-optimization :init
                  :config-task *instant-config-task*
                  :pre-process-func pre-process-func
                  :post-process-func post-process-func
                  ))
  (send *sqp-opt* :optimize :loop-num loop-num)
  t)
(warn "(sample-robot-reach-iks-raw)~%")

(defun sample-robot-reach-iks
    (&key
     (optimize-torque? t)
     (target-coords
      (make-coords :pos (float-vector 400 -200 600) :rpy (list 0 0 0)))
     (pre-process-func)
     (post-process-func)
     (stop 50)
     &allow-other-keys
     )
  (setq *robot* (instance sample-robot :init))
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  (create-support-polygon *robot* (send (send *robot* :rleg :end-coords) :parent))
  (create-support-polygon *robot* (send (send *robot* :lleg :end-coords) :parent))
  (setq *arrow* (instance arrow-object :init))
  (send *arrow* :newcoords target-coords)
  (objects (list *robot* *arrow*))

  (send *robot*
        :inverse-kinematics-statics-optmotiongen
        (list (send (send *robot* :rleg :end-coords) :copy-worldcoords)
              (send (send *robot* :lleg :end-coords) :copy-worldcoords)
              (send *arrow* :copy-worldcoords)
              )
        :move-target
        (list (send *robot* :rleg :end-coords)
              (send *robot* :lleg :end-coords)
              (send *robot* :rarm :end-coords)
              )
        :contact-target-coords
        (list (send (send *robot* :rleg :end-coords) :copy-worldcoords)
              (send (send *robot* :lleg :end-coords) :copy-worldcoords)
              (send *arrow* :copy-worldcoords)
              )
        :contact-move-target
        (list (send *robot* :rleg :end-coords)
              (send *robot* :lleg :end-coords)
              (send *robot* :rarm :end-coords)
              )
        :contact-constraint
        (list
         (instance default-contact-constraint
                   :init-from-support-polygon
                   :support-polygon (send (send (send *robot* :rleg :end-coords) :parent) :get :support-polygon)
                   :support-polygon-coords (send (send *robot* :rleg :end-coords) :worldcoords)
                   )
         (instance default-contact-constraint
                   :init-from-support-polygon
                   :support-polygon (send (send (send *robot* :lleg :end-coords) :parent) :get :support-polygon)
                   :support-polygon-coords (send (send *robot* :lleg :end-coords) :worldcoords)
                   )
         (instance 6d-min-max-contact-constraint
                   :init (float-vector 20 20 20 0 0 0))
         )
        :translation-axis (list t t t)
        :rotation-axis (list t t t)
        :debug-view t
        :root-virtual-mode :6dof
        :optimize-torque? optimize-torque?
        :stop stop
        :min-loop 30
        :pre-process-func pre-process-func
        :post-process-func post-process-func
        )
  )
(warn "(sample-robot-reach-iks)~%")

(defun sample-robot-reach-iks-face
    (&key
     (optimize-torque? t)
     (pre-process-func)
     (post-process-func)
     (stop 50)
     &allow-other-keys
     )
  (setq *robot* (instance sample-robot :init))
  (send *robot* :put :larm-contact-coords
        (make-cascoords
         :coords (send (send (send *robot* :larm :end-coords) :copy-worldcoords) :rotate -pi/2 :y)
         :parent (send (send *robot* :larm :end-coords) :parent)
         ))
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  (create-support-polygon *robot* (send (send *robot* :rleg :end-coords) :parent))
  (create-support-polygon *robot* (send (send *robot* :lleg :end-coords) :parent))
  (setq *arrow* (instance arrow-object :init))
  (send *arrow* :newcoords
        (make-coords :pos (float-vector 600 -400 800) :rpy (list 0 0 0)))
  (setq *wall* (make-cube 300 200 5))
  (send *wall* :newcoords
        (make-coords :pos (float-vector 500 200 500)
                     :rpy (list 0 (deg2rad -60) 0)))
  (send *wall* :set-color (float-vector 1.0 0.6 0.6))
  (objects (list *robot* *arrow* *wall*))

  (setq *face* (elt (send *wall* :faces) 1))
  (send *face* :put :margin 20) ;; [mm]
  (send *face* :put :normal-move-target-axis :z)
  (send *face* :name :face-larm)
  (send *robot*
        :inverse-kinematics-statics-optmotiongen
        (list (send (send *robot* :rleg :end-coords) :copy-worldcoords)
              (send (send *robot* :lleg :end-coords) :copy-worldcoords)
              (send *arrow* :copy-worldcoords)
              *face*
              )
        :move-target
        (list (send *robot* :rleg :end-coords)
              (send *robot* :lleg :end-coords)
              (send *robot* :rarm :end-coords)
              (send *robot* :get :larm-contact-coords)
              )
        :contact-target-coords
        (list (send (send *robot* :rleg :end-coords) :copy-worldcoords)
              (send (send *robot* :lleg :end-coords) :copy-worldcoords)
              *face*
              )
        :contact-move-target
        (list (send *robot* :rleg :end-coords)
              (send *robot* :lleg :end-coords)
              (send *robot* :get :larm-contact-coords)
              )
        :contact-constraint
        (list
         (instance default-contact-constraint
                   :init-from-support-polygon
                   :support-polygon (send (send (send *robot* :rleg :end-coords) :parent) :get :support-polygon)
                   :support-polygon-coords (send (send *robot* :rleg :end-coords) :worldcoords)
                   )
         (instance default-contact-constraint
                   :init-from-support-polygon
                   :support-polygon (send (send (send *robot* :lleg :end-coords) :parent) :get :support-polygon)
                   :support-polygon-coords (send (send *robot* :lleg :end-coords) :worldcoords)
                   )
         (instance default-contact-constraint
                   :init-from-support-polygon :support-polygon nil)
         )
        :translation-axis (list t t t t)
        :rotation-axis (list t t nil t)
        :debug-view t
        :root-virtual-mode :6dof
        :optimize-torque? optimize-torque?
        :stop stop
        :min-loop 30
        :pre-process-func pre-process-func
        :post-process-func post-process-func
        )
  )
(warn "(sample-robot-reach-iks-face)~%")

(defun sample-robot-reach-trajectory-iks-raw
    (&key
     (optimize-torque? nil)
     (optimize-start-end-torque? t)
     (pre-process-func)
     (post-process-func)
     (visualize-callback-func)
     (loop-num 50)
     &allow-other-keys
     )
  (setq *robot* (instance sample-robot :init))
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  (create-support-polygon *robot* (send (send *robot* :rleg :end-coords) :parent))
  (create-support-polygon *robot* (send (send *robot* :lleg :end-coords) :parent))
  (setq *robot-env*
        (instance robot-environment :init :robot *robot* :root-virtual-mode :6dof))
  (setq *arrow* (instance arrow-object :init))
  (send *arrow* :newcoords
        (make-coords :pos (float-vector 400 -200 600) :rpy (list 0 0 0)))

  (setq *robot-env-list* nil)
  (setq *instant-config-task-list* nil)

  (push (copy-object *robot-env*) *robot-env-list*)
  (push
   (instance instant-configuration-task :init
             :name :both-feet-support
             :robot-env (car *robot-env-list*)
             :drive-joint-list
             (if (or optimize-torque? optimize-start-end-torque?) (send (car *robot-env-list*) :drive-joint-list) nil)
             :kin-target-coords-list
             (list (send (send (car *robot-env-list*) :robot :rleg :end-coords) :copy-worldcoords)
                   (send (send (car *robot-env-list*) :robot :lleg :end-coords) :copy-worldcoords)
                   )
             :kin-attention-coords-list
             (list (send (car *robot-env-list*) :robot :rleg :end-coords)
                   (send (car *robot-env-list*) :robot :lleg :end-coords)
                   )
             :contact-target-coords-list
             (list (send (send (car *robot-env-list*) :robot :rleg :end-coords) :copy-worldcoords)
                   (send (send (car *robot-env-list*) :robot :lleg :end-coords) :copy-worldcoords)
                   )
             :contact-attention-coords-list
             (list (send (car *robot-env-list*) :robot :rleg :end-coords)
                   (send (car *robot-env-list*) :robot :lleg :end-coords)
                   )
             :contact-constraint-list
             (list
              (instance default-contact-constraint
                        :init-from-support-polygon
                        :support-polygon (send (send (send (car *robot-env-list*) :robot :rleg :end-coords) :parent) :get :support-polygon)
                        :support-polygon-coords (send (send (car *robot-env-list*) :robot :rleg :end-coords) :worldcoords)
                        )
              (instance default-contact-constraint
                        :init-from-support-polygon
                        :support-polygon (send (send (send (car *robot-env-list*) :robot :lleg :end-coords) :parent) :get :support-polygon)
                        :support-polygon-coords (send (send (car *robot-env-list*) :robot :lleg :end-coords) :worldcoords)
                        )
              )
             :posture-joint-list
             (send (car *robot-env-list*) :robot :joint-list)
             :posture-joint-angle-list
             (send-all (send (car *robot-env-list*) :robot :joint-list) :joint-angle)
             :target-posture-scale 1e-2
             )
   *instant-config-task-list*)

  (push (copy-object *robot-env*) *robot-env-list*)
  (push
   (instance instant-configuration-task :init
             :name :reach-right-hand
             :robot-env (car *robot-env-list*)
             :drive-joint-list
             (if optimize-torque? (send (car *robot-env-list*) :drive-joint-list))
             :kin-target-coords-list
             (list (send (send (car *robot-env-list*) :robot :rleg :end-coords) :copy-worldcoords)
                   (send (send (car *robot-env-list*) :robot :lleg :end-coords) :copy-worldcoords)
                   (send *arrow* :copy-worldcoords)
                   )
             :kin-attention-coords-list
             (list (send (car *robot-env-list*) :robot :rleg :end-coords)
                   (send (car *robot-env-list*) :robot :lleg :end-coords)
                   (send (car *robot-env-list*) :robot :rarm :end-coords)
                   )
             :contact-target-coords-list
             (list (send (send (car *robot-env-list*) :robot :rleg :end-coords) :copy-worldcoords)
                   (send (send (car *robot-env-list*) :robot :lleg :end-coords) :copy-worldcoords)
                   )
             :contact-attention-coords-list
             (list (send (car *robot-env-list*) :robot :rleg :end-coords)
                   (send (car *robot-env-list*) :robot :lleg :end-coords)
                   )
             :contact-constraint-list
             (list
              (instance default-contact-constraint
                        :init-from-support-polygon
                        :support-polygon (send (send (send (car *robot-env-list*) :robot :rleg :end-coords) :parent) :get :support-polygon)
                        :support-polygon-coords (send (send (car *robot-env-list*) :robot :rleg :end-coords) :worldcoords)
                        )
              (instance default-contact-constraint
                        :init-from-support-polygon
                        :support-polygon (send (send (send (car *robot-env-list*) :robot :lleg :end-coords) :parent) :get :support-polygon)
                        :support-polygon-coords (send (send (car *robot-env-list*) :robot :lleg :end-coords) :worldcoords)
                        )
              )
             )
   *instant-config-task-list*)

  (push (copy-object *robot-env*) *robot-env-list*)
  (push
   (instance instant-configuration-task :init
             :name :add-right-hand-contact
             :robot-env (car *robot-env-list*)
             :drive-joint-list
             (if optimize-torque? (send (car *robot-env-list*) :drive-joint-list))
             :kin-target-coords-list
             (list (send (send (car *robot-env-list*) :robot :rleg :end-coords) :copy-worldcoords)
                   (send (send (car *robot-env-list*) :robot :lleg :end-coords) :copy-worldcoords)
                   (send *arrow* :copy-worldcoords)
                   )
             :kin-attention-coords-list
             (list (send (car *robot-env-list*) :robot :rleg :end-coords)
                   (send (car *robot-env-list*) :robot :lleg :end-coords)
                   (send (car *robot-env-list*) :robot :rarm :end-coords)
                   )
             :contact-target-coords-list
             (list (send (send (car *robot-env-list*) :robot :rleg :end-coords) :copy-worldcoords)
                   (send (send (car *robot-env-list*) :robot :lleg :end-coords) :copy-worldcoords)
                   (send *arrow* :copy-worldcoords)
                   )
             :contact-attention-coords-list
             (list (send (car *robot-env-list*) :robot :rleg :end-coords)
                   (send (car *robot-env-list*) :robot :lleg :end-coords)
                   (send (car *robot-env-list*) :robot :rarm :end-coords)
                   )
             :contact-constraint-list
             (list
              (instance default-contact-constraint
                        :init-from-support-polygon
                        :support-polygon (send (send (send (car *robot-env-list*) :robot :rleg :end-coords) :parent) :get :support-polygon)
                        :support-polygon-coords (send (send (car *robot-env-list*) :robot :rleg :end-coords) :worldcoords)
                        )
              (instance default-contact-constraint
                        :init-from-support-polygon
                        :support-polygon (send (send (send (car *robot-env-list*) :robot :lleg :end-coords) :parent) :get :support-polygon)
                        :support-polygon-coords (send (send (car *robot-env-list*) :robot :lleg :end-coords) :worldcoords)
                        )
              (instance 6d-min-max-contact-constraint
                        :init (float-vector 100 100 100 10 10 10))
              )
             )
   *instant-config-task-list*)

  (push (copy-object *robot-env*) *robot-env-list*)
  (push
   (instance instant-configuration-task :init
             :name :remove-right-foot-contact
             :robot-env (car *robot-env-list*)
             :drive-joint-list
             (if (or optimize-torque? optimize-start-end-torque?) (send (car *robot-env-list*) :drive-joint-list) nil)
             :kin-target-coords-list
             (list (send (send (send (car *robot-env-list*) :robot :rleg :end-coords) :copy-worldcoords) :translate (float-vector 0 0 100) :world)
                   (send (send (car *robot-env-list*) :robot :lleg :end-coords) :copy-worldcoords)
                   (send *arrow* :copy-worldcoords)
                   )
             :kin-attention-coords-list
             (list (send (car *robot-env-list*) :robot :rleg :end-coords)
                   (send (car *robot-env-list*) :robot :lleg :end-coords)
                   (send (car *robot-env-list*) :robot :rarm :end-coords)
                   )
             :contact-target-coords-list
             (list (send (send (car *robot-env-list*) :robot :lleg :end-coords) :copy-worldcoords)
                   (send *arrow* :copy-worldcoords)
                   )
             :contact-attention-coords-list
             (list (send (car *robot-env-list*) :robot :lleg :end-coords)
                   (send (car *robot-env-list*) :robot :rarm :end-coords)
                   )
             :contact-constraint-list
             (list
              (instance default-contact-constraint
                        :init-from-support-polygon
                        :support-polygon (send (send (send (car *robot-env-list*) :robot :lleg :end-coords) :parent) :get :support-polygon)
                        :support-polygon-coords (send (send (car *robot-env-list*) :robot :lleg :end-coords) :worldcoords)
                        )
              (instance 6d-min-max-contact-constraint
                        :init (float-vector 100 100 100 10 10 10))
              )
             )
   *instant-config-task-list*)

  (objects (append *robot-env-list* (list *arrow*)))

  (setq *trajectory-config-task*
        (instance trajectory-configuration-task :init
                  :instant-config-task-list (reverse *instant-config-task-list*)
                  :adjacent-regular-scale 1e-5
                  :torque-regular-scale 1e-3
                  ))

  (setq *sqp-opt*
        (instance sqp-optimization :init
                  :config-task *trajectory-config-task*
                  :pre-process-func pre-process-func
                  :post-process-func post-process-func
                  ))
  (send *sqp-opt* :optimize :loop-num loop-num)

  (objects (append (list *robot-env* *arrow*)))
  (send *trajectory-config-task* :play-animation
        :robot-env *robot-env*
        :loop? nil
        :visualize-callback-func visualize-callback-func
        )
  t)
(warn "(sample-robot-reach-trajectory-iks-raw)~%")

(defun sample-robot-reach-trajectory-iks
    (&key
     (optimize-torque? nil)
     (optimize-start-end-torque? t)
     (target-coords
      (make-coords :pos (float-vector 400 -200 600) :rpy (list 0 0 0)))
     (pre-process-func)
     (post-process-func)
     (visualize-callback-func)
     (stop 50)
     &allow-other-keys
     )
  (setq *robot* (instance sample-robot :init))
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  (create-support-polygon *robot* (send (send *robot* :rleg :end-coords) :parent))
  (create-support-polygon *robot* (send (send *robot* :lleg :end-coords) :parent))
  (setq *arrow* (instance arrow-object :init))
  (send *arrow* :newcoords target-coords)

  (setq
   *ret*
   (send *robot*
         :inverse-kinematics-statics-trajectory-optmotiongen
         (list
          (list (send (send *robot* :rleg :end-coords) :copy-worldcoords)
                (send (send *robot* :lleg :end-coords) :copy-worldcoords)
                )
          (list (send (send *robot* :rleg :end-coords) :copy-worldcoords)
                (send (send *robot* :lleg :end-coords) :copy-worldcoords)
                (send *arrow* :copy-worldcoords)
                )
          (list (send (send *robot* :rleg :end-coords) :copy-worldcoords)
                (send (send *robot* :lleg :end-coords) :copy-worldcoords)
                (send *arrow* :copy-worldcoords)
                )
          (list (send (send (send *robot* :rleg :end-coords) :copy-worldcoords) :translate (float-vector 0 0 100) :world)
                (send (send *robot* :lleg :end-coords) :copy-worldcoords)
                (send *arrow* :copy-worldcoords)
                )
          )
         :move-target-list
         (list
          (list (send *robot* :rleg :end-coords)
                (send *robot* :lleg :end-coords)
                )
          (list (send *robot* :rleg :end-coords)
                (send *robot* :lleg :end-coords)
                (send *robot* :rarm :end-coords)
                )
          (list (send *robot* :rleg :end-coords)
                (send *robot* :lleg :end-coords)
                (send *robot* :rarm :end-coords)
                )
          (list (send *robot* :rleg :end-coords)
                (send *robot* :lleg :end-coords)
                (send *robot* :rarm :end-coords)
                )
          )
         :contact-target-coords-list
         (list
          (list (send (send *robot* :rleg :end-coords) :copy-worldcoords)
                (send (send *robot* :lleg :end-coords) :copy-worldcoords)
                )
          (list (send (send *robot* :rleg :end-coords) :copy-worldcoords)
                (send (send *robot* :lleg :end-coords) :copy-worldcoords)
                )
          (list (send (send *robot* :rleg :end-coords) :copy-worldcoords)
                (send (send *robot* :lleg :end-coords) :copy-worldcoords)
                (send *arrow* :copy-worldcoords)
                )
          (list (send (send *robot* :lleg :end-coords) :copy-worldcoords)
                (send *arrow* :copy-worldcoords)
                )
          )
         :contact-move-target-list
         (list
          (list (send *robot* :rleg :end-coords)
                (send *robot* :lleg :end-coords)
                )
          (list (send *robot* :rleg :end-coords)
                (send *robot* :lleg :end-coords)
                )
          (list (send *robot* :rleg :end-coords)
                (send *robot* :lleg :end-coords)
                (send *robot* :rarm :end-coords)
                )
          (list (send *robot* :lleg :end-coords)
                (send *robot* :rarm :end-coords)
                )
          )
         :contact-constraint-list
         (list
          (list
           (instance default-contact-constraint
                     :init-from-support-polygon
                     :support-polygon (send (send (send *robot* :rleg :end-coords) :parent) :get :support-polygon)
                     :support-polygon-coords (send (send *robot* :rleg :end-coords) :worldcoords)
                     )
           (instance default-contact-constraint
                     :init-from-support-polygon
                     :support-polygon (send (send (send *robot* :lleg :end-coords) :parent) :get :support-polygon)
                     :support-polygon-coords (send (send *robot* :lleg :end-coords) :worldcoords)
                     )
           )
          (list
           (instance default-contact-constraint
                     :init-from-support-polygon
                     :support-polygon (send (send (send *robot* :rleg :end-coords) :parent) :get :support-polygon)
                     :support-polygon-coords (send (send *robot* :rleg :end-coords) :worldcoords)
                     )
           (instance default-contact-constraint
                     :init-from-support-polygon
                     :support-polygon (send (send (send *robot* :lleg :end-coords) :parent) :get :support-polygon)
                     :support-polygon-coords (send (send *robot* :lleg :end-coords) :worldcoords)
                     )
           )
          (list
           (instance default-contact-constraint
                     :init-from-support-polygon
                     :support-polygon (send (send (send *robot* :rleg :end-coords) :parent) :get :support-polygon)
                     :support-polygon-coords (send (send *robot* :rleg :end-coords) :worldcoords)
                     )
           (instance default-contact-constraint
                     :init-from-support-polygon
                     :support-polygon (send (send (send *robot* :lleg :end-coords) :parent) :get :support-polygon)
                     :support-polygon-coords (send (send *robot* :lleg :end-coords) :worldcoords)
                     )
           (instance 6d-min-max-contact-constraint
                     :init (float-vector 100 100 100 10 10 10))
           )
          (list
           (instance default-contact-constraint
                     :init-from-support-polygon
                     :support-polygon (send (send (send *robot* :lleg :end-coords) :parent) :get :support-polygon)
                     :support-polygon-coords (send (send *robot* :lleg :end-coords) :worldcoords)
                     )
           (instance 6d-min-max-contact-constraint
                     :init (float-vector 100 100 100 10 10 10))
           )
          )
         :debug-view (list :additional-draw-objects (list *arrow*))
         :root-virtual-mode :6dof
         :optimize-torque? optimize-torque?
         :optimize-start-end-torque? optimize-start-end-torque?
         :stop stop
         :min-loop 30
         :adjacent-regular-scale 1e-5
         :torque-regular-scale 1e-5
         :pre-process-func pre-process-func
         :post-process-func post-process-func
         ))
  (objects (list *robot* *arrow*))
  (visualize-trajectory-result *ret* :visualize-callback-func visualize-callback-func)
  *ret*
  )
(warn "(sample-robot-reach-trajectory-iks)~%")

(defun sample-robot-reach-trajectory-iks-face
    (&key
     (optimize-torque? nil)
     (optimize-start-end-torque? t)
     (pre-process-func)
     (post-process-func)
     (visualize-callback-func)
     (stop 100)
     &allow-other-keys
     )
  (setq *robot* (instance sample-robot :init))
  (send *robot* :put :larm-contact-coords
        (make-cascoords
         :coords (send (send (send *robot* :larm :end-coords) :copy-worldcoords) :rotate -pi/2 :y)
         :parent (send (send *robot* :larm :end-coords) :parent)
         ))
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  (create-support-polygon *robot* (send (send *robot* :rleg :end-coords) :parent))
  (create-support-polygon *robot* (send (send *robot* :lleg :end-coords) :parent))
  (setq *arrow* (instance arrow-object :init))
  (send *arrow* :newcoords
        (make-coords :pos (float-vector 600 -400 800) :rpy (list 0 0 0)))
  (setq *wall* (make-cube 300 200 5))
  (send *wall* :newcoords
        (make-coords :pos (float-vector 500 200 500)
                     :rpy (list 0 (deg2rad -60) 0)))
  (send *wall* :set-color (float-vector 1.0 0.6 0.6))
  (objects (list *robot* *arrow* *wall*))

  (setq *face* (elt (send *wall* :faces) 1))
  (send *face* :put :margin 20) ;; [mm]
  (send *face* :put :normal-move-target-axis :z)
  (send *face* :name :face-larm)

  (setq
   *ret*
   (send *robot*
         :inverse-kinematics-statics-trajectory-optmotiongen
         (list
          (list (send (send *robot* :rleg :end-coords) :copy-worldcoords)
                (send (send *robot* :lleg :end-coords) :copy-worldcoords)
                (send (send *wall* :copy-worldcoords) :translate (float-vector 0 0 200) :local)
                )
          (list (send (send *robot* :rleg :end-coords) :copy-worldcoords)
                (send (send *robot* :lleg :end-coords) :copy-worldcoords)
                *face*
                )
          (list (send (send *robot* :rleg :end-coords) :copy-worldcoords)
                (send (send *robot* :lleg :end-coords) :copy-worldcoords)
                *face*
                )
          (list (send (send *robot* :rleg :end-coords) :copy-worldcoords)
                (send (send *robot* :lleg :end-coords) :copy-worldcoords)
                (send *arrow* :copy-worldcoords)
                *face*
                )
          )
         :move-target-list
         (list
          (list (send *robot* :rleg :end-coords)
                (send *robot* :lleg :end-coords)
                (send *robot* :get :larm-contact-coords)
                )
          (list (send *robot* :rleg :end-coords)
                (send *robot* :lleg :end-coords)
                (send *robot* :get :larm-contact-coords)
                )
          (list (send *robot* :rleg :end-coords)
                (send *robot* :lleg :end-coords)
                (send *robot* :get :larm-contact-coords)
                )
          (list (send *robot* :rleg :end-coords)
                (send *robot* :lleg :end-coords)
                (send *robot* :rarm :end-coords)
                (send *robot* :get :larm-contact-coords)
                )
          )
         :contact-target-coords-list
         (list
          (list (send (send *robot* :rleg :end-coords) :copy-worldcoords)
                (send (send *robot* :lleg :end-coords) :copy-worldcoords)
                )
          (list (send (send *robot* :rleg :end-coords) :copy-worldcoords)
                (send (send *robot* :lleg :end-coords) :copy-worldcoords)
                )
          (list (send (send *robot* :rleg :end-coords) :copy-worldcoords)
                (send (send *robot* :lleg :end-coords) :copy-worldcoords)
                *face*
                )
          (list (send (send *robot* :rleg :end-coords) :copy-worldcoords)
                (send (send *robot* :lleg :end-coords) :copy-worldcoords)
                *face*
                )
          )
         :contact-move-target-list
         (list
          (list (send *robot* :rleg :end-coords)
                (send *robot* :lleg :end-coords)
                )
          (list (send *robot* :rleg :end-coords)
                (send *robot* :lleg :end-coords)
                )
          (list (send *robot* :rleg :end-coords)
                (send *robot* :lleg :end-coords)
                (send *robot* :get :larm-contact-coords)
                )
          (list (send *robot* :rleg :end-coords)
                (send *robot* :lleg :end-coords)
                (send *robot* :get :larm-contact-coords)
                )
          )
         :contact-constraint-list
         (list
          (list
           (instance default-contact-constraint
                     :init-from-support-polygon
                     :support-polygon (send (send (send *robot* :rleg :end-coords) :parent) :get :support-polygon)
                     :support-polygon-coords (send (send *robot* :rleg :end-coords) :worldcoords)
                     )
           (instance default-contact-constraint
                     :init-from-support-polygon
                     :support-polygon (send (send (send *robot* :lleg :end-coords) :parent) :get :support-polygon)
                     :support-polygon-coords (send (send *robot* :lleg :end-coords) :worldcoords)
                     )
           )
          (list
           (instance default-contact-constraint
                     :init-from-support-polygon
                     :support-polygon (send (send (send *robot* :rleg :end-coords) :parent) :get :support-polygon)
                     :support-polygon-coords (send (send *robot* :rleg :end-coords) :worldcoords)
                     )
           (instance default-contact-constraint
                     :init-from-support-polygon
                     :support-polygon (send (send (send *robot* :lleg :end-coords) :parent) :get :support-polygon)
                     :support-polygon-coords (send (send *robot* :lleg :end-coords) :worldcoords)
                     )
           )
          (list
           (instance default-contact-constraint
                     :init-from-support-polygon
                     :support-polygon (send (send (send *robot* :rleg :end-coords) :parent) :get :support-polygon)
                     :support-polygon-coords (send (send *robot* :rleg :end-coords) :worldcoords)
                     )
           (instance default-contact-constraint
                     :init-from-support-polygon
                     :support-polygon (send (send (send *robot* :lleg :end-coords) :parent) :get :support-polygon)
                     :support-polygon-coords (send (send *robot* :lleg :end-coords) :worldcoords)
                     )
           (instance default-contact-constraint
                     :init-from-support-polygon :support-polygon nil)
           )
          (list
           (instance default-contact-constraint
                     :init-from-support-polygon
                     :support-polygon (send (send (send *robot* :rleg :end-coords) :parent) :get :support-polygon)
                     :support-polygon-coords (send (send *robot* :rleg :end-coords) :worldcoords)
                     )
           (instance default-contact-constraint
                     :init-from-support-polygon
                     :support-polygon (send (send (send *robot* :lleg :end-coords) :parent) :get :support-polygon)
                     :support-polygon-coords (send (send *robot* :lleg :end-coords) :worldcoords)
                     )
           (instance default-contact-constraint
                     :init-from-support-polygon :support-polygon nil)
           )
          )
         :rotation-axis-list
         (list
          (list t t nil)
          (list t t t)
          (list t t t)
          (list t t t t)
          )
         :debug-view (list :additional-draw-objects (list *arrow* *wall*))
         :root-virtual-mode :6dof
         :optimize-torque? optimize-torque?
         :optimize-start-end-torque? optimize-start-end-torque?
         :stop stop
         :min-loop 30
         :norm-regular-scale-offset 1e-5
         :adjacent-regular-scale 1e-5
         :torque-regular-scale 1e-6
         :thre 5
         :rthre (deg2rad 2)
         :pre-process-func pre-process-func
         :post-process-func post-process-func
         ))
  (objects (list *robot* *arrow* *wall*))
  (visualize-trajectory-result *ret* :visualize-callback-func visualize-callback-func)
  *ret*
  )
(warn "(sample-robot-reach-trajectory-iks-face)~%")
