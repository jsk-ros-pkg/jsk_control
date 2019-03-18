(load "../inverse-kinematics-wrapper.l")
(load "../discrete-kinematics-configuration-task.l")
(load "../sqp-msc-optimization.l")
(require "irteus/demo/sample-arm-model.l")
(require "irteus/demo/sample-robot-model.l")
(require "models/arrow-object.l")


(defun sample-arm-reach-ik-raw
    (&key
     (pre-process-func)
     (post-process-func)
     &allow-other-keys
     )
  (setq *robot* (instance sarmclass :init))
  (setq *robot-env*
        (instance robot-environment :init :robot *robot* :root-virtual-mode :fix))
  (setq *arrow* (instance arrow-object :init))
  (send *arrow* :newcoords
        (make-coords :pos (float-vector 400 -200 300) :rpy (list (deg2rad 60) 0 0)))
  (objects (list *robot* *arrow*))

  (setq *instant-config-task*
        (instance instant-configuration-task :init
                  :robot-env *robot-env*
                  :drive-joint-list nil
                  :only-kinematics? t
                  :kin-target-coords-list
                  (list (send *arrow* :copy-worldcoords))
                  :kin-attention-coords-list
                  (list (send *robot-env* :robot :end-coords))
                  ))

  (setq *sqp-opt*
        (instance sqp-optimization :init
                  :config-task *instant-config-task*
                  :pre-process-func pre-process-func
                  :post-process-func post-process-func
                  ))
  (send *sqp-opt* :optimize :loop-num 50)
  t)
(warn "(sample-arm-reach-ik-raw)~%")

(defun sample-arm-reach-ik-raw-msc
    (&key
     (pre-process-func)
     (post-process-func)
     &allow-other-keys
     )
  (setq *arrow* (instance arrow-object :init))
  (send *arrow* :newcoords
        (make-coords :pos (float-vector 400 -200 300) :rpy (list (deg2rad 60) 0 0)))
  (setq *robot* (instance sarmclass :init))
  (setq *contact-list*
        (list
         (instance contact-ik-arg :init
                   :name "target-coords"
                   :target-coords (send *arrow* :copy-worldcoords)
                   :translation-axis t
                   :rotation-axis nil
                   )
         ))
  (setq *robot-env*
        (instance robot-environment :init
                  :robot *robot*
                  :contact-list *contact-list*
                  :root-virtual-mode :fix
                  ))

  (setq *instant-config-task*
        (instance instant-configuration-task :init
                  :robot-env *robot-env*
                  :drive-joint-list nil
                  :only-kinematics? t
                  :kin-target-coords-list
                  (list (send (send *robot-env* :contact "target-coords") :contact-coords))
                  :kin-attention-coords-list
                  (list (send *robot-env* :robot :end-coords))
                  ))

  (setq *sqp-opt*
        (instance sqp-msc-optimization :init
                  :config-task *instant-config-task*
                  :pre-process-func pre-process-func
                  :post-process-func post-process-func
                  ))
  (objects (append (list *arrow*) (send-all (send *sqp-opt* :config-task-list) :robot-env)))
  (send *sqp-opt* :optimize :loop-num 50)
  t)
(warn "(sample-arm-reach-ik-raw-msc)~%")

(defun sample-arm-reach-ik-discrete-raw
    (&key
     (pre-process-func)
     (post-process-func)
     &allow-other-keys
     )
  (setq *robot* (instance sarmclass :init))
  (setq *robot-env*
        (instance robot-environment :init :robot *robot* :root-virtual-mode :fix))
  (setq *arrow-list* (mapcar #'(lambda (x) (instance arrow-object :init)) (make-list 3)))
  (send (elt *arrow-list* 0) :newcoords
        (make-coords :pos (float-vector 1000 200 300) :rpy (list (deg2rad 20) 0 0)))
  (send (elt *arrow-list* 1) :newcoords
        (make-coords :pos (float-vector 300 -200 0) :rpy (list (deg2rad 20) 0 0)))
  (send (elt *arrow-list* 2) :newcoords
        (make-coords :pos (float-vector 0 500 200) :rpy (list (deg2rad 20) 0 0)))
  (objects (append (list *robot*) *arrow-list*))

  (setq *instant-config-task*
        (instance discrete-kinematics-configuration-task :init
                  :robot-env *robot-env*
                  :drive-joint-list nil
                  :only-kinematics? t
                  :kin-target-coords-list
                  (list (send-all *arrow-list* :copy-worldcoords))
                  :kin-attention-coords-list
                  (list (make-list (length *arrow-list*) :initial-element (send *robot-env* :robot :end-coords)))
                  ))

  (setq *sqp-opt*
        (instance sqp-optimization :init
                  :config-task *instant-config-task*
                  :pre-process-func pre-process-func
                  :post-process-func post-process-func
                  ))
  (send *sqp-opt* :optimize :loop-num 50)
  t)
(warn "(sample-arm-reach-ik-discrete-raw)~%")

(defun sample-arm-reach-ik
    (&key
     (target-coords
      (make-coords :pos (float-vector 400 -200 300) :rpy (list (deg2rad 60) 0 0)))
     (irteus-ik? nil)
     (pre-process-func)
     (post-process-func)
     &allow-other-keys
     )
  (setq *robot* (instance sarmclass :init))
  (setq *arrow* (instance arrow-object :init))
  (send *arrow* :newcoords target-coords)
  (objects (list *robot* *arrow*))

  (send *robot*
        (if irteus-ik? :inverse-kinematics :inverse-kinematics-optmotiongen)
        (send *arrow* :copy-worldcoords)
        :link-list (send *robot* :link-list (send (send *robot* :end-coords) :parent))
        :move-target (send *robot* :end-coords)
        :translation-axis t
        :rotation-axis t
        :debug-view t
        :pre-process-func pre-process-func
        :post-process-func post-process-func
        )
  )
(warn "(sample-arm-reach-ik)~%")

(defun sample-arm-reach-ik-obstacle
    (&key
     (irteus-ik? nil)
     (pre-process-func)
     (post-process-func)
     &allow-other-keys
     )
  (setq *robot* (instance sarmclass :init))
  (setq *arrow* (instance arrow-object :init))
  (send *arrow* :newcoords
        (make-coords :pos (float-vector 400 -200 300) :rpy (list (deg2rad 60) 0 0)))
  (setq *obstacle* (make-cube 250 250 250))
  (send *obstacle* :newcoords
        (make-coords :pos (float-vector 200 -200 600)))
  (send *obstacle* :set-color (float-vector 1 0 0) 0.5)
  (objects (list *robot* *arrow* *obstacle*))

  (send *robot*
        (if irteus-ik? :inverse-kinematics :inverse-kinematics-optmotiongen)
        (send *arrow* :copy-worldcoords)
        :link-list (send *robot* :link-list (send (send *robot* :end-coords) :parent))
        :move-target (send *robot* :end-coords)
        :translation-axis t
        :rotation-axis nil
        :debug-view t
        :obstacles (list *obstacle*)
        :pre-process-func pre-process-func
        :post-process-func post-process-func
        )
  )
(warn "(sample-arm-reach-ik-obstacle)~%")

(defun sample-arm-reach-ik-posture
    (&key
     (pre-process-func)
     (post-process-func)
     &allow-other-keys
     )
  (setq *robot* (instance sarmclass :init))
  (setq *arrow* (instance arrow-object :init))
  (send *arrow* :newcoords
        (make-coords :pos (float-vector 400 -200 300) :rpy (list (deg2rad 60) 0 0)))
  (objects (list *robot* *arrow*))

  (send *robot*
        :inverse-kinematics-optmotiongen
        (send *arrow* :copy-worldcoords)
        :link-list (send *robot* :link-list (send (send *robot* :end-coords) :parent))
        :move-target (send *robot* :end-coords)
        :translation-axis t
        :rotation-axis nil
        :debug-view t
        :posture-joint-list (list (send *robot* :joint :joint1))
        :posture-joint-angle-list (list -10.0)
        :stop 50
        :min-loop 30
        :pre-process-func pre-process-func
        :post-process-func post-process-func
        )
  )
(warn "(sample-arm-reach-ik-posture)~%")

(defun sample-arm-reach-ik-with-root-virtual-joint
    (&key
     (pre-process-func)
     (post-process-func)
     &allow-other-keys
     )
  (setq *robot* (instance sarmclass :init))
  (setq *arrow* (instance arrow-object :init))
  (send *arrow* :newcoords
        (make-coords :pos (float-vector 1400 -200 300) :rpy (list (deg2rad 60) 0 0)))
  (objects (list *robot* *arrow*))

  (send *robot*
        :inverse-kinematics-optmotiongen
        (send *arrow* :copy-worldcoords)
        :move-target (send *robot* :end-coords)
        :translation-axis t
        :rotation-axis t
        :debug-view t
        :root-virtual-mode :planar
        :pre-process-func pre-process-func
        :post-process-func post-process-func
        )
  )
(warn "(sample-arm-reach-ik-with-root-virtual-joint)~%")

(defun sample-arm-reach-ik-face
    (&key
     (pre-process-func)
     (post-process-func)
     &allow-other-keys
     )
  (setq *robot* (instance sarmclass :init))
  (setq *board* (make-cube 100 200 5))
  (send *board* :newcoords
        (make-coords :pos (float-vector 400 -200 300)
                     :rpy (list (deg2rad 30) (deg2rad -30) (deg2rad -30))))
  (send *board* :set-color (float-vector 1.0 0.6 0.6))
  (objects (list *robot* *board*))

  (setq *face* (elt (send *board* :faces) 1))
  (send *face* :put :margin 20) ;; [mm]
  (send *face* :put :normal-move-target-axis :z)
  (send *face* :put :normal-tolerance 10.0) ;; [deg]
  (send *robot* :inverse-kinematics-optmotiongen
        *face*
        :move-target (send *robot* :end-coords)
        :debug-view t
        :pre-process-func pre-process-func
        :post-process-func post-process-func
        )
  )
(warn "(sample-arm-reach-ik-face)~%")

(defun sample-arm-reach-ik-line
    (&key
     (pre-process-func)
     (post-process-func)
     &allow-other-keys
     )
  (setq *robot* (instance sarmclass :init))
  (setq *pipe* (make-cylinder 3 400))
  (send *pipe* :newcoords
        (make-coords :pos (float-vector 400 -300 400)
                     :rpy (list (deg2rad -30) (deg2rad 30) (deg2rad 30))))
  (send *pipe* :set-color (float-vector 1.0 0.6 0.6))
  (objects (list *robot* *pipe*))

  (setq *edge* (elt (send *pipe* :edges) 15))
  (send *edge* :put :margin 20) ;; [mm]
  (send *edge* :put :direction-move-target-axis :-y)
  (send *edge* :put :normal-move-target-axis :z)
  (send *edge* :put :normal-tolerance 30.0) ;; [deg]
  (send *robot* :inverse-kinematics-optmotiongen
        *edge*
        :move-target (send *robot* :end-coords)
        :debug-view t
        :pre-process-func pre-process-func
        :post-process-func post-process-func
        )
  )
(warn "(sample-arm-reach-ik-line)~%")

(defun sample-robot-reach-ik-raw
    (&key
     (set-posture-joint? nil)
     (look-at? nil)
     (pre-process-func)
     (post-process-func)
     &allow-other-keys
     )
  (setq *robot* (instance sample-robot :init))
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  (setq *arrow* (instance arrow-object :init))
  (send *arrow* :newcoords
        (make-coords :pos (float-vector 300 -300 700) :rpy (list 0 0 0)))
  (objects (list *robot* *arrow*))

  (setq *look-at-contact*
        (instance look-at-contact :init
                  :target-pos (send *arrow* :worldpos)
                  :camera-axis :z
                  :angle-of-view 10
                  ))
  (setq *robot-env*
        (instance robot-environment :init
                  :robot *robot*
                  :contact-list (if look-at? (list *look-at-contact*))
                  :root-virtual-mode :fix
                  ))

  (setq *instant-config-task*
        (instance instant-configuration-task :init
                  :robot-env *robot-env*
                  :drive-joint-list nil
                  :only-kinematics? t
                  :posture-joint-list (if set-posture-joint? (send *robot* :joint-list))
                  :posture-joint-angle-list (if set-posture-joint? (send-all (send *robot* :joint-list) :joint-angle))
                  :kin-target-coords-list
                  (remove
                   nil
                   (list (send *arrow* :copy-worldcoords)
                         (if look-at? (send *look-at-contact* :contact-coords))))
                  :kin-attention-coords-list
                  (remove
                   nil
                   (list (send *robot* :rarm :end-coords)
                         (if look-at? (send *robot* :head :end-coords))))
                  ))

  (setq *sqp-opt*
        (instance sqp-optimization :init
                  :config-task *instant-config-task*
                  :pre-process-func pre-process-func
                  :post-process-func post-process-func
                  ))
  (send *sqp-opt* :optimize :loop-num 50)
  t)
(warn "(sample-robot-reach-ik-raw)~%")

(defun sample-robot-reach-ik
    (&key
     (irteus-ik? nil)
     (pre-process-func)
     (post-process-func)
     &allow-other-keys
     )
  (setq *robot* (instance sample-robot :init))
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  (setq *arrow* (instance arrow-object :init))
  (send *arrow* :newcoords
        (make-coords :pos (float-vector 300 -300 700) :rpy (list 0 0 0)))
  (objects (list *robot* *arrow*))

  (send *robot*
        (if irteus-ik? :inverse-kinematics :inverse-kinematics-optmotiongen)
        (send *arrow* :copy-worldcoords)
        :link-list (send *robot* :link-list (send (send *robot* :rarm :end-coords) :parent))
        :move-target (send *robot* :rarm :end-coords)
        :debug-view t
        :pre-process-func pre-process-func
        :post-process-func post-process-func
        )
  )
(warn "(sample-robot-reach-ik)~%")

(defun sample-robot-reach-ik-limb
    (&key
     (irteus-ik? nil)
     (pre-process-func)
     (post-process-func)
     &allow-other-keys
     )
  (setq *robot* (instance sample-robot :init))
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  (setq *arrow* (instance arrow-object :init))
  (send *arrow* :newcoords
        (make-coords :pos (float-vector 300 -300 700) :rpy (list 0 0 0)))
  (objects (list *robot* *arrow*))

  (send *robot* :rarm
        (if irteus-ik? :inverse-kinematics :inverse-kinematics-optmotiongen)
        (send *arrow* :copy-worldcoords)
        :debug-view t
        :pre-process-func pre-process-func
        :post-process-func post-process-func
        )
  )
(warn "(sample-robot-reach-ik-limb)~%")

(defun sample-robot-reach-ik-dual-arm
    (&key
     (irteus-ik? nil)
     (pre-process-func)
     (post-process-func)
     &allow-other-keys
     )
  (setq *robot* (instance sample-robot :init))
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  (setq *arrow-right* (instance arrow-object :init))
  (send *arrow-right* :newcoords
        (make-coords :pos (float-vector 300 -300 700) :rpy (list 0 0 0)))
  (setq *arrow-left* (instance arrow-object :init))
  (send *arrow-left* :newcoords
        (make-coords :pos (float-vector 400 200 600) :rpy (list 0 0 0)))
  (objects (list *robot* *arrow-right* *arrow-left*))

  (send *robot*
        (if irteus-ik? :inverse-kinematics :inverse-kinematics-optmotiongen)
        (list (send *arrow-right* :copy-worldcoords) (send *arrow-left* :copy-worldcoords))
        :link-list
        (list (send *robot* :link-list (send (send *robot* :rarm :end-coords) :parent) (send *robot* :rarm :root-link))
              (send *robot* :link-list (send (send *robot* :larm :end-coords) :parent) (send *robot* :larm :root-link)))
        :move-target
        (list (send *robot* :rarm :end-coords) (send *robot* :larm :end-coords))
        :debug-view t
        :pre-process-func pre-process-func
        :post-process-func post-process-func
        )
  )
(warn "(sample-robot-reach-ik-dual-arm)~%")

(defun sample-robot-reach-ik-dual-arm-with-torso
    (&key
     (irteus-ik? nil)
     (pre-process-func)
     (post-process-func)
     &allow-other-keys
     )
  (setq *robot* (instance sample-robot :init))
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  (setq *arrow-right* (instance arrow-object :init))
  (send *arrow-right* :newcoords
        (make-coords :pos (float-vector 300 -300 700) :rpy (list 0 0 0)))
  (setq *arrow-left* (instance arrow-object :init))
  (send *arrow-left* :newcoords
        (make-coords :pos (float-vector 400 200 600) :rpy (list 0 0 0)))
  (objects (list *robot* *arrow-right* *arrow-left*))

  (send *robot*
        (if irteus-ik? :inverse-kinematics :inverse-kinematics-optmotiongen)
        (list (send *arrow-right* :copy-worldcoords) (send *arrow-left* :copy-worldcoords))
        :link-list
        (list (send *robot* :link-list (send (send *robot* :rarm :end-coords) :parent))
              (send *robot* :link-list (send (send *robot* :larm :end-coords) :parent)))
        :move-target
        (list (send *robot* :rarm :end-coords) (send *robot* :larm :end-coords))
        :debug-view t
        :pre-process-func pre-process-func
        :post-process-func post-process-func
        )
  )
(warn "(sample-robot-reach-ik-dual-arm-with-torso)~%")

(defun sample-robot-reach-ik-fullbody
    (&key
     (irteus-ik? nil)
     (pre-process-func)
     (post-process-func)
     &allow-other-keys
     )
  (setq *robot* (instance sample-robot :init))
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  (setq *arrow-right* (instance arrow-object :init))
  (send *arrow-right* :newcoords
        (make-coords :pos (float-vector 300 -300 700) :rpy (list 0 0 0)))
  (setq *arrow-left* (instance arrow-object :init))
  (send *arrow-left* :newcoords
        (make-coords :pos (float-vector 400 200 600) :rpy (list 0 0 0)))
  (objects (list *robot* *arrow-right* *arrow-left*))
  (setq *target-centroid-pos* (float-vector 0 -100 0))

  (send *robot*
        (if irteus-ik? :fullbody-inverse-kinematics :inverse-kinematics-optmotiongen)
        (remove
         nil
         (list (send (send *robot* :rleg :end-coords) :copy-worldcoords)
               (send (send *robot* :lleg :end-coords) :copy-worldcoords)
               (send *arrow-right* :copy-worldcoords)
               (send *arrow-left* :copy-worldcoords)
               (if (not irteus-ik?) (make-coords :pos *target-centroid-pos*))
               ))
        :link-list
        (list (send *robot* :link-list (send (send *robot* :rleg :end-coords) :parent))
              (send *robot* :link-list (send (send *robot* :lleg :end-coords) :parent))
              (send *robot* :link-list (send (send *robot* :rarm :end-coords) :parent))
              (send *robot* :link-list (send (send *robot* :larm :end-coords) :parent))
              )
        :move-target
        (remove
         nil
         (list (send *robot* :rleg :end-coords)
               (send *robot* :lleg :end-coords)
               (send *robot* :rarm :end-coords)
               (send *robot* :larm :end-coords)
               (if (not irteus-ik?) :cog)
               ))
        :translation-axis
        (remove nil (list t t t t (if (not irteus-ik?) :z)))
        :debug-view t
        :root-virtual-mode :6dof ;; only for optmotiongen ik
        :target-centroid-pos *target-centroid-pos* ;; only for irteus ik
        :pre-process-func pre-process-func
        :post-process-func post-process-func
        )
  )
(warn "(sample-robot-reach-ik-fullbody)~%")

(defun sample-arm-reach-trajectory-ik-raw
    (&key
     (pre-process-func)
     (post-process-func)
     (visualize-callback-func)
     &allow-other-keys
     )
  (setq *robot* (instance sarmclass :init))
  (setq *robot-env*
        (instance robot-environment :init :robot *robot* :root-virtual-mode :fix))
  (send *robot-env* :invariant-joint-list
        (send *robot-env* :root-virtual-joint-list))
  (setq *arrow-list* (mapcar #'(lambda (x) (instance arrow-object :init)) (make-list 10)))
  (send (elt *arrow-list* 0) :newcoords (make-coords :pos (float-vector 100 -400 300) :rpy (list (deg2rad 60) 0 0)))
  (dotimes (i (1- (length *arrow-list*)))
    (send (elt *arrow-list* (1+ i)) :newcoords
          (send (send (elt *arrow-list* i) :copy-worldcoords) :translate (float-vector 100 0 0))))

  (setq *robot-env-list* nil)
  (setq *instant-config-task-list* nil)

  (dolist (arrow *arrow-list*)
    (push (copy-object *robot-env*) *robot-env-list*)
    (push
     (instance instant-configuration-task :init
               :robot-env (car *robot-env-list*)
               :drive-joint-list nil
               :only-kinematics? t
               :kin-target-coords-list
               (list (send arrow :copy-worldcoords))
               :kin-attention-coords-list
               (list (send (car *robot-env-list*) :robot :end-coords))
               )
     *instant-config-task-list*
     )
    )

  (objects (append *robot-env-list* *arrow-list*))

  (setq *trajectory-config-task*
        (instance trajectory-configuration-task :init
                  :instant-config-task-list (reverse *instant-config-task-list*)
                  :norm-regular-scale-max 1e-3
                  :adjacent-regular-scale 1e-5
                  ))

  (setq *sqp-opt*
        (instance sqp-optimization :init
                  :config-task *trajectory-config-task*
                  :pre-process-func pre-process-func
                  :post-process-func post-process-func
                  ))
  (send *sqp-opt* :optimize :loop-num 50)

  (objects (append (list *robot-env*) *arrow-list*))
  (send *trajectory-config-task* :play-animation
        :robot-env *robot-env*
        :loop? nil
        :visualize-callback-func visualize-callback-func
        )
  t)
(warn "(sample-arm-reach-trajectory-ik-raw)~%")

(defun sample-arm-reach-trajectory-ik
    (&key
     (pre-process-func)
     (post-process-func)
     (visualize-callback-func)
     &allow-other-keys
     )
  (setq *robot* (instance sarmclass :init))
  (setq *arrow-list* (mapcar #'(lambda (x) (instance arrow-object :init)) (make-list 10)))
  (send (elt *arrow-list* 0) :newcoords (make-coords :pos (float-vector 100 -400 300) :rpy (list (deg2rad 60) 0 0)))
  (dotimes (i (1- (length *arrow-list*)))
    (send (elt *arrow-list* (1+ i)) :newcoords
          (send (send (elt *arrow-list* i) :copy-worldcoords) :translate (float-vector 100 0 0))))
  (setq
   *ret*
   (send *robot*
         :inverse-kinematics-trajectory-optmotiongen
         (send-all *arrow-list* :copy-worldcoords)
         :move-target-list (send *robot* :end-coords)
         :debug-view (list :additional-draw-objects *arrow-list*)
         :rotation-axis-list (make-list (length *arrow-list*) :initial-element nil)
         :stop 50
         :min-loop 30
         :adjacent-regular-scale 1e-5
         :pre-process-func pre-process-func
         :post-process-func post-process-func
         ))
  (objects (append (list *robot*) *arrow-list*))
  (visualize-trajectory-result *ret* :visualize-callback-func visualize-callback-func)
  *ret*
  )
(warn "(sample-arm-reach-trajectory-ik)~%")

(defun sample-arm-reach-trajectory-ik-with-root-virtual-joint
    (&key
     (target-coords
      (make-coords :pos (float-vector 400 -200 300) :rpy (list (deg2rad 60) 0 0)))
     (rotation-axis-list (list t t t))
     (root-virtual-joint-min-angle-list)
     (root-virtual-joint-max-angle-list)
     (pre-process-func)
     (post-process-func)
     &allow-other-keys
     )
  (setq *robot* (instance sarmclass :init))
  (setq *arrow-list* (mapcar #'(lambda (x) (instance arrow-object :init)) (make-list 3)))
  (send (elt *arrow-list* 0) :newcoords target-coords)
  (send (elt *arrow-list* 1) :newcoords
        (make-coords :pos (float-vector 100 0 400) :rpy (list 0 (deg2rad 30) 0)))
  (send (elt *arrow-list* 2) :newcoords
        (make-coords :pos (float-vector -100 300 200) :rpy (list 0 0 0)))

  (send *robot*
        :inverse-kinematics-trajectory-optmotiongen
        (send-all *arrow-list* :copy-worldcoords)
        :move-target-list (send *robot* :end-coords)
        :rotation-axis-list rotation-axis-list
        :debug-view (list :additional-draw-objects *arrow-list*)
        :root-virtual-mode :planar
        :root-virtual-joint-invariant? t
        :root-virtual-joint-min-angle-list root-virtual-joint-min-angle-list
        :root-virtual-joint-max-angle-list root-virtual-joint-max-angle-list
        :pre-process-func pre-process-func
        :post-process-func post-process-func
        )
  )
(warn "(sample-arm-reach-trajectory-ik-with-root-virtual-joint)~%")

(defun sample-arm-reach-trajectory-ik-with-root-virtual-joint-obstacle
    (&key
     (pre-process-func)
     (post-process-func)
     &allow-other-keys
     )
  (setq *robot* (instance sarmclass :init))
  (setq *arrow-list* (mapcar #'(lambda (x) (instance arrow-object :init)) (make-list 3)))
  (send (elt *arrow-list* 0) :newcoords
        (make-coords :pos (float-vector 400 -200 300) :rpy (list (deg2rad 60) 0 0)))
  (send (elt *arrow-list* 1) :newcoords
        (make-coords :pos (float-vector 100 0 400) :rpy (list 0 (deg2rad 30) 0)))
  (send (elt *arrow-list* 2) :newcoords
        (make-coords :pos (float-vector -100 300 200) :rpy (list 0 0 0)))
  (setq *obstacle* (make-cube 250 1000 250))
  (send *obstacle* :newcoords
        (make-coords :pos (float-vector 400 400 600)))
  (send *obstacle* :set-color (float-vector 1 0 0) 0.5)

  (send *robot*
        :inverse-kinematics-trajectory-optmotiongen
        (send-all *arrow-list* :copy-worldcoords)
        :move-target-list (send *robot* :end-coords)
        :translation-axis-list
        (list t t t)
        :rotation-axis-list
        (list :x :x :x)
        :debug-view (list :additional-draw-objects (append *arrow-list* (list *obstacle*)))
        :obstacles (list *obstacle*)
        :root-virtual-mode :planar
        :root-virtual-joint-invariant? t
        :pre-process-func pre-process-func
        :post-process-func post-process-func
        )
  )
(warn "(sample-arm-reach-trajectory-ik-with-root-virtual-joint-obstacle)~%")

(defun sample-robot-reach-trajectory-ik-dual-arm-with-torso
    (&key
     (pre-process-func)
     (post-process-func)
     (visualize-callback-func)
     &allow-other-keys
     )
  (setq *robot* (instance sample-robot :init))
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  (setq *arrow-right-list* (mapcar #'(lambda (x) (instance arrow-object :init)) (make-list 3)))
  (send (elt *arrow-right-list* 0) :newcoords
        (make-coords :pos (float-vector 500 -300 500) :rpy (list 0 0 0)))
  (send (elt *arrow-right-list* 1) :newcoords
        (make-coords :pos (float-vector 400 -300 600) :rpy (list 0 0 0)))
  (send (elt *arrow-right-list* 2) :newcoords
        (make-coords :pos (float-vector 300 -300 700) :rpy (list 0 0 0)))
  (setq *arrow-left-list* (mapcar #'(lambda (x) (instance arrow-object :init)) (make-list 3)))
  (send-all *arrow-left-list* :newcoords
            (make-coords :pos (float-vector 400 200 600) :rpy (list 0 0 0)))

  (setq
   *ret*
   (send *robot*
         :inverse-kinematics-trajectory-optmotiongen
         (mapcar
          #'(lambda (rc lc) (list rc lc))
          (send-all *arrow-right-list* :copy-worldcoords)
          (send-all *arrow-left-list* :copy-worldcoords)
          )
         :move-target-list (list (send *robot* :rarm :end-coords) (send *robot* :larm :end-coords))
         :debug-view (list :additional-draw-objects (append *arrow-right-list* *arrow-left-list*))
         :root-virtual-mode :planar
         :root-virtual-joint-invariant? t
         :pre-process-func pre-process-func
         :post-process-func post-process-func
         ))
  (objects (append (list *robot*) *arrow-right-list* *arrow-left-list*))
  (visualize-trajectory-result *ret* :visualize-callback-func visualize-callback-func)
  *ret*
  )
(warn "(sample-robot-reach-trajectory-ik-dual-arm-with-torso)~%")

(defun sample-robot-reach-trajectory-ik-dual-arm-with-torso-no-mid-constraint
    (&key
     (pre-process-func)
     (post-process-func)
     (visualize-callback-func)
     &allow-other-keys
     )
  (setq *robot* (instance sample-robot :init))
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  (setq *arrow-right-start* (instance arrow-object :init))
  (send *arrow-right-start* :newcoords
        (make-coords :pos (float-vector 500 -300 500) :rpy (list 0 0 0)))
  (setq *arrow-right-finish* (instance arrow-object :init))
  (send *arrow-right-finish* :newcoords
        (make-coords :pos (float-vector 300 -300 700) :rpy (list 0 0 0)))
  (setq *arrow-left* (instance arrow-object :init))
  (send *arrow-left* :newcoords
        (make-coords :pos (float-vector 400 200 600) :rpy (list 0 0 0)))

  (setq
   *ret*
   (send *robot*
         :inverse-kinematics-trajectory-optmotiongen
         (list (list (send *arrow-right-start* :copy-worldcoords) (send *arrow-left* :copy-worldcoords))
               (list (send *arrow-left* :copy-worldcoords))
               (list (send *arrow-right-finish* :copy-worldcoords) (send *arrow-left* :copy-worldcoords)))
         :move-target-list
         (list (list (send *robot* :rarm :end-coords) (send *robot* :larm :end-coords))
               (list (send *robot* :larm :end-coords))
               (list (send *robot* :rarm :end-coords) (send *robot* :larm :end-coords)))
         :debug-view (list :additional-draw-objects (list *arrow-right-start* *arrow-right-finish* *arrow-left*))
         :root-virtual-mode :planar
         :root-virtual-joint-invariant? t
         :adjacent-regular-scale 1e-5
         :stop 50
         :min-loop 10
         :pre-process-func pre-process-func
         :post-process-func post-process-func
         ))
  (objects (list *robot* *arrow-right-start* *arrow-right-finish* *arrow-left*))
  (visualize-trajectory-result *ret* :visualize-callback-func visualize-callback-func)
  *ret*
  )
(warn "(sample-robot-reach-trajectory-ik-dual-arm-with-torso-no-mid-constraint)~%")

(defun sample-robot-reach-trajectory-ik-fullbody
    (&key
     (pre-process-func)
     (post-process-func)
     (visualize-callback-func)
     &allow-other-keys
     )
  (setq *robot* (instance sample-robot :init))
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  (setq *arrow-right-list* (mapcar #'(lambda (x) (instance arrow-object :init)) (make-list 3)))
  (send (elt *arrow-right-list* 0) :newcoords
        (make-coords :pos (float-vector 500 -300 500) :rpy (list 0 0 0)))
  (send (elt *arrow-right-list* 1) :newcoords
        (make-coords :pos (float-vector 400 -300 600) :rpy (list 0 0 0)))
  (send (elt *arrow-right-list* 2) :newcoords
        (make-coords :pos (float-vector 300 -300 700) :rpy (list 0 0 0)))
  (setq *arrow-left-list* (mapcar #'(lambda (x) (instance arrow-object :init)) (make-list 3)))
  (send-all *arrow-left-list* :newcoords
            (make-coords :pos (float-vector 400 200 600) :rpy (list 0 0 0)))
  (setq *target-centroid-pos* (float-vector 0 -100 0))

  (setq
   *ret*
   (send *robot*
         :inverse-kinematics-trajectory-optmotiongen
         (mapcar
          #'(lambda (rc lc)
              (list (send (send *robot* :rleg :end-coords) :copy-worldcoords)
                    (send (send *robot* :lleg :end-coords) :copy-worldcoords)
                    rc
                    lc
                    (make-coords :pos *target-centroid-pos*)
                    ))
          (send-all *arrow-right-list* :copy-worldcoords)
          (send-all *arrow-left-list* :copy-worldcoords)
          )
         :move-target-list
         (list (send *robot* :rleg :end-coords)
               (send *robot* :lleg :end-coords)
               (send *robot* :rarm :end-coords)
               (send *robot* :larm :end-coords)
               :cog
               )
         :translation-axis-list
         (list (list t t t t :z) (list t t t t :z) (list t t t t :z))
         :debug-view (list :additional-draw-objects (append *arrow-right-list* *arrow-left-list*))
         :root-virtual-mode :6dof
         :root-virtual-joint-invariant? nil
         :pre-process-func pre-process-func
         :post-process-func post-process-func
         ))
  (objects (append (list *robot*) *arrow-right-list* *arrow-left-list*))
  (visualize-trajectory-result *ret* :visualize-callback-func visualize-callback-func)
  *ret*
  )
(warn "(sample-robot-reach-trajectory-ik-fullbody)~%")
