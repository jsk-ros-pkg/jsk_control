(load "./inverse-kinematics-wrapper.l")


;;;;;;;;;;;;;;;;;;;;;;;;;
;; cascaded-link (add methods)
;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod cascaded-link
  (:inverse-kinematics-statics-optmotiongen
   (target-coords
    &key
    ;; same arguments with irteus inverse-kinematics
    (stop 50)
    (link-list)
    (move-target)
    (debug-view)
    (revert-if-fail t)
    (transform-coords
     target-coords)
    (translation-axis
     (cond
      ((atom move-target) t)
      (t (make-list (length move-target) :initial-element t))))
    (rotation-axis
     (cond
      ((atom move-target) t)
      (t (make-list (length move-target) :initial-element t))))
    (thre
     (cond
      ((atom move-target) 1)
      (t (make-list (length move-target) :initial-element 1))))
    (rthre
     (cond
      ((atom move-target) (deg2rad 1))
      (t (make-list (length move-target) :initial-element (deg2rad 1)))))
    (collision-avoidance-link-pair :nil)
    (collision-distance-limit 10.0) ;; [mm]
    (obstacles)
    (min-loop)
    ;; new arguments
    (contact-target-coords)
    (contact-move-target)
    (contact-constraint)
    (root-virtual-mode :fix)
    (root-virtual-joint-min-angle-list)
    (root-virtual-joint-max-angle-list)
    (joint-angle-margin 0.0)
    (posture-joint-list)
    (posture-joint-angle-list)
    (optimize-torque?)
    (eom-trans-thre 1e-1)
    (eom-rot-thre 1e-1)
    (torque-thre 1e-1)
    (target-posture-scale 1e-3)
    (norm-regular-scale-max 1e-3)
    (norm-regular-scale-offset 1e-6)
    (torque-regular-scale 1e-5)
    (pre-process-func)
    (post-process-func)
    &allow-other-keys
    )
   "
Solve inverse kinematics problem with sqp optimization.
   ;; target-coords, move-target, rotation-axis, translation-axis
   ;; -> both list and atom OK.
    target-coords : The coordinate of the target that returns coordinates. Use a list of targets to solve the IK relative to multiple end links simultaneously. Function is not available to target-coords.
    link-list : List of links to control. When the target-coords is list, this should be a list of lists.
    move-target : Specify end-effector coordinate. When the target-coords is list, this should be list too.
    stop : Maximum number for IK iteration. Default is 50.
    debug-view : Set t to show debug message and visualization. Use :no-message to just show the irtview image. Default is nil.
    revert-if-fail : Set nil to keep the angle posture of IK solve iteration. Default is t, which return to original position when IK fails.
    translation-axis : :x :y :z for constraint along the x, y, z axis. :xy :yz :zx for plane. Default is t.
    rotation-axis : Use nil for position only IK. :x, :y, :z for the constraint around axis with plus direction. When the target-coords is list, this should be list too. Default is t.
    thre : Threshold for position error to terminate IK iteration. Default is 1 [mm].
    rthre : Threshold for rotation error to terminate IK iteration. Default is 0.017453 [rad] (1 deg).
    contact-target-coords : target coords for contact
    contact-move-target : end-effector coords for contact
    contact-constraint : contact constraint
   "
   (if (atom target-coords) (setq target-coords (list target-coords)))
   (if (and link-list (atom (car link-list))) (setq link-list (list link-list)))
   (if (atom move-target) (setq move-target (list move-target)))
   (if (atom transform-coords) (setq transform-coords (list transform-coords)))
   (if (atom translation-axis) (setq translation-axis (list translation-axis)))
   (if (atom rotation-axis) (setq rotation-axis (list rotation-axis)))
   (if (atom thre) (setq thre (list thre)))
   (if (atom rthre) (setq rthre (list rthre)))
   (if (and debug-view (atom debug-view)) (setq debug-view (list debug-view)))
   (let* ((av-orig
           (send self :angle-vector))
          (rc-orig
           (send self :copy-worldcoords))
          (union-link-list
           (send self :calc-union-link-list link-list))
          (contact-list
           (mapcar
            #'(lambda
                (tc trans-axis rot-axis trans-coords)
                (cond ((derivedp tc polygon)
                       (generate-contact-ik-arg-from-rect-face
                        :rect-face tc)
                       )
                      ((derivedp tc edge)
                       (generate-contact-ik-arg-from-line-segment
                        :line-seg tc)
                       )
                      (t
                       (if (not (and (equal trans-axis t) (equal rot-axis t)))
                           (instance contact-ik-arg :init
                                     :target-coords tc
                                     :translation-axis trans-axis
                                     :rotation-axis rot-axis
                                     :transform-coords trans-coords
                                     ))
                       ))
                )
            target-coords
            translation-axis
            rotation-axis
            transform-coords
            ))
          (robot-env
           (instance robot-environment :init
                     :robot self
                     :contact-list (remove nil contact-list)
                     :root-virtual-mode root-virtual-mode
                     :root-virtual-joint-min-angle-list root-virtual-joint-min-angle-list
                     :root-virtual-joint-max-angle-list root-virtual-joint-max-angle-list
                     ))
          (variant-joint-list
           (cond (union-link-list
                  (append (send-all union-link-list :joint) (send robot-env :root-virtual-joint-list))
                  )
                 (t
                  (send robot-env :variant-joint-list)
                  ))
           )
          (collision-pair-list
           (cond ((not (equal collision-avoidance-link-pair :nil))
                  collision-avoidance-link-pair
                  )
                 (t
                  (send self :collision-avoidance-link-pair-from-link-list
                        links
                        :obstacles obstacles)
                  ))
           )
          (instant-config-task
           (instance instant-configuration-task :init
                     :robot-env robot-env
                     :variant-joint-list variant-joint-list
                     :drive-joint-list (if optimize-torque? (send robot-env :drive-joint-list))
                     :posture-joint-list posture-joint-list
                     :posture-joint-angle-list posture-joint-angle-list
                     :target-posture-scale target-posture-scale
                     :kin-target-coords-list
                     (mapcar
                      #'(lambda (c tc) (if c (send c :contact-coords) tc))
                      contact-list target-coords)
                     :kin-attention-coords-list move-target
                     :contact-attention-coords-list contact-move-target
                     :contact-target-coords-list
                     (mapcar
                      #'(lambda (tc)
                          (cond ((coordinates-p tc)
                                 tc
                                 )
                                (t
                                 (send (elt contact-list (position tc target-coords :test #'equal)) :contact-coords)
                                 )))
                      contact-target-coords)
                     :contact-constraint-list contact-constraint
                     :collision-pair-list collision-pair-list
                     :collision-distance-margin (* 1e-3 collision-distance-limit) ;; [m]
                     :variant-joint-angle-margin joint-angle-margin
                     :invariant-joint-angle-margin joint-angle-margin
                     :norm-regular-scale-max norm-regular-scale-max
                     :norm-regular-scale-offset norm-regular-scale-offset
                     :torque-regular-scale torque-regular-scale
                     ))
          (sqp-opt
           (instance sqp-optimization :init
                     :config-task instant-config-task
                     :convergence-check-func
                     #'(lambda (sqp)
                         (let* ((config-task
                                 (send sqp :config-task))
                                (kin-task-value
                                 (send config-task :kinematics-task-value :update? nil))
                                (pos-diff-list
                                 (divide-vector kin-task-value :segment-length 3 :interval-length 3))
                                (rot-diff-list
                                 (divide-vector (subseq kin-task-value 3) :segment-length 3 :interval-length 3))
                                (thre
                                 (send config-task :get :thre))
                                (rthre
                                 (send config-task :get :rthre))
                                (eom-trans-diff
                                 (send config-task :eom-trans-task-value :update? nil))
                                (eom-rot-diff
                                 (send config-task :eom-rot-task-value :update? nil))
                                (torque-diff
                                 (send config-task :torque-task-value :update? nil))
                                (eom-trans-thre
                                 (send config-task :get :eom-trans-thre))
                                (eom-rot-thre
                                 (send config-task :get :eom-rot-thre))
                                (torque-thre
                                 (send config-task :get :torque-thre))
                                )
                           (and (every #'(lambda (pos-diff pos-th) (< (* 1e3 (norm pos-diff)) pos-th)) pos-diff-list thre) ;; [mm]
                                (every #'(lambda (rot-diff rot-th) (< (rad2deg (norm rot-diff)) rot-th)) rot-diff-list rthre) ;; [deg]
                                (< (norm eom-trans-diff) eom-trans-thre) ;; [N]
                                (< (norm eom-rot-diff) eom-rot-thre) ;; [Nm]
                                (< (norm torque-diff) torque-thre) ;; [Nm] [N]
                                )
                           ))
                     :failure-callback-func
                     #'(lambda (sqp)
                         (let* ((config-task
                                 (send sqp :config-task))
                                (kin-task-value
                                 (send config-task :kinematics-task-value :update? nil))
                                (pos-diff-list
                                 (divide-vector kin-task-value :segment-length 3 :interval-length 3))
                                (rot-diff-list
                                 (divide-vector (subseq kin-task-value 3) :segment-length 3 :interval-length 3))
                                (thre
                                 (send config-task :get :thre))
                                (rthre
                                 (send config-task :get :rthre))
                                (eom-trans-diff
                                 (send config-task :eom-trans-task-value :update? nil))
                                (eom-rot-diff
                                 (send config-task :eom-rot-task-value :update? nil))
                                (torque-diff
                                 (send config-task :torque-task-value :update? nil))
                                (eom-trans-thre
                                 (send config-task :get :eom-trans-thre))
                                (eom-rot-thre
                                 (send config-task :get :eom-rot-thre))
                                (torque-thre
                                 (send config-task :get :torque-thre))
                                )
                           (warning-message 3 "inverse-kinematics-optmotiongen failed.~%")
                           (map
                            nil
                            #'(lambda (i pos-diff rot-diff pos-th rot-th)
                                (warning-message
                                 (if (>= (* 1e3 (norm pos-diff)) pos-th) 1 3)
                                 "pos[~1d]  ~5,3f / ~5,3f  ~a~%" i (* 1e3 (norm pos-diff)) pos-th pos-diff) ;; [mm]
                                (warning-message
                                 (if (>= (rad2deg (norm rot-diff)) rot-th) 1 3)
                                 "rot[~1d]  ~5,3f / ~5,3f  ~a~%" i (rad2deg (norm rot-diff)) rot-th rot-diff) ;; [deg]
                                )
                            (range (length pos-diff-list))
                            pos-diff-list
                            rot-diff-list
                            thre
                            rthre
                            )
                           (warning-message
                            (if (>= (norm eom-trans-diff) eom-trans-thre) 1 3)
                            "eom-trans  ~5,3f / ~5,3f  ~a~%" (norm eom-trans-diff) eom-trans-thre eom-trans-diff) ;; [N]
                           (warning-message
                            (if (>= (norm eom-rot-diff) eom-rot-thre) 1 3)
                            "eom-rot  ~5,3f / ~5,3f  ~a~%" (norm eom-rot-diff) eom-rot-thre eom-rot-diff) ;; [Nm]
                           (warning-message
                            (if (>= (norm torque-diff) torque-thre) 1 3)
                            "torque  ~5,3f / ~5,3f  ~a~%" (norm torque-diff) torque-thre torque-diff) ;; [Nm] [N]
                           ))
                     :pre-process-func pre-process-func
                     :post-process-func post-process-func
                     :no-visualize? (not debug-view)
                     :no-print? (not (and debug-view (not (memq :no-message debug-view))))
                     ))
          )
     (send instant-config-task :put :thre thre)
     (send instant-config-task :put :rthre rthre)
     (send instant-config-task :put :eom-trans-thre eom-trans-thre)
     (send instant-config-task :put :eom-rot-thre eom-rot-thre)
     (send instant-config-task :put :torque-thre torque-thre)
     (send sqp-opt :optimize :loop-num stop :loop-num-min min-loop)
     (send robot-env :dissoc-root-virtual)
     (cond ((and (not (equal (send sqp-opt :status) :success)) revert-if-fail)
            (send self :angle-vector av-orig)
            (send self :newcoords rc-orig)
            nil
            )
           (t
            (send self :angle-vector)
            ))
     ))
  )

(defmethod cascaded-link
  (:inverse-kinematics-statics-trajectory-optmotiongen
   (target-coords-list
    &key
    ;; same arguments with irteus inverse-kinematics
    (stop 50)
    (move-target-list)
    (debug-view)
    (revert-if-fail t)
    (transform-coords-list :nil)
    (translation-axis-list :nil)
    (rotation-axis-list :nil)
    (thre 1.0)
    (rthre (deg2rad 1))
    (thre-list :nil)
    (rthre-list :nil)
    (collision-avoidance-link-pair :nil)
    (collision-distance-limit 10.0) ;; [mm]
    (obstacles)
    (min-loop)
    ;; new arguments
    (contact-target-coords-list)
    (contact-move-target-list)
    (contact-constraint-list)
    (root-virtual-mode :fix)
    (root-virtual-joint-invariant? nil)
    (root-virtual-joint-min-angle-list)
    (root-virtual-joint-max-angle-list)
    (joint-angle-margin 0.0)
    (posture-joint-list
     (make-list (length target-coords-list) :initial-element nil))
    (posture-joint-angle-list
     (make-list (length target-coords-list) :initial-element nil))
    (optimize-torque?)
    (optimize-start-end-torque?)
    (eom-trans-thre 1e-1)
    (eom-rot-thre 1e-1)
    (torque-thre 1e-1)
    (target-posture-scale 1e-3)
    (norm-regular-scale-max 1e-3)
    (norm-regular-scale-offset 1e-6)
    (torque-regular-scale 1e-5)
    (adjacent-regular-scale 0.0)
    (pre-process-func)
    (post-process-func)
    &allow-other-keys
    )
   "
Solve inverse kinematics problem with sqp optimization.
    target-coords-list : The coordinate of the target that returns coordinates. Use a list of targets to solve the IK relative to multiple end links simultaneously. Function is not available to target-coords.
    move-target-list : Specify end-effector coordinate. When the target-coords is list, this should be list too.
    stop : Maximum number for IK iteration. Default is 50.
    debug-view : Set t to show debug message and visualization. Use :no-message to just show the irtview image. Default is nil.
    revert-if-fail : Set nil to keep the angle posture of IK solve iteration. Default is t, which return to original position when IK fails.
    translation-axis-list : :x :y :z for constraint along the x, y, z axis. :xy :yz :zx for plane. Default is t.
    rotation-axis-list : Use nil for position only IK. :x, :y, :z for the constraint around axis with plus direction. When the target-coords is list, this should be list too. Default is t.
    thre : Threshold for position error to terminate IK iteration. Default is 1 [mm].
    rthre : Threshold for rotation error to terminate IK iteration. Default is 0.017453 [rad] (1 deg).
    contact-target-coords-list : target coords for contact
    contact-move-target-list : end-effector coords for contact
    contact-constraint-list : contact constraint
   "
   (setq target-coords-list
         (mapcar #'(lambda (x) (if (atom x) (list x) x)) target-coords-list))
   (cond ((atom move-target-list)
          (setq move-target-list (make-list (length target-coords-list) :initial-element (list move-target-list)))
          )
         ((every #'atom move-target-list)
          (setq move-target-list (make-list (length target-coords-list) :initial-element move-target-list))
          )
         (t
          (setq move-target-list (mapcar #'(lambda (x) (if (atom x) (list x) x)) move-target-list))
          ))
   (cond ((equal transform-coords-list :nil)
          (setq transform-coords-list target-coords-list)
          )
         ((every #'atom transform-coords-list)
          (setq transform-coords-list (make-list (length target-coords-list) :initial-element transform-coords-list))
          ))
   (cond ((equal translation-axis-list :nil)
          (setq translation-axis-list (mapcar #'(lambda (tc) (make-list (length tc) :initial-element t)) target-coords-list))
          )
         ((every #'atom translation-axis-list)
          (setq translation-axis-list (make-list (length target-coords-list) :initial-element translation-axis-list))
          ))
   (cond ((equal rotation-axis-list :nil)
          (setq rotation-axis-list (mapcar #'(lambda (tc) (make-list (length tc) :initial-element t)) target-coords-list))
          )
         ((every #'atom rotation-axis-list)
          (setq rotation-axis-list (make-list (length target-coords-list) :initial-element rotation-axis-list))
          ))
   (cond ((equal thre-list :nil)
          (setq thre-list (mapcar #'(lambda (tc) (make-list (length tc) :initial-element thre)) target-coords-list))
          )
         ((every #'atom thre-list)
          (setq thre-list (make-list (length target-coords-list) :initial-element thre-list))
          ))
   (cond ((equal rthre-list :nil)
          (setq rthre-list (mapcar #'(lambda (tc) (make-list (length tc) :initial-element rthre)) target-coords-list))
          )
         ((every #'atom rthre-list)
          (setq rthre-list (make-list (length target-coords-list) :initial-element rthre-list))
          ))
   (if (and debug-view (atom debug-view)) (setq debug-view (list debug-view)))
   (let* ((av-orig
           (send self :angle-vector))
          (rc-orig
           (send self :copy-worldcoords))
          (collision-pair-list
           (cond ((not (equal collision-avoidance-link-pair :nil))
                  collision-avoidance-link-pair
                  )
                 (t
                  (send self :collision-avoidance-link-pair-from-link-list
                        links
                        :obstacles obstacles)
                  ))
           )
          (physical-target-coords
           (remove-if-not
            #'(lambda (tc) (or (derivedp tc polygon) (derivedp tc edge)))
            (remove-duplicates (flatten target-coords-list) :test #'equal)
            ))
          (physical-contact
           (mapcar
            #'(lambda
                (i tc)
                (send tc :put :physical-id i)
                (let* ((contact
                        (cond ((derivedp tc polygon)
                               (generate-contact-ik-arg-from-rect-face
                                :rect-face tc)
                               )
                              ((derivedp tc edge)
                               (generate-contact-ik-arg-from-line-segment
                                :line-seg tc)
                               )))
                       )
                  (send contact :put :target-coords tc)
                  contact
                  ))
            (range (length physical-target-coords))
            physical-target-coords
            ))
          (robot-env-list
           (progn
             ;; By putting cascaded coords and copying object, cascaded structure is holded in copied object and accessible by :get method.
             (send self :put :collision-pair-list collision-pair-list)
             (mapcar
              #'(lambda
                  (target-coords
                   move-target
                   translation-axis
                   rotation-axis
                   transform-coords
                   )
                  (send self :put :move-target move-target)
                  (let* ((robot
                          (progn
                            (send self :put :target-coords target-coords)
                            (copy-object self)
                            ))
                         (contact-list
                          (append
                           physical-contact
                           (remove
                            nil
                            (mapcar
                             #'(lambda
                                 (tc trans-axis rot-axis trans-coords)
                                 (when (and (not
                                             (find-if
                                              #'(lambda (ptc) (equal (send tc :get :physical-id) (send ptc :get :physical-id)))
                                              physical-target-coords))
                                            (not (and (equal trans-axis t) (equal rot-axis t)))
                                            )
                                   (let* ((contact
                                           (instance contact-ik-arg :init
                                                     :target-coords tc
                                                     :translation-axis trans-axis
                                                     :rotation-axis rot-axis
                                                     :transform-coords trans-coords
                                                     ))
                                          )
                                     (send contact :put :target-coords tc)
                                     contact
                                     )))
                             (send robot :get :target-coords)
                             translation-axis
                             rotation-axis
                             transform-coords
                             )))
                          )
                         (robot-env
                          (instance robot-environment :init
                                    :robot robot
                                    :contact-list contact-list
                                    :root-virtual-mode root-virtual-mode
                                    :root-virtual-joint-min-angle-list root-virtual-joint-min-angle-list
                                    :root-virtual-joint-max-angle-list root-virtual-joint-max-angle-list
                                    ))
                         )
                    (send robot-env :invariant-joint-list
                          (append
                           (flatten
                            (send-all
                             (remove-if-not
                              #'(lambda (c) (numberp (send (send c :get :target-coords) :get :physical-id)))
                              contact-list)
                             :joint-list))
                           (if root-virtual-joint-invariant? (send robot-env :root-virtual-joint-list))
                           ))
                    (send robot-env :variant-joint-list
                          (set-difference (send robot-env :joint-list) (send robot-env :invariant-joint-list)))
                    (send robot-env :put :move-target
                          (send robot-env :robot :get :move-target))
                    (send robot-env :put :collision-pair-list
                          (send robot-env :robot :get :collision-pair-list))
                    robot-env
                    ))
              target-coords-list
              move-target-list
              translation-axis-list
              rotation-axis-list
              transform-coords-list
              )
             ))
          (instant-config-task-list
           (mapcar
            #'(lambda
                (i
                 robot-env
                 target-coords
                 thre
                 rthre
                 contact-move-target
                 contact-target-coords
                 contact-constraint
                 posture-jl
                 posture-jal)
                (let* ((config-task
                        (instance instant-configuration-task :init
                                  :robot-env robot-env
                                  :variant-joint-list
                                  (send robot-env :variant-joint-list)
                                  :invariant-joint-list
                                  (send robot-env :invariant-joint-list)
                                  :drive-joint-list
                                  (if (or optimize-torque?
                                          (and optimize-start-end-torque?
                                               (or (= i 0) (= i (1- (length robot-env-list)))))
                                          )
                                      (send robot-env :drive-joint-list))
                                  :posture-joint-list posture-jl
                                  :posture-joint-angle-list posture-jal
                                  :kin-target-coords-list
                                  (mapcar
                                   #'(lambda (tc)
                                       (let* ((contact
                                               (find-if
                                                #'(lambda (c) (equal (send c :get :target-coords) tc))
                                                (send robot-env :contact-list)))
                                              )
                                         (cond (contact
                                                (send contact :contact-coords)
                                                )
                                               (t
                                                tc))
                                         ))
                                   target-coords
                                   )
                                  :kin-attention-coords-list
                                  (send robot-env :get :move-target)
                                  :contact-attention-coords-list contact-move-target
                                  :contact-target-coords-list
                                  (mapcar
                                   #'(lambda (tc)
                                       (let* ((contact
                                               (find-if
                                                #'(lambda (c) (equal (send c :get :target-coords) tc))
                                                (send robot-env :contact-list)))
                                              )
                                         (cond (contact
                                                (send contact :contact-coords)
                                                )
                                               (t
                                                tc))
                                         ))
                                   contact-target-coords
                                   )
                                  :contact-constraint-list contact-constraint
                                  :collision-pair-list
                                  (send robot-env :get :collision-pair-list)
                                  :collision-distance-margin
                                  (* 1e-3 collision-distance-limit) ;; [m]
                                  :variant-joint-angle-margin joint-angle-margin
                                  :invariant-joint-angle-margin joint-angle-margin
                                  ))
                       )
                  (send config-task :put :thre thre)
                  (send config-task :put :rthre rthre)
                  (send config-task :put :eom-trans-thre eom-trans-thre)
                  (send config-task :put :eom-rot-thre eom-rot-thre)
                  (send config-task :put :torque-thre torque-thre)
                  config-task
                  ))
            (range (length robot-env-list))
            robot-env-list
            target-coords-list
            thre-list
            rthre-list
            contact-move-target-list
            contact-target-coords-list
            contact-constraint-list
            posture-joint-list
            posture-joint-angle-list
            ))
          (trajectory-config-task
           (instance trajectory-configuration-task :init
                     :instant-config-task-list instant-config-task-list
                     :norm-regular-scale-max norm-regular-scale-max
                     :norm-regular-scale-offset norm-regular-scale-offset
                     :adjacent-regular-scale adjacent-regular-scale
                     :torque-regular-scale torque-regular-scale
                     ))
          (sqp-opt
           (instance sqp-optimization :init
                     :config-task trajectory-config-task
                     :convergence-check-func
                     #'(lambda (sqp)
                         (let* ((converge? t)
                                )
                           (dolist (config-task (send (send sqp :config-task) :instant-config-task-list))
                             (let* ((kin-task-value
                                     (send config-task :kinematics-task-value :update? nil))
                                    (pos-diff-list
                                     (divide-vector kin-task-value :segment-length 3 :interval-length 3))
                                    (rot-diff-list
                                     (divide-vector (subseq kin-task-value 3) :segment-length 3 :interval-length 3))
                                    (thre
                                     (send config-task :get :thre))
                                    (rthre
                                     (send config-task :get :rthre))
                                    (eom-trans-diff
                                     (send config-task :eom-trans-task-value :update? nil))
                                    (eom-rot-diff
                                     (send config-task :eom-rot-task-value :update? nil))
                                    (torque-diff
                                     (send config-task :torque-task-value :update? nil))
                                    (eom-trans-thre
                                     (send config-task :get :eom-trans-thre))
                                    (eom-rot-thre
                                     (send config-task :get :eom-rot-thre))
                                    (torque-thre
                                     (send config-task :get :torque-thre))
                                    )
                               (unless (and (every #'(lambda (pos-diff pos-th) (< (* 1e3 (norm pos-diff)) pos-th)) pos-diff-list thre) ;; [mm]
                                            (every #'(lambda (rot-diff rot-th) (< (rad2deg (norm rot-diff)) rot-th)) rot-diff-list rthre) ;; [deg]
                                            (< (norm eom-trans-diff) eom-trans-thre) ;; [N]
                                            (< (norm eom-rot-diff) eom-rot-thre) ;; [Nm]
                                            (< (norm torque-diff) torque-thre) ;; [Nm] [N]
                                            )
                                 (setq converge? nil)
                                 (return-from nil nil)
                                 )
                               ))
                           converge?
                           ))
                     :failure-callback-func
                     #'(lambda (sqp)
                         (warning-message 3 "inverse-kinematics-optmotiongen failed.~%")
                         (dolist (config-task (send (send sqp :config-task) :instant-config-task-list))
                           (let* ((kin-task-value
                                   (send config-task :kinematics-task-value :update? nil))
                                  (pos-diff-list
                                   (divide-vector kin-task-value :segment-length 3 :interval-length 3))
                                  (rot-diff-list
                                   (divide-vector (subseq kin-task-value 3) :segment-length 3 :interval-length 3))
                                  (thre
                                   (send config-task :get :thre))
                                  (rthre
                                   (send config-task :get :rthre))
                                  (eom-trans-diff
                                   (send config-task :eom-trans-task-value :update? nil))
                                  (eom-rot-diff
                                   (send config-task :eom-rot-task-value :update? nil))
                                  (torque-diff
                                   (send config-task :torque-task-value :update? nil))
                                  (eom-trans-thre
                                   (send config-task :get :eom-trans-thre))
                                  (eom-rot-thre
                                   (send config-task :get :eom-rot-thre))
                                  (torque-thre
                                   (send config-task :get :torque-thre))
                                  )
                             (map
                              nil
                              #'(lambda (i pos-diff rot-diff pos-th rot-th)
                                  (warning-message
                                   (if (>= (* 1e3 (norm pos-diff)) pos-th) 1 3)
                                   "pos[~1d]  ~5,3f / ~5,3f  ~a~%" i (* 1e3 (norm pos-diff)) pos-th pos-diff) ;; [mm]
                                  (warning-message
                                   (if (>= (rad2deg (norm rot-diff)) rot-th) 1 3)
                                   "rot[~1d]  ~5,3f / ~5,3f  ~a~%" i (rad2deg (norm rot-diff)) rot-th rot-diff) ;; [deg]
                                  )
                              (range (length pos-diff-list))
                              pos-diff-list
                              rot-diff-list
                              thre
                              rthre
                              )
                             (warning-message
                              (if (>= (norm eom-trans-diff) eom-trans-thre) 1 3)
                              "eom-trans  ~5,3f / ~5,3f  ~a~%" (norm eom-trans-diff) eom-trans-thre eom-trans-diff) ;; [N]
                             (warning-message
                              (if (>= (norm eom-rot-diff) eom-rot-thre) 1 3)
                              "eom-rot  ~5,3f / ~5,3f  ~a~%" (norm eom-rot-diff) eom-rot-thre eom-rot-diff) ;; [Nm]
                             (warning-message
                              (if (>= (norm torque-diff) torque-thre) 1 3)
                              "torque  ~5,3f / ~5,3f  ~a~%" (norm torque-diff) torque-thre torque-diff) ;; [Nm] [N]
                             ))
                         )
                     :pre-process-func pre-process-func
                     :post-process-func post-process-func
                     :no-visualize? (not debug-view)
                     :no-print? (not (and debug-view (not (memq :no-message debug-view))))
                     ))
          )
     (when debug-view
       (objects (append robot-env-list (cadr (memq :additional-draw-objects debug-view)))))
     (send sqp-opt :optimize :loop-num stop :loop-num-min min-loop)
     (send-all robot-env-list :dissoc-root-virtual)
     (cond ((and (not (equal (send sqp-opt :status) :success)) revert-if-fail)
            (send self :angle-vector av-orig)
            (send self :newcoords rc-orig)
            nil
            )
           (t
            (send self :angle-vector (send (car robot-env-list) :robot :angle-vector))
            (send self :newcoords (send (car robot-env-list) :robot :copy-worldcoords))
            (mapcar
             #'(lambda (av rc)
                 (list (list :angle-vector av) (list :root-coords rc)))
             (send-all robot-env-list :robot :angle-vector)
             (send-all robot-env-list :robot :copy-worldcoords)
             )
            ))
     ))
  )
