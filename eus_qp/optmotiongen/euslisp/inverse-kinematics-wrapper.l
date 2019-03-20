(load "./trajectory-configuration-task.l")
(load "./sqp-optimization.l")


;;;;;;;;;;;;;;;;;;;;;;;;;
;; cascaded-link (add methods)
;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod cascaded-link
  (:inverse-kinematics-optmotiongen
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
    (root-virtual-mode :fix)
    (root-virtual-joint-min-angle-list)
    (root-virtual-joint-max-angle-list)
    (joint-angle-margin 0.0)
    (posture-joint-list)
    (posture-joint-angle-list)
    (target-posture-scale 1e-3)
    (norm-regular-scale-max 1e-2)
    (norm-regular-scale-offset 1e-7)
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
                     :drive-joint-list nil
                     :only-kinematics? t
                     :posture-joint-list posture-joint-list
                     :posture-joint-angle-list posture-joint-angle-list
                     :target-posture-scale target-posture-scale
                     :kin-target-coords-list
                     (mapcar
                      #'(lambda (c tc) (if c (send c :contact-coords) tc))
                      contact-list target-coords)
                     :kin-attention-coords-list move-target
                     :collision-pair-list collision-pair-list
                     :collision-distance-margin (* 1e-3 collision-distance-limit) ;; [m]
                     :variant-joint-angle-margin joint-angle-margin
                     :invariant-joint-angle-margin joint-angle-margin
                     :norm-regular-scale-max norm-regular-scale-max
                     :norm-regular-scale-offset norm-regular-scale-offset
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
                                )
                           (and (every #'(lambda (pos-diff pos-th) (< (* 1e3 (norm pos-diff)) pos-th)) pos-diff-list thre) ;; [mm]
                                (every #'(lambda (rot-diff rot-th) (< (rad2deg (norm rot-diff)) rot-th)) rot-diff-list rthre)) ;; [deg]
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
                           ))
                     :pre-process-func pre-process-func
                     :post-process-func post-process-func
                     :no-visualize? (not debug-view)
                     :no-print? (not (and debug-view (not (memq :no-message debug-view))))
                     ))
          )
     (send instant-config-task :put :thre thre)
     (send instant-config-task :put :rthre rthre)
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
  (:inverse-kinematics-trajectory-optmotiongen
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
    (root-virtual-mode :fix)
    (root-virtual-joint-invariant? nil)
    (root-virtual-joint-min-angle-list)
    (root-virtual-joint-max-angle-list)
    (joint-angle-margin 0.0)
    (posture-joint-list
     (make-list (length target-coords-list) :initial-element nil))
    (posture-joint-angle-list
     (make-list (length target-coords-list) :initial-element nil))
    (norm-regular-scale-max 1e-3)
    (norm-regular-scale-offset 1e-7)
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
                           (send robot :get :target-coords)
                           translation-axis
                           rotation-axis
                           transform-coords
                           ))
                         (robot-env
                          (instance robot-environment :init
                                    :robot robot
                                    :contact-list (remove nil contact-list)
                                    :root-virtual-mode root-virtual-mode
                                    :root-virtual-joint-min-angle-list root-virtual-joint-min-angle-list
                                    :root-virtual-joint-max-angle-list root-virtual-joint-max-angle-list
                                    ))
                         )
                    (send robot-env :invariant-joint-list
                          (if root-virtual-joint-invariant? (send robot-env :root-virtual-joint-list)))
                    (send robot-env :variant-joint-list
                          (set-difference (send robot-env :joint-list) (send robot-env :invariant-joint-list)))
                    (send robot-env :put :contact-list contact-list)
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
                (robot-env
                 target-coords
                 thre
                 rthre
                 posture-jl
                 posture-jal)
                (let* ((config-task
                        (instance instant-configuration-task :init
                                  :robot-env robot-env
                                  :variant-joint-list
                                  (send robot-env :variant-joint-list)
                                  :invariant-joint-list
                                  (send robot-env :invariant-joint-list)
                                  :drive-joint-list nil
                                  :only-kinematics? t
                                  :posture-joint-list posture-jl
                                  :posture-joint-angle-list posture-jal
                                  :kin-target-coords-list
                                  (mapcar
                                   #'(lambda (c tc) (if c (send c :contact-coords) tc))
                                   (send robot-env :get :contact-list)
                                   target-coords
                                   )
                                  :kin-attention-coords-list
                                  (send robot-env :get :move-target)
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
                  config-task
                  ))
            robot-env-list
            target-coords-list
            thre-list
            rthre-list
            posture-joint-list
            posture-joint-angle-list
            ))
          (trajectory-config-task
           (instance trajectory-configuration-task :init
                     :instant-config-task-list instant-config-task-list
                     :norm-regular-scale-max norm-regular-scale-max
                     :norm-regular-scale-offset norm-regular-scale-offset
                     :adjacent-regular-scale adjacent-regular-scale
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
                                    )
                               (unless (and (every #'(lambda (pos-diff pos-th) (< (* 1e3 (norm pos-diff)) pos-th)) pos-diff-list thre) ;; [mm]
                                            (every #'(lambda (rot-diff rot-th) (< (rad2deg (norm rot-diff)) rot-th)) rot-diff-list rthre)) ;; [deg]
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


;;;;;;;;;;;;;;;;;;;;;;;;;
;; robot-model (add or overwrite methods)
;;;;;;;;;;;;;;;;;;;;;;;;;

(unless (assoc :limb-orig (send robot-model :methods))
  (rplaca (assoc :limb (send robot-model :methods)) :limb-orig))
(defmethod robot-model
  (:limb
   (limb method &rest args)
   "
Extend to support to call :inverse-kinematics-optmotiongen.
   "
   (let (ret)
     (case
      method
      (:inverse-kinematics-optmotiongen
       (let* ((link-list
               (if (memq :link-list args)
                   (cadr (memq :link-list args))
                 (send self :link-list
                       (send self limb :end-coords :parent)
                       (send self limb :root-link))))
              )
         (send* self :inverse-kinematics-optmotiongen
                (car args)
                :move-target
                (if (memq :move-target args)
                    (cadr (memq :move-target args))
                  (send self limb :end-coords))
                :link-list link-list
                (cdr args))))
      (t
       (send* self :limb-orig limb method args)
       ))
     ))
  )


;;;;;;;;;;;;;;;;;;;;;;;;;
;; contact-ik-arg
;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass contact-ik-arg
  :super cascaded-link
  :slots ((_contact-coords "$T_{\mathit{cnt}}$")
          )
  :documentation
  "
inverse-kinematics-optmotiongenの{\it target-coords, translation-axis, rotation-axis, transform-coords}引数に対応する接触座標を表す仮想の関節・リンク構造．
  "
  )

(defmethod contact-ik-arg
  (:init
   (&key
    (target-coords)
    (translation-axis)
    (rotation-axis)
    (transform-coords target-coords)
    (name (send target-coords :name))
    )
   "
Initialize instance
   "
   (send-super :init :name name)
   (let* (rv-ll
          rv-jl
          (string-name
           (string-left-trim ":" (format nil "~a" (send self :name))))
          (translation-joint-axis
           (if (atom translation-axis) (ik-arg-axis->axis-list translation-axis) translation-axis))
          (rotation-joint-axis
           (if (atom rotation-axis) (ik-arg-axis->axis-list rotation-axis) rotation-axis))
          (joint-class-list
           (append (make-list (length translation-joint-axis) :initial-element linear-joint)
                   (make-list (length rotation-joint-axis) :initial-element rotational-joint)))
          (joint-axis-list
           (append translation-joint-axis rotation-joint-axis))
          (root-link
           (instance bodyset-link :init
                     (make-cascoords)
                     :bodies (list (make-cube 1e-3 1e-3 1e-3))
                     :name (read-from-string (format nil ":~a-root" string-name))
                     :weight 0.0
                     ))
          )
     ;; 1. setup link
     (setq rv-ll
           (append
            (list root-link)
            (mapcar
             #'(lambda (joint-class
                        joint-axis
                        )
                 (instance bodyset-link :init
                           (make-cascoords)
                           :bodies (list (make-cube 1e-3 1e-3 1e-3))
                           :name
                           (read-from-string
                            (format nil ":~a-~a-~a" string-name (send joint-class :name) (symbol-name joint-axis)))
                           :weight 0.0
                           ))
             joint-class-list
             joint-axis-list
             )))
     (dotimes (i (1- (length rv-ll)))
       (send (elt rv-ll i) :assoc (elt rv-ll (1+ i))))
     (send self :assoc (car rv-ll))
     (setq links rv-ll)
     ;; 2. setup joint
     (setq rv-jl
           (mapcar
            #'(lambda (joint-class
                       joint-axis
                       parent-link
                       child-link
                       )
                (instance joint-class :init
                          :name
                          (read-from-string
                           (format nil ":~a-~a-~a" string-name (send joint-class :name) (symbol-name joint-axis)))
                          :parent-link parent-link :child-link child-link
                          :axis joint-axis :min -1e10 :max 1e10
                          :max-joint-torque 0.0
                          ))
            joint-class-list
            joint-axis-list
            (butlast rv-ll)
            (cdr rv-ll)
            ))
     (setq joint-list rv-jl)
     (send self :init-ending)
     ;; 3. set transform-coords and target-coords
     (send self :newcoords
           (make-coords :pos (send target-coords :worldpos)
                        :rot (send (send transform-coords :copy-worldcoords) :worldrot)))
     (setq _contact-coords
           (make-cascoords :parent (car (last rv-ll))
                           :coords (send target-coords :copy-worldcoords)
                           ))
     self
     ))
  (:contact-coords
   (&rest args)
   "
return $T_{\mathit{cnt}} := \{ \bm{p}_{\mathit{cnt}}, \bm{R}_{\mathit{cnt}} \}$
   "
   (forward-message-to _contact-coords args)
   )
  )


;;;;;;;;;;;;;;;;;;;;;;;;;
;; util
;;;;;;;;;;;;;;;;;;;;;;;;;

(defun ik-arg-axis->axis-list
    (ik-arg-axis)
  "
Convert translation-axis / rotatoin-axis to axis list.
  "
  (case ik-arg-axis
        ((t) nil)
        ((:x) (list :x))
        ((:y) (list :y))
        ((:z) (list :z))
        ((:xy :yx) (list :x :y))
        ((:yz :zy) (list :y :z))
        ((:zx :xz) (list :z :x))
        ((nil :xyz) (list :x :y :z))
        ))

(defun axis->index
    (axis)
  (case axis
        ((:x :-x) 0)
        ((:y :-y) 1)
        ((:z :-z) 2)
        )
  )

(defun axis->sgn
    (axis)
  (case axis
        ((:x :y :z) 1)
        ((:-x :-y :-z) -1)
        )
  )

(defun generate-contact-ik-arg-from-rect-face
    (&key
     (rect-face)
     (name (send rect-face :name))
     (margin (or (send rect-face :get :margin) 0))
     )
  "
Generate contact-ik-arg instance from rectangle face.
  "
  (let* ((x-edge
          (elt (send rect-face :edges) 0))
         (x-dir
          (send x-edge :direction))
         (x-len
          (send x-edge :length))
         (y-edge
          (elt (send rect-face :edges) 1))
         (y-dir
          (send y-edge :direction))
         (y-len
          (send y-edge :length))
         (z-dir
          (send rect-face :normal))
         (rot
          (transpose (matrix x-dir y-dir z-dir)))
         (pos
          (elt (send rect-face :centroid) 1))
         (transform-coords
          (make-coords :name name :pos pos :rot rot))
         (target-rotation
          (cond ((send rect-face :get :target-rotation)
                 (send rect-face :get :target-rotation)
                 )
                ((send rect-face :get :normal-move-target-axis)
                 (send (orient-coords-to-axis (make-coords) z-dir (send rect-face :get :normal-move-target-axis)) :worldrot)
                 ))
          )
         (target-coords
          (cond (target-rotation
                 (make-coords
                  :name (send transform-coords :name)
                  :pos (send transform-coords :worldpos)
                  :rot target-rotation)
                 )
                (t
                 transform-coords
                 ))
          )
         (contact
          (instance contact-ik-arg :init
                    :target-coords target-coords
                    :translation-axis :xy
                    :rotation-axis :xyz
                    :transform-coords transform-coords
                    ))
         (linear-x-joint
          (elt (send contact :joint-list) 0))
         (linear-y-joint
          (elt (send contact :joint-list) 1))
         (rotational-x-joint
          (elt (send contact :joint-list) 2))
         (rotational-y-joint
          (elt (send contact :joint-list) 3))
         (rotational-z-joint
          (elt (send contact :joint-list) 4))
         (normal-tolerance-x
          (or (send rect-face :get :normal-tolerance-x)
              (send rect-face :get :normal-tolerance)
              0))
         (normal-tolerance-y
          (or (send rect-face :get :normal-tolerance-y)
              (send rect-face :get :normal-tolerance)
              0))
         )
    (send linear-x-joint :max-angle (max (- (* 0.5 x-len) margin) 0))
    (send linear-x-joint :min-angle (min (+ (* -0.5 x-len) margin) 0))
    (send linear-y-joint :max-angle (max (- (* 0.5 y-len) margin) 0))
    (send linear-y-joint :min-angle (min (+ (* -0.5 y-len) margin) 0))
    (send rotational-x-joint :max-angle normal-tolerance-x)
    (send rotational-x-joint :min-angle (- normal-tolerance-x))
    (send rotational-y-joint :max-angle normal-tolerance-y)
    (send rotational-y-joint :min-angle (- normal-tolerance-y))
    contact
    ))

(defun generate-contact-ik-arg-from-line-segment
    (&key
     (line-seg)
     (name (send line-seg :name))
     (margin (or (send line-seg :get :margin) 0))
     )
  "
Generate contact-ik-arg instance from line segment.
  "
  (let* ((x-dir
          (send line-seg :direction))
         (x-len
          (send line-seg :length))
         (pos
          (send line-seg :point 0.5))
         (transform-coords
          (orient-coords-to-axis
           (make-coords :name name :pos pos)
           x-dir :x))
         (y-dir
          (send transform-coords :rotate-vector (float-vector 0 1 0)))
         (z-dir
          (send transform-coords :rotate-vector (float-vector 0 0 1)))
         (target-rotation
          (cond ((send line-seg :get :target-rotation)
                 (send line-seg :get :target-rotation)
                 )
                ((and (send line-seg :get :direction-move-target-axis)
                      (send line-seg :get :normal-move-target-axis))
                 (let* ((mat (make-matrix 3 3))
                        (direction-axis
                         (send line-seg :get :direction-move-target-axis))
                        (direction-index
                         (axis->index direction-axis))
                        (direction-sgn
                         (axis->sgn direction-axis))
                        (normal-axis
                         (send line-seg :get :normal-move-target-axis))
                        (normal-index
                         (axis->index normal-axis))
                        (normal-sgn
                         (axis->sgn normal-axis))
                        (other-index
                         (car (remove direction-index (remove normal-index (list 0 1 2)))))
                        )
                   (setf (matrix-column mat direction-index) (scale direction-sgn x-dir))
                   (setf (matrix-column mat normal-index) (scale normal-sgn z-dir))
                   (setf (matrix-column mat other-index) y-dir)
                   ;; set right hand coordinate system
                   (when (< (v. (v* (matrix-column mat 0) (matrix-column mat 1)) (matrix-column mat 2)) 0)
                     (setf (matrix-column mat other-index) (scale -1 (matrix-column mat other-index))))
                   mat
                   ))
                ((send line-seg :get :direction-move-target-axis)
                 (send (orient-coords-to-axis (make-coords) x-dir (send line-seg :get :direction-move-target-axis)) :worldrot)
                 ))
          )
         (target-coords
          (cond (target-rotation
                 (make-coords
                  :name (send transform-coords :name)
                  :pos (send transform-coords :worldpos)
                  :rot target-rotation)
                 )
                (t
                 transform-coords
                 ))
          )
         (contact
          (instance contact-ik-arg :init
                    :target-coords target-coords
                    :translation-axis :x
                    :rotation-axis (list :x :y) ;; :y should be after :x
                    :transform-coords transform-coords
                    ))
         (linear-x-joint
          (elt (send contact :joint-list) 0))
         (rotational-x-joint
          (elt (send contact :joint-list) 1))
         (rotational-y-joint
          (elt (send contact :joint-list) 2))
         (normal-tolerance
          (or (send line-seg :get :normal-tolerance) 0))
         )
    (send linear-x-joint :max-angle (max (- (* 0.5 x-len) margin) 0))
    (send linear-x-joint :min-angle (min (+ (* -0.5 x-len) margin) 0))
    (send rotational-y-joint :max-angle normal-tolerance)
    (send rotational-y-joint :min-angle (- normal-tolerance))
    contact
    ))
