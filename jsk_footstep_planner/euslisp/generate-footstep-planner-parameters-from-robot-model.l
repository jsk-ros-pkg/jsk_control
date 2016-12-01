;;;;;;;;;;;;;;;;
;; Generate parameters for footstep planner using robot model
;;;;;;;;;;;;;;;;

;;;;;;;;;;;;;;;;
;; utilities
;;;;;;;;;;;;;;;;
(defun eus-vector->yaml-vector-string
  (vec)
  "Generate yaml-style vector string from eus vector.
   For example, #f(1 2 3) -> [1, 2, 3] string."
  (let ((str "["))
    (dotimes (i (length vec))
      (setq str (format nil "~A~A~A" str (elt vec i) (if (= i (- (length vec) 1)) "" ", "))))
    (format nil "~A]" str)))

(defun neglect-small-value
  (value thre)
  "Neglect small value.
   If abs of value is larger than thre, return value. Otherwise, return 0."
  (if (< (abs value) thre) 0.0 value))

;;;;;;;;;;;;;;;;
;; generate functions
;;;;;;;;;;;;;;;;
(defun gen-default_lfoot_to_rfoot_offset-string
  (robot &key (return-value))
  "Generate default_lfoot_to_rfoot_offset."
  (let ((ret (scale (* -1 2 1e-3) (cadr (memq :default-half-offset (send robot :footstep-parameter))))))
    (if return-value ret (format nil "default_lfoot_to_rfoot_offset: ~A" (eus-vector->yaml-vector-string ret)))
    ))

(defun gen-footstep_size_xy-string
  (robot &key (return-value))
  "Generate footstep_size_[xy]."
  (let* ((vs (mapcar #'(lambda (v) (send (send robot :lleg :end-coords) :inverse-transform-vector v)) (send (send robot :support-polygon :lleg) :vertices)))
         (fs-size-x (* 1e-3 (- (elt (find-extream vs #'(lambda (x) (elt x 0)) #'>) 0) (elt (find-extream vs #'(lambda (x) (elt x 0)) #'<) 0))))
         (fs-size-y (* 1e-3 (- (elt (find-extream vs #'(lambda (x) (elt x 1)) #'>) 1) (elt (find-extream vs #'(lambda (x) (elt x 1)) #'<) 1)))))
    (if return-value
        (list fs-size-x fs-size-y)
      (format nil "footstep_size_x: ~A~%footstep_size_y: ~A" fs-size-x fs-size-y))
    ))

(defun gen-rlleg_footstep_offset-string
  (robot &key (return-value))
  "Generate [rl]leg_footstep_offset."
  (let* ((neglect-thre (* 1e-3 0.1)) ;; 0.1 [mm]
         (off-list
          (mapcar #'(lambda (leg)
                      (let ((tmp (scale 1e-3 (send (send robot leg :end-coords) :inverse-transform-vector (cadr (send (send robot :support-polygon leg) :centroid))))))
                        (float-vector (neglect-small-value (elt tmp 0) neglect-thre)
                                      (neglect-small-value (elt tmp 1) neglect-thre)
                                      (neglect-small-value (elt tmp 2) neglect-thre))
                        ))
                  '(:rleg :lleg))))
    (if return-value
        off-list
      (format nil "rleg_footstep_offset: ~A~%lleg_footstep_offset: ~A" (eus-vector->yaml-vector-string (car off-list)) (eus-vector->yaml-vector-string (cadr off-list))))
    ))

(defun gen-collision_bbox_sizeoffset-string
  (robot &key (return-value))
  "Generate collision_bbox_size and collision_bbox_offset."
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  (send *robot* :update-descendants)
  (let* ((bb (send (make-bounding-box (flatten (send-all (flatten (send-all (flatten (append (send robot :torso) (send robot :legs))) :bodies)) :vertices))) :body))
         (bbox-size
          (float-vector
           (* 1e-3 (- (elt (find-extream (send bb :vertices) #'(lambda (x) (elt x 0)) #'>) 0) (elt (find-extream (send bb :vertices) #'(lambda (x) (elt x 0)) #'<) 0)))
           (* 1e-3 (- (elt (find-extream (send bb :vertices) #'(lambda (x) (elt x 1)) #'>) 1) (elt (find-extream (send bb :vertices) #'(lambda (x) (elt x 1)) #'<) 1)))
           (* 1e-3 (- (elt (find-extream (send bb :vertices) #'(lambda (x) (elt x 2)) #'>) 2) (elt (find-extream (send bb :vertices) #'(lambda (x) (elt x 2)) #'<) 2)))))
         (neglect-thre (* 1e-3 0.1)) ;; 0.1[mm]
         (bbox-offset
          (let ((tmp (scale 1e-3 (send (send robot :foot-midcoords) :inverse-transform-vector (send bb :centroid)))))
            (float-vector (neglect-small-value (elt tmp 0) neglect-thre) (neglect-small-value (elt tmp 1) neglect-thre) 0.0)
            )))
    (if return-value
        (list bbox-size bbox-offset)
      (format nil "collision_bbox_size: ~A~%collision_bbox_offset: ~A" (eus-vector->yaml-vector-string bbox-size) (eus-vector->yaml-vector-string bbox-offset)))
    ))

(defun solve-ik-for-foostep-planning-successors
  (robot
   &key (div-xy 50.0) (div-th 10.0)
        (xmin -100.0) (xmax 200.0)
        (ymin -150.0) (ymax 100.0)
        (thmin -20.0) (thmax 20.0))
  "Solve IK for foostep-planning successors.
   div-xy, xmin, xmax, ymin, ymax are [mm].
   div-th, thmin, thmax are [deg]."
  (let* ((ret) (org-analysis-level (send-all (send *robot* :links) :analysis-level)))
    (do ((x xmin (+ x div-xy))) ((< (+ xmax *epsilon*) x))
        (do ((y ymin (+ y div-xy))) ((< (+ ymax *epsilon*) y))
            (do ((th thmin (+ th div-th))) ((< (+ thmax *epsilon*) th))
                ;; Reset robot
                (send *robot* :reset-pose)
                (send *robot* :fix-leg-to-coords (make-coords))
                (mapcar #'(lambda (l) (Send l :analysis-level :coords)) (send *robot* :links))
                (let ((ik-solved-p
                       ;; Move rleg xyth, check move-centroid-on-foot for all COM cases.
                       (prog1
                           (and (send *robot* :rleg :move-end-pos (float-vector x y 0) :world :warnp nil)
                                (send *robot* :rleg :move-end-rot th :z :world :warnp nil)
                                (send *robot* :move-centroid-on-foot :lleg '(:rleg :lleg) :warnp nil)
                                (send *robot* :move-centroid-on-foot :both '(:rleg :lleg) :warnp nil)
                                (send *robot* :move-centroid-on-foot :rleg '(:rleg :lleg) :warnp nil)
                                )
                         (mapcar #'(lambda (l al) (Send l :analysis-level al)) (send *robot* :links) org-analysis-level)
                         ))
                      (foot-collision-free-p
                       (progn
                         (send *robot* :update-descendants)
                         (= (pqp-collision-check (send *robot* :get :rleg-sole-body) (send *robot* :get :lleg-sole-body)) 0))))
                  (when (and foot-collision-free-p ik-solved-p)
                    (push (float-vector x y th) ret)
                    (send *irtviewer* :draw-objects))
                  )
                )))
    (list :successors ret :div-xy div-xy :div-th div-th :max-th (elt (find-extream ret #'(lambda (x) (abs (elt x 2))) #'>) 2))
    ))

(defun gen-successors-string
  (robot &key (return-value) (ik-results))
  "Generate successors."
  (unless ik-results
    (setq ik-results (solve-ik-for-foostep-planning-successors robot)))
  (let ((str (format nil "successors:~%")))
    (dolist (ikr (cadr (memq :successors ik-results)))
      (setq str (format nil "~A  - x: ~A~%" str (* 1e-3 (elt ikr 0))))
      (setq str (format nil "~A    y: ~A~%" str (* 1e-3 (elt ikr 1))))
      (setq str (format nil "~A    theta: ~A~%" str (deg2rad (elt ikr 2))))
      )
    (setq str (format nil "~Agoal_pos_thr: ~A~%" str (* 1e-3 (cadr (memq :div-xy ik-results)))))
    (setq str (format nil "~Agoal_rot_thr: ~A~%" str (deg2rad (cadr (memq :div-th ik-results)))))
    (setq str (format nil "~Atransition_limit_yaw: ~A~%" str (deg2rad (+ 0.1 (cadr (memq :max-th ik-results))))))
    str))

(defun gen-all-footstep-planner-parameter-string
  (robot &key (ik-results))
  "Generate all robot-dependent parameters for footstep planners."
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  (send *robot* :update-descendants)
  (format nil "~A~%~A~%~A~%~A~%~A~%"
          (gen-default_lfoot_to_rfoot_offset-string robot)
          (gen-footstep_size_xy-string robot)
          (gen-rlleg_footstep_offset-string robot)
          (gen-collision_bbox_sizeoffset-string robot)
          (gen-successors-string robot :ik-results ik-results)
          )
  )

#|
;; usage

(load "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknt-interface.l")
(hrp2jsknt)
(setq *robot* *hrp2jsknt*)
(objects (list *robot*))
;(gen-all-footstep-planner-parameter-string *robot*)

(load "irteus/demo/sample-robot-model.l")
(setq *robot* (instance sample-robot :init))
(objects (list *robot*))
(gen-all-footstep-planner-parameter-string *robot*)
|#
