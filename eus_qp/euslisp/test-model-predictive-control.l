(load "package://eus_qp/euslisp/model-predictive-control.l")

(defun gen-default-contact-constraint
  (l)
  (instance* default-contact-constraint
             :init
             :mu-trans 1.0
             :mu-rot 0.05
             (if (send *robot* :support-polygon l)
                 (let* ((vs (mapcar #'(lambda (v) (send *robot* l :end-coords :inverse-transform-vector v)) (send (send *robot* :support-polygon l) :vertices))))
                   (list :l-min-x (elt (find-extream vs #'(lambda (v) (elt v 0)) #'<) 0)
                         :l-max-x (elt (find-extream vs #'(lambda (v) (elt v 0)) #'>) 0)
                         :l-min-y (elt (find-extream vs #'(lambda (v) (elt v 1)) #'<) 1)
                         :l-max-y (elt (find-extream vs #'(lambda (v) (elt v 1)) #'>) 1)
                         ))
               (list :l-min-x 0 :l-max-x 0 :l-min-y 0 :l-max-y 0)))
  )

(defun setup ()
  ;; (load "package://hrpsys_ros_bridge_tutorials/models/hrp2jsk.l")
  ;; (unless (boundp '*hrp2jsk*)
  ;;   (objects (list (setq *robot* (hrp2jsk)))))
  (load "irteus/demo/sample-robot-model")
  (unless (boundp '*robot*)
    (objects (list (setq *robot* (instance sample-robot :init)))))
  )

(defun demo-MPC-gen-motion-1
  (&key (limbs '(:rleg :lleg))
        (foot-pos-diff (list (float-vector 0 0 0) (float-vector 0 0 0)))
        (foot-rot-diff (list (list 0 0 0) (list 0 0 0))))
  "COG XY MPC test. Move COG in X and Y axes."
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  (mapcar #'(lambda (l d)
              (send *robot* l :move-end-pos d :world))
          limbs foot-pos-diff)
  (mapcar #'(lambda (l d)
              (mapcar #'(lambda (dd a)
                          (send *robot* l :move-end-rot dd a))
                      d '(:x :y :z)))
          limbs foot-rot-diff)
  (send *robot* :fix-leg-to-coords (make-coords))
  (send *robot* :move-centroid-on-foot (if (> (length limbs) 1) :both (car limbs)) '(:rleg :lleg))
  (let* ((preview-window 10)
         (dt 0.05) ;; [s]
         (total-mass (* 1e-3 (send *robot* :weight)))
         (mg (* total-mass 1e-3 (elt *g-vec* 2)))
         (cog-z (* 1e-3 (elt (send *robot* :centroid) 2)))
         (initial-state
          (float-vector (* total-mass 1e-3 (elt (send *robot* :centroid) 0))
                        0
                        (* total-mass 1e-3 (elt (send *robot* :centroid) 1))
                        0
                        0 0))
         (mpc (instance MPC-horizontal-cog-motion-generator
                        :init preview-window
                        :initial-state initial-state))
         (prm-list) (cog-ret))
    (setq prm-list
          (append
           (mapcar #'(lambda (x)
                (instance MPC-horizontal-cog-motion-generator-param
                          :init
                          dt
                          (copy-object initial-state)
                          (mapcar #'(lambda (l) (send *robot* l :end-coords :copy-worldcoords)) limbs)
                          (mapcar #'(lambda (l) (gen-default-contact-constraint l)) limbs)
                          limbs
                          mg cog-z total-mass
                          :all-limbs limbs))
                   (make-list (round (* 1.5 preview-window))))
           (mapcar #'(lambda (x)
                (instance MPC-horizontal-cog-motion-generator-param
                          :init
                          dt
                          (v+ (float-vector (* total-mass 30 1e-3) 0 (* total-mass 20 1e-3) 0 0 0) (copy-object initial-state)) ;; Move COG
                          (mapcar #'(lambda (l) (send *robot* l :end-coords :copy-worldcoords)) limbs)
                          (mapcar #'(lambda (l) (gen-default-contact-constraint l)) limbs)
                          limbs
                          mg cog-z total-mass
                          :all-limbs limbs))
                   (make-list (round (* 2.5 preview-window))))))
    (dolist (prm prm-list)
      (let ((r (send mpc :append-param prm)))
        ;;(print (list (position prm prm-list) r))
        (when r
          (format t ";; initial state~%")
          (format t ";;   state = ~A~%" (mpc . current-state))
          (send mpc :proc-model-predictive-control)
          (push (list :cog (send mpc :get-cog-from-preview-index) :refcog (send mpc :get-ref-cog-from-preview-index)
                      :end-coords (send mpc :get-all-end-coords-from-preview-index)
                      :wrench-list (send mpc :calc-wrench-list-from-result-input-value)
                      :contact-constraint-list (send mpc :get-contact-constraint-list-from-preview-index))
                cog-ret)
          )))
    (setq cog-ret (reverse cog-ret))
    (dolist (r cog-ret)
      (send *robot* :fullbody-inverse-kinematics
            (cadr (memq :end-coords r))
            :move-target (mapcar #'(lambda (x) (send *robot* x :end-coords)) limbs)
            :link-list (mapcar #'(lambda (x) (send *robot* :link-list (send *robot* x :end-coords :parent))) limbs)
            :target-centroid-pos (cadr (memq :cog r)))
      (send *irtviewer* :draw-objects :flush nil)
      (draw-force-value r))
    (if (boundp 'gnuplot)
        (graph-view (list (mapcar #'(lambda (x) (elt (cadr (memq :cog x)) 0)) cog-ret)
                          (mapcar #'(lambda (x) (elt (cadr (memq :cog x)) 1)) cog-ret)
                          (mapcar #'(lambda (x) (elt (cadr (memq :refcog x)) 0)) cog-ret)
                          (mapcar #'(lambda (x) (elt (cadr (memq :refcog x)) 1)) cog-ret))))
    cog-ret))

(defun demo-MPC-gen-motion-2 ()
  "COG-Z MPC test. Vertical jumping."
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  (send *robot* :move-centroid-on-foot :both '(:rleg :lleg))
  (let* ((limbs '(:rleg :lleg))
         (preview-window 10)
         (dt 0.05) ;; [s]
         (total-mass (* 1e-3 (send *robot* :weight)))
         (initial-state (float-vector (* total-mass 1e-3 (elt (send *robot* :centroid) 2)) 0))
         (mpc (instance MPC-vertical-cog-motion-generator
                        :init preview-window
                        :initial-state initial-state))
         (contact-constraint-list
          (mapcar #'(lambda (l)
                      (instance* default-contact-constraint
                                 :init
                                 :mu-trans 1.0
                                 :mu-rot 0.05
                                 (let* ((vs (mapcar #'(lambda (v) (send *robot* l :end-coords :inverse-transform-vector v)) (send (send *robot* :support-polygon l) :vertices))))
                                   (list :l-min-x (elt (find-extream vs #'(lambda (v) (elt v 0)) #'<) 0)
                                         :l-max-x (elt (find-extream vs #'(lambda (v) (elt v 0)) #'>) 0)
                                         :l-min-y (elt (find-extream vs #'(lambda (v) (elt v 1)) #'<) 1)
                                         :l-max-y (elt (find-extream vs #'(lambda (v) (elt v 1)) #'>) 1)
                                         ))))
                  limbs))
         (no-contact-costraint-list
          (mapcar #'(lambda (l)
                      (instance no-contact-constraint :init))
                  limbs))
         (count -1)
         (prm-list) (cog-ret))
    (setq prm-list
          (append
           ;; on the ground
           (mapcar #'(lambda (x)
                       (instance MPC-vertical-cog-motion-generator-param
                                 :init
                                 dt
                                 (copy-object initial-state)
                                 (mapcar #'(lambda (l) (send *robot* l :end-coords :copy-worldcoords)) limbs)
                                 contact-constraint-list limbs
                                 total-mass
                                 :all-limbs limbs
                                 ))
                   (make-list (round (* 1.5 preview-window))))
           ;; in the air
           (mapcar #'(lambda (x)
                       (instance MPC-vertical-cog-motion-generator-param
                                 :init
                                 dt
                                 (copy-object initial-state)
                                 (mapcar #'(lambda (l) (send *robot* l :end-coords :copy-worldcoords)) limbs)
                                 nil nil
                                 total-mass
                                 :all-limbs limbs
                                 ))
                   (make-list 6))
           ;; on the ground
           (mapcar #'(lambda (x)
                       (instance MPC-vertical-cog-motion-generator-param
                                 :init
                                 dt
                                 (copy-object initial-state)
                                 (mapcar #'(lambda (l) (send *robot* l :end-coords :copy-worldcoords)) limbs)
                                 contact-constraint-list limbs
                                 total-mass
                                 :all-limbs limbs
                                 ))
                   (make-list (round (* 1.5 preview-window))))))
    (dolist (prm prm-list)
      (let ((r (send mpc :append-param prm)))
        ;;(print (list (position prm prm-list) r))
        (when r
          (format t ";; initial state~%")
          (format t ";;   state = ~A~%" (mpc . current-state))
          (send mpc :proc-model-predictive-control)
          (push (list :cog (float-vector (elt (send *robot* :centroid nil) 0) (elt (send *robot* :centroid nil) 1) (send mpc :get-cog-z-from-preview-index))
                      :refcog (float-vector (elt (send *robot* :centroid nil) 0) (elt (send *robot* :centroid nil) 1) (send mpc :get-ref-cog-z-from-preview-index))
                      :end-coords (send mpc :get-all-end-coords-from-preview-index)
                      :wrench-list (send mpc :calc-wrench-list-from-result-input-value)
                      :contact-constraint-list (send mpc :get-contact-constraint-list-from-preview-index))
                cog-ret)
          )))
    (setq cog-ret (reverse cog-ret))
    (dolist (r cog-ret)
      (send *robot* :translate (v- (cadr (memq :cog r)) (send *robot* :centroid)) :world)
      (send *robot* :fullbody-inverse-kinematics
            (cadr (memq :end-coords r))
            :move-target (mapcar #'(lambda (x) (send *robot* x :end-coords)) limbs)
            :link-list (mapcar #'(lambda (x) (send *robot* :link-list (send *robot* x :end-coords :parent))) limbs)
            :target-centroid-pos (cadr (memq :cog r)))
      (send *irtviewer* :draw-objects :flush nil)
      (draw-force-value r)
      )
    (if (boundp 'gnuplot)
        (graph-view (list (mapcar #'(lambda (x) (elt (cadr (memq :cog x)) 2)) cog-ret)
                          (mapcar #'(lambda (x) (elt (cadr (memq :refcog x)) 2)) cog-ret))))
    cog-ret))

(defun demo-MPC-gen-motion-3
  (&key (limbs '(:rleg :lleg))
        (foot-pos-diff (list (float-vector 0 0 0) (float-vector 0 0 0)))
        (foot-rot-diff (list (list 0 0 0) (list 0 0 0))))
  "COG XY MPC test. Walking by changing contact constraints"
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  (mapcar #'(lambda (l d)
              (send *robot* l :move-end-pos d :world))
          limbs foot-pos-diff)
  (mapcar #'(lambda (l d)
              (mapcar #'(lambda (dd a)
                          (send *robot* l :move-end-rot dd a))
                      d '(:x :y :z)))
          limbs foot-rot-diff)
  (send *robot* :fix-leg-to-coords (make-coords))
  (send *robot* :move-centroid-on-foot (if (> (length limbs) 1) :both (car limbs)) '(:rleg :lleg))
  (let* ((limbs '(:rleg :lleg))
         (preview-window 10)
         (dt 0.05) ;; [s]
         (total-mass (* 1e-3 (send *robot* :weight)))
         (mg (* total-mass 1e-3 (elt *g-vec* 2)))
         (cog-z (* 1e-3 (elt (send *robot* :centroid) 2)))
         (initial-state
          (float-vector (* total-mass 1e-3 (elt (send *robot* :centroid) 0))
                        0
                        (* total-mass 1e-3 (elt (send *robot* :centroid) 1))
                        0
                        0 0))
         (mpc (instance MPC-horizontal-cog-motion-generator
                        :init preview-window
                        :initial-state initial-state))
         (prm-list) (cog-ret)
         (count -1))
    (setq prm-list
          (append
           (mapcar #'(lambda (x)
                       (instance MPC-horizontal-cog-motion-generator-param
                                 :init
                                 dt
                                 (copy-object initial-state)
                                 (mapcar #'(lambda (l) (send *robot* l :end-coords :copy-worldcoords)) limbs)
                                 (list (gen-default-contact-constraint :rleg) (gen-default-contact-constraint :lleg))
                                 limbs
                                 mg cog-z total-mass
                                 :all-limbs limbs
                                 ))
                   (make-list (* 1 preview-window)))
           (mapcar #'(lambda (x)
                       (instance MPC-horizontal-cog-motion-generator-param
                                 :init
                                 dt
                                 (v+ (float-vector 0 0 (* total-mass 1e-3 (elt (send *robot* :rleg :end-coords :worldpos) 1)) 0 0 0) (copy-object initial-state))
                                 (mapcar #'(lambda (l) (send *robot* l :end-coords :copy-worldcoords)) limbs)
                                 (list (gen-default-contact-constraint :rleg))
                                 (list :rleg)
                                 mg cog-z total-mass
                                 :all-limbs limbs
                                 ))
                   (make-list (round (* 1.6 preview-window))))
           (mapcar #'(lambda (x)
                       (instance MPC-horizontal-cog-motion-generator-param
                                 :init
                                 dt
                                 (v+ (float-vector 0 0 (* total-mass 1e-3 (elt (send *robot* :lleg :end-coords :worldpos) 1)) 0 0 0) (copy-object initial-state))
                                 (mapcar #'(lambda (l) (send *robot* l :end-coords :copy-worldcoords)) limbs)
                                 (list (gen-default-contact-constraint :lleg))
                                 (list :lleg)
                                 mg cog-z total-mass
                                 :all-limbs limbs
                                 ))
                   (make-list (round (* 1.6 preview-window))))
           (mapcar #'(lambda (x)
                       (instance MPC-horizontal-cog-motion-generator-param
                                 :init
                                 dt
                                 (copy-object initial-state)
                                 (mapcar #'(lambda (l) (send *robot* l :end-coords :copy-worldcoords)) limbs)
                                 (list (gen-default-contact-constraint :rleg) (gen-default-contact-constraint :lleg))
                                 limbs
                                 mg cog-z total-mass
                                 :all-limbs limbs
                                 ))
                   (make-list (* 3 preview-window)))))
    (dolist (prm prm-list)
      (let ((r (send mpc :append-param prm)))
        ;;(print (list (position prm prm-list) r))
        (when r
          (format t ";; initial state~%")
          (format t ";;   state = ~A~%" (mpc . current-state))
          (send mpc :proc-model-predictive-control)
          (format t ";;   input = ~A~%" (elt (send mpc :input-value-list) 0))
          (push (list :cog (send mpc :get-cog-from-preview-index) :refcog (send mpc :get-ref-cog-from-preview-index)
                      :end-coords (send mpc :get-all-end-coords-from-preview-index)
                      :wrench-list (send mpc :calc-wrench-list-from-result-input-value)
                      :contact-constraint-list (send mpc :get-contact-constraint-list-from-preview-index))
                cog-ret)
          )))
    (setq cog-ret (reverse cog-ret))
    (dolist (r cog-ret)
      (send *robot* :fullbody-inverse-kinematics
            (cadr (memq :end-coords r))
            :move-target (mapcar #'(lambda (x) (send *robot* x :end-coords)) limbs)
            :link-list (mapcar #'(lambda (x) (send *robot* :link-list (send *robot* x :end-coords :parent))) limbs)
            :target-centroid-pos (cadr (memq :cog r)))
      (send *irtviewer* :draw-objects :flush nil)
      (draw-force-value r))
    (if (boundp 'gnuplot)
        (graph-view (list (mapcar #'(lambda (x) (elt (cadr (memq :cog x)) 0)) cog-ret)
                          (mapcar #'(lambda (x) (elt (cadr (memq :cog x)) 1)) cog-ret)
                          (mapcar #'(lambda (x) (elt (cadr (memq :refcog x)) 0)) cog-ret)
                          (mapcar #'(lambda (x) (elt (cadr (memq :refcog x)) 1)) cog-ret))))
    cog-ret))

(defun demo-MPC-gen-motion-4
  (&key (limbs '(:rleg :lleg :rarm :larm))
        (ee-pos-diff (list (float-vector 0 0 0) (float-vector 0 0 0)))
        (ee-rot-diff (list (list 0 0 0) (list 0 0 0))))
  "COG XY MPC test. Walking by changing contact constraints trot 4leg."
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  (send *robot* :move-coords
        (make-coords :rpy (list 0 pi/2 0)
                     :pos (float-vector 0 0 300))
        (car (send *robot* :links)))
  (setq tc (list
            (make-coords :pos #f(-200 -120 0))
            (make-coords :pos #f(-200 120 0))
            ;; (make-coords :pos #f(200 -120 0) :rpy (list 0 pi/2 0))
            ;; (make-coords :pos #f(200 120 0) :rpy (list 0 pi/2 0))
            (make-coords :pos #f(200 -120 0) :rpy (list 0 0 0))
            (make-coords :pos #f(200 120 0) :rpy (list 0 0 0))
            ))
  (send *robot* :fullbody-inverse-kinematics
        tc
        :joint-args '(:absolute-p t)
        :min (float-vector -1e10 -1e10 -1e10 -180 -180 -180)
        :max (float-vector  1e10  1e10  1e10  180  180  180)
        :move-target (mapcar #'(lambda (l) (send *robot* l :end-coords)) limbs)
        :link-list (mapcar #'(lambda (l) (send *robot* :link-list (send *robot* l :end-coords :parent))) limbs)
        :target-centroid-pos (vector-mean (send-all tc :worldpos)))
  (let* ((preview-window 10)
         (dt 0.05) ;; [s]
         (total-mass (* 1e-3 (send *robot* :weight)))
         (mg (* total-mass 1e-3 (elt *g-vec* 2)))
         (cog-z (* 1e-3 (elt (send *robot* :centroid) 2)))
         (initial-state
          (float-vector (* total-mass 1e-3 (elt (send *robot* :centroid) 0))
                        0
                        (* total-mass 1e-3 (elt (send *robot* :centroid) 1))
                        0
                        0 0))
         (mpc (instance MPC-horizontal-cog-motion-generator
                        :init preview-window
                        :initial-state initial-state))
         (prm-list) (cog-ret)
         (count -1))
    (setq prm-list
          (append
           (mapcar #'(lambda (x)
                       (instance MPC-horizontal-cog-motion-generator-param
                                 :init
                                 dt
                                 (copy-object initial-state)
                                 (mapcar #'(lambda (l) (send *robot* l :end-coords :copy-worldcoords)) limbs)
                                 (mapcar #'(lambda (l) (gen-default-contact-constraint l)) limbs)
                                 limbs
                                 mg cog-z total-mass
                                 :all-limbs limbs
                                 ))
                   (make-list (* 1 preview-window)))
           (mapcar #'(lambda (x)
                       (instance MPC-horizontal-cog-motion-generator-param
                                 :init
                                 dt
                                 (v+ (float-vector 0 0 (* total-mass 1e-3 (elt (send *robot* :rleg :end-coords :worldpos) 1)) 0 0 0) (copy-object initial-state))
                                 (mapcar #'(lambda (l) (send *robot* l :end-coords :copy-worldcoords)) limbs)
                                 (list (gen-default-contact-constraint :rleg) (gen-default-contact-constraint :larm))
                                 (list :rleg :larm)
                                 mg cog-z total-mass
                                 :all-limbs limbs
                                 ))
                   (make-list (round (* 1.6 preview-window))))
           (mapcar #'(lambda (x)
                       (instance MPC-horizontal-cog-motion-generator-param
                                 :init
                                 dt
                                 (v+ (float-vector 0 0 (* total-mass 1e-3 (elt (send *robot* :lleg :end-coords :worldpos) 1)) 0 0 0) (copy-object initial-state))
                                 (mapcar #'(lambda (l) (send *robot* l :end-coords :copy-worldcoords)) limbs)
                                 (list (gen-default-contact-constraint :lleg) (gen-default-contact-constraint :rarm))
                                 (list :lleg :rarm)
                                 mg cog-z total-mass
                                 :all-limbs limbs
                                 ))
                   (make-list (round (* 1.6 preview-window))))
           (mapcar #'(lambda (x)
                       (instance MPC-horizontal-cog-motion-generator-param
                                 :init
                                 dt
                                 (v+ (float-vector 0 0 (* total-mass 1e-3 (elt (send *robot* :rleg :end-coords :worldpos) 1)) 0 0 0) (copy-object initial-state))
                                 (mapcar #'(lambda (l) (send *robot* l :end-coords :copy-worldcoords)) limbs)
                                 (list (gen-default-contact-constraint :rleg) (gen-default-contact-constraint :larm))
                                 (list :rleg :larm)
                                 mg cog-z total-mass
                                 :all-limbs limbs
                                 ))
                   (make-list (round (* 1.6 preview-window))))
           (mapcar #'(lambda (x)
                       (instance MPC-horizontal-cog-motion-generator-param
                                 :init
                                 dt
                                 (copy-object initial-state)
                                 (mapcar #'(lambda (l) (send *robot* l :end-coords :copy-worldcoords)) limbs)
                                 (mapcar #'(lambda (l) (gen-default-contact-constraint l)) limbs)
                                 limbs
                                 mg cog-z total-mass
                                 :all-limbs limbs
                                 ))
                   (make-list (* 2 preview-window)))))
    (dolist (prm prm-list)
      (let ((r (send mpc :append-param prm)))
        ;;(print (list (position prm prm-list) r))
        (when r
          (format t ";; initial state~%")
          (format t ";;   state = ~A~%" (mpc . current-state))
          (send mpc :proc-model-predictive-control)
          (format t ";;   input = ~A~%" (elt (send mpc :input-value-list) 0))
          (push (list :cog (send mpc :get-cog-from-preview-index) :refcog (send mpc :get-ref-cog-from-preview-index)
                      :end-coords (send mpc :get-all-end-coords-from-preview-index)
                      :wrench-list (send mpc :calc-wrench-list-from-result-input-value)
                      :contact-constraint-list (send mpc :get-contact-constraint-list-from-preview-index))
                cog-ret)
          )))
    (setq cog-ret (reverse cog-ret))
    (dolist (r cog-ret)
      (send *robot* :fullbody-inverse-kinematics
            (cadr (memq :end-coords r))
            :move-target (mapcar #'(lambda (x) (send *robot* x :end-coords)) limbs)
            :link-list (mapcar #'(lambda (x) (send *robot* :link-list (send *robot* x :end-coords :parent))) limbs)
            :target-centroid-pos (cadr (memq :cog r)))
      (send *irtviewer* :draw-objects :flush nil)
      (draw-force-value r)
      )
    (if (boundp 'gnuplot)
        (graph-view (list (mapcar #'(lambda (x) (elt (cadr (memq :cog x)) 0)) cog-ret)
                          (mapcar #'(lambda (x) (elt (cadr (memq :cog x)) 1)) cog-ret)
                          (mapcar #'(lambda (x) (elt (cadr (memq :refcog x)) 0)) cog-ret)
                          (mapcar #'(lambda (x) (elt (cadr (memq :refcog x)) 1)) cog-ret))))
    cog-ret))

(defun get-max-id-for-demo-functions
  (demo-function-sym)
  (let ((demo-ids (mapcar #'(lambda (x) (let ((str (string-left-trim (string-upcase demo-function-sym) (string x)))) (unless (string= "" str) (read-from-string str)))) (apropos-list demo-function-sym))))
    (apply #'max (remove-if-not #'numberp demo-ids))))

(warn ";; MPC motion gen demos~%")
(dotimes (i (get-max-id-for-demo-functions 'demo-mpc-gen-motion-))
  (warn ";;   (demo-mpc-gen-motion-~d) ;; ~A~%" (1+ i)
        (documentation (read-from-string (format nil "demo-mpc-gen-motion-~d" (1+ i))))))

(defun demo-mpc-gen-motion-all
  ()
  (let ((ret))
    (dotimes (i (get-max-id-for-demo-functions 'demo-mpc-gen-motion-))  ;; Get max number of demo function
      (format t ";; demo-mpc-gen-motion-~d~%" (1+ i))
      (push (funcall (eval (read-from-string (format nil "#'demo-mpc-gen-motion-~d" (1+ i))))) ret)
      )
    ret))
(warn ";;   (demo-mpc-gen-motion-all)~%")

(defun test-predictive-matrices-common
  (Ak-list Bk-list
   init-value input-value-list
   mpc &optional (drift-coeff-list))
  (let ((state-value init-value)
        (ret) (ret1) (ret0))
    (mapcar #'(lambda (ak bk)
                (send mpc :append-param
                      (instance model-predictive-control-param
                                :init nil nil Ak Bk
                                nil nil nil nil nil)))
            Ak-list Bk-list)
    (dotimes (i (length Ak-list))
      (setq state-value (v+ (transform (elt Ak-list i) state-value) (transform (elt Bk-list i) (elt input-value-list i))))
      (if drift-coeff-list
          (setq state-value (v+ state-value (elt drift-coeff-list i))))
      (push state-value ret)
      )
    (setq ret0 (apply #'concatenate float-vector (reverse ret)))
    (setq ret1 (v+
                (transform (send mpc :calc-phi-matrix Ak-list) init-value)
                (transform (send mpc :calc-psi-matrix Ak-list Bk-list) (apply #'concatenate float-vector input-value-list))
                ))
    (if drift-coeff-list
        (setq ret1 (v+ ret1 (transform (send mpc :calc-lambda-matrix Ak-list) (apply #'concatenate float-vector drift-coeff-list)))))
    ;;(format-array ret0)
    ;;(format-array ret1)
    (format-array (v- ret0 ret1) ";;   diff")
    (format t ";;   same? ~A~%" (eps= (distance ret0 ret1) 0.0))
    (eps= (distance ret0 ret1) 0.0)))

(defun test-predictive-matrices-1
  ()
  "Test fixed size and values. Linear time-invariant system."
  (let* ((mpc (instance model-predictive-control :init 4 3))
         (Ak-list (list (scale-matrix 0.5 (unit-matrix 3)) (scale-matrix 1 (unit-matrix 3)) (scale-matrix 1.5 (unit-matrix 3)) (scale-matrix 2 (unit-matrix 3))))
         (Bk-list (list (scale-matrix 0.1 (unit-matrix 3)) (scale-matrix 0.2 (unit-matrix 3)) (scale-matrix 0.3 (unit-matrix 3)) (scale-matrix 0.4 (unit-matrix 3))))
         (input-value-list (list (float-vector -1 -2 -3) (float-vector -4 -5 -6) (float-vector -7 -8 -9) (float-vector -10 -11 -12))))
    (format t ";; ~A~%" (documentation 'test-predictive-matrices-0))
    (test-predictive-matrices-common
     Ak-list Bk-list (mpc . current-state) input-value-list mpc)
    ))

(defun test-predictive-matrices-2
  ()
  "Test random size and values. Linear time-invariant system."
  (let* ((state-dim (1+ (random 5)))
         (input-dim (1+ (random 3)))
         (predict-len (1+ (random 10)))
         (initial-state (make-random-vector state-dim :random-range 1.0))
         (mpc (instance model-predictive-control :init predict-len state-dim :initial-state initial-state))
         (tmpAk (make-random-matrix state-dim state-dim :random-range 5.0))
         (Ak-list (mapcar #'(lambda (x)
                              (copy-object tmpAk))
                          (make-list predict-len)))
         (tmpBk (make-random-matrix state-dim input-dim :random-range 1.0))
         (Bk-list (mapcar #'(lambda (x)
                              (copy-object tmpBk))
                          (make-list predict-len)))
         (input-value-list (mapcar #'(lambda (x)
                                       (make-random-vector input-dim :random-range 1.0))
                                   (make-list predict-len))))
    (format t ";; ~A~%" (documentation 'test-predictive-matrices-1))
    (test-predictive-matrices-common
     Ak-list Bk-list (mpc . current-state) input-value-list mpc)
    ))

(defun test-predictive-matrices-3
  ()
  "Test random size and values. Linear time-variant system."
  (let* ((state-dim (1+ (random 5)))
         (input-dim (1+ (random 3)))
         (predict-len (1+ (random 10)))
         (initial-state (make-random-vector state-dim :random-range 1.0))
         (mpc (instance model-predictive-control :init predict-len state-dim :initial-state initial-state))
         (Ak-list (mapcar #'(lambda (x)
                              (make-random-matrix state-dim state-dim :random-range 5.0))
                          (make-list predict-len)))
         (Bk-list (mapcar #'(lambda (x)
                              (make-random-matrix state-dim input-dim :random-range 1.0))
                          (make-list predict-len)))
         (input-value-list (mapcar #'(lambda (x)
                                       (make-random-vector input-dim :random-range 1.0))
                                   (make-list predict-len))))
    (format t ";; ~A~%" (documentation 'test-predictive-matrices-2))
    (test-predictive-matrices-common
     Ak-list Bk-list (mpc . current-state) input-value-list mpc)
    ))

(defun test-predictive-matrices-4
  ()
  "Test random size and values. Linear time-variant system with drift coeff."
  (let* ((state-dim (1+ (random 5)))
         (input-dim (1+ (random 3)))
         (predict-len (1+ (random 10)))
         (initial-state (make-random-vector state-dim :random-range 1.0))
         (mpc (instance model-predictive-control :init predict-len state-dim :initial-state initial-state))
         (Ak-list (mapcar #'(lambda (x)
                              (make-random-matrix state-dim state-dim :random-range 5.0))
                          (make-list predict-len)))
         (Bk-list (mapcar #'(lambda (x)
                              (make-random-matrix state-dim input-dim :random-range 1.0))
                          (make-list predict-len)))
         (input-value-list (mapcar #'(lambda (x)
                                       (make-random-vector input-dim :random-range 1.0))
                                   (make-list predict-len)))
         (drift-coeff-list (mapcar #'(lambda (x)
                                       (make-random-vector state-dim :random-range 1.0))
                                   (make-list predict-len))))
    (format t ";; ~A~%" (documentation 'test-predictive-matrices-3))
    (test-predictive-matrices-common
     Ak-list Bk-list (mpc . current-state) input-value-list mpc drift-coeff-list)
    ))

(warn ";; MPC check funcs~%")
(dotimes (i (get-max-id-for-demo-functions 'test-predictive-matrices-)) ;; Get max number of demo function
  (warn ";;   (test-predictive-matrices-~d) ;; ~A~%" (1+ i)
        (documentation (read-from-string (format nil "test-predictive-matrices-~d" (1+ i))))))

(defun test-predictive-matrices-all
  ()
  (let ((ret))
    (dotimes (i (get-max-id-for-demo-functions 'test-predictive-matrices-))  ;; Get max number of demo function
      (format t ";; test-predictive-matrices-~d~%" (1+ i))
      (push (funcall (eval (read-from-string (format nil "#'test-predictive-matrices-~d" (1+ i))))) ret)
      )
    ret))
(warn ";;   (test-predictive-matrices-all)~%")

(defun make-random-matrix
  (row-dim column-dim &key (random-range 1e10))
  "Make matrix with given row dimension and column dimension.
   Component of matrix is random value."
  (make-matrix row-dim column-dim
               (mapcar #'(lambda (row)
                           (mapcar #'(lambda (column) (random random-range)) (make-list column-dim)))
                       (make-list row-dim))))

(defun make-random-vector
  (dim &key (random-range 1e10))
  "Make vector with given length.
   Component of vector is random value."
  (concatenate float-vector (mapcar #'(lambda (dim) (random random-range)) (make-list dim)))
  )

(defmacro with-set-color-line-width ;; just for coloring
  (vw color line-width &rest bodies)
  (with-gensyms
   (prev-color prev-width)
   `(let* ((,prev-color (send ,vw :viewsurface :color))
           (,prev-width (send ,vw :viewsurface :line-width)))
      (unwind-protect
          (progn
            (send ,vw :viewsurface :color ,color)
            (send ,vw :viewsurface :line-width ,line-width)
            ,@bodies)
        (send ,vw :viewsurface :color ,prev-color)
        (send ,vw :viewsurface :line-width ,prev-width)
        ))))

(defun draw-force-value
  (rs-list
   &key (robot *robot*))
  ;; draw contact wrench
  (send-all (cadr (memq :contact-constraint-list rs-list)) :draw-on :flush nil)
  (mapcar
   #'(lambda (f m cc)
       ;; draw cop
       (let* ((cop (send robot :calc-cop-from-force-moment ;; dummy using of robot
                         f m cc cc)))
         (when cop (send cop :draw-on :flush nil :color #F(1 0 0) :width 4 :size 30))))
   (car (cadr (memq :wrench-list rs-list))) (cadr (cadr (memq :wrench-list rs-list))) (cadr (memq :end-coords rs-list)))
  (mapcar
   #'(lambda (f m cc)
       ;; draw force
       (with-set-color-line-width
        *viewer* #F(1 0.3 1) 5
        (send *irtviewer* :viewer :draw-arrow
              (send cc :worldpos)
              (v+ (send cc :worldpos) (scale 1 f)))))
   (car (cadr (memq :wrench-list rs-list))) (cadr (cadr (memq :wrench-list rs-list))) (cadr (memq :end-coords rs-list)))
  (send *irtviewer* :viewer :viewsurface :flush)
  )

