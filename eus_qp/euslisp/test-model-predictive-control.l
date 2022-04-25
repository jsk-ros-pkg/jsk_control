(load "package://eus_qp/euslisp/model-predictive-control.l")

(defun gen-default-contact-constraint
  (l &key (mu-trans 1.0) (mu-rot 0.05) (cop-margin-ratio 1.0))
  (instance* default-contact-constraint
             :init
             :name l
             :mu-trans mu-trans
             :mu-rot mu-rot
             (if (send *robot* :support-polygon l)
                 (let* ((vs (mapcar #'(lambda (v) (send *robot* l :end-coords :inverse-transform-vector v)) (send (send *robot* :support-polygon l) :vertices))))
                   (list :l-min-x (* cop-margin-ratio (elt (find-extream vs #'(lambda (v) (elt v 0)) #'<) 0))
                         :l-max-x (* cop-margin-ratio (elt (find-extream vs #'(lambda (v) (elt v 0)) #'>) 0))
                         :l-min-y (* cop-margin-ratio (elt (find-extream vs #'(lambda (v) (elt v 1)) #'<) 1))
                         :l-max-y (* cop-margin-ratio (elt (find-extream vs #'(lambda (v) (elt v 1)) #'>) 1))
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
        (foot-rot-diff (list (list 0 0 0) (list 0 0 0)))
        ((:receding-horizon-proc-count rhpc) 1))
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
                        :initial-state initial-state
                        :receding-horizon-proc-count rhpc))
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
    (if (and (boundp 'gnuplot) (/= x::*display* 0))
        (graph-view (list (mapcar #'(lambda (x) (elt (cadr (memq :cog x)) 0)) cog-ret)
                          (mapcar #'(lambda (x) (elt (cadr (memq :cog x)) 1)) cog-ret)
                          (mapcar #'(lambda (x) (elt (cadr (memq :refcog x)) 0)) cog-ret)
                          (mapcar #'(lambda (x) (elt (cadr (memq :refcog x)) 1)) cog-ret))))
    cog-ret))

(defun demo-MPC-gen-motion-2
  (&key ((:receding-horizon-proc-count rhpc) 1))
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
                        :initial-state initial-state
                        :receding-horizon-proc-count rhpc))
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
    (if (and (boundp 'gnuplot) (/= x::*display* 0))
        (graph-view (list (mapcar #'(lambda (x) (elt (cadr (memq :cog x)) 2)) cog-ret)
                          (mapcar #'(lambda (x) (elt (cadr (memq :refcog x)) 2)) cog-ret))))
    cog-ret))

(defun demo-MPC-gen-motion-3
  (&key (limbs '(:rleg :lleg))
        (foot-pos-diff (list (float-vector 0 0 0) (float-vector 0 0 0)))
        (foot-rot-diff (list (list 0 0 0) (list 0 0 0)))
        ((:receding-horizon-proc-count rhpc) 1))
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
                        :initial-state initial-state
                        :receding-horizon-proc-count rhpc))
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
    (if (and (boundp 'gnuplot) (/= x::*display* 0))
        (graph-view (list (mapcar #'(lambda (x) (elt (cadr (memq :cog x)) 0)) cog-ret)
                          (mapcar #'(lambda (x) (elt (cadr (memq :cog x)) 1)) cog-ret)
                          (mapcar #'(lambda (x) (elt (cadr (memq :refcog x)) 0)) cog-ret)
                          (mapcar #'(lambda (x) (elt (cadr (memq :refcog x)) 1)) cog-ret))))
    cog-ret))

(defun demo-MPC-gen-motion-4
  (&key (limbs '(:rleg :lleg :rarm :larm))
        (ee-pos-diff (list (float-vector 0 0 0) (float-vector 0 0 0)))
        (ee-rot-diff (list (list 0 0 0) (list 0 0 0)))
        ((:receding-horizon-proc-count rhpc) 1))
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
                        :initial-state initial-state
                        :receding-horizon-proc-count rhpc))
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
    (if (and (boundp 'gnuplot) (/= x::*display* 0))
        (graph-view (list (mapcar #'(lambda (x) (elt (cadr (memq :cog x)) 0)) cog-ret)
                          (mapcar #'(lambda (x) (elt (cadr (memq :cog x)) 1)) cog-ret)
                          (mapcar #'(lambda (x) (elt (cadr (memq :refcog x)) 0)) cog-ret)
                          (mapcar #'(lambda (x) (elt (cadr (memq :refcog x)) 1)) cog-ret))))
    cog-ret))

(defun demo-MPC-gen-motion-5
  (&key (limbs '(:rleg :lleg))
        (foot-pos-diff (list (float-vector 0 0 0) (float-vector 0 0 0)))
        (foot-rot-diff (list (list 0 0 0) (list 0 0 0)))
        (dt 0.05) ;; [s]
        (default-step-time 1.0) ;; [s]
        (default-step-height 50.0) ;; [mm]
        (double-support-ratio 0.2) ;; [0, 1]
        (mu-trans 0.2) (cop-margin-ratio 1.0)
        (fs-list)
        ((:receding-horizon-proc-count rhpc) 1))
  "COG XY MPC test. Walking by footstep list. Please set :fs-list argument."
  ;; Set initial pose
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
  (unless fs-list
    ;;(Setq fs-list (send *robot* :go-pos-params->footstep-list 200 0 0)))
    (Setq fs-list (send *robot* :go-pos-params->footstep-list 100 0 0)))
  (let* ((limbs '(:rleg :lleg))
         (preview-window 10)
         (total-mass (* 1e-3 (send *robot* :weight))) ;; [kg]
         (mg (* total-mass 1e-3 (elt *g-vec* 2))) ;; [N]
         (cog-z (* 1e-3 (elt (send *robot* :centroid) 2))) ;; [m]
         (tmp-gait-generator (instance gait-generator :init *robot* dt))
         (support-limb-list (send-all (butlast fs-list) :name))
         (support-coords-list (send-all (butlast fs-list) :copy-worldcoords))
         (swing-dst-coords-list (send-all (subseq fs-list 1) :copy-worldcoords))
         (swing-src-coords-list (append (list (send (send *robot* (if (eq (car support-limb-list) :rleg) :lleg :rleg) :end-coords) :copy-worldcoords))
                                        (send-all (butlast fs-list 2) :copy-worldcoords)))
         (half-double-support-ratio (* 0.5 double-support-ratio))
         (time-count (/ default-step-time dt))
         (prm-list) (cog-ret) (ret))
    (labels ((COM-pos->state
              (com total-mass)
              (float-vector (* total-mass 1e-3 (elt com 0)) 0
                            (* total-mass 1e-3 (elt com 1)) 0
                            0 0))
             (counter-foot-step
              (fs &key (double-support-p nil))
              (send (send fs :copy-worldcoords) :translate
                    (scale (* (if double-support-p 1.0 2.0)
                              (if (eq (send fs :name) :rleg) 1.0 -1.0))
                           (cadr (memq :default-half-offset (send *robot* :footstep-parameter))))))
             (calc-double-support-param
              (state end-coords-list)
              (instance MPC-horizontal-cog-motion-generator-param
                        :init
                        dt
                        state
                        end-coords-list
                        (mapcar #'(lambda (l) (gen-default-contact-constraint l :mu-trans mu-trans :cop-margin-ratio cop-margin-ratio)) limbs)
                        limbs
                        mg cog-z total-mass
                        :all-limbs limbs)))
      ;; Print
      (mapcar #'(lambda (l sp swd sws)
                  (format t ";; sp ~A, spc ~A, sw ~A, swc ~A->~A~%"
                          l (send sp :worldpos)
                          (if (eq l :rleg) :lleg :rleg) (send sws :worldpos) (send swd :worldpos)))
              support-limb-list support-coords-list
              swing-dst-coords-list swing-src-coords-list)
      ;; Generate param list
      (setq prm-list
            (append
             ;; Initial double support phase
             (mapcar #'(lambda (x)
                         (calc-double-support-param
                          (com-pos->state (send (counter-foot-step (car fs-list) :double-support-p t) :worldpos) total-mass)
                          (mapcar #'(lambda (l) (send *robot* l :end-coords :copy-worldcoords)) limbs)
                          ))
                     (make-list (round time-count)))
             ;; 2 ~ pre-last foot steps
             (apply
              #'append
              (mapcar
               #'(lambda (spl spc swd sws)
                   (append
                    ;; Start double support
                    (mapcar #'(lambda (x)
                                (calc-double-support-param
                                 (com-pos->state (midpoint 0.5 (send spc :worldpos) (send sws :worldpos)) total-mass)
                                 (mapcar #'(lambda (l) (if (eq l spl) spc sws)) limbs)
                                 ))
                            (make-list (round (* half-double-support-ratio time-count))))
                    ;; Swing phase
                    (let ((count -1) (time-count2 (* (- 1.0 double-support-ratio) time-count)))
                      (mapcar #'(lambda (x)
                                  (incf count)
                                  (instance MPC-horizontal-cog-motion-generator-param
                                            :init
                                            dt
                                            (com-pos->state (send spc :worldpos) total-mass)
                                            (mapcar #'(lambda (l)
                                                        (if (eq l spl)
                                                            spc
                                                          (send tmp-gait-generator
                                                                :cycloid-midcoords
                                                                (/ count time-count2)
                                                                sws swd default-step-height)))
                                                    limbs)
                                            (list (gen-default-contact-constraint spl :mu-trans mu-trans :cop-margin-ratio cop-margin-ratio))
                                            (list spl)
                                            mg cog-z total-mass
                                            :all-limbs limbs
                                            ))
                              (make-list (round time-count2))))
                    ;; End double support
                    (mapcar #'(lambda (x)
                                (calc-double-support-param
                                 (com-pos->state (midpoint 0.5 (send spc :worldpos) (send swd :worldpos)) total-mass)
                                 (mapcar #'(lambda (l) (if (eq l spl) spc swd)) limbs)
                                 ))
                            (make-list (round (* half-double-support-ratio time-count))))))
               support-limb-list
               support-coords-list
               swing-dst-coords-list
               swing-src-coords-list))
             ;; Final double support phase
             (mapcar #'(lambda (x)
                         (calc-double-support-param
                          (com-pos->state (send (counter-foot-step (car (last fs-list)) :double-support-p t) :worldpos) total-mass)
                          (mapcar #'(lambda (l)
                                      (if (eq (send (car (last fs-list)) :name) l)
                                          (send (car (last fs-list)) :copy-worldcoords)
                                        (counter-foot-step (car (last fs-list)))))
                                  limbs)
                          ))
                     (make-list (round (* 2 time-count))))))
      ;; MPC
      (let ((tm 0.0)
            (mpc (instance MPC-horizontal-cog-motion-generator
                           :init preview-window
                           :initial-state (com-pos->state (send (counter-foot-step (car fs-list) :double-support-p t) :worldpos) total-mass)
                           :receding-horizon-proc-count rhpc)))
        (dolist (prm prm-list)
          (let ((r (send mpc :append-param prm)))
            ;;(print (list (position prm prm-list) r))
            (when r
              (format t ";; initial state~%")
              (format t ";;   state = ~A~%" (mpc . current-state))
              (send mpc :proc-model-predictive-control)
              (format t ";;   input = ~A~%" (elt (send mpc :input-value-list) 0))
              (setq tm (+ tm ((elt (mpc . param-list) 0) . dt)))
              (let ((wr-list (send mpc :calc-wrench-list-from-result-input-value))
                    (ec-list (send mpc :get-all-end-coords-from-preview-index)))
                (push (list :cog (send mpc :get-cog-from-preview-index) :refcog (send mpc :get-ref-cog-from-preview-index)
                            :end-coords ec-list :wrench-list wr-list
                            :zmp (send *robot* :calc-zmp-from-forces-moments
                                       (car wr-list) (cadr wr-list)
                                       :limbs limbs :force-sensors ec-list :cop-coords ec-list)
                            :contact-constraint-list (send mpc :get-contact-constraint-list-from-preview-index)
                            :time tm)
                      cog-ret)
                )))))
      (setq cog-ret (reverse cog-ret))
      (dolist (r cog-ret)
        (send *robot* :fullbody-inverse-kinematics
              (cadr (memq :end-coords r))
              :move-target (mapcar #'(lambda (x) (send *robot* x :end-coords)) limbs)
              :link-list (mapcar #'(lambda (x) (send *robot* :link-list (send *robot* x :end-coords :parent))) limbs)
              :root-link-virtual-joint-weight #f(0.1 0.1 0 0 0 0.1)
              :target-centroid-pos (cadr (memq :cog r)))
        (send *irtviewer* :draw-objects :flush nil)
        (push (append (list :angle-vector (send *robot* :angle-vector)
                            :root-coords (send (car (send *robot* :links)) :copy-worldcoords))
                      r)
              ret)
        (draw-force-value r))
      (when (and (boundp 'gnuplot) (/= x::*display* 0))
        (print ";; COM plot")
        (graph-view (list (mapcar #'(lambda (x) (elt (cadr (memq :cog x)) 0)) cog-ret)
                          (mapcar #'(lambda (x) (elt (cadr (memq :cog x)) 1)) cog-ret)
                          (mapcar #'(lambda (x) (elt (cadr (memq :refcog x)) 0)) cog-ret)
                          (mapcar #'(lambda (x) (elt (cadr (memq :refcog x)) 1)) cog-ret)
                          (mapcar #'(lambda (x) (elt (cadr (memq :zmp x)) 0)) cog-ret)
                          (mapcar #'(lambda (x) (elt (cadr (memq :zmp x)) 1)) cog-ret)
                          )
                    (mapcar #'(lambda (x) (cadr (memq :time x))) cog-ret)
                    :xlabel "Time [s]" :ylabel "COM [mm]" :title "COM and ZMP"
                    :keylist (list "Act COM X" "Act COM Y" "Ref COM X" "Ref COM Y" "ZMP X" "ZMP Y")
                    )
        (print ";; COM plot done")
        (read-line)
        (print ";; Force plot")
        (graph-view
         (apply
          #'append
          (mapcar #'(lambda (axis-idx)
                      (mapcar #'(lambda (limb-idx)
                                  (mapcar #'(lambda (x)
                                              (let ((w (elt (car (cadr (memq :wrench-list x))) limb-idx)))
                                                (if (eps= (elt w 2) 0.0) 0.0 (/ (elt w axis-idx) (elt w 2))))) cog-ret))
                              (list 0 1)))
                  (list 0 1)))
         (mapcar #'(lambda (x) (cadr (memq :time x))) cog-ret)
         :xlabel "Time [s]" :ylabel "Fxy/Fz" :title "Fxy/Fz"
         :keylist (apply
                   #'append
                   (mapcar #'(lambda (axis-name)
                               (mapcar #'(lambda (limb-name)
                                           (format nil "~A F~A/Fz" (string-downcase limb-name) axis-name))
                                       limbs))
                           (list "x" "y")))
         )
        (print ";; Force plot done")
        )
      (reverse ret))))

(defun get-max-id-for-demo-functions
  (demo-function-sym)
  (let ((demo-ids (mapcar #'(lambda (x) (let ((str (string-left-trim (string-upcase demo-function-sym) (string x)))) (unless (string= "" str) (read-from-string str)))) (apropos-list demo-function-sym))))
    (apply #'max (remove-if-not #'numberp demo-ids))))

(warn ";; MPC motion gen demos~%")
(dotimes (i (get-max-id-for-demo-functions 'demo-mpc-gen-motion-))
  (warn ";;   (demo-mpc-gen-motion-~d) ;; ~A~%" (1+ i)
        (documentation (read-from-string (format nil "demo-mpc-gen-motion-~d" (1+ i))))))

(defun demo-mpc-gen-motion-all
  (&key ((:receding-horizon-proc-count rhpc) 1))
  (let ((ret))
    (dotimes (i (get-max-id-for-demo-functions 'demo-mpc-gen-motion-))  ;; Get max number of demo function
      (format t ";; demo-mpc-gen-motion-~d~%" (1+ i))
      (push (funcall (eval (read-from-string (format nil "#'demo-mpc-gen-motion-~d" (1+ i)))) :receding-horizon-proc-count rhpc) ret)
      )
    ret))
(warn ";;   (demo-mpc-gen-motion-all)~%")

(defun test-predictive-matrices-common
  (Ak-list Bk-list
   init-value input-value-list
   mpc
   &optional (drift-coeff-list) (Ck-list) (Dk-list))
  (let ((state-value init-value)
        (state-value-list) (output-value-list) (ret1) (ret0) (ret2) (ret3))
    (dotimes (i (length Ak-list))
      (send mpc :append-param
            (instance model-predictive-control-param
                      :init nil nil (elt Ak-list i) (elt Bk-list i)
                      nil nil nil nil nil
                      nil nil
                      (if Ck-list (elt Ck-list i))
                      (if Dk-list (elt Dk-list i)))))
    (dotimes (i (length Ak-list))
      (if Ck-list
          (push (if Dk-list
                    (v+ (transform (elt Ck-list i) state-value) (transform (elt Dk-list i) (elt input-value-list i)))
                  (transform (elt Ck-list i) state-value))
                output-value-list))
      (setq state-value (v+ (transform (elt Ak-list i) state-value) (transform (elt Bk-list i) (elt input-value-list i))))
      (if drift-coeff-list
          (setq state-value (v+ state-value (elt drift-coeff-list i))))
      (push state-value state-value-list)
      )
    (setq ret0 (apply #'concatenate float-vector (reverse state-value-list)))
    (setq ret1 (v+
                (transform (send mpc :calc-total-system-matrix Ak-list) init-value)
                (transform (send mpc :calc-total-input-matrix Ak-list Bk-list) (apply #'concatenate float-vector input-value-list))
                ))
    (if drift-coeff-list
        (setq ret1 (v+ ret1 (transform (send mpc :calc-total-drift-matrix Ak-list) (apply #'concatenate float-vector drift-coeff-list)))))
    (when Ck-list
      (setq ret2 (apply #'concatenate float-vector (reverse output-value-list)))
      (setq ret3 (v+ (transform (send mpc :calc-total-output-matrix Ak-list Ck-list) init-value)
                     (transform (send mpc :calc-total-feedforward-matrix Ak-list Bk-list Ck-list Dk-list)
                                (apply #'concatenate float-vector input-value-list))))
      (if drift-coeff-list
          (setq ret3 (v+ ret3 (transform (send mpc :calc-total-output-drift-matrix Ak-list Ck-list)
                                         (apply #'concatenate float-vector drift-coeff-list)))))
      )
    ;;(format-array ret0)
    ;;(format-array ret1)
    (format-array (v- ret0 ret1) ";;   state diff")
    (format t ";;   state same? ~A~%" (eps= (distance ret0 ret1) 0.0))
    (when Ck-list
      (format-array (v- ret2 ret3) ";;   output diff")
      (format t ";;   output same? ~A~%" (eps= (distance ret2 ret3) 0.0)))
    (if Ck-list
        (and (eps= (distance ret0 ret1) 0.0) (eps= (distance ret2 ret3) 0.0))
      (eps= (distance ret0 ret1) 0.0))
    ))

(defun test-predictive-matrices-1
  ()
  "Test fixed size and values. Linear time-invariant system."
  (format t ";; ~A~%" (documentation 'test-predictive-matrices-1))
  (let* ((mpc (instance model-predictive-control :init 4 3 :output-dim 3))
         (Ak-list (list (scale-matrix 0.5 (unit-matrix 3)) (scale-matrix 1 (unit-matrix 3)) (scale-matrix 1.5 (unit-matrix 3)) (scale-matrix 2 (unit-matrix 3))))
         (Bk-list (list (scale-matrix 0.1 (unit-matrix 3)) (scale-matrix 0.2 (unit-matrix 3)) (scale-matrix 0.3 (unit-matrix 3)) (scale-matrix 0.4 (unit-matrix 3))))
         (input-value-list (list (float-vector -1 -2 -3) (float-vector -4 -5 -6) (float-vector -7 -8 -9) (float-vector -10 -11 -12))))
    (test-predictive-matrices-common
     Ak-list Bk-list (mpc . current-state) input-value-list mpc)
    ))

(defun test-predictive-matrices-2
  ()
  "Test random size and values. Linear time-invariant system."
  (format t ";; ~A~%" (documentation 'test-predictive-matrices-2))
  (let* ((state-dim (1+ (random 5)))
         (input-dim (1+ (random 3)))
         (output-dim (1+ (random 5)))
         (predict-len (1+ (random 10)))
         (initial-state (make-random-vector state-dim :random-range 1.0))
         (mpc (instance model-predictive-control :init predict-len state-dim :initial-state initial-state :output-dim output-dim))
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
    (test-predictive-matrices-common
     Ak-list Bk-list (mpc . current-state) input-value-list mpc)
    ))

(defun test-predictive-matrices-3
  (&key (use-drift) (use-Ck-list) (use-Dk-list))
  "Test random size and values. Linear time-variant system."
  (unless (or use-drift use-Ck-list use-Dk-list)
    (format t ";; ~A~%" (documentation 'test-predictive-matrices-3)))
  (let* ((state-dim (1+ (random 5)))
         (output-dim (1+ (random 5)))
         (input-dim (1+ (random 3)))
         (predict-len (1+ (random 10)))
         (initial-state (make-random-vector state-dim :random-range 1.0))
         (mpc (instance model-predictive-control :init predict-len state-dim :initial-state initial-state :output-dim output-dim))
         (Ak-list (mapcar #'(lambda (x)
                              (make-random-matrix state-dim state-dim :random-range 5.0))
                          (make-list predict-len)))
         (Bk-list (mapcar #'(lambda (x)
                              (make-random-matrix state-dim input-dim :random-range 1.0))
                          (make-list predict-len)))
         (Ck-list (mapcar #'(lambda (x)
                              (make-random-matrix output-dim state-dim :random-range 1.0))
                          (make-list predict-len)))
         (Dk-list (mapcar #'(lambda (x)
                              (make-random-matrix output-dim input-dim :random-range 1.0))
                          (make-list predict-len)))
         (input-value-list (mapcar #'(lambda (x)
                                       (make-random-vector input-dim :random-range 1.0))
                                   (make-list predict-len)))
         (drift-coeff-list (mapcar #'(lambda (x)
                                       (make-random-vector state-dim :random-range 1.0))
                                   (make-list predict-len))))
    (test-predictive-matrices-common
     Ak-list Bk-list (mpc . current-state) input-value-list mpc
     drift-coeff-list (if use-Ck-list Ck-list) (if use-Dk-list Dk-list))
    ))

(defun test-predictive-matrices-4
  ()
  "Test random size and values. Linear time-variant system with drift."
  (format t ";; ~A~%" (documentation 'test-predictive-matrices-4))
  (test-predictive-matrices-3 :use-drift t)
  )

(defun test-predictive-matrices-5
  ()
  "Test random size and values. Linear time-variant system with output."
  (format t ";; ~A~%" (documentation 'test-predictive-matrices-5))
  (test-predictive-matrices-3 :use-Ck-list t)
  )

(defun test-predictive-matrices-6
  ()
  "Test random size and values. Linear time-variant system with output+feedforward."
  (format t ";; ~A~%" (documentation 'test-predictive-matrices-6))
  (test-predictive-matrices-3 :use-Ck-list t :use-Dk-list t)
  )

(defun test-predictive-matrices-7
  ()
  "Test random size and values. Linear time-variant system with drift+output."
  (format t ";; ~A~%" (documentation 'test-predictive-matrices-7))
  (test-predictive-matrices-3 :use-drift t :use-Ck-list t)
  )

(defun test-predictive-matrices-8
  ()
  "Test random size and values. Linear time-variant system with drift+output+feedforward."
  (format t ";; ~A~%" (documentation 'test-predictive-matrices-8))
  (test-predictive-matrices-3 :use-drift t :use-Ck-list t :use-Dk-list t)
  )

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
  (mapcar #'(lambda (cconst ccoords)
              (send cconst :update-contact-coords ccoords))
          (cadr (memq :contact-constraint-list rs-list))
          (cadr (memq :end-coords rs-list)))
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

