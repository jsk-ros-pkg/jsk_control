(load "cfr-cwc-calculation.l")

(defun setup ()
  (unless (boundp '*robot*)
    (load "irteus/demo/sample-robot-model.l")
    (setq *robot* (instance sample-robot :init))
    (send-all (send *robot* :joint-list) :max-joint-torque 15.0))
  (unless (boundp '*irtviewer*)
    (make-irtviewer))
  (objects (list *robot*))
  ;; Define contact constraints
  (let ((mu-trans 0.4))
    (send *robot* :put :contact-constraint-list
          (append
           ;; Default foot contact without max-fz constraints
           (mapcar #'(lambda (l)
                       (instance* default-contact-constraint
                                  :init
                                  :name l
                                  :mu-trans mu-trans
                                  :mu-rot 0.05
                                  (let* ((vs (mapcar #'(lambda (v) (send *robot* l :end-coords :inverse-transform-vector v)) (send (send *robot* :support-polygon l) :vertices))))
                                    (list :l-min-x (elt (find-extream vs #'(lambda (v) (elt v 0)) #'<) 0)
                                          :l-max-x (elt (find-extream vs #'(lambda (v) (elt v 0)) #'>) 0)
                                          :l-min-y (elt (find-extream vs #'(lambda (v) (elt v 1)) #'<) 1)
                                          :l-max-y (elt (find-extream vs #'(lambda (v) (elt v 1)) #'>) 1)
                                          ))))
                   '(:rleg :lleg))
           ;; Default hand contact
           (mapcar #'(lambda (l)
                       (instance* default-contact-constraint
                                  :init
                                  :name l
                                  :mu-trans mu-trans
                                  :max-fz 100
                                  :mu-rot 0.05
                                  (let* ((vs
                                          (mapcar #'(lambda (v) (send *robot* l :end-coords :inverse-transform-vector v))
                                                  (send (find-if #'(lambda (fc) (memq :bottom (send fc :id))) (remove-if-not #'(lambda (fc) (eq :cube (caar (send (fc . mbody) :csg)))) (send (send *robot* l :end-coords :parent) :faces))) :vertices))))
                                    (list :l-min-x (elt (find-extream vs #'(lambda (v) (elt v 0)) #'<) 0)
                                          :l-max-x (elt (find-extream vs #'(lambda (v) (elt v 0)) #'>) 0)
                                          :l-min-y (elt (find-extream vs #'(lambda (v) (elt v 1)) #'<) 1)
                                          :l-max-y (elt (find-extream vs #'(lambda (v) (elt v 1)) #'>) 1)
                                          ))))
                   '(:rarm :larm))
           ;; Default foot contact with max-fz
           (mapcar #'(lambda (l)
                       (instance* default-contact-constraint
                                  :init
                                  :name (read-from-string (format nil "~A2" l))
                                  :max-fz (* 9.8 1e-3 (send *robot* :weight) 2) ;; 2 times of robot's weight
                                  :mu-trans mu-trans
                                  :mu-rot 0.05
                                  (let* ((vs (mapcar #'(lambda (v) (send *robot* l :end-coords :inverse-transform-vector v)) (send (send *robot* :support-polygon l) :vertices))))
                                    (list :l-min-x (elt (find-extream vs #'(lambda (v) (elt v 0)) #'<) 0)
                                          :l-max-x (elt (find-extream vs #'(lambda (v) (elt v 0)) #'>) 0)
                                          :l-min-y (elt (find-extream vs #'(lambda (v) (elt v 1)) #'<) 1)
                                          :l-max-y (elt (find-extream vs #'(lambda (v) (elt v 1)) #'>) 1)
                                          ))))
                   '(:rleg :lleg)))
          )
    ))

(defun demo-calc-CFR-1
  (&key (use-lp t) (debug-view t) (dxy 10)
        (ref-cog-z 0)
        (cog-acc (float-vector 0 0 0))
        (calc-min-max-cog-pos-func #'calc-static-min-max-cog-pos)
        (return-value-mode :polygon))
  "Demo for calc CFR on reset-pose."
  ;; SCFRは足支持多角形と一致
  ;; Init pose
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  (if debug-view (send *irtviewer* :draw-objects))
  ;; Calc and display CFR
  (let* ((cc-list (remove-if-not #'(lambda (l) (memq (send l :name) '(:rleg :lleg))) (send *robot* :get :contact-constraint-list)))
         (scfr
          (if use-lp
              (calc-feasible-cog-region-from-contact-constraint-lp
               ref-cog-z
               (list (send *robot* :rleg :end-coords) (send *robot* :lleg :end-coords))
               cc-list
               :calc-min-max-cog-pos-func calc-min-max-cog-pos-func
               :cog-acc cog-acc
               :return-value-mode return-value-mode
               :div 50)
            (calc-static-feasible-cog-region-from-contact-constraint-iterative-qp
             0
             (list (send *robot* :rleg :end-coords) (send *robot* :lleg :end-coords))
             cc-list
             :x-min -70 :x-max 170 :y-min -140 :y-max 140 :dxy dxy))))
    (if debug-view (send scfr :draw-on :flush t))
    scfr))

(defun demo-calc-CFR-2
  (&key (rfoot-height 0) (use-lp t) (debug-view t) (dxy 10)
        (ref-cog-z 0)
        (cog-acc (float-vector 0 0 0))
        (calc-min-max-cog-pos-func #'calc-static-min-max-cog-pos)
        (return-value-mode :polygon)
        (div 50)
        ((:contact-constraint-list cc-list)
         (remove-if-not #'(lambda (l) (memq (send l :name) '(:rleg :lleg))) (send *robot* :get :contact-constraint-list)))
        )
  "Demo for calc CFR on foot diff standing pose.
   rfoot-height is right foot height offset [mm]."
  ;; rfoot-height = 0と rfoot-height=100とで後者の方がCFRが太っている
  ;; 静止するためのモーメントを、鉛直力だけでなく摩擦力でも保持できるため
  ;; Init pose
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  (send *robot* :rleg :move-end-pos (float-vector 50 -50 rfoot-height) :world)
  (send *robot* :lleg :move-end-pos (float-vector -50 50 0) :world)
  (if debug-view (send *irtviewer* :draw-objects))
  ;; Calc and display CFR
  (let* ((scfr
          (if use-lp
              (calc-feasible-cog-region-from-contact-constraint-lp
               ref-cog-z
               (list (send *robot* :rleg :end-coords) (send *robot* :lleg :end-coords))
               cc-list
               :calc-min-max-cog-pos-func calc-min-max-cog-pos-func
               :cog-acc cog-acc
               :return-value-mode return-value-mode
               :div div)
            (calc-static-feasible-cog-region-from-contact-constraint-iterative-qp
             ref-cog-z
             (list (send *robot* :rleg :end-coords) (send *robot* :lleg :end-coords))
             cc-list
             :x-min -120 :x-max 220 :y-min -190 :y-max 190 :dxy dxy))))
    (if debug-view (send scfr :draw-on :flush t))
    scfr))

(defun demo-calc-CFR-3
  (&key (rfoot-pitch -20)
        (lfoot-pitch (- rfoot-pitch))
        (rfoot-roll -10)
        (lfoot-roll (- rfoot-roll))
        (use-lp t) (debug-view t) (dxy 10)
        (ref-cog-z 0)
        (cog-acc (float-vector 0 0 0))
        (calc-min-max-cog-pos-func #'calc-static-min-max-cog-pos)
        (return-value-mode :polygon))
  "Demo for calc CFR on foot diff standing pose.
   You can change foot rot from rfoot-pitch, lfoot-pitch, rfoot-roll, and lfoot-roll arguments."
  ;; Init pose
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  (send *robot* :rleg :move-end-pos #f(50 -50 0) :world)
  (send *robot* :lleg :move-end-pos #f(-50 50 0) :world)
  (send *robot* :rleg :move-end-rot rfoot-pitch :y :local)
  (send *robot* :rleg :move-end-rot rfoot-roll :x :local)
  (send *robot* :lleg :move-end-rot lfoot-pitch :y :local)
  (send *robot* :lleg :move-end-rot lfoot-roll :x :local)
  (if debug-view (send *irtviewer* :draw-objects))
  ;; Calc and display CFR
  (let* ((cc-list (remove-if-not #'(lambda (l) (memq (send l :name) '(:rleg :lleg))) (send *robot* :get :contact-constraint-list)))
         (scfr
          (if use-lp
              (calc-feasible-cog-region-from-contact-constraint-lp
               ref-cog-z
               (list (send *robot* :rleg :end-coords) (send *robot* :lleg :end-coords))
               cc-list
               :calc-min-max-cog-pos-func calc-min-max-cog-pos-func
               :cog-acc cog-acc
               :return-value-mode return-value-mode
               :div 50)
            (calc-static-feasible-cog-region-from-contact-constraint-iterative-qp
             0
             (list (send *robot* :rleg :end-coords) (send *robot* :lleg :end-coords))
             cc-list
             :x-min -120 :x-max 220 :y-min -190 :y-max 190 :dxy dxy))))
    (if debug-view (send scfr :draw-on :flush t))
    scfr))

(defun demo-calc-CFR-4
  (&key (use-lp t) (debug-view t) (dxy 10)
        (ref-cog-z 0)
        (cog-acc (float-vector 0 0 0))
        (calc-min-max-cog-pos-func #'calc-static-min-max-cog-pos)
        (return-value-mode :polygon))
  "Demo for calc CFR on 4leg walking pose."
  ;; 4脚支持状態.手先位置までCFRが伸びてないのは、max-fzを与えているから.handのmax-fzをなくすと手先位置も含まれる
  ;; Init pose
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  (if debug-view (send *irtviewer* :draw-objects))
  ;; Calc and display CFR
  (let* ((cc-list (remove-if-not #'(lambda (l) (memq (send l :name) '(:rleg :lleg :rarm :larm))) (send *robot* :get :contact-constraint-list)))
         (c-coords-list (list
                         (make-coords :pos #f(0   -75 0)) ;; rleg
                         (make-coords :pos #f(0    75 0)) ;; lleg
                         (make-coords :pos #f(400 -75 0)) ;; rarm
                         (make-coords :pos #f(400  75 0)) ;; larm
                         ))
         (scfr
          (if use-lp
              (calc-feasible-cog-region-from-contact-constraint-lp
               ref-cog-z
               c-coords-list
               cc-list
               :calc-min-max-cog-pos-func calc-min-max-cog-pos-func
               :cog-acc cog-acc
               :return-value-mode return-value-mode
               :div 50)
            (calc-static-feasible-cog-region-from-contact-constraint-iterative-qp
             0
             c-coords-list
             cc-list
             :x-min -70 :x-max 410 :y-min -140 :y-max 140 :dxy dxy))))
    (if debug-view (send scfr :draw-on :flush t))
    scfr))

(defun demo-calc-CFR-5
  (&key (use-lp t) (debug-view t) (dxy 10)
        (ref-cog-z 0)
        (cog-acc (float-vector 0 0 0))
        (calc-min-max-cog-pos-func #'calc-static-min-max-cog-pos)
        (return-value-mode :polygon))
  "Demo for calc CFR on one hand contact"
  ;; Init pose
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  ;;(send *robot* :rleg :move-end-pos #f(50 -50 0) :world)
  ;;(send *robot* :lleg :move-end-pos #f(-50 50 0) :world)
  (send *robot* :rarm :inverse-kinematics (make-coords :pos (float-vector 300 -150 600) :rpy (list (deg2rad 10) (deg2rad 20) (deg2rad -5))))
  (if debug-view (send *irtviewer* :draw-objects))
  ;; Calc and display CFR
  (let* ((cc-list (remove-if-not #'(lambda (l) (memq (send l :name) '(:rleg :lleg :rarm))) (send *robot* :get :contact-constraint-list)))
         (c-coords-list (list
                         (send *robot* :rleg :end-coords)
                         (send *robot* :lleg :end-coords)
                         (send (send *robot* :rarm :end-coords :copy-worldcoords) :rotate -pi/2 :y)
                         ))
         (scfr
          (if use-lp
              (calc-feasible-cog-region-from-contact-constraint-lp
               ref-cog-z
               c-coords-list
               cc-list
               :calc-min-max-cog-pos-func calc-min-max-cog-pos-func
               :cog-acc cog-acc
               :return-value-mode return-value-mode
               :div 50)
            (calc-static-feasible-cog-region-from-contact-constraint-iterative-qp
             0
             c-coords-list
             cc-list
             :x-min -70 :x-max 410 :y-min -220 :y-max 190 :dxy dxy))))
    (if debug-view (send scfr :draw-on :flush t))
    scfr))

(defun demo-calc-CFR-6
  (&key (use-lp t) (debug-view t) (dxy 10)
        (ref-cog-z 0)
        (cog-acc (float-vector 0 0 0))
        (calc-min-max-cog-pos-func #'calc-static-min-max-cog-pos)
        (return-value-mode :polygon))
  "Demo for calc CFR on two hand contact."
  ;; Init pose
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  ;;(send *robot* :rleg :move-end-pos #f(50 -50 0) :world)
  ;;(send *robot* :lleg :move-end-pos #f(-50 50 0) :world)
  (send *robot* :rarm :inverse-kinematics (make-coords :pos (float-vector 300 -150 600) :rpy (list (deg2rad 10) (deg2rad 20) (deg2rad -5))))
  (send *robot* :larm :inverse-kinematics (make-coords :pos (float-vector 200 100 500) :rpy (list (deg2rad -10) (deg2rad -20) (deg2rad 5))))
  (if debug-view (send *irtviewer* :draw-objects))
  ;; Calc and display CFR
  (let* ((cc-list (remove-if-not #'(lambda (l) (memq (send l :name) '(:rleg :lleg :rarm :larm))) (send *robot* :get :contact-constraint-list)))
         (c-coords-list (list
                         (send *robot* :rleg :end-coords)
                         (send *robot* :lleg :end-coords)
                         (send (send *robot* :rarm :end-coords :copy-worldcoords) :rotate -pi/2 :y)
                         (send (send *robot* :larm :end-coords :copy-worldcoords) :rotate -pi/2 :y)
                         ))
         (scfr
          (if use-lp
              (calc-feasible-cog-region-from-contact-constraint-lp
               ref-cog-z
               c-coords-list
               cc-list
               :calc-min-max-cog-pos-func calc-min-max-cog-pos-func
               :cog-acc cog-acc
               :return-value-mode return-value-mode
               :div 50)
            (calc-static-feasible-cog-region-from-contact-constraint-iterative-qp
             0
             c-coords-list
             cc-list
             :x-min -70 :x-max 440 :y-min -340 :y-max 290 :dxy dxy))))
    (if debug-view (send scfr :draw-on :flush t))
    scfr))

(defun demo-calc-CFR-7
  (&key (use-fz-max t) (use-lp t) (debug-view t) (dxy 10)
        (ref-cog-z 0)
        (cog-acc (float-vector 0 0 0))
        (calc-min-max-cog-pos-func #'calc-static-min-max-cog-pos)
        (return-value-mode :polygon))
  "Demo for calc CFR for spider walk."
  ;; use-fz-max tだととける
  ;; use-fz-max nilだとwrenchが有界でないのでCFRも有界にならず、現状のLP繰り返しプログラムの仕様上エラる
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  (if debug-view (send *irtviewer* :draw-objects))
  (let* ((cc-list
          (if use-fz-max
              (remove-if-not #'(lambda (l) (memq (send l :name) '(:rleg2 :lleg2))) (send *robot* :get :contact-constraint-list))
            (remove-if-not #'(lambda (l) (memq (send l :name) '(:rleg :lleg))) (send *robot* :get :contact-constraint-list))))
         (c-coords-list (list
                         (make-coords :pos (float-vector 0 -200 0) :rpy (list 0 0 -pi/2))
                         (make-coords :pos (float-vector 0  200 0) :rpy (list 0 0  pi/2))
                         ))
         (scfr
          (if use-lp
              (calc-feasible-cog-region-from-contact-constraint-lp
               ref-cog-z
               c-coords-list
               cc-list
               :calc-min-max-cog-pos-func calc-min-max-cog-pos-func
               :cog-acc cog-acc
               :return-value-mode return-value-mode
               :div 50)
            (calc-static-feasible-cog-region-from-contact-constraint-iterative-qp
             0
             c-coords-list
             cc-list
             :x-min -210 :x-max 210 :y-min -340 :y-max 340 :dxy dxy))))
    (if debug-view (send scfr :draw-on :flush t))
    scfr))

(warn ";; (setup)~%")

(defun get-max-number-of-demo-calc-cfr-functions
  ()
  (apply #'max (remove-if-not #'numberp (mapcar #'(lambda (x) (let ((str (string-left-trim "DEMO-CALC-CFR-" (string x)))) (unless (string= "" str) (read-from-string str)))) (apropos-list 'demo-calc-cfr)))))

(dotimes (i (get-max-number-of-demo-calc-cfr-functions)) ;; Get max number of demo function
  (warn ";;   (demo-calc-cfr-~d) ;; ~A~%" (1+ i)
        (documentation (read-from-string (format nil "demo-calc-cfr-~d" (1+ i))))))
(warn ";; (demo-calc-cfr-all)~%")

(defun demo-calc-cfr-all
  (&key (press-enter-p t) (use-lp t) (debug-view t) (return-values nil) (dxy 10)
        (calc-min-max-cog-pos-func #'calc-static-min-max-cog-pos) (ref-cog-z 0) (cog-acc (float-vector 0 0 0)) (return-value-mode :polygon))
  "for all test"
  (let ((ret))
    (dotimes (i (get-max-number-of-demo-calc-cfr-functions))  ;; Get max number of demo function
      (format t ";; demo-calc-cfr-~d ~A~%" (1+ i) (if use-lp "LP" "Iterative QP"))
      (push (funcall (eval (read-from-string (format nil "#'demo-calc-cfr-~d" (1+ i))))
                     :use-lp use-lp :debug-view debug-view :dxy dxy
                     :calc-min-max-cog-pos-func calc-min-max-cog-pos-func :ref-cog-z ref-cog-z :cog-acc cog-acc :return-value-mode return-value-mode) ret)
      (when press-enter-p (format t ";; press enter~%") (read-line)))
    (if return-values
        ret
      (every #'identity ret))
    ))

(defun compare-calc-scfr-lp-qp-all
  (&key (return-values nil) (debug-view t) (dxy 10))
  "Compare SCFR approximation error between LP and Iterative-QP."
  (let ((plgs-lp
         (bench :print-string "SCFR Incremental LP"
                (demo-calc-cfr-all :press-enter-p nil :use-lp t :debug-view nil :return-values t)))
        (plgs-qp
         (bench :print-string "SCFR Iterative QP"
                (demo-calc-cfr-all :press-enter-p nil :use-lp nil :debug-view nil :return-values t :dxy dxy))))
    (if (and debug-view (and x::*display* (> x::*display* 0)))
        (mapcar #'(lambda (plg-lp plg-qp)
                    (send *irtviewer* :draw-objects :flush nil)
                    (send plg-lp :draw-on :flush nil :color #F(1 0 0) :width 50)
                    (send plg-qp :draw-on :flush t :color #F(0 1 0) :width 3)
                    (send *irtviewer* :viewer :viewsurface :flush)
                    (read-line))
                plgs-lp plgs-qp))
    (if return-values
        (list plgs-lp plgs-qp)
      (mapcar #'(lambda (plg-lp plg-qp)
                  (let* ((vv (send plg-lp :vertices))
                         (x-max (elt (find-extream vv #'(lambda (x) (elt x 0)) #'>) 0))
                         (x-min (elt (find-extream vv #'(lambda (x) (elt x 0)) #'<) 0))
                         (y-max (elt (find-extream vv #'(lambda (x) (elt x 1)) #'>) 1))
                         (y-min (elt (find-extream vv #'(lambda (x) (elt x 1)) #'<) 1)))
                    (> (- (* (+ dxy dxy (- x-max x-min)) (+ dxy dxy (- y-max y-min))) (* (- x-max x-min) (- y-max y-min)))
                       (abs (- (send plg-lp :area) (send plg-qp :area))))))
              plgs-lp plgs-qp))
    ))

(defun compare-calc-dcfr-all
  (&key (return-values nil) (debug-view t) (dxy 10) (ref-cog-z 0) (cog-acc (float-vector 0.4 0.5 0)))
  "Compare DCFR approximation error between local version and ref-cog-z-given version"
  (let ((plgs-dcfr-1
         (bench :print-string "Local DCFR"
                (demo-calc-cfr-all :press-enter-p nil :use-lp t :debug-view nil :return-values t :calc-min-max-cog-pos-func #'calc-dynamic-min-max-cog-pos :ref-cog-z ref-cog-z :cog-acc cog-acc :return-value-mode :prism-polygon)))
        (plgs-dcfr-2
         (bench :print-string "DCFR from COGZ"
                (demo-calc-cfr-all :press-enter-p nil :use-lp t :debug-view nil :return-values t :calc-min-max-cog-pos-func #'calc-dynamic-min-max-cog-pos-from-cog-z :ref-cog-z ref-cog-z :cog-acc cog-acc))))
    (if (and debug-view (and x::*display* (> x::*display* 0)))
        (mapcar #'(lambda (plg-1 plg-2)
                    (send *irtviewer* :draw-objects :flush nil)
                    (send plg-1 :draw-on :flush nil :color #F(1 0 0) :width 50)
                    (send plg-2 :draw-on :flush t :color #F(0 1 0) :width 3)
                    (send *irtviewer* :viewer :viewsurface :flush)
                    (read-line))
                plgs-dcfr-1 plgs-dcfr-2))
    (if return-values
        (list plgs-dcfr-1 plgs-dcfr-2)
      (mapcar #'(lambda (plg-1 plg-2)
                  (let* ((vv (send plg-1 :vertices))
                         (x-max (elt (find-extream vv #'(lambda (x) (elt x 0)) #'>) 0))
                         (x-min (elt (find-extream vv #'(lambda (x) (elt x 0)) #'<) 0))
                         (y-max (elt (find-extream vv #'(lambda (x) (elt x 1)) #'>) 1))
                         (y-min (elt (find-extream vv #'(lambda (x) (elt x 1)) #'<) 1)))
                    ;;(print (list (send plg-1 :area) (send plg-2 :area) (- (send plg-1 :area) (send plg-2 :area))))
                    (> 5000 (abs (- (send plg-1 :area) (send plg-2 :area))))
                    ))
              plgs-dcfr-1 plgs-dcfr-2))
    ))
(warn ";; (compare-calc-scfr-lp-qp-all)~%")
(warn ";; (compare-calc-dcfr-all)~%")
