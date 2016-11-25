(load "package://eus_qp/euslisp/contact-optimization.l")
(load "package://eus_qp/euslisp/eiquadprog.l")
(load "package://eus_qpoases/euslisp/eus-qpoases.l")

(defun setup ()
  ;;(setup-model-for-hrp2)
  ;; (unless (boundp '*robot*)
  ;;   (load "package://hrpsys_ros_bridge_tutorials/models/hrp2jsk.l")
  ;;   (setq *robot* (hrp2jsk)))
  (unless (boundp '*robot*)
    (load "irteus/demo/sample-robot-model.l")
    (setq *robot* (instance sample-robot :init))
    (send-all (send *robot* :joint-list) :max-joint-torque 15.0))
  (unless (boundp '*cbox*)
    (load "models/cardboard-box-object.l")
    (setq *cbox* (cardboard-box))
    (send (car (send *cbox* :links)) :weight 20000.0) ;; 20kg
    (let ((ccoords2 (make-cascoords :coords (send (send *cbox* :copy-worldcoords) :transform (make-coords :pos (float-vector 0 215 185) :rpy (list 0 0 pi/2))))))
      (send *cbox* :assoc ccoords2)
      (send *cbox* :put :face-coords-2 ccoords2)))
  (unless (boundp '*irtviewer*)
    (make-irtviewer))
  (objects (list *robot*))
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
  (ret-list
   &key (union-link-list)
        (robot *robot*)
        (limbs '(:rleg :lleg))
        (contact-coords-list (mapcar #'(lambda (l) (send robot l :end-coords)) limbs)))
  ;; draw contact wrench
  (send-all (send robot :get :contact-constraint-list) :draw-on :flush nil)
  (mapcar
   #'(lambda (f m cc)
       ;; draw cop
       (let* ((cop (send robot :calc-cop-from-force-moment ;; dummy using of robot
                         f m cc cc)))
         (when cop (send cop :draw-on :flush nil :color #F(1 0 0) :width 4 :size 30))))
   (car (cadr (memq :wrench-list ret-list))) (cadr (cadr (memq :wrench-list ret-list))) contact-coords-list)
  (mapcar
   #'(lambda (f m cc)
       ;; draw force
       (with-set-color-line-width
        *viewer* #F(1 0.3 1) 5
        (send *irtviewer* :viewer :draw-arrow
              (send cc :worldpos)
              (v+ (send cc :worldpos) (scale 1 f)))))
   (car (cadr (memq :wrench-list ret-list))) (cadr (cadr (memq :wrench-list ret-list))) contact-coords-list)
  ;; draw torque
  (if (cadr (memq :torque-vector ret-list))
      (with-set-color-line-width
       *viewer* #F(0 1 1) 5
       (send robot :draw-torque *viewer*
             :torque-vector (cadr (memq :torque-vector ret-list))
             :joint-list (send-all union-link-list :joint))))
  (send *irtviewer* :viewer :viewsurface :flush)
  )

(defun test-wrench-distribute-by-pseudo-inverse
  (&key (limbs '(:rleg :lleg))
        (equality-error-weight) ;; dummy
        (contact-coords-list (mapcar #'(lambda (l) (send *robot* l :end-coords)) limbs)))
  (let ((ret
         (list :wrench-list
               (send *robot* :calc-contact-wrenches-from-total-wrench
                     (send-all contact-coords-list :worldpos)))))
    ;; just for drawing
    (mapcar #'(lambda (c-constraint c-coords)
                (send c-constraint :update-contact-coords c-coords))
            (send *robot* :get :contact-constraint-list) contact-coords-list)
    (draw-force-value ret :limbs limbs)
    (send *irtviewer* :viewer :viewsurface :flush)
    ret))

(defun test-wrench-distribute-by-qp
  (&key (limbs '(:rleg :lleg))
        (contact-coords-list (mapcar #'(lambda (l) (send *robot* l :end-coords)) limbs))
        (contact-constraint-matrix-list)
        (min-inequality-violation-weight)
        (equality-error-weight))
  (unless contact-constraint-matrix-list
    (setq contact-constraint-matrix-list
          (mapcar #'(lambda (c-constraint c-coords)
                      (send c-constraint :calc-constraint-matrix c-coords))
                  (send *robot* :get :contact-constraint-list) contact-coords-list)))
  (send *robot* :calc-torque :calc-statics-p t)
  (let ((ret
         (wrench-distribute-from-total-wrench
          contact-coords-list contact-constraint-matrix-list
          :robot *robot*
          :min-inequality-violation-weight min-inequality-violation-weight
          :equality-error-weight equality-error-weight
          :contact-constraint-vector-list (send-all (send *robot* :get :contact-constraint-list) :get-constraint-vector))))
    (draw-force-value ret :limbs limbs)
    (send *irtviewer* :viewer :viewsurface :flush)
    ret))

(defun test-wrench-torque-distribute-by-qp
  (&key (limbs '(:rleg :lleg))
        (contact-coords-list (mapcar #'(lambda (l) (send *robot* l :end-coords)) limbs))
        (contact-constraint-matrix-list)
        (min-inequality-violation-weight)
        (equality-error-weight)
        (func #'wrench-torque-distribute-from-total-wrench-inertial-torque))
  (unless contact-constraint-matrix-list
    (setq contact-constraint-matrix-list
          (mapcar #'(lambda (c-constraint c-coords)
                      (send c-constraint :calc-constraint-matrix c-coords))
                  (send *robot* :get :contact-constraint-list) contact-coords-list)))
  (send *robot* :calc-torque :calc-statics-p t)
  (let* ((link-list
          (mapcar #'(lambda (x) (send *robot* :link-list (send x :parent)))
                  contact-coords-list))
         (union-link-list (send *robot* :calc-union-link-list link-list))
         (ret (funcall func
                       contact-coords-list contact-constraint-matrix-list
                       :robot *robot*
                       :contact-constraint-vector-list (send-all (send *robot* :get :contact-constraint-list) :get-constraint-vector)
                       :min-inequality-violation-weight min-inequality-violation-weight
                       :equality-error-weight equality-error-weight
                       )))
    (draw-force-value ret :union-link-list union-link-list :limbs limbs)
    (send *irtviewer* :viewer :viewsurface :flush)
    ret))

(defun demo-robot-contact-wrench-distribution-common
  (&key (func #'test-wrench-distribute-by-qp) (press-enter-p t) (mu-trans 0.5) (ret-list nil)
        (mu-margin-ratio 1.0) (cop-margin-ratio 1.0)
        (equality-error-weight (if (boundp '*force-use-equality-error-weight*) 1e8))
        (use-no-contact nil) (use-hand))
  (let ((ret) (limbs))
    (send *robot* :put :contact-constraint-list
          (append
           (mapcar #'(lambda (l)
                       (if (and (eq l :lleg) use-no-contact)
                           (instance no-contact-constraint :init :name l)
                         (instance* default-contact-constraint
                                    :init
                                    :name l
                                    :mu-margin-ratio mu-margin-ratio :cop-margin-ratio cop-margin-ratio
                                    :mu-trans mu-trans
                                    :mu-rot 0.05
                                    (let* ((vs (mapcar #'(lambda (v) (send *robot* l :end-coords :inverse-transform-vector v)) (send (send *robot* :support-polygon l) :vertices))))
                                      (list :l-min-x (elt (find-extream vs #'(lambda (v) (elt v 0)) #'<) 0)
                                            :l-max-x (elt (find-extream vs #'(lambda (v) (elt v 0)) #'>) 0)
                                            :l-min-y (elt (find-extream vs #'(lambda (v) (elt v 1)) #'<) 1)
                                            :l-max-y (elt (find-extream vs #'(lambda (v) (elt v 1)) #'>) 1)
                                            )))))
                   '(:rleg :lleg))
           (if use-hand
               (mapcar #'(lambda (l)
                           (instance default-gripper-contact-constraint
                                     :init :name l
                                     :mu-margin-ratio mu-margin-ratio :cop-margin-ratio cop-margin-ratio
                                     :mu-trans 0.4 :mu-rot 0.05
                                     :l-min -12.5 :l-max 12.5 :max-fx 300 :max-fy 70.0 :min-fy -70.0 :max-nx 5.0 :min-nx -5.0))
                       '(:rarm :larm)))
           ))
    (setq limbs (send-all (send *robot* :get :contact-constraint-list) :name))
    (send *robot* :reset-pose)
    (if use-hand (send *robot* :arms :shoulder-p :joint-angle -40))
    (send *robot* :fix-leg-to-coords (make-coords))
    (send *irtviewer* :draw-objects)
    (push (funcall func :limbs limbs :equality-error-weight equality-error-weight) ret)
    (when press-enter-p (format t ";; press enter~%") (read-line))
    (send *robot* :reset-pose)
    (send *robot* :fix-leg-to-coords (make-coords))
    (send *robot* :legs :move-end-pos #f(0 0 50))
    (send *robot* :fix-leg-to-coords (make-coords))
    (send *robot* :lleg :move-end-pos #f(100 50 0))
    (if use-hand (send *robot* :arms :shoulder-p :joint-angle -40))
    (send *robot* :move-centroid-on-foot :rleg '(:rleg :lleg))
    (send *irtviewer* :draw-objects)
    (push (funcall func :limbs limbs :equality-error-weight equality-error-weight) ret)
    (when press-enter-p (format t ";; press enter~%") (read-line))
    (send *robot* :reset-pose)
    (if use-hand (send *robot* :arms :shoulder-p :joint-angle -40))
    (send *robot* :fix-leg-to-coords (make-coords :pos #f(200 200 300)))
    (send *robot* :move-centroid-on-foot :both '(:rleg :lleg))
    (send *irtviewer* :draw-objects)
    (push (funcall func :limbs limbs :equality-error-weight equality-error-weight) ret)
    (when press-enter-p (format t ";; press enter~%") (read-line))
    (send *robot* :reset-pose)
    (send *robot* :fix-leg-to-coords (make-coords))
    (send *robot* :lleg :move-end-pos #f(-100 0 50) :world)
    (send *robot* :rleg :move-end-rot -40 :y)
    (send *robot* :lleg :move-end-rot 40 :y)
    (send *robot* :rleg :move-end-rot -10 :x)
    (send *robot* :lleg :move-end-rot 10 :x)
    (send *robot* :rleg :move-end-rot -5 :z)
    (send *robot* :lleg :move-end-rot 5 :z)
    (if use-hand (send *robot* :arms :shoulder-p :joint-angle -40))
    (send *robot* :move-centroid-on-foot :both '(:rleg :lleg))
    (send *irtviewer* :draw-objects)
    (push (funcall func :limbs limbs :equality-error-weight equality-error-weight) ret)
    (if ret-list
        (reverse ret)
      (every #'identity (reverse ret)))))

(defun demo-robot-contact-wrench-distribution-1
  (&key (press-enter-p t))
  "Demo for robot contact wrench distribution by pseudo-inverse."
  (demo-robot-contact-wrench-distribution-common :func #'test-wrench-distribute-by-pseudo-inverse :press-enter-p press-enter-p))

(defun demo-robot-contact-wrench-distribution-2
  (&key (press-enter-p t))
  "Demo for robot contact wrench distribution by QP."
  (demo-robot-contact-wrench-distribution-common :func #'test-wrench-distribute-by-qp :press-enter-p press-enter-p))

(defun demo-robot-contact-wrench-distribution-3
  (&key (press-enter-p t))
  "Demo for robot contact wrench and joint torque distribution by QP."
  (demo-robot-contact-wrench-distribution-common :func #'test-wrench-torque-distribute-by-qp :press-enter-p press-enter-p))

(defun demo-robot-contact-wrench-distribution-4
  (&key (press-enter-p t))
  "Demo for robot contact wrench and joint torque distribution by QP. Infeasible solution."
  (not (demo-robot-contact-wrench-distribution-common :func #'test-wrench-torque-distribute-by-qp :mu-trans 0.01 :press-enter-p press-enter-p)))

(defun demo-robot-contact-wrench-distribution-5
  (&key (press-enter-p t))
  "Demo for robot contact wrench and joint torque distribution by QP. Originally infeasible solution, but obtain solution by allowing min inequality boundary violation."
  (demo-robot-contact-wrench-distribution-common :func #'(lambda (&rest args) (apply #'test-wrench-torque-distribute-by-qp :min-inequality-violation-weight 1 args))
                                                 :mu-trans 0.01 :press-enter-p press-enter-p))

(defun demo-robot-contact-wrench-distribution-6
  (&key (press-enter-p t))
  "Demo for robot contact wrench distribution with no-contact constraint. Only the second example successes."
  (let ((ret (demo-robot-contact-wrench-distribution-common :func #'test-wrench-distribute-by-qp :press-enter-p press-enter-p :use-no-contact t :ret-list t)))
    (and (not (elt ret 0)) (elt ret 1) (not (elt ret 2)) (not (elt ret 3)))
    ))

(defun demo-robot-contact-wrench-distribution-7
  (&key (press-enter-p t))
  "Demo for robot contact wrench distribution with mu/cop-margin-ratio."
  (let ((ret (demo-robot-contact-wrench-distribution-common :func #'test-wrench-torque-distribute-by-qp :press-enter-p press-enter-p :ret-list t
                                                            :mu-margin-ratio 0.5 :cop-margin-ratio 0.5)))
    (and (elt ret 0) (not (elt ret 1)) (elt ret 2) (not (elt ret 3)))
    ))

(defun demo-robot-contact-wrench-distribution-8
  (&key (press-enter-p t))
  "Demo for robot contact wrench distribution with hand gripper contacts."
  (demo-robot-contact-wrench-distribution-common :func #'test-wrench-torque-distribute-by-qp :press-enter-p press-enter-p :ret-list t :use-hand t))

(defun demo-robot-contact-wrench-distribution-all
  (&key (press-enter-p t) (force-use-equality-error-weight))
  "for all test"
  (let ((ret))
    (makunbound '*force-use-equality-error-weight*)
    (if force-use-equality-error-weight (setq *force-use-equality-error-weight* t))
    (dotimes (i (apply #'max (remove-if-not #'numberp (mapcar #'(lambda (x) (let ((str (string-left-trim "DEMO-ROBOT-CONTACT-WRENCH-DISTRIBUTION-" (string x)))) (unless (string= "" str) (read-from-string str)))) (apropos-list 'demo-robot-contact-wrench-distribution)))))  ;; Get max number of demo function
      (format t ";; demo-robot-contact-wrench-distribution-~d~%" (1+ i))
      (push (funcall (eval (read-from-string (format nil "#'demo-robot-contact-wrench-distribution-~d" (1+ i)))) :press-enter-p press-enter-p) ret)
      (when press-enter-p (format t ";; press enter~%") (read-line)))
    (print (mapcar #'(lambda (x) (not (not x))) ret))
    (every #'identity ret)
    ))

(warn ";; (setup)~%")
(dotimes (i (apply #'max (remove-if-not #'numberp (mapcar #'(lambda (x) (let ((str (string-left-trim "DEMO-ROBOT-CONTACT-WRENCH-DISTRIBUTION-" (string x)))) (unless (string= "" str) (read-from-string str)))) (apropos-list 'demo-robot-contact-wrench-distribution))))) ;; Get max number of demo function
  (warn ";;   (demo-robot-contact-wrench-distribution-~d) ;; ~A~%" (1+ i)
        (documentation (read-from-string (format nil "demo-robot-contact-wrench-distribution-~d" (1+ i))))))
(warn ";; (demo-robot-contact-wrench-distribution-all)~%")
(warn ";; (demo-robot-contact-wrench-distribution-all :force-use-equality-error-weight t)~%")
(warn ";; (test-compare-wrench-torque-calc)~%")

(defun demo-robot-wrench-distribute-from-total-wrench-and-given-wrench
  ()
  ;; Currently calculation with contact constraints does not work.
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  (send *robot* :calc-torque :calc-statics-p t) ;; Statics
  (wrench-distribute-from-total-wrench-and-given-wrench
   (send *robot* :arms :end-coords)
   (send *robot* :legs :end-coords)
   (list (float-vector 0 0 (* (send *robot* :weight) 1e-3 9.8 0.45) 0 0 0)
         (float-vector 0 0 (* (send *robot* :weight) 1e-3 9.8 0.45) 0 0 0))
   :robot *robot*)
   )
(warn ";; (demo-robot-wrench-distribute-from-total-wrench-and-given-wrench)~%")

(defun calc-diff-wrench
  (tq-wr-list1 tq-wr-list2)
  (distance (send *robot* :wrench-list->wrench-vector (cadr (memq :wrench-list tq-wr-list1)))
            (send *robot* :wrench-list->wrench-vector (cadr (memq :wrench-list tq-wr-list2))))
  )

(defun calc-diff-torque
  (tq-wr-list1 tq-wr-list2)
  (distance (cadr (memq :torque-vector tq-wr-list1)) (cadr (memq :torque-vector tq-wr-list2)))
  )

(defun test-compare-wrench-torque-calc ()
  (let* ((ret1
          (bench :print-string "wrench torque distribute"
                 (test-distribute-all
                  :func #'test-wrench-torque-distribute-by-qp
                  :press-enter-p nil)))
         (ret2
          (bench :print-string "wrench torque distribute old"
                 (test-distribute-all
                  ;; does not work?
                  ;;:func #'(lambda (&rest args) (apply #'test-wrench-torque-distribute-by-qp :func #'wrench-torque-distribute-from-total-wrench-inertial-torque-old args))
                  :func #'test-wrench-torque-distribute-by-qp
                  :press-enter-p nil)))
         (ret3
          (bench :print-string "wrench distribute qp"
                 (test-distribute-all
                  :func #'test-wrench-distribute-by-qp
                  :press-enter-p nil)))
         (ret4
          (bench :print-string "wrench distribute pinv"
                 (test-distribute-all
                  :func #'test-wrench-distribute-by-pseudo-inverse
                  :press-enter-p nil))))
    (format t ";; wrench torque dist vs wrench torque dist old~%")
    (format t ";;   diff wrench ~A, diff torque ~A~%"
            (mapcar #'calc-diff-wrench ret1 ret2) (mapcar #'calc-diff-torque ret1 ret2))
    (format t ";; wrench torque dist vs wrench dist~%")
    (format t ";;   diff wrench ~A (wrench torque ~A, wrench ~A)~%"
            (mapcar #'calc-diff-wrench ret1 ret3)
            (mapcar #'(lambda (x) (norm (send *robot* :wrench-list->wrench-vector (cadr (memq :wrench-list x))))) ret1)
            (mapcar #'(lambda (x) (norm (send *robot* :wrench-list->wrench-vector (cadr (memq :wrench-list x))))) ret3))
    (format t ";; wrench dist qp vs wrench dist pinv~%")
    (format t ";;   diff wrench ~A (wrench dist qp ~A, wrench dist pinv ~A)~%"
            (mapcar #'calc-diff-wrench ret3 ret4)
            (mapcar #'(lambda (x) (norm (send *robot* :wrench-list->wrench-vector (cadr (memq :wrench-list x))))) ret3)
            (mapcar #'(lambda (x) (norm (send *robot* :wrench-list->wrench-vector (cadr (memq :wrench-list x))))) ret4))
    ))

(defun demo-cbox-wrench-calc-comon
  (contact-constraint-list
   contact-coords-list
   &key (min-inequality-violation-weight))
  (send *cbox* :put :contact-constraint-list contact-constraint-list)
  ;; calc total static wrench
  (send *cbox* :calc-torque :calc-statics-p t)
  (let ((ret
         (wrench-distribute-from-total-wrench
          contact-coords-list
          (mapcar #'(lambda (c-constraint c-coords)
                      (send c-constraint :calc-constraint-matrix c-coords))
                  contact-constraint-list contact-coords-list)
          :robot *cbox*
          :min-inequality-violation-weight min-inequality-violation-weight
          :contact-constraint-vector-list (send-all contact-constraint-list :get-constraint-vector))))
    (draw-force-value
     ret :limbs nil :robot *cbox*
     :contact-coords-list contact-coords-list)
    ret))

(defun set-cbox-pose-neutral
  ()
  (send *cbox* :newcoords (make-coords))
  (objects (list *cbox*))
  )

(defun set-cbox-pose-tilting
  ()
  ;; move models
  (send *cbox* :newcoords (make-coords))
  (send *cbox* :rotate (deg2rad -30) :x)
  (send *cbox* :rotate (deg2rad -10) :y)
  (objects (list *cbox*))
  )

(defun demo-cbox-wrench-calc-1
  ()
  "Demo for cbox wrench calculation by default-contact-constraint. cbox is neutral pos rot."
  (set-cbox-pose-neutral)
  (demo-cbox-wrench-calc-comon
   ;; setup contact constraint
   ;; box size 190, 215, 185
   (list (instance default-contact-constraint
                   :init
                   :mu-trans 0.5 :mu-rot 0.05
                   :l-min-x -190 :l-max-x 190
                   :l-min-y -215 :l-max-y 215))
   (list (send *cbox* :worldcoords))
   ))

(defun demo-cbox-wrench-calc-2
  ()
  "Demo for cbox wrench calculation by norm-contact-constraint. cbox is neutral pos rot."
  (set-cbox-pose-neutral)
  (demo-cbox-wrench-calc-comon
   (list (instance norm-contact-constraint :init :fz))
   (list (send *cbox* :worldcoords))
   ))

(defun demo-cbox-wrench-calc-3
  ()
  "Demo for cbox wrench calculation by 2D-translational-friction-contact-constraint. cbox is neutral pos rot."
  (set-cbox-pose-neutral)
  (demo-cbox-wrench-calc-comon
   (list (instance 2D-translational-friction-contact-constraint :init 0.5))
   (list (send *cbox* :worldcoords))
   ))

(defun demo-cbox-wrench-calc-4
  ()
  "Demo for cbox wrench calculation by rotational-friction-contact-constraint. cbox is neutral pos rot."
  (set-cbox-pose-neutral)
  (demo-cbox-wrench-calc-comon
   (list (instance rotational-friction-contact-constraint :init 0.05 :fz))
   (list (send *cbox* :worldcoords))
   ))

(defun demo-cbox-wrench-calc-5
  ()
  "Demo for cbox wrench calculation by 2D-cop-contact-constraint. cbox is neutral pos rot."
  (set-cbox-pose-neutral)
  (demo-cbox-wrench-calc-comon
   (list (instance 2D-cop-contact-constraint :init 190 -190 215 -215))
   (list (send *cbox* :worldcoords))
   ))

(defun demo-cbox-wrench-calc-6
  (&key (mu-trans 0.5) (l-min-x -190) (l-max-x 190))
  "Demo for cbox wrench calculation by default-contact-constraint. cbox is tilting."
  (set-cbox-pose-tilting)
  (demo-cbox-wrench-calc-comon
   ;; setup contact constraint
   ;; box size 190, 215, 185
   (list (instance default-contact-constraint
                   :init
                   :mu-trans mu-trans :mu-rot 0.05
                   :l-min-x l-min-x :l-max-x l-max-x
                   :l-min-y -215 :l-max-y 215)
         (instance default-contact-constraint
                   :init
                   :mu-trans mu-trans :mu-rot 0.05
                   :l-min-x l-min-x :l-max-x l-max-x
                   :l-min-y -185 :l-max-y 185)
         )
   (list (send *cbox* :worldcoords) (send *cbox* :get :face-coords-2))
   ))

(defun demo-cbox-wrench-calc-7
  ()
  "Demo for cbox wrench calculation by norm-contact-constraint. cbox is tilting."
  (set-cbox-pose-tilting)
  (demo-cbox-wrench-calc-comon
   (list (instance norm-contact-constraint :init :fz) (instance norm-contact-constraint :init :fz))
   (list (send *cbox* :worldcoords) (send *cbox* :get :face-coords-2))
   ))

(defun demo-cbox-wrench-calc-8
  ()
  "Demo for cbox wrench calculation by 2D-translational-friction-contact-constraint. cbox is tilting."
  (set-cbox-pose-tilting)
  (demo-cbox-wrench-calc-comon
   (list (instance 2D-translational-friction-contact-constraint :init 0.5) (instance 2D-translational-friction-contact-constraint :init 0.5))
   (list (send *cbox* :worldcoords) (send *cbox* :get :face-coords-2))
   ))

(defun demo-cbox-wrench-calc-9
  ()
  "Demo for cbox wrench calculation by rotational-friction-contact-constraint. cbox is tilting."
  (set-cbox-pose-tilting)
  (demo-cbox-wrench-calc-comon
   (list (instance rotational-friction-contact-constraint :init 0.05 :fz) (instance rotational-friction-contact-constraint :init 0.05 :fz))
   (list (send *cbox* :worldcoords) (send *cbox* :get :face-coords-2))
   ))

(defun demo-cbox-wrench-calc-10
  ()
  "Demo for cbox wrench calculation by 2D-cop-contact-constraint. cbox is tilting."
  (set-cbox-pose-tilting)
  (demo-cbox-wrench-calc-comon
   ;; setup contact constraint
   ;; box size 190, 215, 185
   (list (instance 2D-cop-contact-constraint :init 190 -190 215 -215)
         (instance 2D-cop-contact-constraint :init 190 -190 185 -185))
   (list (send *cbox* :worldcoords) (send *cbox* :get :face-coords-2))
   ))

(defun demo-cbox-wrench-calc-11
  (&key (min-inequality-violation-weight))
  "Demo for cbox wrench calculation by default-contact-constraint. cbox is tilting. Infeasible solution."
  (set-cbox-pose-tilting)
  (let ((ret
         (demo-cbox-wrench-calc-comon
          ;; setup contact constraint
          ;; box size 190, 215, 185
          (list (instance default-contact-constraint
                          :init
                          :mu-trans 0.05 :mu-rot 0.05
                          :l-min-x -50 :l-max-x 50
                          :l-min-y -50 :l-max-y 50)
                (instance default-contact-constraint
                          :init
                          :mu-trans 0.05 :mu-rot 0.05
                          :l-min-x -50 :l-max-x 50
                          :l-min-y -50 :l-max-y 50)
                )
          (list (send *cbox* :worldcoords) (send *cbox* :get :face-coords-2))
          :min-inequality-violation-weight min-inequality-violation-weight
          )))
    (if min-inequality-violation-weight
        ret
      (not ret))
    ))

(defun demo-cbox-wrench-calc-12
  ()
  "Demo for cbox wrench calculation by default-contact-constraint. cbox is tilting. Originally infeasible solution, but obtain solution by allowing min inequality boundary violation."
  (demo-cbox-wrench-calc-11 :min-inequality-violation-weight 1e5)
  )

(defun demo-cbox-wrench-calc-13
  ()
  "Demo for cbox wrench calculation by min-max-contact-constraint. cbox is tilting."
  (set-cbox-pose-tilting)
  (demo-cbox-wrench-calc-comon
   (list (instance min-max-contact-constraint
                   :init :fz 40.0 :min/max :max)
         (instance min-max-contact-constraint
                   :init :nz 5.0 :min/max :min))
   (list (send *cbox* :worldcoords) (send *cbox* :get :face-coords-2))
   ))

(defun demo-cbox-wrench-calc-14
  (&key (min-inequality-violation-weight))
  "Demo for cbox wrench calculation by 2D-translational-sliding-contact-constraint. cbox is neutral pos rot. This sample is infeasible but it's correct."
  (set-cbox-pose-neutral)
  (let ((ret
         (demo-cbox-wrench-calc-comon
          (list (instance 2D-translational-sliding-contact-constraint :init 0.5 :slide-axis :fx))
          (list (send *cbox* :worldcoords))
          :min-inequality-violation-weight min-inequality-violation-weight
          )))
    (if min-inequality-violation-weight
        ret
      (not ret))
    ))

(defun demo-cbox-wrench-calc-15
  ()
  "Demo for cbox wrench calculation by 2D-translational-sliding-contact-constraint. cbox is neutral pos rot. This sample is originally infeasible, but obtain solution by allowing min inequality boundary violation."
  (demo-cbox-wrench-calc-14 :min-inequality-violation-weight 1e5))

(defun demo-cbox-wrench-calc-16
  ()
  "Demo for cbox wrench calculation by default-contact-constraint. cbox is neutral pos rot. cbox is sliding in the -y direction."
  (set-cbox-pose-neutral)
  (demo-cbox-wrench-calc-comon
   (list (instance default-contact-constraint
                   :init
                   :mu-trans 0.5 :mu-rot 0.05
                   :slide-axis :-fy
                   :l-min-x -150 :l-max-x 150
                   :l-min-y -150 :l-max-y 150)
         (instance default-contact-constraint
                   :init
                   :mu-trans 0.05 :mu-rot 0.05
                   :l-min-x -50 :l-max-x 50
                   :l-min-y -50 :l-max-y 50))
   (list (send *cbox* :worldcoords) (send *cbox* :get :face-coords-2))
   ))

(defun demo-cbox-wrench-calc-17
  ()
  "Demo for cbox wrench calculation by 6d-min-max-contact-constraint. cbox is neutral pos rot."
  (set-cbox-pose-neutral)
  (demo-cbox-wrench-calc-comon
   (list (instance 6d-min-max-contact-constraint :init (float-vector 1000 1000 1000 100 100 100)))
   (list (send *cbox* :get :face-coords-2))
   ))

(defun demo-cbox-wrench-calc-18
  ()
  "Demo for cbox wrench calculation by polygon-cop-contact-constraint. cbox is neutral pos rot."
  (set-cbox-pose-neutral)
  (demo-cbox-wrench-calc-comon
   (list (instance polygon-cop-contact-constraint :init (elt (send *cbox* :faces) 1))) ;; bottom face
   (list (send *cbox* :worldcoords))
   ))

(defun demo-cbox-wrench-calc-19
  ()
  "Demo for cbox wrench calculation by symmetric-friction-polycone-contact-constraint. cbox is neutral pos rot."
  (set-cbox-pose-neutral)
  (demo-cbox-wrench-calc-comon
   (list (instance symmetric-friction-polycone-contact-constraint :init 0.5 :div 16))
   (list (send *cbox* :worldcoords))
   ))

(defun demo-cbox-wrench-calc-20
  ()
  "Demo for cbox wrench calculation by rotational-sliding-contact-constraint. cbox is neutral pos rot."
  (set-cbox-pose-neutral)
  (let ((ret (demo-cbox-wrench-calc-comon
              (list (instance rotational-sliding-contact-constraint :init 0.5 :fz :slide-axis :nz))
              (list (send *cbox* :worldcoords))
              )))
    (not ret))
  )

(defun demo-cbox-wrench-calc-21
  ()
  "Demo for cbox wrench calculation by default-contact-constraint. cbox is neutral pos rot. cbox is sliding in the -y direction."
  (set-cbox-pose-neutral)
  (demo-cbox-wrench-calc-comon
   (list (instance default-contact-constraint
                   :init
                   :mu-trans 0.5 :mu-rot 0.5
                   :slide-axis :nz
                   :l-min-x -150 :l-max-x 150
                   :l-min-y -150 :l-max-y 150)
         (instance 6d-min-max-contact-constraint
                   :init
                   (float-vector 100 100 100 100 100 100)))
   (list (send *cbox* :worldcoords) (send *cbox* :get :face-coords-2))
   ))

(defun demo-cbox-wrench-calc-all
  (&key (press-enter-p t))
  (let ((ret))
    (dotimes (i (apply #'max (remove-if-not #'numberp (mapcar #'(lambda (x) (let ((str (string-left-trim "DEMO-CBOX-WRENCH-CALC-" (string x)))) (unless (string= "" str) (read-from-string str)))) (apropos-list 'demo-cbox-wrench-calc))))) ;; Get max number of demo function
      (format t ";; demo-cbox-wrench-calc-~d~%" (1+ i))
      (push (funcall (eval (read-from-string (format nil "#'demo-cbox-wrench-calc-~d" (1+ i))))) ret)
      (when press-enter-p (format t ";; press enter~%") (read-line)))
    (every #'identity ret)
    ))

;; Usage
(warn ";; cbox demo functions~%")
(dotimes (i (apply #'max (remove-if-not #'numberp (mapcar #'(lambda (x) (let ((str (string-left-trim "DEMO-CBOX-WRENCH-CALC-" (string x)))) (unless (string= "" str) (read-from-string str)))) (apropos-list 'demo-cbox-wrench-calc))))) ;; Get max number of demo function
  (warn ";;   (demo-cbox-wrench-calc-~d) ;; ~A~%" (1+ i)
        (documentation (read-from-string (format nil "demo-cbox-wrench-calc-~d" (1+ i))))))
(warn ";;   (demo-cbox-wrench-calc-all) ;; for all tests~%")

;; test for calc-constraint-param-list-for-polygon-cop and calc-constraint-param-list-for-symmetric-friction-polycone
(defun test-cop-polygon-constraint
  (&key (max-itr 1000) (debug-view))
  (let* ((vertices-size (+ 1 (random 10)))
         (aface-vertices
          (quickhull
           (remove-duplicates
            (append
             (mapcar #'(lambda (x) (float-vector (random 20.0) (random 20.0) 0.0)) (make-list vertices-size))
             (mapcar #'(lambda (x) (float-vector (- (random 20.0)) (random 20.0) 0.0)) (make-list vertices-size))
             (mapcar #'(lambda (x) (float-vector (- (random 20.0)) (- (random 20.0)) 0.0)) (make-list vertices-size))
             (mapcar #'(lambda (x) (float-vector (random 20.0) (- (random 20.0)) 0.0)) (make-list vertices-size)))
            :test #'eps-v=)))
         ret fc cc)
    ;; Initialize
    (setq vertices-size (length aface-vertices))
    (setq fc (instance face :init :vertices aface-vertices))
    (setq cc (calc-constraint-param-list-for-polygon-cop fc (make-coords)))
    (format t ";; div = ~A, vertices = ~A~%" vertices-size aface-vertices)
    (when debug-view
      (objects (list (make-cube 50 50 10)))
      (send *irtviewer* :draw-objects :flush nil))
    (dotimes (i max-itr)
      (let* ((pp (float-vector (- (random 50.0) 25.0) (- (random 50.0) 25.0) 0.0))
             (fz (+ 1e-5 (random 300.0)))
             ;; Check whether pp is included in fc face using :insidep method.
             (fp (send (instance polygon2d :init (mapcar #'(lambda (x) (subseq x 0 2)) (cdr (send fc :vertices))))
                       :insidep (float-vector (elt pp 0) (elt pp 1))))
             ;; Check whether pp is included in contact-constraint
             (constraint-matrix (make-matrix (length (cadr (memq :matrix cc))) 6 (cadr (memq :matrix cc))))
             (mp (mapcar #'(lambda (x y) (not (< x y)))
                         (concatenate cons (transform constraint-matrix (float-vector 0 0 fz (* 1e-3 fz (elt pp 1)) (* fz -1e-3 (elt pp 0)) 0)))
                         (cadr (memq :vector cc)))))
        ;; Debug
        (when debug-view
          (if (every #'identity mp)
              (send pp :draw-on :flush nil :color #f(0 1 0) :size 2)
            (send pp :draw-on :flush nil :color #f(0 0 1) :size 2)))
        (unless (eq :border fp) ;; neglect border
          (when debug-view
            (format t ";; (insidep = constraint?) = ~A, pos = ~A[mm], fz = ~A[N], insidep = ~A, constraint = ~A~%"
                    (eq fp (every #'identity mp)) pp fz fp (every #'identity mp)))
          (push (eq fp (every #'identity mp)) ret)
          (unless (eq fp (every #'identity mp))
            (when debug-view
              (send pp :draw-on :flush t :color #f(1 0 0) :size 2))
            (error ";; err~%")
            ))
        ))
    (when debug-view
      (send fc :draw-on :flush nil :color #(0 0 0) :width 5)
      (send *viewer* :viewsurface :flush))
    (every #'identity (reverse ret))
    ))

(defun test-friction-polycone-constraint
  (&key (max-itr 1000) (debug-view))
  (let* ((vertices-size (+ 3 (random 10)))
         (mu (random 5.0))
         vertices directions ret
         fc cc)
    ;; Initialize
    (multiple-value-setq
     (vertices directions)
     (calc-vertices-directions-for-regular-polygon mu vertices-size :offset-angle (/ pi vertices-size)))
    (setq fc (instance face :init :vertices vertices))
    (setq cc (calc-constraint-param-list-for-symmetric-friction-polycone mu :div vertices-size))
    (format t ";; mu ~A, div = ~A~%" mu vertices-size)
    (when debug-view
      (objects (list (make-cube 10 10 10)))
      (send *irtviewer* :draw-objects :flush nil))
    ;; Check
    (dotimes (i max-itr)
      (let* ((pp (float-vector (- (random 10.0) 5.0) (- (random 10.0) 5.0) 0.0))
             (fz (+ 1e-5 (random 300.0)))
             ;; Check whether pp is included in fc face using :insidep method.
             (fp (send (instance polygon2d :init (mapcar #'(lambda (x) (subseq x 0 2)) (cdr (send fc :vertices))))
                       :insidep (float-vector (elt pp 0) (elt pp 1))))
             ;; Check whether pp is included in contact-constraint
             (constraint-matrix (make-matrix (length (cadr (memq :matrix cc))) 6 (cadr (memq :matrix cc))))
             (mp (mapcar #'(lambda (x y) (not (< x y)))
                         (concatenate cons (transform constraint-matrix (float-vector (* fz (elt pp 0)) (* (elt pp 1) fz) fz 0 0 0)))
                         (cadr (memq :vector cc)))))
        ;; Debug
        (when debug-view
          (if (every #'identity mp)
              (send pp :draw-on :flush nil :color #f(0 1 0) :size 2)
            (send pp :draw-on :flush nil :color #f(0 0 1) :size 2)))
        (unless (eq :border fp) ;; neglect border
          (when debug-view
            (format t ";; (insidep = constraint?) = ~A, pos = ~A[mm], fz = ~A[N], insidep = ~A, constraint = ~A~%"
                    (eq fp (every #'identity mp)) pp fz fp (every #'identity mp)))
          (push (eq fp (every #'identity mp)) ret)
          (unless (eq fp (every #'identity mp))
            (when debug-view
              (send pp :draw-on :flush t :color #f(1 0 0) :size 2))
            (error ";; err~%")
            ))
        ))
    (when debug-view
      (send fc :draw-on :flush nil :color #(0 0 0) :width 5)
      (send *viewer* :viewsurface :flush))
    (every #'identity (reverse ret))
    ))
(warn ";; (test-cop-polygon-constraint)~%")
(warn ";; (test-friction-polycone-constraint)~%")

#|
(defun test-wrench-comp ()
  (let* ((contact-coords-list
          (mapcar #'(lambda (l) (send *hrp2* l :end-coords)) '(:rleg :lleg :rarm :larm))))
  (labels ((calc-wrench
            ()
            (dotimes (i 3) (send *hrp2* :calc-torque :dt 0.005 :calc-statics-p nil))
            (concatenate float-vector
                         (send (car (send *hrp2* :links)) :force)
                         (send (car (send *hrp2* :links)) :moment))
            )
           (comp-wrench
            (weight)
            (let ((ret-pseudo-inverse
                   (send *hrp2* :calc-contact-wrenches-from-total-wrench
                         (send-all contact-coords-list :worldpos) :total-wrench (calc-wrench)
                         :weight weight))
                  (opt-weight (copy-object weight)))
              (dotimes (i (length opt-weight)) (setf (elt opt-weight i) (/ 1.0 (elt weight i))))
              (let* ((ret-qp
                      (wrench-distribute-from-total-wrench
                       contact-coords-list
                       nil
                       (v- (calc-wrench))
                       :robot *hrp2*
                       :optimize-weight-vector opt-weight)))
                ;;(print ret-pseudo-inverse)
                ;;(print ret-qp)
                (mapcar #'(lambda (f1 m1 f2 m2)
                            (and (eps-v= f1 f2) (eps-v= m1 m2)))
                        (car ret-pseudo-inverse) (cadr ret-pseudo-inverse)
                        (car ret-qp) (cadr ret-qp))
                ))))
    (let* ((weight (fill (instantiate float-vector (* 6 (length contact-coords-list))) 1)))
      (print (comp-wrench weight))
      (dotimes (i 6)
        (setf (elt weight i) 0.5)
        (setf (elt weight (+ 12 i)) 0.5)
        )
      (print (comp-wrench weight))
      (dotimes (i 6)
        (setf (elt weight i) 1e-5)
        (setf (elt weight (+ 12 i)) 1e-5)
        )
      (print (comp-wrench weight))
      (dotimes (i 6)
        (setf (elt weight i) 1e-9)
        (setf (elt weight (+ 12 i)) 1e-9)
        )
      (print (comp-wrench weight))
      t))))
|#
