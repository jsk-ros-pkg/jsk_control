(load (format nil "~A/euslisp/eus-cddlib-compiled.l" (ros::resolve-ros-path "package://eus_cddlib")))

;; |0| + | 1  0| x >= 0
;; |0|   | 0  1|
;; |1|   |-1 -1|
;; |1|   | 0 -1|
;;
;; x =|0 1  0 -1|y + | | z
;;    |1 0 -1  0|    | |
;; sum y = 1, y >= 0, z >= 0
(defun test1 ()
  (let* ((A (matrix #F(1 0)
                   #F(0 1)
                   #F(-1 0)
                   #F(0 -1)))
         (b (float-vector 0
                         0
                         1
                         1))
         (ret1 (cddlib-H-to-V
                :A_ineq A
                :b_ineq b
                :verbose 1))
         (ret2 (cddlib-V-to-H
                :V (elt ret1 0)
                :R_nonneg (elt ret1 1)
                :R_free (elt ret1 2)
                :verbose 1))
         )
    )
  )



(defun test2 ()
  (let* ((A (matrix #F(0 0 1 0 0 0)
                    #F(1 0 0.2 0 0 0)
                    #F(-1 0 0.2 0 0 0)
                    #F(0 1 0.2 0 0 0)
                    #F(0 -1 0.2 0 0 0)
                    #F(0 0 0.09 1 0 0)
                    #F(0 0 0.09 -1 0 0)
                    #F(0 0 0.05 0 1 0)
                    #F(0 0 0.05 0 -1 0)
                    #F(0 0 0.01 0 0 1)
                    #F(0 0 0.01 0 0 -1)))
         (b (float-vector 0
                          0
                          0
                          0
                          0
                          0
                          0
                          0
                          0
                          0
                          0))
         (ret1 (cddlib-H-to-V
                :A_ineq A
                :b_ineq b
                :verbose 1))
         (ret2 (cddlib-V-to-H
                :V (elt ret1 0)
                :R_nonneg (elt ret1 1)
                :R_free (elt ret1 2)
                :verbose 1))
         )
    )
  )

(defun block-matrix
  (m &optional (i 0) (j 0) (ii nil) (jj nil))
  ;; i, j : start row and column idx
  ;; ii, jj : row and column length of block matrix
  (unless ii (setq ii (- (array-dimension m 0) i)))
  (unless jj (setq jj (- (array-dimension m 1) j)))
  (let ((ret (make-matrix ii jj)))
    (dotimes (_i ii)
      (dotimes (_j jj)
        (setf (aref ret _i _j) (aref m (+ _i i) (+ _j j)))))
    ret))

(defun draw-region (V R_nonneg R_free &optional (z 0))
  (print V)
  (cond
    ((= (array-dimension R_free 1) 2)
     (let ((pwidth (send *viewer* :viewsurface :line-width))
           (psize (send *viewer* :viewsurface :point-size))
           (pcolor (send *viewer* :viewsurface :color))
           (vec (if (= (array-dimension V 1) 0)
                    (float-vector 0 0 z)
                    (float-vector (* 1e3 (aref V 0 0))
                                  (* 1e3 (aref V 1 0))
                                  z))))
       (unwind-protect
            (progn
              (send *viewer* :viewsurface :color #f(0 0 1))
              (send *viewer* :viewsurface :line-width 3)
              (send *viewer* :viewsurface :point-size 3)
              (send *viewer* :viewsurface :3d-line
                    (v+ vec (float-vector (* 1e6 (aref R_free 0 0))
                                          (* 1e6 (aref R_free 1 0))
                                          0))
                    (v+ vec (float-vector (* -1e6 (aref R_free 0 0))
                                          (* -1e6 (aref R_free 1 0))
                                          0))
                    :depth-test t)
              (send *viewer* :viewsurface :3d-line
                    (v+ vec (float-vector (* 1e6 (aref R_free 0 1))
                                          (* 1e6 (aref R_free 1 1))
                                          0))
                    (v+ vec (float-vector (* -1e6 (aref R_free 0 1))
                                          (* -1e6 (aref R_free 1 1))
                                          0))
                    :depth-test t)
              (send *viewer* :flush)
              )
         (progn
           (send *viewer* :viewsurface :line-width pwidth)
           (send *viewer* :viewsurface :point-size psize)
           (send *viewer* :viewsurface :color pcolor))))
     (return-from draw-region t))
    ((and (= (array-dimension R_free 1) 1)
          (= (array-dimension R_nonneg 1) 1))
     (let ((pwidth (send *viewer* :viewsurface :line-width))
           (psize (send *viewer* :viewsurface :point-size))
           (pcolor (send *viewer* :viewsurface :color))
           (vec (if (= (array-dimension V 1) 0)
                    (float-vector 0 0 z)
                    (float-vector (+ 1e3 (aref V 0 0))
                                  (+ 1e3 (aref V 1 0))
                                  z))))
       (unwind-protect
            (progn
              (send *viewer* :viewsurface :color #f(0 0 1))
              (send *viewer* :viewsurface :line-width 3)
              (send *viewer* :viewsurface :point-size 3)
              (send *viewer* :viewsurface :3d-line
                    (v+ vec (float-vector (* 1e6 (aref R_free 0 0))
                                          (* 1e6 (aref R_free 1 0))
                                          0))
                    (v+ vec (float-vector (* -1e6 (aref R_free 0 0))
                                          (* -1e6 (aref R_free 1 0))
                                          0))
                    :depth-test t)
              (send *viewer* :viewsurface :3d-line
                    vec
                    (v+ vec (float-vector (* 1e6 (aref R_nonneg 0 0))
                                          (* 1e6 (aref R_nonneg 1 0))
                                          0))
                    :depth-test t)
              (send *viewer* :flush)
              )
         (progn
           (send *viewer* :viewsurface :line-width pwidth)
           (send *viewer* :viewsurface :point-size psize)
           (send *viewer* :viewsurface :color pcolor))))
     (return-from draw-region t))
    ((and  (= (array-dimension R_free 1) 1)
           (= (array-dimension R_nonneg 1) 0))
     (let ((pwidth (send *viewer* :viewsurface :line-width))
           (psize (send *viewer* :viewsurface :point-size))
           (pcolor (send *viewer* :viewsurface :color))
           (vec (if (= (array-dimension V 1) 0)
                    (list (float-vector 0 0 z))
                    (let ((vec))
                      (dotimes (i (array-dimension V 1))
                        (push (float-vector (+ 1e3 (aref V 0 i))
                                            (+ 1e3 (aref V 1 i))
                                            z) vec))
                      vec))))
       (unwind-protect
            (progn
              (send *viewer* :viewsurface :color #f(0 0 1))
              (send *viewer* :viewsurface :line-width 3)
              (send *viewer* :viewsurface :point-size 3)
              (dolist (vv vec)
                (send *viewer* :viewsurface :3d-line
                      (v+ vv (float-vector (* 1e6 (aref R_free 0 0))
                                           (* 1e6 (aref R_free 1 0))
                                           0))
                      (v+ vv (float-vector (* -1e6 (aref R_free 0 0))
                                           (* -1e6 (aref R_free 1 0))
                                           0))
                      :depth-test t))
              (send *viewer* :flush)
              )
         (progn
           (send *viewer* :viewsurface :line-width pwidth)
           (send *viewer* :viewsurface :point-size psize)
           (send *viewer* :viewsurface :color pcolor))))
     (return-from draw-region t))
    (t nil))
  (let* ((points (let ((points nil))
                   (dotimes (i (array-dimension V 1))
                     (push (float-vector (* 1e3 (aref V 0 i)) (* 1e3 (aref V 1 i)) z) points))
                   points))
         (hull (instance polygon :init :vertices (if points (quickhull points) (list (float-vector 0 0 z)))))
         (vec (cond ((= (array-dimension R_nonneg 1) 0)
                     nil)
                    ((= (array-dimension R_nonneg 1) 1)
                     (list (float-vector (aref R_nonneg 0 0) (aref R_nonneg 1 0) 0)
                           (float-vector (aref R_nonneg 0 0) (aref R_nonneg 1 0) 0)))
                    (t
                     (let ((v1 (float-vector (aref R_nonneg 0 0) (aref R_nonneg 1 0) 0))
                           (v2 (float-vector (aref R_nonneg 0 1) (aref R_nonneg 1 1) 0)))
                       (if (> (elt (v* v1 v2) 2) 0)
                           (list v1 v2)
                           (list v2 v1))))))
         (ext (if vec
                  (let ((v1 (v* (elt vec 0) #F(0 0 1)))
                        (v2 (v* #F(0 0 1) (elt vec 0))))
                    (list
                     (find-extream (send hull :vertices) #'(lambda (v) (v. v v1)) #'>)
                     (find-extream (send hull :vertices) #'(lambda (v) (v. v v2)) #'>)))
                  nil))
         )
    (let ((pwidth (send *viewer* :viewsurface :line-width))
          (psize (send *viewer* :viewsurface :point-size))
          (pcolor (send *viewer* :viewsurface :color)))
      (unwind-protect
           (progn
             (send *viewer* :viewsurface :color #f(0 0 1))
             (send *viewer* :viewsurface :line-width 3)
             (send *viewer* :viewsurface :point-size 3)
             (mapc #'(lambda (e)
                       (unless (and (find (elt (send e :vertices) 0) vec) (find (elt (send e :vertices) 1) vec))
                         (send *viewer* :viewsurface :3d-line
                               (elt (send e :vertices) 0)
                               (elt (send e :vertices) 1)
                               :depth-test t)))
                   (send hull :edges))
             (mapc #'(lambda (v e)
                       (send *viewer* :viewsurface :3d-line
                             e
                             (v+ e (scale 1e6 v))
                             :depth-test t))
                   vec
                   ext)
             (send *viewer* :flush)
             )
        (progn
          (send *viewer* :viewsurface :line-width pwidth)
          (send *viewer* :viewsurface :point-size psize)
          (send *viewer* :viewsurface :color pcolor)))
      )
    )
  )

(defun setup ()
  (unless (boundp '*robot*)
    (load "package://eus_qp/euslisp/contact-optimization.l")
    (load "irteus/demo/sample-robot-model.l")
    (setq *robot* (instance sample-robot :init))
    (send *robot* :make-support-polygons)
    (dolist (l (list :rleg :lleg))
      (send (send *robot* l :end-coords) :put :contact-constraint
            (instance* default-contact-constraint
                       :init
                       :name l
                       :mu-margin-ratio 1.0 :cop-margin-ratio 1.0
                       :mu-trans 0.5
                       :mu-rot 0.05
                       (let* ((vs (mapcar #'(lambda (v) (send (send *robot* l :end-coords) :inverse-transform-vector v)) (send (send *robot* :support-polygon l) :vertices))))
                         (list :l-min-x (elt (find-extream vs #'(lambda (v) (elt v 0)) #'<) 0)
                               :l-max-x (elt (find-extream vs #'(lambda (v) (elt v 0)) #'>) 0)
                               :l-min-y (elt (find-extream vs #'(lambda (v) (elt v 1)) #'<) 1)
                               :l-max-y (elt (find-extream vs #'(lambda (v) (elt v 1)) #'>) 1)
                               )))
            ))
    )

  (unless (boundp '*irtviewer*)
    (make-irtviewer))
  (objects (list *robot*))
  )

(defun calc-scfr (&optional (limbs (list :rleg :lleg)))
  (let* ((mg (* 1e-6 (send *robot* :weight) (elt *g-vec* 2)))
         (A (apply
             #'concatenate-matrix-diagonal
             (mapcar #'(lambda (l)
                         (send (send (send *robot* l :end-coords) :get :contact-constraint) :calc-constraint-matrix (send (send *robot* l :end-coords) :worldcoords)))
                     limbs)))
         (b (v- (apply
                 #'concatenate
                 float-vector
                 (mapcar #'(lambda (l)
                             (send (send (send *robot* l :end-coords) :get :contact-constraint) :get-constraint-vector))
                         limbs))))
         (G (send *robot* :calc-grasp-matrix
                  (mapcar #'(lambda (l)
                              (send (send *robot* l :end-coords) :worldpos))
                          limbs)))
         (total-wrench (v- (float-vector 0
                                         0
                                         mg
                                         0
                                         0
                                         0)))
         (A2  (concatenate-matrix-row A (make-matrix (array-dimension A 0) 2)))
         (G2 (concatenate-matrix-row G (matrix (float-vector 0 0)
                                               (float-vector 0 0)
                                               (float-vector 0 0)
                                               (float-vector 0 (- mg))
                                               (float-vector mg 0)
                                               (float-vector 0 0))))
         (ret (cddlib-H-to-V :A_ineq A2 :b_ineq b :A_eq G2 :b_eq total-wrench))
         (hoge (print "############"))
         (Vc (block-matrix (elt ret 0) (- (array-dimension (elt ret 0) 0) 2) 0 2 (array-dimension (elt ret 0) 1)))
         (Rc_nonneg (block-matrix (elt ret 1) (- (array-dimension (elt ret 1) 0) 2) 0 2 (array-dimension (elt ret 1) 1)))
         (Rc_free (block-matrix (elt ret 2) (- (array-dimension (elt ret 2) 0) 2) 0 2 (array-dimension (elt ret 2) 1)))
         (ret2 (cddlib-V-to-H :V Vc :R_nonneg Rc_nonneg))
         (A_eq (elt ret2 0))
         (b_eq (elt ret2 1))
         (A_ineq (elt ret2 2))
         (b_ineq (elt ret2 3))
         (ret3 (cddlib-H-to-V :A_ineq A_ineq :b_ineq b_ineq :A_eq A_eq :b_eq b_eq))
         (V (elt ret3 0))
         (R_nonneg (elt ret3 1))
         (R_free (elt ret3 2))
         )
    (format t "equality: (=0)~%")
    (dotimes (i (array-dimension A_eq 0))
      (format-array (float-vector (aref A_eq i 0) (aref A_eq i 1) (elt b_eq i))))
    (format t "inequality: (>=0)~%")
    (dotimes (i (array-dimension A_ineq 0))
      (format-array (float-vector (aref A_ineq i 0) (aref A_ineq i 1) (elt b_ineq i))))
    (dolist (l limbs)
      (send (send (send *robot* l :end-coords) :get :contact-constraint) :draw-on :flush nil))
    (draw-region V R_nonneg R_free (elt (send *robot* :centroid) 2))
    )
  )


(defun test3 ()
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  (send *irtviewer* :draw-objects)
  (calc-scfr (list :rleg :lleg)))

(defun test4 ()
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  (send *robot* :lleg :move-end-pos #f(-100 0 50) :world)
  (send *robot* :rleg :move-end-rot -40 :y)
  (send *robot* :lleg :move-end-rot 40 :y)
  (send *robot* :rleg :move-end-rot -10 :x)
  (send *robot* :lleg :move-end-rot 10 :x)
  (send *robot* :rleg :move-end-rot -5 :z)
  (send *robot* :lleg :move-end-rot 5 :z)
  (send *irtviewer* :draw-objects)
  (calc-scfr (list :rleg :lleg)))

(defun test5 ()
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  (send *robot* :lleg :crotch-p :joint-angle -100)
  (send *robot* :lleg :crotch-y :joint-angle -30)
  (send *irtviewer* :draw-objects)
  (calc-scfr (list :rleg :lleg)))

(defun test6 ()
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  (send *robot* :lleg :crotch-p :joint-angle -90)
  (send *robot* :lleg :knee-p :joint-angle 30)
  (send *robot* :lleg :ankle-p :joint-angle 60)
  (send *robot* :rleg :crotch-p :joint-angle 90)
  (send *robot* :rleg :knee-p :joint-angle 30)
  (send *robot* :rleg :ankle-p :joint-angle 60)
  (send *irtviewer* :draw-objects)
  (calc-scfr (list :rleg :lleg)))


(defun test7 ()
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  (send *robot* :lleg :crotch-p :joint-angle -80)
  (send *robot* :rleg :crotch-p :joint-angle 80)
  (send *irtviewer* :draw-objects)
  (calc-scfr (list :rleg :lleg)))


