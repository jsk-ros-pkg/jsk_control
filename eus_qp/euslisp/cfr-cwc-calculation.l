(load "contact-optimization.l")
(require :eus-qpoases "package://eus_qpoases/euslisp/eus-qpoases.l")

;;;;;;;;;;;;;;;;
;; Definitions
;;   SCFR : Static  COM Feasible Region
;;   DCFR : Dynamic COM Feasible Region
;;;;;;;;;;;;;;;;


;;;;;;;;;;;;;;;;
;; Functions for scan min/max
;;;;;;;;;;;;;;;;

(defun calc-static-min-max-cog-pos-org
  (contact-coords-list
   contact-constraint-matrix-list
   contact-constraint-vector-list
   local-contact-constraint-matrix-list
   &key (min/max :min) (axis :x)
        (ref-cog-x) (ref-cog-z)
        (use-local-contact)
        (grasp-matrix (if use-local-contact
                          (send *robot* :calc-grasp-matrix (send-all contact-coords-list :worldpos)
                                :contact-rots (send-all contact-coords-list :worldrot))
                        (send *robot* :calc-grasp-matrix (send-all contact-coords-list :worldpos))))
        (total-mass (* 1e-3 (send *robot* :weight))) ;; [kg]
        (contact-constraint-matrix
          (if use-local-contact
              (apply #'concatenate-matrix-diagonal local-contact-constraint-matrix-list)
            (apply #'concatenate-matrix-diagonal contact-constraint-matrix-list)))
        (contact-constraint-vector
         (apply #'concatenate float-vector contact-constraint-vector-list))
        (contact-constraint-list)
        (solve-lp-mode :lp)
        (object-model)
        (lp-solver #'solve-qpoases-lp))
  "Calculate min/max COG position [mm] considering statics and contact constraints using LP.
   Used for SCFR.
   min/max argument -> :min or :max.
   axis argument -> :x, :y, :z.
   ref-cog-x -> If nil, cog x and y are added to unknown variable. Otherwise, cog-x is given variable and only cog-y is unknown variable."
  (let* ((wrench-dim (array-dimension contact-constraint-matrix 1))
         (ineq-dim (array-dimension contact-constraint-matrix 0))
         (eval-coeff-vector (instantiate float-vector (+ wrench-dim (if ref-cog-x 1 2))))
         (g (* 1e-3 (elt *g-vec* 2))) ;; [m/s^2]
         (mg (* total-mass g))
         (tmp-cog-mat
          (if ref-cog-x
                (make-matrix
                 6 1
                 (list (list 0)
                       (list 0)
                       (list 0)
                       (list (- mg))
                       (list 0)
                       (list 0)))
              (make-matrix
               6 2
               (list (list 0 0)
                     (list 0 0)
                     (list 0 0)
                     (list 0 (- mg))
                     (list mg 0)
                     (list 0 0)))))
         (equality-matrix
          (concatenate-matrix-row
           tmp-cog-mat
           grasp-matrix)) ;; G
         (equality-vector
          (concatenate float-vector
                       (float-vector 0 0 mg)
                       (if ref-cog-x
                           (float-vector 0 (* -1 mg 1e-3 ref-cog-x) 0)
                         (float-vector 0 0 0))))
         (inequality-matrix
          (concatenate-matrix-row
           (make-matrix ineq-dim (if ref-cog-x 1 2)) ;; 0
           contact-constraint-matrix ;; C
           ))
         (inequality-min-vector contact-constraint-vector))
    ;;(format-array equality-vector)
    ;;(format-array equality-matrix)
    ;;(format-array inequality-matrix)
    ;; Set eval-coeff-vector to extract x or y
    (case axis
      (:x (setf (elt eval-coeff-vector 0) (case min/max (:max -1.0) (:min 1.0))))
      (:y (setf (elt eval-coeff-vector (if ref-cog-x 0 1)) (case min/max (:max -1.0) (:min 1.0))))
      (:z (setf (elt eval-coeff-vector 2) (case min/max (:max -1.0) (:min 1.0))))
      )
    ;; Solve LP
    (let ((ret
           (funcall
            lp-solver
            :eval-coeff-vector eval-coeff-vector
            :equality-vector equality-vector
            :equality-matrix equality-matrix
            :inequality-matrix inequality-matrix
            :inequality-min-vector inequality-min-vector
            :debug nil
            :solve-lp-mode solve-lp-mode
            ;;:print-level :pl-medium
            )))
      ;;(print ret)
      ;;(* (case min/max (:max -1e3) (:min 1e3)) (v. eval-coeff-vector ret))
      (if ret
          (* 1e3 (elt ret (case axis (:x 0) (:y (if ref-cog-x 0 1)) (:z 2)))))
      )))

(defun calc-static-min-max-cog-pos
  (contact-coords-list
   contact-constraint-matrix-list
   contact-constraint-vector-list
   local-contact-constraint-matrix-list
   &key (min/max :min) (axis :x)
        (ref-cog-x) (ref-cog-z) (cog-acc)
        (use-local-contact)
        (grasp-matrix (if use-local-contact
                          (send *robot* :calc-grasp-matrix (send-all contact-coords-list :worldpos)
                                :contact-rots (send-all contact-coords-list :worldrot))
                        (send *robot* :calc-grasp-matrix (send-all contact-coords-list :worldpos))))
        (total-mass (* 1e-3 (send *robot* :weight))) ;; [kg]
        (contact-constraint-matrix
          (if use-local-contact
              (apply #'concatenate-matrix-diagonal local-contact-constraint-matrix-list)
            (apply #'concatenate-matrix-diagonal contact-constraint-matrix-list)))
        (contact-constraint-vector
         (apply #'concatenate float-vector contact-constraint-vector-list))
        (contact-constraint-list)
        (solve-lp-mode :lp)
        ((:object-model obj))
        (lp-solver #'solve-qpoases-lp))
  "Calculate min/max COG position [mm] considering statics and contact constraints using LP.
   Direct projection version (state is wrench vector, does not include COM).
   Used for SCFR.
   min/max argument -> :min or :max.
   axis argument -> :x, :y.
   ref-cog-x -> If nil, cog x and y are added to unknown variable. Otherwise, cog-x is given variable and only cog-y is unknown variable."
  (let* ((g (* 1e-3 (elt *g-vec* 2))) ;; [m/s^2]
         (mg (* total-mass g))
         (eval-coeff-vector
          (transform (transpose grasp-matrix)
                     (if ref-cog-x
                         (float-vector 0 0 0 (/ 1 mg) 0 0)
                       (float-vector 0 0 0 0 (/ -1 mg) 0))))
         (tmp-eq-mat
          (make-matrix 4 6
                       (list (list 1 0 0 0 0 0)
                             (list 0 1 0 0 0 0)
                             (list 0 0 1 0 0 0)
                             (list 0 0 0 0 0 1))))
         (tmp-eq-vec (float-vector 0 0 mg 0))
         (equality-matrix (if ref-cog-x
                              (m*
                               (concatenate-matrix-column
                                (make-matrix 1 6 (list (list 0 0 0 0 (/ -1 mg) 0)))
                                tmp-eq-mat)
                               grasp-matrix)
                            (m* tmp-eq-mat grasp-matrix)))
         (equality-vector (if ref-cog-x
                              (concatenate float-vector (float-vector (* 1e-3 ref-cog-x)) tmp-eq-vec)
                            tmp-eq-vec))
         (inequality-matrix contact-constraint-matrix)
         (inequality-min-vector contact-constraint-vector))
    (when obj
      (multiple-value-setq
       (inequality-matrix inequality-min-vector)
       (calc-constraint-matrix-vector-for-obj-env-constraint
        contact-coords-list
        :use-local-contact use-local-contact
        :contact-constraint-matrix contact-constraint-matrix
        :contact-constraint-vector contact-constraint-vector
        :contact-constraint-list contact-constraint-list
        :object-model obj)))
    (if (eq min/max :max) (setq eval-coeff-vector (scale -1 eval-coeff-vector)))
    ;; Solve LP
    (let ((ret
           (funcall
            lp-solver
            :eval-coeff-vector eval-coeff-vector
            :equality-vector equality-vector
            :equality-matrix equality-matrix
            :inequality-matrix inequality-matrix
            :inequality-min-vector inequality-min-vector
            :debug nil
            :solve-lp-mode solve-lp-mode
            ;;:print-level :pl-medium
            )))
      (if ret
          (* (if (eq min/max :max) -1 1) 1e3 (v. eval-coeff-vector ret))
        )
      )))

(defun calc-static-min-max-cog-pos-with-obj-org
  (contact-coords-list
   contact-constraint-matrix-list
   contact-constraint-vector-list
   local-contact-constraint-matrix-list
   &key (min/max :min) (axis :x)
        (ref-cog-x) (ref-cog-z) (cog-acc)
        (use-local-contact)
        (grasp-matrix (if use-local-contact
                          (send *robot* :calc-grasp-matrix (send-all contact-coords-list :worldpos)
                                :contact-rots (send-all contact-coords-list :worldrot))
                        (send *robot* :calc-grasp-matrix (send-all contact-coords-list :worldpos))))
        (total-mass (* 1e-3 (send *robot* :weight))) ;; [kg]
        (contact-constraint-matrix
          (if use-local-contact
              (apply #'concatenate-matrix-diagonal local-contact-constraint-matrix-list)
            (apply #'concatenate-matrix-diagonal contact-constraint-matrix-list)))
        (contact-constraint-vector
         (apply #'concatenate float-vector contact-constraint-vector-list))
        (contact-constraint-list)
        (solve-lp-mode :lp)
        ((:object-model obj))
        (lp-solver #'solve-qpoases-lp))
  "Calculate min/max COG position [mm] considering statics and contact constraints using LP.
   Direct projection version (state is wrench vector, does not include COM).
   Used for SCFR.
   min/max argument -> :min or :max.
   axis argument -> :x, :y.
   ref-cog-x -> If nil, cog x and y are added to unknown variable. Otherwise, cog-x is given variable and only cog-y is unknown variable."
  (let* ((obj-contact-idx-list (remove nil (mapcar #'(lambda (x) (if (send x :get :is-obj-contact) (position x contact-constraint-list))) contact-constraint-list)))
         (non-obj-contact-idx-list (remove-if #'(lambda (idx)
                                                  (memq idx obj-contact-idx-list))
                                              (mapcar #'(lambda (c) (position c contact-coords-list)) contact-coords-list)))
         (tmp-contact-constraint (send obj :get :object-environment-contact-constraint))
         (grasp-matrix
          (concatenate-matrix-row
           (concatenate-matrix-column
            (send *robot* :calc-grasp-matrix
                  (mapcar #'(lambda (idx) (send (elt contact-coords-list idx) :worldpos))
                          (append obj-contact-idx-list non-obj-contact-idx-list)))
            (concatenate-matrix-row
             (scale-matrix -1 (send *robot* :calc-grasp-matrix
                                    (mapcar #'(lambda (idx) (send (elt contact-coords-list idx) :worldpos)) obj-contact-idx-list)))
             (make-matrix 6 (* 6 (length non-obj-contact-idx-list)))))
           (concatenate-matrix-column
            (make-matrix 6 6)
            (send *robot* :calc-grasp-matrix (list (send obj :worldpos))))
           ))
         (contact-constraint-matrix
          (apply #'concatenate-matrix-diagonal
                 (append
                  (mapcar #'(lambda (idx) (elt contact-constraint-matrix-list idx))
                          (append obj-contact-idx-list non-obj-contact-idx-list))
                  (list (send tmp-contact-constraint :calc-constraint-matrix (send obj :worldcoords))))))
         (contact-constraint-vector
          (apply #'concatenate
                 float-vector
                 (append
                  (mapcar #'(lambda (idx) (elt contact-constraint-vector-list idx))
                          (append obj-contact-idx-list non-obj-contact-idx-list))
                  (list (send tmp-contact-constraint :get-constraint-vector)))))
         (g (* 1e-3 (elt *g-vec* 2))) ;; [m/s^2]
         (mg (* total-mass g))
         (obj-mg (* 1e-3 (send obj :weight) g))
         (obj-cen (scale 1e-3 (send obj :centroid nil)))
         (eval-coeff-vector
          (transform (transpose grasp-matrix)
                     (concatenate
                      float-vector
                      (if ref-cog-x
                          (float-vector 0 0 0 (/ 1 mg) 0 0)
                        (float-vector 0 0 0 0 (/ -1 mg) 0))
                      (instantiate float-vector 6)
                      )))
         (tmp-eq-mat
          (concatenate-matrix-diagonal
           (make-matrix 4 6
                        (list (list 1 0 0 0 0 0)
                              (list 0 1 0 0 0 0)
                              (list 0 0 1 0 0 0)
                              (list 0 0 0 0 0 1)))
           (unit-matrix 6)))
         (tmp-eq-vec (concatenate float-vector
                                  (float-vector 0 0 mg 0)
                                  (float-vector 0 0 obj-mg (* (elt obj-cen 1) obj-mg) (* -1 (elt obj-cen 0) obj-mg) 0)
                                  ))
         (equality-matrix (if ref-cog-x
                              (m*
                               (concatenate-matrix-column
                                (make-matrix 1 12 (list (append (list 0 0 0 0 (/ -1 mg) 0) (make-list 6 :initial-element 0))))
                                tmp-eq-mat)
                               grasp-matrix)
                            (m* tmp-eq-mat grasp-matrix)))
         (equality-vector (if ref-cog-x
                              (concatenate float-vector (float-vector (* 1e-3 ref-cog-x)) tmp-eq-vec)
                            tmp-eq-vec))
         (inequality-matrix contact-constraint-matrix)
         (inequality-min-vector contact-constraint-vector))
    (if (eq min/max :max) (setq eval-coeff-vector (scale -1 eval-coeff-vector)))
    ;; Solve LP
    (let ((ret
           (funcall
            lp-solver
            :eval-coeff-vector eval-coeff-vector
            :equality-vector equality-vector
            :equality-matrix equality-matrix
            :inequality-matrix inequality-matrix
            :inequality-min-vector inequality-min-vector
            :debug nil
            :solve-lp-mode solve-lp-mode
            ;;:print-level :pl-medium
            )))
      (if ret
          (* (if (eq min/max :max) -1 1) 1e3 (v. eval-coeff-vector ret))
        )
      )))

(defun calc-constraint-matrix-vector-for-obj-env-constraint
  (contact-coords-list
   &key (use-local-contact)
        (contact-constraint-matrix)
        (contact-constraint-vector)
        (contact-constraint-list)
        (concatenate-return-values t)
        ((:object-model obj)))
  "Calculate object-environment constraints.
   Return (list constraint-matrix constraint-vector)."
  (let ((obj-contact-idx-list (remove nil (mapcar #'(lambda (x) (if (send x :get :is-obj-contact) (position x contact-constraint-list))) contact-constraint-list))))
    (unless obj-contact-idx-list
      (return-from calc-constraint-matrix-vector-for-obj-env-constraint
                   (if concatenate-return-values
                       (list contact-constraint-matrix contact-constraint-vector)
                     (list nil nil))))
    (let* ((tmp-contact-constraint (send obj :get :object-environment-contact-constraint))
           (g (* 1e-3 (elt *g-vec* 2))) ;; [m/s^2]
           (obj-mg (* 1e-3 (send obj :weight) g)) ;; [N]
           (tmp-grasp-matrix-obj-1
            (send *robot* :calc-grasp-matrix (list (v- (send obj :centroid nil) (send obj :worldpos)))))
           (tmp-grasp-matrix-obj-2
            (apply
             #'concatenate-matrix-row
             (mapcar #'(lambda (c)
                         (if (memq (position c contact-coords-list) obj-contact-idx-list)
                             (if use-local-contact
                                 (send *robot* :calc-grasp-matrix (list (v- (send c :worldpos) (send obj :worldpos)))
                                       :contact-rots (list (send c :worldrot)))
                               (send *robot* :calc-grasp-matrix (list (v- (send c :worldpos) (send obj :worldpos)))))
                           (make-matrix 6 6)))
                     contact-coords-list)))
           (obj-constraint-matrix
            (send tmp-contact-constraint :calc-constraint-matrix (send obj :worldcoords)))
           (obj-constraint-vector
            (send tmp-contact-constraint :get-constraint-vector))
           (new-obj-constraint-matrix
            (m* obj-constraint-matrix tmp-grasp-matrix-obj-2))
           (new-obj-constraint-vector
            (v- obj-constraint-vector
                (transform (m* obj-constraint-matrix tmp-grasp-matrix-obj-1)
                           (float-vector 0 0 obj-mg 0 0 0)))))
      (when (send tmp-contact-constraint :get :estimated-constraint)
        (warn ";; overwrite by estimated constraint ~A=>~A~%"
              new-obj-constraint-vector
              (send tmp-contact-constraint :get :estimated-constraint))
        (setq new-obj-constraint-vector (send tmp-contact-constraint :get :estimated-constraint))
        )
      (if concatenate-return-values
          (list (concatenate-matrix-column contact-constraint-matrix new-obj-constraint-matrix)
                (concatenate float-vector contact-constraint-vector new-obj-constraint-vector))
        (list new-obj-constraint-matrix new-obj-constraint-vector))
      )))

(defun calc-static-min-value-for-objenv-constraint
  (contact-coords-list
   contact-constraint-matrix-list
   contact-constraint-vector-list
   local-contact-constraint-matrix-list
   &key (min/max :min) (axis :x)
        (ref-cog-x) (ref-cog-z) (cog-acc)
        (use-local-contact)
        (grasp-matrix (if use-local-contact
                          (send *robot* :calc-grasp-matrix (send-all contact-coords-list :worldpos)
                                :contact-rots (send-all contact-coords-list :worldrot))
                        (send *robot* :calc-grasp-matrix (send-all contact-coords-list :worldpos))))
        (total-mass (* 1e-3 (send *robot* :weight))) ;; [kg]
        (contact-constraint-matrix
          (if use-local-contact
              (apply #'concatenate-matrix-diagonal local-contact-constraint-matrix-list)
            (apply #'concatenate-matrix-diagonal contact-constraint-matrix-list)))
        (contact-constraint-vector
         (apply #'concatenate float-vector contact-constraint-vector-list))
        (contact-constraint-list)
        (element-index 0)
        (solve-lp-mode :lp)
        ((:object-model obj))
        (lp-solver #'solve-qpoases-lp))
  "Calc min value for obj-env constraint"
  (let* ((g (* 1e-3 (elt *g-vec* 2))) ;; [m/s^2]
         (mg (* total-mass g))
         (eval-coeff-vector
          (transform (transpose grasp-matrix)
                     (if ref-cog-x
                         (float-vector 0 0 0 (/ 1 mg) 0 0)
                       (float-vector 0 0 0 0 (/ -1 mg) 0))))
         (tmp-eq-mat
          (make-matrix 4 6
                       (list (list 1 0 0 0 0 0)
                             (list 0 1 0 0 0 0)
                             (list 0 0 1 0 0 0)
                             (list 0 0 0 0 0 1))))
         (tmp-eq-vec (float-vector 0 0 mg 0))
         (equality-matrix (if ref-cog-x
                              (m*
                               (concatenate-matrix-column
                                (make-matrix 1 6 (list (list 0 0 0 0 (/ -1 mg) 0)))
                                tmp-eq-mat)
                               grasp-matrix)
                            (m* tmp-eq-mat grasp-matrix)))
         (equality-vector (if ref-cog-x
                              (concatenate float-vector (float-vector (* 1e-3 ref-cog-x)) tmp-eq-vec)
                            tmp-eq-vec))
         (inequality-matrix contact-constraint-matrix)
         (inequality-min-vector contact-constraint-vector)
         (add-inequality-matrix) (add-inequality-min-vector))
    (when obj
      (multiple-value-setq
       (add-inequality-matrix add-inequality-min-vector)
       (calc-constraint-matrix-vector-for-obj-env-constraint
        contact-coords-list
        :use-local-contact use-local-contact
        :contact-constraint-matrix contact-constraint-matrix
        :contact-constraint-vector contact-constraint-vector
        :contact-constraint-list contact-constraint-list
        :concatenate-return-values nil
        :object-model obj)))
    ;;(if (eq min/max :max) (setq eval-coeff-vector (scale -1 eval-coeff-vector)))
    (setq eval-coeff-vector
          (m*
           (make-matrix 1 (array-dimension add-inequality-matrix 0)
                        (list (let ((vec (instantiate float-vector (array-dimension add-inequality-matrix 0))))
                                (setf (elt vec element-index) 1.0)
                                vec)))
           add-inequality-matrix))
    ;;(if (eq min/max :max) (setq eval-coeff-vector ))
    (setq eval-coeff-vector (array-entity eval-coeff-vector))
    ;;(print eval-coeff-vector)
    ;;(print add-inequality-min-vector)
    ;;(read-line)
    ;; Solve LP
    (let ((ret
           (funcall
            lp-solver
            :eval-coeff-vector eval-coeff-vector
            :equality-vector equality-vector
            :equality-matrix equality-matrix
            :inequality-matrix inequality-matrix
            :inequality-min-vector inequality-min-vector
            :debug nil
            :solve-lp-mode solve-lp-mode
            ;;:print-level :pl-medium
            )))
      (if ret
          (list (v. eval-coeff-vector ret)
                (elt add-inequality-min-vector element-index))
        )
      )))

(defun calc-min-max-total-inertial-forces
  (contact-coords-list
   contact-constraint-matrix-list
   contact-constraint-vector-list
   local-contact-constraint-matrix-list
   &key (min/max :min) (axis :fx)
        (use-local-contact t)
        (grasp-matrix
         (if use-local-contact
             (send *robot* :calc-grasp-matrix (send-all contact-coords-list :worldpos)
                    :contact-rots (send-all contact-coords-list :worldrot))
           (send *robot* :calc-grasp-matrix (send-all contact-coords-list :worldpos))))
        (lp-solver #'solve-qpoases-lp)
        (object-model)
        (debug nil))
  "Calculate min/max of total inertial force."
  (let* ((contact-constraint-matrix
          (if use-local-contact
              (apply #'concatenate-matrix-diagonal local-contact-constraint-matrix-list)
            (apply #'concatenate-matrix-diagonal contact-constraint-matrix-list)))
         (inequality-min-vector
          (apply #'concatenate float-vector contact-constraint-vector-list))
         (wrench-dim (array-dimension contact-constraint-matrix 1))
         (ineq-dim (array-dimension contact-constraint-matrix 0))
         (eval-coeff-vector (instantiate float-vector (+ 6 wrench-dim)))
         (equality-matrix
          (concatenate-matrix-row
           (scale-matrix -1 (unit-matrix 6)) ;; -E
           grasp-matrix)) ;; G
         (equality-vector (instantiate float-vector 6))
         (inequality-matrix
          (concatenate-matrix-row
           (make-matrix ineq-dim 6) ;; 0
           contact-constraint-matrix ;; C
           ))
         (idx
          (case axis
            (:fx 0)
            (:fy 1)
            (:fz 2)
            (:nx 3)
            (:ny 4)
            (:nz 5)
            )))
    (setf (elt eval-coeff-vector idx) (case min/max (:max -1.0) (:min 1.0)))
    (let ((ret
           (funcall
            lp-solver
            :eval-coeff-vector eval-coeff-vector
            :equality-vector equality-vector
            :equality-matrix equality-matrix
            :inequality-matrix inequality-matrix
            :inequality-min-vector inequality-min-vector
            :debug nil
            ;; :solve-type *solve-type*
            ;; :presolve-type *presolve-type*
            ;;:print-level :pl-medium
            )))
      (if ret
          (elt ret idx))
      )))

(defun calc-min-max-ZMP-z-static
  (contact-coords-list
   contact-constraint-matrix-list
   contact-constraint-vector-list
   local-contact-constraint-matrix-list
   &key (min/max :min) (axis :x)
        (ref-cog-x) (ref-cog-z)
        (use-local-contact)
        (grasp-matrix (if use-local-contact
                          (send *robot* :calc-grasp-matrix (send-all contact-coords-list :worldpos)
                                :contact-rots (send-all contact-coords-list :worldrot))
                        (send *robot* :calc-grasp-matrix (send-all contact-coords-list :worldpos))))
        (total-mass (* 1e-3 (send *robot* :weight))) ;; [kg]
        (zmp-z 0)
        (contact-constraint-matrix
          (if use-local-contact
              (apply #'concatenate-matrix-diagonal local-contact-constraint-matrix-list)
            (apply #'concatenate-matrix-diagonal contact-constraint-matrix-list)))
        (contact-constraint-vector
         (apply #'concatenate float-vector contact-constraint-vector-list))
        (contact-constraint-list)
        (solve-lp-mode :lp)
        (object-model)
        (lp-solver #'solve-qpoases-lp))
        ;;(lp-solver #'solve-coinclp-lp))
  "Calculate min/max ZMP [mm] considering dynamics and contact constraints using LP.
   Direct projection version (state is wrench vector, does not include COM).
   min/max argument -> :min or :max.
   axis argument -> :x, :y.
   ref-cog-x -> If nil, cog x and y are added to unknown variable. Otherwise, cog-x is given variable and only cog-y is unknown variable."
  (let* ((g (* 1e-3 (elt *g-vec* 2))) ;; [m/s^2]
         (mg (* total-mass g))
         (Xmat-list
          (mapcar #'(lambda (c)
                      (make-matrix 1 6
                                   (list (concatenate
                                          float-vector
                                          (scale -1 (matrix-row (outer-product-matrix (v- (scale 1e-3 (send c :worldpos)) (float-vector 0 0 (* 1e-3 zmp-z)))) 1))
                                          (float-vector 0 -1 0)))))
                  contact-coords-list))
         (Ymat-list
          (mapcar #'(lambda (c)
                      (make-matrix 1 6
                                   (list (concatenate
                                          float-vector
                                          (matrix-row (outer-product-matrix (v- (scale 1e-3 (send c :worldpos)) (float-vector 0 0 (* 1e-3 zmp-z)))) 0)
                                          (float-vector 1 0 0)))))
                  contact-coords-list))
         (Zmat-list
          (mapcar #'(lambda (c) (float-vector 0 0 1 0 0 0)) contact-coords-list))
         (Xmat (apply #'concatenate-matrix-row Xmat-list))
         (Ymat (apply #'concatenate-matrix-row Ymat-list))
         (Zmat (make-matrix 1 (* 6 (length contact-coords-list)) (list (apply #'concatenate float-vector Zmat-list))))
         (eval-coeff-vector
          (if ref-cog-x
              (array-entity Ymat)
            (array-entity Xmat)))
         (equality-matrix
          (if ref-cog-x
              (concatenate-matrix-column Xmat Zmat)
            Zmat))
         (equality-vector
          (if ref-cog-x
              (float-vector (* 1e-3 ref-cog-x mg) mg)
            (float-vector mg)))
         (inequality-matrix contact-constraint-matrix)
         (inequality-min-vector contact-constraint-vector))
    (if (eq min/max :max) (setq eval-coeff-vector (scale -1 eval-coeff-vector)))
    ;; Solve LP
    (let ((ret
           (funcall
            lp-solver
            :eval-coeff-vector eval-coeff-vector
            :equality-vector equality-vector
            :equality-matrix equality-matrix
            :inequality-matrix inequality-matrix
            :inequality-min-vector inequality-min-vector
            :debug nil
            :solve-lp-mode solve-lp-mode
            ;;:print-level :pl-medium
            )))
      (if ret
          (* (if (eq min/max :max) -1 1) 1e3 (/ (v. eval-coeff-vector ret) (elt (transform Zmat ret) 0)))
        )
      )))

(defun calc-rotation-matrix-from-acc-vec-old
  (acc)
  ;; TODO
  (let (g a b acc2)
    (setq g (sqrt (+ (* (elt acc 0) (elt acc 0)) (* (elt acc 1) (elt acc 1)) (* (elt acc 2) (elt acc 2))))
          b (atan2 (/ (- (elt acc 0)) g) (sqrt (+ (* (/ (elt acc 1) g) (/ (elt acc 1) g)) (* (/ (elt acc 2) g) (/ (elt acc 2) g)))))
          a (atan2 (/ (elt acc 1) g) (/ (elt acc 2) g)))
    (setq acc2 (transform (transpose (rpy-matrix 0 (- b) (- a))) acc) R (rpy-matrix 0 (- b) (- a)))
    (setq g (sqrt (+ (* (elt acc2 0) (elt acc2 0)) (* (elt acc2 1) (elt acc2 1)) (* (elt acc2 2) (elt acc2 2))))
          b (atan2 (/ (- (elt acc2 0)) g) (sqrt (+ (* (/ (elt acc2 1) g) (/ (elt acc2 1) g)) (* (/ (elt acc2 2) g) (/ (elt acc2 2) g)))))
          a (atan2 (/ (elt acc2 1) g) (/ (elt acc2 2) g)))
    ;; (list (transform (transpose (rpy-matrix 0 (- b) (- a))) acc2)
    ;;       (transform (transpose (m* R (rpy-matrix 0 (- b) (- a)))) acc))
    (setq acc2 (transform (transpose (rpy-matrix 0 (- b) (- a))) acc2) R (m* R (rpy-matrix 0 (- b) (- a))))
    (setq g (sqrt (+ (* (elt acc2 0) (elt acc2 0)) (* (elt acc2 1) (elt acc2 1)) (* (elt acc2 2) (elt acc2 2))))
          b (atan2 (/ (- (elt acc2 0)) g) (sqrt (+ (* (/ (elt acc2 1) g) (/ (elt acc2 1) g)) (* (/ (elt acc2 2) g) (/ (elt acc2 2) g)))))
          a (atan2 (/ (elt acc2 1) g) (/ (elt acc2 2) g)))
    (m* R (rpy-matrix 0 (- b) (- a)))
    ))

(defun calc-rotation-matrix-from-acc-vec-old2
  (acc)
  ;; XYZ euler matrix
  ;; https://ja.wikipedia.org/wiki/%E3%82%AA%E3%82%A4%E3%83%A9%E3%83%BC%E8%A7%92
  (let* ((g (sqrt (+ (* (elt acc 0) (elt acc 0)) (* (elt acc 1) (elt acc 1)) (* (elt acc 2) (elt acc 2)))))
         (b (atan2 (/ (elt acc 0) g) (sqrt (+ (* (/ (elt acc 1) g) (/ (elt acc 1) g)) (* (/ (elt acc 2) g) (/ (elt acc 2) g))))))
         (a (atan2 (/ (- (elt acc 1)) g) (/ (elt acc 2) g))))
    (m* (rotation-matrix a :x) (rotation-matrix b :y))
    ))

(defun calc-rotation-matrix-from-acc-vec
  (acc)
  "Calculate rotation matrix from acceleration vector.
   Rotation matrix Z axis aligns to acceleration vector."
  ;; AxisAngle
  (send (orient-coords-to-axis (make-coords) (normalize-vector acc) :z 1e-10) :worldrot)
  )

(defun calc-dynamic-min-max-cog-pos
  (contact-coords-list
   contact-constraint-matrix-list
   contact-constraint-vector-list
   local-contact-constraint-matrix-list
   &key (min/max :min) (axis :x)
        (ref-cog-x) (ref-cog-z)
        (use-local-contact)
        (grasp-matrix (if use-local-contact
                          (send *robot* :calc-grasp-matrix (send-all contact-coords-list :worldpos)
                                :contact-rots (send-all contact-coords-list :worldrot))
                        (send *robot* :calc-grasp-matrix (send-all contact-coords-list :worldpos))))
        (total-mass (* 1e-3 (send *robot* :weight))) ;; [kg]
        (contact-constraint-matrix
          (if use-local-contact
              (apply #'concatenate-matrix-diagonal local-contact-constraint-matrix-list)
            (apply #'concatenate-matrix-diagonal contact-constraint-matrix-list)))
        (contact-constraint-vector
         (apply #'concatenate float-vector contact-constraint-vector-list))
        (contact-constraint-list)
        (solve-lp-mode :lp)
        (cog-acc (float-vector 0 0 0))
        (cog-total-acc (v+ cog-acc (float-vector 0 0 (* 1e-3 (elt *g-vec* 2))))) ;; [m/s^2], ddp-[0 0 -g]^T = ddp + [0 0 g]
        (grav-rot (calc-rotation-matrix-from-acc-vec cog-total-acc))
        (grav-grasp-matrix (m* (concatenate-matrix-diagonal (transpose grav-rot) (transpose grav-rot)) grasp-matrix))
        (diff-angular-momentum (float-vector 0 0 0))
        (grav-local-diff-angular-momentum (transform (transpose grav-rot) diff-angular-momentum))
        ((:object-model obj))
        (lp-solver #'solve-qpoases-lp))
  "Calculate min/max COG position [mm] considering dynamics and contact constraints using LP.
   Direct projection version (state is wrench vector, does not include COM).
   Used for DCFR.
   min/max argument -> :min or :max.
   axis argument -> :x, :y.
   ref-cog-x -> If nil, cog x and y are added to unknown variable. Otherwise, cog-x is given variable and only cog-y is unknown variable."
  (let* ((mg (* total-mass (norm cog-total-acc)))
         (dx-vec (float-vector 0 0 0 0 (/ -1 mg) 0))
         (dy-vec (float-vector 0 0 0 (/ 1 mg) 0 0))
         (eval-coeff-vector
          (transform (transpose grav-grasp-matrix)
                     (if ref-cog-x dy-vec dx-vec)))
         (tmp-eq-mat
          (make-matrix 4 6
                       (list (list 1 0 0 0 0 0)
                             (list 0 1 0 0 0 0)
                             (list 0 0 1 0 0 0)
                             (list 0 0 0 0 0 1))))
         (tmp-eq-vec (concatenate float-vector (float-vector 0 0 mg 0)))
         (equality-matrix (if ref-cog-x
                              (m*
                               (concatenate-matrix-column
                                (make-matrix 1 6 (list dx-vec))
                                tmp-eq-mat)
                               grav-grasp-matrix)
                            (m* tmp-eq-mat grav-grasp-matrix)))
         (equality-vector (if ref-cog-x
                              (concatenate float-vector (float-vector (* 1e-3 ref-cog-x)) tmp-eq-vec)
                            tmp-eq-vec))
         (inequality-matrix contact-constraint-matrix)
         (inequality-min-vector contact-constraint-vector))
    (when obj
      (multiple-value-setq
       (inequality-matrix inequality-min-vector)
       (calc-constraint-matrix-vector-for-obj-env-constraint
        contact-coords-list
        :use-local-contact use-local-contact
        :contact-constraint-matrix contact-constraint-matrix
        :contact-constraint-vector contact-constraint-vector
        :contact-constraint-list contact-constraint-list
        :object-model obj)))
    (setq *grav-rot* grav-rot)
    (if (eq min/max :max) (setq eval-coeff-vector (scale -1 eval-coeff-vector)))
    ;; Solve LP
    (let ((ret
           (funcall
            lp-solver
            :eval-coeff-vector eval-coeff-vector
            :equality-vector equality-vector
            :equality-matrix equality-matrix
            :inequality-matrix inequality-matrix
            :inequality-min-vector inequality-min-vector
            :debug nil
            :solve-lp-mode solve-lp-mode
            ;;:print-level :pl-medium
            )))
      (if ret
          (* (if (eq min/max :max) -1 1) 1e3 (v. eval-coeff-vector ret))
        )
      )))

(defun calc-dynamic-min-max-cog-pos-from-cog-z
  (contact-coords-list
   contact-constraint-matrix-list
   contact-constraint-vector-list
   local-contact-constraint-matrix-list
   &key (min/max :min) (axis :x)
        (use-local-contact t)
        (ref-cog-x) (ref-cog-z 0)
        (grasp-matrix (if use-local-contact
                          (send *robot* :calc-grasp-matrix (send-all contact-coords-list :worldpos)
                                :contact-rots (send-all contact-coords-list :worldrot))
                        (send *robot* :calc-grasp-matrix (send-all contact-coords-list :worldpos))))
        (contact-constraint-matrix
          (if use-local-contact
              (apply #'concatenate-matrix-diagonal local-contact-constraint-matrix-list)
            (apply #'concatenate-matrix-diagonal contact-constraint-matrix-list)))
        (contact-constraint-vector
         (apply #'concatenate float-vector contact-constraint-vector-list))
        (contact-constraint-list)
        (total-mass (* 1e-3 (send *robot* :weight))) ;; [kg]
        (cog-acc (float-vector 0 0 0))
        (cog-inertial-force (scale total-mass (v+ cog-acc (float-vector 0 0 (* 1e-3 (elt *g-vec* 2))))))
        (diff-angular-momentum (float-vector 0 0 0))
        (solve-lp-mode :lp)
        (object-model)
        (lp-solver #'solve-qpoases-lp))
  "Calculate min/max COG position [mm] considering dynamics and contact constraints using LP.
   Direct projection version (state is wrench vector, does not include COM).
   Used for DCFR.
   Use ref-cog-z.
   min/max argument -> :min or :max.
   axis argument -> :x, :y.
   ref-cog-x -> If nil, cog x and y are added to unknown variable. Otherwise, cog-x is given variable and only cog-y is unknown variable."
  (let* ((contact-constraint-matrix
          (if use-local-contact
              (apply #'concatenate-matrix-diagonal local-contact-constraint-matrix-list)
            (apply #'concatenate-matrix-diagonal contact-constraint-matrix-list)))
         (wrench-dim (array-dimension contact-constraint-matrix 1))
         (eval-coeff-vector (instantiate float-vector (+ wrench-dim (if ref-cog-x 1 2))))
         (tmp-cog-mat
          (if ref-cog-x
              (concatenate-matrix-column
               (make-matrix 3 1)
               (make-matrix 3 1
                            (list
                             (list (* -1 (elt cog-inertial-force 2)))
                             (list 0)
                             (list (elt cog-inertial-force 0))
                             )))
            (concatenate-matrix-column
             (make-matrix 3 2)
             (make-matrix 3 2
                          (list
                           (list 0 (* -1 (elt cog-inertial-force 2)))
                           (list (elt cog-inertial-force 2) 0)
                           (list (- (elt cog-inertial-force 1)) (elt cog-inertial-force 0))
                           )))))
         (equality-matrix
          (concatenate-matrix-row tmp-cog-mat grasp-matrix))
         (equality-vector
          (concatenate float-vector
                       cog-inertial-force
                       (v- diff-angular-momentum
                           (if ref-cog-x
                               (float-vector (* (elt cog-inertial-force 1) (* 1e-3 ref-cog-z))
                                             (- (* (elt cog-inertial-force 2) (* 1e-3 ref-cog-x)) (* (elt cog-inertial-force 0) (* 1e-3 ref-cog-z)))
                                             (* -1 (elt cog-inertial-force 1) (* 1e-3 ref-cog-x)))
                             (float-vector (* (elt cog-inertial-force 1) (* 1e-3 ref-cog-z))
                                           (* -1 (elt cog-inertial-force 0) (* 1e-3 ref-cog-z))
                                           0)))))
         (inequality-matrix
          (concatenate-matrix-row
           (make-matrix (array-dimension contact-constraint-matrix 0) (if ref-cog-x 1 2))
           contact-constraint-matrix))
         (inequality-min-vector contact-constraint-vector))
    (setf (elt eval-coeff-vector 0) (case min/max (:max -1.0) (:min 1.0)))
    (let ((ret
           (funcall
            lp-solver
            :eval-coeff-vector eval-coeff-vector
            :equality-matrix equality-matrix
            :equality-vector equality-vector
            :inequality-matrix inequality-matrix
            :inequality-min-vector inequality-min-vector
            :debug nil
            :solve-lp-mode solve-lp-mode
            ;;:solve-type 2;;*solve-type*
            ;;:presolve-type *presolve-type*
            ;;:print-level :pl-medium
            )))
      (if ret (* 1e3 (elt ret 0)))
      )))


;;;;;;;;;;;;;;;;
;; Calc region and min/max
;;;;;;;;;;;;;;;;

(defun calc-feasible-total-inertial-fz-minmax-from-contact-constraint-lp
  (contact-coords-list
   contact-constraint-matrix-list
   contact-constraint-vector-list
   local-contact-constraint-matrix-list
   &key (lp-solver #'solve-qpoases-lp))
  "Calc min/max of Fz.
   Return is (list min-fz max-fz)."
  (mapcar #'(lambda (minmax)
              (calc-min-max-total-inertial-forces
               contact-coords-list
               contact-constraint-matrix-list
               contact-constraint-vector-list
               local-contact-constraint-matrix-list
               :min/max minmax :axis :fz
               :lp-solver lp-solver))
          '(:min :max))
  )

(defun calc-feasible-cog-region-from-contact-constraint-lp
  (ref-cog-z
   contact-coords-list
   contact-constraint-list
   &key (debug nil) (div 20) (lp-solver #'solve-qpoases-lp)
        (use-local-contact t)
        (cog-acc (float-vector 0 0 0))
        ;;(use-local-contact nil)
        (return-value-mode :polygon)
        ;;(return-value-mode :prism)
        (prism-height 4000)
        (object-model)
        ;;(calc-min-max-cog-pos-func #'calc-static-min-max-cog-pos-org))
        (calc-min-max-cog-pos-func #'calc-static-min-max-cog-pos))
        ;;(calc-min-max-cog-pos-func #'calc-dynamic-min-max-cog-pos))
        ;;(calc-min-max-cog-pos-func #'calc-dynamic-min-max-cog-pos-from-cog-z))
        ;;(calc-min-max-cog-pos-func #'calc-min-max-ZMP-z-static))
        ;;(calc-min-max-cog-pos-func #'calc-min-max-cog-pos-with-given-inertial-forces3))
  "Calculate feasible cog region (CFR) considering statics/dynamics and contact constraint using LP.
   div is division number of Y axis.
   calc-min-max-cog-pos-func is function to calculate min/max, which specifies SCFR or DCFR,..etc.
   ref-cog-z[mm] is reference cog height because cog height is not determined from CFR."
  (let* ((contact-constraint-matrix-list
          (mapcar #'(lambda (ccons ccoords) (send ccons :calc-constraint-matrix ccoords))
                  contact-constraint-list contact-coords-list))
         (contact-constraint-vector-list
          (mapcar #'(lambda (x) (send x :get-constraint-vector)) contact-constraint-list))
         (local-contact-constraint-matrix-list
          (mapcar #'(lambda (x) (send x :get-constraint-matrix)) contact-constraint-list))
         (grasp-matrix
          (if use-local-contact
              (send *robot* :calc-grasp-matrix (send-all contact-coords-list :worldpos)
                    :contact-rots (send-all contact-coords-list :worldrot))
            (send *robot* :calc-grasp-matrix (send-all contact-coords-list :worldpos))))
         (total-mass (* 1e-3 (send *robot* :weight nil))) ;; [kg]
         (contact-constraint-matrix
          (if use-local-contact
              (apply #'concatenate-matrix-diagonal local-contact-constraint-matrix-list)
            (apply #'concatenate-matrix-diagonal contact-constraint-matrix-list)))
         (contact-constraint-vector
          (apply #'concatenate float-vector contact-constraint-vector-list))
         (min-x) (max-x) (divx) (ret))
    (if object-model (warn ";;   consider object-environment contact constraint in CFR calculation~%"))
    ;; Step 1. Calc COG X min max
    (setq min-x
          (funcall
           calc-min-max-cog-pos-func
           contact-coords-list
           contact-constraint-matrix-list
           contact-constraint-vector-list
           local-contact-constraint-matrix-list
           :use-local-contact use-local-contact :lp-solver lp-solver
           ;; To reduce calculation
           :contact-constraint-matrix contact-constraint-matrix :contact-constraint-vector contact-constraint-vector
           :contact-constraint-list contact-constraint-list
           :ref-cog-z ref-cog-z :cog-acc cog-acc
           :object-model object-model
           :min/max :min :total-mass total-mass :grasp-matrix grasp-matrix))
    (setq max-x
          (funcall
           calc-min-max-cog-pos-func
           contact-coords-list
           contact-constraint-matrix-list
           contact-constraint-vector-list
           local-contact-constraint-matrix-list
           :use-local-contact use-local-contact :lp-solver lp-solver
           ;; To reduce calculation
           :contact-constraint-matrix contact-constraint-matrix :contact-constraint-vector contact-constraint-vector
           :contact-constraint-list contact-constraint-list
           :solve-lp-mode :slp-with-hotstart
           :ref-cog-z ref-cog-z :cog-acc cog-acc
           :object-model object-model
           :min/max :max :total-mass total-mass :grasp-matrix grasp-matrix))
    ;; For numerical error, shurink the range of [min-x, max-x] by tolerance
    (let ((tolerance 0.01))
      (setq max-x (- max-x tolerance)
            min-x (+ min-x tolerance)))
    (setq divx (/ (- max-x min-x) div))
    ;;(print (list min-x max-x))
    ;; Step 2. Sample from COG X min to COG X max
    (do ((x min-x (+ divx x)))
        ((< (+ (* 1 divx) max-x) x))
        ;; Step 3. Calc COG Y min max
        (let ((max-y
               (funcall
                calc-min-max-cog-pos-func
                contact-coords-list
                contact-constraint-matrix-list
                contact-constraint-vector-list
                local-contact-constraint-matrix-list
                :min/max :max :axis :y :ref-cog-x x :ref-cog-z ref-cog-z
                :use-local-contact use-local-contact :lp-solver lp-solver
                ;; To reduce calculation
                :solve-lp-mode :slp-with-hotstart
                :cog-acc cog-acc
                :contact-constraint-matrix contact-constraint-matrix :contact-constraint-vector contact-constraint-vector
                :contact-constraint-list contact-constraint-list
                :object-model object-model
                :total-mass total-mass :grasp-matrix grasp-matrix)))
          (if max-y (push (float-vector x max-y 0) ret)))
        )
    (do ((x min-x (+ divx x)))
        ((< (+ (* 1 divx) max-x) x))
        (let ((min-y
               (funcall
                calc-min-max-cog-pos-func
                contact-coords-list
                contact-constraint-matrix-list
                contact-constraint-vector-list
                local-contact-constraint-matrix-list
                :min/max :min :axis :y :ref-cog-x x :ref-cog-z ref-cog-z
                :use-local-contact use-local-contact :lp-solver lp-solver
                ;; To reduce calculation
                :solve-lp-mode :slp-with-hotstart
                :cog-acc cog-acc
                :contact-constraint-matrix contact-constraint-matrix :contact-constraint-vector contact-constraint-vector
                :contact-constraint-list contact-constraint-list
                :object-model object-model
                :total-mass total-mass :grasp-matrix grasp-matrix)))
          (if min-y (push (float-vector x min-y 0) ret)))
        )
    ;; Step 4. Calc convex hull
    (case return-value-mode
          (:polygon (if (boundp '*grav-rot*) ;; dirty
                        (let ((dir (transform *grav-rot* (float-vector 0 0 1))))
                          (setq dir (scale (/ ref-cog-z (elt dir 2)) dir))
                          (let* ((local-vertices (quickhull (reverse ret)))
                                 (pl (instance polygon :init :vertices (quickhull
                                                                        (mapcar #'(lambda (pp) (v+ (transform *grav-rot* pp) dir)) (reverse ret))))))
                            (send pl :put :local-vertices local-vertices)
                            (send pl :put :grav-rot *grav-rot*)
                            (makunbound '*grav-rot*)
                            pl))
                      (instance polygon :init :vertices (quickhull (mapcar #'(lambda (x) (v+ (float-vector 0 0 ref-cog-z) x)) (reverse ret))))))
          ((:prism :prism-polygon)
           (let ((bd (make-prism (quickhull (reverse ret)) prism-height)))
             (when (boundp '*grav-rot*) ;; dirty
               (send bd :rotate-with-matrix *grav-rot* :world)
               (send bd :translate (float-vector 0 0 (* -0.5 prism-height)) :local)
               (send bd :worldcoords)
               ;; (send (send *viewer* :get :pickviewer) :draw-objects :flush nil)
               ;; (send bd :draw-on :flush t :color #f(1 0 0))
               ;; (read-line)
               (makunbound '*grav-rot*))
             (if (eq return-value-mode :prism-polygon)
                 (calc-world-dcfr-polygon bd :cog-height ref-cog-z)
               bd)))
          (t (reverse ret)))
    ))

(defun get-intersection-polygon
  (abody aface &key (polygon-offset-vector (float-vector 0 0 0)))
  "Get intersection face with abody and aface."
  (instance polygon :init :vertices
            (mapcar #'(lambda (v)
                        (v+ v polygon-offset-vector))
                    (quickhull (flatten
                                (mapcar #'(lambda (f)
                                            (mapcar #'(lambda (e)
                                                        (let ((a (send aface :intersect-edge e)))
                                                          (if a (cadr a))))
                                                    (send f :edges)))
                                        (send abody :faces))))))
  )

(defun calc-world-dcfr-polygon
  (dcfr-prism
   &key ((:cog-height h) (elt (send *robot* :centroid) 2))
        (polygon-offset-vector (float-vector 0 0 0)))
  "Calc DCFR polygon in world frame."
  (let ((ff (instance face :init :vertices (list (float-vector 5000 5000 h) (float-vector -5000 5000 h) (float-vector -5000 -5000 h) (float-vector 5000 -5000 h)))))
    (get-intersection-polygon dcfr-prism ff :polygon-offset-vector polygon-offset-vector)
    ))

(defun gen-xyz-list-from-xy-min-max
  (&key (x-min -1000) (x-max 100) (y-min -700) (y-max 700)
        (dx 10) (dy 10) (z 1400))
  (let ((ret))
    (do ((x x-min (+ x dx))) ((> x x-max))
      (do ((y y-min (+ y dy))) ((> y y-max))
        (push (float-vector x y z) ret)
        ))
    (reverse ret)))

(defun calc-static-feasible-cog-region-from-contact-constraint-iterative-qp-org
  (ref-cog-z contact-coords-list contact-constraint-list
   &key (debug nil) (return-polygon t) (set-region-bbox)
        (x-min -1000) (x-max 100) (y-min -700) (y-max 700)
        (dxy 10) (dx dxy) (dy dxy)
        (cog-pos-list (gen-xyz-list-from-xy-min-max :x-min x-min :x-max x-max :y-min y-min :y-max y-max :dx dx :dy dy :z ref-cog-z)))
  "Calculate static feasible cog region (SCFR) considering statics and contact constraint using QP iteratively.
   cog-pos-list is list of cog pos [mm] candidate corresponding to searching area."
  (when set-region-bbox
    (setq *region-bbox* (send (make-bounding-box cog-pos-list) :body))
    (send *region-bbox* :set-color #f(0 1 0))
    (gl::transparent *region-bbox* 0.3))
  (let* ((ret) (total-mass (send *robot* :weight))
         (contact-constraint-matrix-list
          (mapcar #'(lambda (ccons ccoords) (send ccons :calc-constraint-matrix ccoords))
                  contact-constraint-list contact-coords-list))
         (contact-constraint-vector-list
          (mapcar #'(lambda (x) (send x :get-constraint-vector)) contact-constraint-list))
         (local-contact-constraint-matrix-list
          (mapcar #'(lambda (x) (send x :get-constraint-matrix)) contact-constraint-list)))
    (dolist (cog-pos cog-pos-list)
      (let ((qp-status (float-vector 0)))
        (labels ((qp-solver
                  (&rest args)
                  (apply #'solve-qpoases-qp (append (list :status qp-status :debug nil) args))))
          (let ((wr
                 (calc-static-wrenches-from-cog
                  cog-pos contact-coords-list contact-constraint-matrix-list contact-constraint-vector-list
                  :total-mass total-mass :qp-solver #'qp-solver)))
            (if (= (round (elt qp-status 0)) 0) ;; Solved
                (progn
                  (if debug (format t ";; ~A OK ~A~%" cog-pos (elt qp-status 0)))
                  (push cog-pos ret))
              (if debug (format t ";; ~A NG ~A~%" cog-pos (elt qp-status 0))))
            ))))
    (format t ";; Solved = ~A/~A~%" (length ret) (length cog-pos-list))
    (if return-polygon
        (instance polygon :init :vertices (quickhull (reverse ret)))
      (reverse ret))))

(defun calc-static-feasible-cog-region-from-contact-constraint-iterative-qp
  (ref-cog-z contact-coords-list contact-constraint-list
   &key (debug nil) (return-polygon t) (set-region-bbox)
        (x-min -1000) (x-max 100) (y-min -700) (y-max 700)
        (dxy 10) (dx dxy) (dy dxy)
        ;;(cog-pos-list (gen-xyz-list-from-xy-min-max :x-min x-min :x-max x-max :y-min y-min :y-max y-max :dx dx :dy dy :z ref-cog-z))
        )
  "Calculate static feasible cog region (SCFR) considering statics and contact constraint using QP iteratively.
   cog-pos-list is list of cog pos [mm] candidate corresponding to searching area."
  (when set-region-bbox
    (setq *region-bbox* (send (make-bounding-box cog-pos-list) :body))
    (send *region-bbox* :set-color #f(0 1 0))
    (gl::transparent *region-bbox* 0.3))
  (let* ((ret) (total-mass (send *robot* :weight))
         (contact-constraint-matrix-list
          (mapcar #'(lambda (ccons ccoords) (send ccons :calc-constraint-matrix ccoords))
                  contact-constraint-list contact-coords-list))
         (contact-constraint-vector-list
          (mapcar #'(lambda (x) (send x :get-constraint-vector)) contact-constraint-list))
         (local-contact-constraint-matrix-list
          (mapcar #'(lambda (x) (send x :get-constraint-matrix)) contact-constraint-list))
         (count 0)
         (max-count
          (* (let ((count 0)) (do ((x x-min (+ x dx))) ((> x x-max)) (incf count)) count)
             (let ((count 0)) (do ((y y-min (+ y dy))) ((> y y-max)) (incf count)) count))))
    (format t ";; Test QP = ~A~%" max-count)
    (do ((x x-min (+ x dx))) ((> x x-max))
        (do ((y y-min (+ y dy))) ((> y y-max))
            (incf count)
            (if (= 0 (mod count (/ max-count 10))) (format t ";;   Current Tested = ~A/~A~%" count max-count))
            (let ((cog-pos (float-vector x y ref-cog-z)))
              (let ((qp-status (float-vector 0)))
                (labels ((qp-solver
                          (&rest args)
                          ;;(apply #'solve-qpoases-qp (append (list :status qp-status :debug nil :solve-qp-mode :sqp-with-hotstart) args))))
                          (apply #'solve-qpoases-qp (append (list :status qp-status :debug nil) args))))
                          ;;(apply #'solve-qpoases-lp (append (list :status qp-status :debug nil :solve-lp-mode :slp-with-hotstart) args))))
                          ;;(apply #'solve-qpoases-lp (append (list :status qp-status :debug nil) args))))
                  (let ((wr
                         (calc-static-wrenches-from-cog
                          cog-pos contact-coords-list contact-constraint-matrix-list contact-constraint-vector-list
                          :total-mass total-mass :qp-solver #'qp-solver)))
                    (if (= (round (elt qp-status 0)) 0) ;; Solved
                        (progn
                          (if debug (format t ";; ~A OK ~A~%" cog-pos (elt qp-status 0)))
                          (push cog-pos ret))
                      (if debug (format t ";; ~A NG ~A~%" cog-pos (elt qp-status 0))))
                    ))))))
    ;;(format t ";; Solved = ~A/~A~%" (length ret) (length cog-pos-list))
    (format t ";; Solved = ~A/~A~%" (length ret) max-count)
    (if return-polygon
        (instance polygon :init :vertices (quickhull (reverse ret)))
      (reverse ret))))

(defun calc-static-feasible-cog-region-from-contact-constraint-iterative-qp-lp
  (ref-cog-z contact-coords-list contact-constraint-list
   &key (debug nil) (return-polygon t) (set-region-bbox)
        (x-min -1000) (x-max 100) (y-min -700) (y-max 700)
        (dxy 10) (dx dxy) (dy dxy)
        )
  "Calculate static feasible cog region (SCFR) considering statics and contact constraint using QP iteratively.
   cog-pos-list is list of cog pos [mm] candidate corresponding to searching area."
  (when set-region-bbox
    (setq *region-bbox* (send (make-bounding-box cog-pos-list) :body))
    (send *region-bbox* :set-color #f(0 1 0))
    (gl::transparent *region-bbox* 0.3))
  (let* ((ret) (total-mass (send *robot* :weight))
         (contact-constraint-matrix-list
          (mapcar #'(lambda (ccons ccoords) (send ccons :calc-constraint-matrix ccoords))
                  contact-constraint-list contact-coords-list))
         (contact-constraint-vector-list
          (mapcar #'(lambda (x) (send x :get-constraint-vector)) contact-constraint-list))
         (local-contact-constraint-matrix-list
          (mapcar #'(lambda (x) (send x :get-constraint-matrix)) contact-constraint-list))
         (count 0)
         (max-count
          (* (let ((count 0)) (do ((x x-min (+ x dx))) ((> x x-max)) (incf count)) count)
             (let ((count 0)) (do ((y y-min (+ y dy))) ((> y y-max)) (incf count)) count))))
    (format t ";; Test QP = ~A~%" max-count)
    (do ((x x-min (+ x dx))) ((> x x-max))
        (let ((flag) (y-min) (y-max))
          ;;(setq lp-solver #'solve-coinclp-lp)
          (setq lp-solver #'solve-qpoases-lp)
          (setq y-min (calc-static-min-max-cog-pos
                        contact-coords-list
                        contact-constraint-matrix-list
                        contact-constraint-vector-list
                        local-contact-constraint-matrix-list
                        :ref-cog-x x
                        :use-local-contact t :lp-solver lp-solver
                        :axis :y
                        :min/max :min)
                y-max (calc-static-min-max-cog-pos
                        contact-coords-list
                        contact-constraint-matrix-list
                        contact-constraint-vector-list
                        local-contact-constraint-matrix-list
                        :ref-cog-x x
                        :use-local-contact t :lp-solver lp-solver
                        :axis :y
                        :min/max :max))
          (when (and y-min y-max)
          (do ((y (- y-min (* 2 dy)) (+ y dy))) ((> y (+ y-min (* 2 dy))))
              (incf count)
              (if (= 0 (mod count (/ max-count 10))) (format t ";;   Current Tested = ~A/~A~%" count max-count))
              (let ((cog-pos (float-vector x y ref-cog-z)))
                (unless (eq flag :skip)
                  (let ((qp-status (float-vector 0)))
                    (labels ((qp-solver
                              (&rest args)
                              (apply #'solve-qpoases-qp (append (list :status qp-status :debug nil) args))))
                      (let ((wr
                             (calc-static-wrenches-from-cog
                              cog-pos contact-coords-list contact-constraint-matrix-list contact-constraint-vector-list
                              :total-mass total-mass :qp-solver #'qp-solver)))
                        (if (and (not (= (round (elt qp-status 0)) 0)) flag)
                            (setq flag :skip))
                        (if (= (round (elt qp-status 0)) 0) ;; Solved
                            (progn
                              (setq flag t)
                              (if debug (format t ";; ~A OK ~A~%" cog-pos (elt qp-status 0)))
                              (push cog-pos ret))
                          (if debug (format t ";; ~A NG ~A~%" cog-pos (elt qp-status 0))))
                        ))))))
          (do ((y (- y-max (* 2 dy)) (+ y dy))) ((> y (+ y-max (* 2 dy))))
              (incf count)
              (if (= 0 (mod count (/ max-count 10))) (format t ";;   Current Tested = ~A/~A~%" count max-count))
              (let ((cog-pos (float-vector x y ref-cog-z)))
                (unless (eq flag :skip)
                  (let ((qp-status (float-vector 0)))
                    (labels ((qp-solver
                              (&rest args)
                              (apply #'solve-qpoases-qp (append (list :status qp-status :debug nil) args))))
                      (let ((wr
                             (calc-static-wrenches-from-cog
                              cog-pos contact-coords-list contact-constraint-matrix-list contact-constraint-vector-list
                              :total-mass total-mass :qp-solver #'qp-solver)))
                        (if (and (not (= (round (elt qp-status 0)) 0)) flag)
                            (setq flag :skip))
                        (if (= (round (elt qp-status 0)) 0) ;; Solved
                            (progn
                              (setq flag t)
                              (if debug (format t ";; ~A OK ~A~%" cog-pos (elt qp-status 0)))
                              (push cog-pos ret))
                          (if debug (format t ";; ~A NG ~A~%" cog-pos (elt qp-status 0))))
                        ))))))
          )))
    ;;(format t ";; Solved = ~A/~A~%" (length ret) (length cog-pos-list))
    (format t ";; Solved = ~A/~A~%" (length ret) max-count)
    (if return-polygon
        (instance polygon :init :vertices (quickhull (reverse ret)))
      (reverse ret))))

;;(defun calc-static-feasible-cog-region-from-contact-constraint-iterative-qp
(defun calc-feasible-ZMP-z-static-region-from-contact-constraint-iterative-qp
  (ref-cog-z contact-coords-list contact-constraint-list
   &key (debug nil) (return-polygon t) (set-region-bbox)
        (x-min -1000) (x-max 100) (y-min -700) (y-max 700)
        (dxy 10) (dx dxy) (dy dxy)
        ;;(cog-pos-list (gen-xyz-list-from-xy-min-max :x-min x-min :x-max x-max :y-min y-min :y-max y-max :dx dx :dy dy :z ref-cog-z))
        )
  "Calculate static feasible cog region (SCFR) considering statics and contact constraint using QP iteratively.
   cog-pos-list is list of cog pos [mm] candidate corresponding to searching area."
  (when set-region-bbox
    (setq *region-bbox* (send (make-bounding-box cog-pos-list) :body))
    (send *region-bbox* :set-color #f(0 1 0))
    (gl::transparent *region-bbox* 0.3))
  (let* ((ret) (total-mass (send *robot* :weight))
         (contact-constraint-matrix-list
          (mapcar #'(lambda (ccons ccoords) (send ccons :calc-constraint-matrix ccoords))
                  contact-constraint-list contact-coords-list))
         (contact-constraint-vector-list
          (mapcar #'(lambda (x) (send x :get-constraint-vector)) contact-constraint-list))
         (local-contact-constraint-matrix-list
          (mapcar #'(lambda (x) (send x :get-constraint-matrix)) contact-constraint-list))
         (count 0)
         (max-count
          (* (let ((count 0)) (do ((x x-min (+ x dx))) ((> x x-max)) (incf count)) count)
             (let ((count 0)) (do ((y y-min (+ y dy))) ((> y y-max)) (incf count)) count))))
    (format t ";; Test QP = ~A~%" max-count)
    (do ((x x-min (+ x dx))) ((> x x-max))
        (do ((y y-min (+ y dy))) ((> y y-max))
            (incf count)
            (if (= 0 (mod count (/ max-count 10))) (format t ";;   Current Tested = ~A/~A~%" count max-count))
            (let ((cog-pos (float-vector x y ref-cog-z)))
              (let ((qp-status (float-vector 0)))
                (labels ((qp-solver
                          (&rest args)
                          (apply #'solve-qpoases-qp (append (list :status qp-status :debug nil) args))))
                  (let ((wr
                         (calc-wrenches-from-zmp-z-static
                          cog-pos contact-coords-list contact-constraint-matrix-list contact-constraint-vector-list local-contact-constraint-matrix-list
                          :total-mass total-mass :qp-solver #'qp-solver)))
                    (if (= (round (elt qp-status 0)) 0) ;; Solved
                        (progn
                          (if debug (format t ";; ~A OK ~A~%" cog-pos (elt qp-status 0)))
                          (push cog-pos ret))
                      (if debug (format t ";; ~A NG ~A~%" cog-pos (elt qp-status 0))))
                    ))))))
    ;;(format t ";; Solved = ~A/~A~%" (length ret) (length cog-pos-list))
    (format t ";; Solved = ~A/~A~%" (length ret) max-count)
    (if return-polygon
        (instance polygon :init :vertices (quickhull (reverse ret)))
      (reverse ret))))

(defun calc-wrenches-from-zmp-z-static-org
  (zmp-pos contact-coords-list contact-constraint-matrix-list contact-constraint-vector-list
           &key (total-mass (send *robot* :weight)) (cog-acc (float-vector 0 0 0)) (diff-angular-momentum (float-vector 0 0 0))
           (qp-solver #'solve-qpoases-qp))
  "Calculate all EE wrenches considering COG statics."
  (let* ((zmp-pos (scale 1e-3 zmp-pos))
         (equality-matrix
          (make-matrix 3 (* 6 (length contact-coords-list))
                       (list
                        (apply
                         #'append
                         (mapcar #'(lambda (c)
                                     (float-vector 0 0 1 0 0 0))
                                 contact-coords-list))
                        (apply
                         #'append
                         (mapcar #'(lambda (c)
                                     (float-vector (elt zmp-pos 2) 0 0 0 -1 0))
                                 contact-coords-list))
                        (apply
                         #'append
                         (mapcar #'(lambda (c)
                                     (float-vector 0 (elt zmp-pos 2) 0 1 0 0))
                                 contact-coords-list)))))
         (mg (* total-mass 1e-3 1e-3 (elt *g-vec* 2)))
         (equality-vector
          (float-vector mg (* (elt zmp-pos 0) mg) (* (elt zmp-pos 1) mg))))
    (funcall
     #'qp-solver
     :eval-weight-matrix (unit-matrix (* 6 (length contact-coords-list)))
     :equality-matrix equality-matrix
     :equality-vector equality-vector
     :inequality-matrix (apply #'concatenate-matrix-diagonal contact-constraint-matrix-list)
     :inequality-min-vector (apply #'concatenate float-vector contact-constraint-vector-list)))
  )

(defun calc-wrenches-from-zmp-z-static
  (zmp-pos contact-coords-list contact-constraint-matrix-list contact-constraint-vector-list local-contact-constraint-matrix-list
           &key (total-mass (send *robot* :weight)) (cog-acc (float-vector 0 0 0)) (diff-angular-momentum (float-vector 0 0 0))
                (use-local-contact t)
                (qp-solver #'solve-qpoases-qp))
  "Calculate all EE wrenches considering COG statics."
  (let* ((Gf (apply #'concatenate-matrix-row
                    (mapcar #'(lambda (x)
                                (concatenate-matrix-row
                                 (if use-local-contact
                                     (send x :worldrot)
                                   (unit-matrix 3))
                                 (make-matrix 3 3)))
                            contact-coords-list)))
         (dG (apply #'concatenate-matrix-row
                    (mapcar #'(lambda (x)
                                (concatenate-matrix-row
                                 (if use-local-contact
                                     (m* (outer-product-matrix (scale 1e-3 (v- (send x :worldpos) zmp-pos))) (send x :worldrot))
                                   (outer-product-matrix (scale 1e-3 (v- (send x :worldpos) zmp-pos))))
                                 (if use-local-contact
                                     (send x :worldrot)
                                   (unit-matrix 3))
                                 ))
                            contact-coords-list)))
         (wrench-dim (* 6 (length contact-coords-list)))
         (equality-matrix
          (concatenate-matrix-column
           (concatenate-matrix-row
            (concatenate-matrix-column
             (m* (make-matrix 2 3 (list (list 1 0 0) (list 0 1 0))) Gf)
             (m* (make-matrix 1 3 (list (list 0 0 1))) dG))
            (scale-matrix -1 (unit-matrix 3))
            )
           (concatenate-matrix-row
            (concatenate-matrix-column
             (m* (make-matrix 1 3 (list (list 0 0 1))) Gf)
             (m* (make-matrix 2 3 (list (list 1 0 0) (list 0 1 0))) dG))
            (make-matrix 3 3))))
         (mg (* total-mass 1e-3 1e-3 (elt *g-vec* 2)))
         (equality-vector (float-vector 0 0 0 mg 0 0))
         (contact-matrix (if use-local-contact
                             (apply #'concatenate-matrix-diagonal local-contact-constraint-matrix-list)
                           (apply #'concatenate-matrix-diagonal contact-constraint-matrix-list)))
         (contact-vector (apply #'concatenate float-vector contact-constraint-vector-list))
         (inequality-matrix (concatenate-matrix-row contact-matrix (make-matrix (array-dimension contact-matrix 0) 3))))
    ;;(print contact-matrix)
    (funcall
     #'qp-solver
     :eval-weight-matrix (unit-matrix (+ 3 wrench-dim))
     :eval-coeff-vector (instantiate float-vector (+ wrench-dim 3))
     :equality-matrix equality-matrix
     :equality-vector equality-vector
     :inequality-matrix inequality-matrix
     :inequality-min-vector contact-vector
     ;;:print-level :pl-low
     ;;:solve-qp-mode :sqp-with-hotstart
     ;;:debug nil
     ))
  )

(defun calc-static-wrenches-from-cog
  (cog-pos contact-coords-list contact-constraint-matrix-list contact-constraint-vector-list
   &key (total-mass (send *robot* :weight)) (cog-acc (float-vector 0 0 0)) (diff-angular-momentum (float-vector 0 0 0))
        (qp-solver #'solve-qpoases-qp))
  "Calculate all EE wrenches considering COG statics."
  (wrench-distribute-from-total-wrench
   contact-coords-list
   contact-constraint-matrix-list
   :robot *robot*
   :total-wrench
   (calc-total-wrench-from-cog-motion :cog-pos cog-pos :total-mass total-mass :cog-acc cog-acc :diff-angular-momentum diff-angular-momentum)
   :contact-constraint-vector-list contact-constraint-vector-list
   :qp-solver qp-solver))

(defun calc-total-wrench-from-cog-motion
  (&key (total-mass (send *robot* :weight))
        (cog-pos (send *robot* :centroid nil))
        (cog-acc (float-vector 0 0 0))
        (diff-angular-momentum (float-vector 0 0 0)))
  "Calculate static total wrench at COG around world origin.
   wrench is [N] and [Nm]."
  (let* ((mg (scale (* 1e-3 total-mass) (v+ cog-acc (scale 1e-3 *g-vec*)))))
    (concatenate float-vector mg (v+ diff-angular-momentum (v* (scale 1e-3 cog-pos) mg)))
    ))

