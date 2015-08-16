(defvar *libeusqpoases* (load-foreign (format nil "~A/lib/libeus_qpoases.so" (ros::resolve-ros-path "package://eus_qpoases"))))

(defforeign _solve-qpoases-qp
  *libeusqpoases*
  "solve_qpoases_qp"
  (:string ;; ret
   :string ;; eval-weight-matrix
   :string ;; eval-coeff-vector
   :string ;; state-min-vector
   :string ;; state-max-vector
   :string ;; inequality-matrix
   :string ;; inequality-min-vector
   :string ;; inequality-max-vector
   :integer ;; state_len
   :integer ;; inequality_len
   :integer ;; print-level
   :string ;; return status
   )
  :string)

(defforeign _solve-qpoases-lp
  *libeusqpoases*
  "solve_qpoases_lp"
  (:string ;; ret
   :string ;; eval-coeff-vector
   :string ;; state-min-vector
   :string ;; state-max-vector
   :string ;; inequality-matrix
   :string ;; inequality-min-vector
   :string ;; inequality-max-vector
   :integer ;; state_len
   :integer ;; inequality_len
   :integer ;; print-level
   :string ;; return status
   )
  :string)

(defun solve-qpoases
  (&rest args)
  (warning-message 1 ";; solve-qpoases is deprecated!! Use solve-qpoases-qp function!!~%")
  (apply #'solve-qpoases-qp args))

(defun solve-qpoases-qp
  (&key (inf-var 1e35)
        (initial-state)
        (eval-weight-matrix)
        (eval-coeff-vector)
        (state-dim (array-dimension eval-weight-matrix 0))
        (equality-matrix)
        (equality-vector)
        (state-min-vector (fill (instantiate float-vector state-dim) (- inf-var)))
        (state-max-vector (fill (instantiate float-vector state-dim) inf-var))
        (inequality-matrix)
        (inequality-min-vector)
        (inequality-max-vector)
        (inequality-dim (if inequality-matrix (array-dimension inequality-matrix 0) 0))
        (print-level :pl-none)
        (debug t)
        (status (float-vector 0))
        (check-status t)
        )
  (if (and inequality-matrix (= (length inequality-min-vector) 0))
      (setq inequality-min-vector (fill (instantiate float-vector inequality-dim) (- inf-var))))
  (if (and inequality-matrix (= (length inequality-max-vector) 0))
      (setq inequality-max-vector (fill (instantiate float-vector inequality-dim) inf-var)))
  (labels ((concatenate-matrix-column
            (&rest args)
            (let (m ml (size-of-column (car (remove nil (mapcar #'(lambda (x) (array-dimension x 1)) args)))))
              (dolist (mat args)
                (if mat
                    (unless (= (array-dimension mat 1) size-of-column)
                      (error ";; concatenate-matrix-column matrix size error (size=~A)~%" (mapcar #'(lambda (x) (array-dimension x 1)) args))))
                (setq m (if mat (length (matrix-column mat 0)) 0))
                (dotimes (i m)
                  (push (matrix-row mat i) ml)))
              (when ml (apply #'matrix (reverse ml)))
              )))
    (let ((ret (instantiate float-vector state-dim))
          (inequality-matrix
           (if equality-matrix
               (concatenate-matrix-column equality-matrix inequality-matrix)
             inequality-matrix))
          (inequality-max-vector
           (if equality-vector
               (concatenate float-vector equality-vector inequality-max-vector)
             inequality-max-vector))
          (inequality-min-vector
           (if equality-vector
               (concatenate float-vector equality-vector inequality-min-vector)
             inequality-min-vector)))
      (when (and (= (length inequality-min-vector) 0) (= (length inequality-max-vector) 0))
        (setq inequality-matrix (make-matrix 1 state-dim))
        (setq inequality-min-vector (fill (instantiate float-vector 1) (- inf-var)))
        (setq inequality-max-vector (fill (instantiate float-vector 1) inf-var)))
      (_solve-qpoases-qp
       ret
       (array-entity eval-weight-matrix)
       eval-coeff-vector
       state-min-vector
       state-max-vector
       (array-entity inequality-matrix)
       inequality-min-vector
       inequality-max-vector
       state-dim
       (array-dimension inequality-matrix 0)
       (case print-level
             (:pl-debug-iter -2)
             (:pl-tabular -1)
             (:pl-none 0)
             (:pl-low 1)
             (:pl-medium 2)
             (:pl-high 3)
             (t 2))
       status)
      ;; Check status and return value
      (let ((int-status (round (elt status 0))))
        ;; Check and print
        (if debug
            (unless (= int-status 0)
              (print
               (case int-status
                 (0 :QP-problem-solved)
                 (1 :QP-could-not-be-solved-within-given-number-of-iterations)
                 (-1 :QP-could-not-be-solved-due-to-an-internal-error)
                 (-2 :QP-is-infeasible-and-thus-could-not-be-solved)
                 (-3 :QP-is-unbounded-and-thus-could-not-be-solved)))))
        ;; Return value
        (cond
         ((not check-status) ret)
         ((= int-status 0) ret)
         (t nil))
        ))))

;; !!Manial says LP by qpOASES in inefficient!!
(defun solve-qpoases-lp
  (&key (inf-var 1e35)
        (initial-state)
        (eval-coeff-vector)
        (state-dim (length eval-coeff-vector))
        (equality-matrix)
        (equality-vector)
        (state-min-vector (fill (instantiate float-vector state-dim) (- inf-var)))
        (state-max-vector (fill (instantiate float-vector state-dim) inf-var))
        (inequality-matrix)
        (inequality-min-vector)
        (inequality-max-vector)
        (inequality-dim (if inequality-matrix (array-dimension inequality-matrix 0) 0))
        (print-level :pl-none)
        (debug t)
        (status (float-vector 0))
        (check-status t)
        )
  (if (and inequality-matrix (= (length inequality-min-vector) 0))
      (setq inequality-min-vector (fill (instantiate float-vector inequality-dim) (- inf-var))))
  (if (and inequality-matrix (= (length inequality-max-vector) 0))
      (setq inequality-max-vector (fill (instantiate float-vector inequality-dim) inf-var)))
  (labels ((concatenate-matrix-column
            (&rest args)
            (let (m ml (size-of-column (car (remove nil (mapcar #'(lambda (x) (array-dimension x 1)) args)))))
              (dolist (mat args)
                (if mat
                    (unless (= (array-dimension mat 1) size-of-column)
                      (error ";; concatenate-matrix-column matrix size error (size=~A)~%" (mapcar #'(lambda (x) (array-dimension x 1)) args))))
                (setq m (if mat (length (matrix-column mat 0)) 0))
                (dotimes (i m)
                  (push (matrix-row mat i) ml)))
              (when ml (apply #'matrix (reverse ml)))
              )))
    (let ((ret (instantiate float-vector state-dim))
          (inequality-matrix
           (concatenate-matrix-column
            equality-matrix
            inequality-matrix))
          (inequality-max-vector
           (concatenate float-vector equality-vector inequality-max-vector))
          (inequality-min-vector
           (concatenate float-vector equality-vector inequality-min-vector)))
      (when (and (= (length inequality-min-vector) 0) (= (length inequality-max-vector) 0))
        (setq inequality-matrix (make-matrix 1 state-dim))
        (setq inequality-min-vector (fill (instantiate float-vector 1) (- inf-var)))
        (setq inequality-max-vector (fill (instantiate float-vector 1) inf-var)))
      (_solve-qpoases-lp
       ret
       eval-coeff-vector
       state-min-vector
       state-max-vector
       (array-entity inequality-matrix)
       inequality-min-vector
       inequality-max-vector
       (length ret)
       (array-dimension inequality-matrix 0)
       (case print-level
             (:pl-debug-iter -2)
             (:pl-tabular -1)
             (:pl-none 0)
             (:pl-low 1)
             (:pl-medium 2)
             (:pl-high 3)
             (t 2))
       status)
      ;; Check status and return value
      (let ((int-status (round (elt status 0))))
        ;; Check and print
        (if debug
            (unless (= int-status 0)
              (print
               (case int-status
                 (0 :QP-problem-solved)
                 (1 :QP-could-not-be-solved-within-given-number-of-iterations)
                 (-1 :QP-could-not-be-solved-due-to-an-internal-error)
                 (-2 :QP-is-infeasible-and-thus-could-not-be-solved)
                 (-3 :QP-is-unbounded-and-thus-could-not-be-solved)))))
        ;; Return value
        (cond
         ((not check-status) ret)
         ((= int-status 0) ret)
         (t nil))
        ))))

