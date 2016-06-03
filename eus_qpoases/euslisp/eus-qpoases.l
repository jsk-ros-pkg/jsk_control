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

(defforeign _solve-qpoases-qp-with-hotstart
  *libeusqpoases*
  "solve_qpoases_qp_with_hotstart"
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

(defforeign _solve-qpoases-lp-with-hotstart
  *libeusqpoases*
  "solve_qpoases_lp_with_hotstart"
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

(defforeign _solve-qpoases-sqp-with-hotstart
  *libeusqpoases*
  "solve_qpoases_sqp_with_hotstart"
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

(defforeign _solve-qpoases-slp-with-hotstart
  *libeusqpoases*
  "solve_qpoases_slp_with_hotstart"
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
        (solve-qp-mode :qp)
        )
  (if (and inequality-matrix (= (length inequality-min-vector) 0))
      (setq inequality-min-vector (fill (instantiate float-vector inequality-dim) (- inf-var))))
  (if (and inequality-matrix (= (length inequality-max-vector) 0))
      (setq inequality-max-vector (fill (instantiate float-vector inequality-dim) inf-var)))
  (let ((ret (instantiate float-vector state-dim))
        (inequality-matrix
         (cond
          ((and equality-vector inequality-matrix)
           (concatenate-matrix-column equality-matrix inequality-matrix))
          (equality-matrix equality-matrix)
          (t inequality-matrix)))
        (inequality-max-vector
         (cond
          ((and equality-vector inequality-max-vector)
           (concatenate float-vector equality-vector inequality-max-vector))
          (equality-vector equality-vector)
          (t inequality-max-vector)))
        (inequality-min-vector
         (cond
          ((and equality-vector inequality-min-vector)
           (concatenate float-vector equality-vector inequality-min-vector))
          (equality-vector equality-vector)
          (t inequality-min-vector))))
    (when (and (= (length inequality-min-vector) 0) (= (length inequality-max-vector) 0))
      (setq inequality-matrix (make-matrix 1 state-dim))
      (setq inequality-min-vector (fill (instantiate float-vector 1) (- inf-var)))
      (setq inequality-max-vector (fill (instantiate float-vector 1) inf-var)))
    (funcall
     (case
      solve-qp-mode
      (:qp #'_solve-qpoases-qp)
      (:qp-with-hotstart #'_solve-qpoases-qp-with-hotstart)
      (:sqp-with-hotstart #'_solve-qpoases-sqp-with-hotstart))
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
               (-3 :QP-is-unbounded-and-thus-could-not-be-solved)
               (-100 :QP-problem-not-solved) ;; Not qpOASES original, check isSolved(). For example, hotstart.
               ))))
      ;; Return value
      (cond
       ((not check-status) ret)
       ((= int-status 0) ret)
       (t nil))
      )))

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
        (solve-lp-mode :lp)
        )
  (if (and inequality-matrix (= (length inequality-min-vector) 0))
      (setq inequality-min-vector (fill (instantiate float-vector inequality-dim) (- inf-var))))
  (if (and inequality-matrix (= (length inequality-max-vector) 0))
      (setq inequality-max-vector (fill (instantiate float-vector inequality-dim) inf-var)))
  (let ((ret (instantiate float-vector state-dim))
        (inequality-matrix
         (cond
          ((and equality-vector inequality-matrix)
           (concatenate-matrix-column equality-matrix inequality-matrix))
          (equality-matrix equality-matrix)
          (t inequality-matrix)))
        (inequality-max-vector
         (cond
          ((and equality-vector inequality-max-vector)
           (concatenate float-vector equality-vector inequality-max-vector))
          (equality-vector equality-vector)
          (t inequality-max-vector)))
        (inequality-min-vector
         (cond
          ((and equality-vector inequality-min-vector)
           (concatenate float-vector equality-vector inequality-min-vector))
          (equality-vector equality-vector)
          (t inequality-min-vector))))
    (when (and (= (length inequality-min-vector) 0) (= (length inequality-max-vector) 0))
      (setq inequality-matrix (make-matrix 1 state-dim))
      (setq inequality-min-vector (fill (instantiate float-vector 1) (- inf-var)))
      (setq inequality-max-vector (fill (instantiate float-vector 1) inf-var)))
    (funcall
     (case
      solve-lp-mode
      (:lp #'_solve-qpoases-lp)
      (:lp-with-hotstart #'_solve-qpoases-lp-with-hotstart)
      (:slp-with-hotstart #'_solve-qpoases-slp-with-hotstart))
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
               (-3 :QP-is-unbounded-and-thus-could-not-be-solved)
               (-100 :QP-problem-not-solved) ;; Not qpOASES original, check isSolved(). For example, hotstart.
               ))))
      ;; Return value
      (cond
       ((not check-status) ret)
       ((= int-status 0) ret)
       (t nil))
      )))

