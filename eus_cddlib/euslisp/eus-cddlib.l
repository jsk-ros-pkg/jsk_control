(defvar *libeuscddlib* (load-foreign (format nil "~A/lib/libeus_cddlib.so" (ros::resolve-ros-path "package://eus_cddlib"))))

(defforeign _cddlib-initialize
  *libeuscddlib*
  "cddlib_initialize"
  ()
  :integer)

(defforeign _cddlib-finalize
  *libeuscddlib*
  "cddlib_finalize"
  ()
  :integer)

(defforeign _cddlib-h-to-v
  *libeuscddlib*
  "cddlib_H_to_V"
  (:string ;; A_eq
   :string ;; b_eq
   :string ;; A_ineq
   :string ;; b_ineq
   :integer ;; d
   :integer ;; m_eq
   :integer ;; m_ineq
   :string ;; out_n
   :string ;; ont_s
   :integer ;; verbose
   )
  :integer)

(defforeign _cddlib-get-v
  *libeuscddlib*
  "cddlib_get_V"
  (:string ;; V
   :string ;; R
   :integer ;; d
   :integer ;; n
   :integer ;; s
   )
  :integer)

(defforeign _cddlib-v-to-h
  *libeuscddlib*
  "cddlib_V_to_H"
  (:string ;; V
   :string ;; R
   :integer ;; d
   :integer ;; n
   :integer ;; s
   :string ;; m_eq
   :string ;; m_ineq
   :integer ;; verbose
   )
  :integer)

(defforeign _cddlib-get-h
  *libeuscddlib*
  "cddlib_get_H"
  (:string ;; A_eq
   :string ;; b_eq
   :string ;; A_ineq
   :string ;; b_ineq
   :integer ;; d
   :integer ;; m_eq
   :integer ;; m_ineq
   )
  :integer)

;;input
;; A_eq x + b_eq = 0
;; A_ineq x + b_ineq >= 0
;;output
;; (list V R)
;; x = V y + R z (sum of y = 1, y >= 0, z >= 0)
(defun cddlib-H-to-V
    (&key
       (A_eq)
       (b_eq)
       (A_ineq)
       (b_ineq)
       (verbose 0)
       )
  (cond
    ((and A_eq A_ineq) t)
    (A_eq (setq A_ineq (make-matrix 0 (array-dimension A_eq 1)))
          (setq b_ineq (instantiate float-vector (array-dimension A_eq 1))))
    (A_ineq (setq A_eq (make-matrix 0 (array-dimension A_ineq 1)))
            (setq b_eq (instantiate float-vector (array-dimension A_ineq 1))))
    (t (return-from cddlib-H-to-V (list (make-matrix 0 0) (make-matrix 0 0)))))
  (let ((n (instantiate integer-vector 1))
        (s (instantiate integer-vector 1))
        (d (array-dimension A_eq 1))
        )
    (_cddlib-H-to-V
     (array-entity A_eq)
     b_eq
     (array-entity A_ineq)
     b_ineq
     d
     (array-dimension A_eq 0)
     (array-dimension A_ineq 0)
     n
     s
     verbose)
    (let ((V (make-matrix d (elt n 0)))
          (R (make-matrix d (elt s 0)))
          )
      (_cddlib-get-V
       (array-entity V)
       (array-entity R)
       d
       (elt n 0)
       (elt s 0))
      (list V R)
      )
    )
  )

;;input
;; x = V y + R z (sum of y = 1, y >= 0, z >= 0)
;;output
;; (list A_eq b_eq A_ineq b_ineq)
;; A_eq x + b_eq = 0
;; A_ineq x + b_ineq >= 0
(defun cddlib-V-to-H
    (&key
       (V)
       (R)
       (verbose 0)
       )
  (cond
    ((and V R) t)
    (V (setq R (make-matrix (array-dimension V 0) 0)))
    (R (setq V (make-matrix (array-dimension R 0) 0)))
    (t (return-from cddlib-V-to-H (list (make-matrix 0 0) (make-matrix 0 0) (make-matrix 0 0) (make-matrix 0 0)))))
  (let ((m_eq (instantiate integer-vector 1))
        (m_ineq (instantiate integer-vector 1))
        (d (array-dimension V 0))
        )
    (_cddlib-V-to-H
     (array-entity V)
     (array-entity R)
     d
     (array-dimension V 1)
     (array-dimension R 1)
     m_eq
     m_ineq
     verbose)
    (let ((A_eq (make-matrix (elt m_eq 0) d))
          (b_eq (instantiate float-vector (elt m_eq 0)))
          (A_ineq (make-matrix (elt m_ineq 0) d))
          (b_ineq (instantiate float-vector (elt m_ineq 0)))
          )
      (_cddlib-get-H
       (array-entity A_eq)
       b_eq
       (array-entity A_ineq)
       b_ineq
       d
       (elt m_eq 0)
       (elt m_ineq 0))
      (list A_eq b_eq A_ineq b_ineq)
      )
    )
  )

(_cddlib-initialize)
