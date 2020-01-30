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
   :string ;; n
   :string ;; s_nonneg
   :string ;; s_free
   :integer ;; verbose
   )
  :integer)

(defforeign _cddlib-get-v
  *libeuscddlib*
  "cddlib_get_V"
  (:string ;; V
   :string ;; R_nonneg
   :string ;; R_free
   :integer ;; d
   :integer ;; n
   :integer ;; s_nonneg
   :integer ;; s_free
   )
  :integer)

(defforeign _cddlib-v-to-h
  *libeuscddlib*
  "cddlib_V_to_H"
  (:string ;; V
   :string ;; R_nonneg
   :string ;; R_free
   :integer ;; d
   :integer ;; n
   :integer ;; s_nonneg
   :integer ;; s_free
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
;; (list V R_nonneg R_free)
;; x = V y + R_nonneg z + R_free w (sum y = 1, y >= 0, z >= 0)
(defun cddlib-H-to-V
    (&key
       (A_eq)
       (b_eq)
       (A_ineq)
       (b_ineq)
       (verbose 0)
       )
  (cond
    ((and A_eq A_ineq) (unless (= (array-dimension A_eq 1) (array-dimension A_ineq 1))
                         (error "column size mismatch A_eq=~A, A_ineq=~A" (array-dimension A_eq 1) (array-dimension A_ineq 1))))
    (A_eq (setq A_ineq (make-matrix 0 (array-dimension A_eq 1)))
          (setq b_ineq (instantiate float-vector (array-dimension A_eq 1))))
    (A_ineq (setq A_eq (make-matrix 0 (array-dimension A_ineq 1)))
            (setq b_eq (instantiate float-vector (array-dimension A_ineq 1))))
    (t (return-from cddlib-H-to-V (list (make-matrix 0 0) (make-matrix 0 0) (make-matrix 0 0)))))
  (let ((n (instantiate integer-vector 1))
        (s_nonneg (instantiate integer-vector 1))
        (s_free (instantiate integer-vector 1))
        (d (array-dimension A_eq 1))
        )
    (if (= 0 (_cddlib-H-to-V
              (array-entity A_eq)
              b_eq
              (array-entity A_ineq)
              b_ineq
              d
              (array-dimension A_eq 0)
              (array-dimension A_ineq 0)
              n
              s_nonneg
              s_free
              verbose))
        (let ((V (make-matrix d (elt n 0)))
              (R_nonneg (make-matrix d (elt s_nonneg 0)))
              (R_free (make-matrix d (elt s_free 0)))
              )
          (_cddlib-get-V
           (array-entity V)
           (array-entity R_nonneg)
           (array-entity R_free)
           d
           (elt n 0)
           (elt s_nonneg 0)
           (elt s_free 0))
          (list V R_nonneg R_free)
          )
        nil)
    )
  )

;;input
;; x = V y + R_nonneg z + R_free w (sum y = 1, y >= 0, z >= 0)
;;output
;; (list A_eq b_eq A_ineq b_ineq)
;; A_eq x + b_eq = 0
;; A_ineq x + b_ineq >= 0
(defun cddlib-V-to-H
    (&key
       (V)
       (R_nonneg)
       (R_free)
       (verbose 0)
       )
  (cond
    ((and V R_nonneg R_free) (unless (= (array-dimension V 0) (array-dimension R_nonneg 0) (array-dimension R_free 0))
                               (error "row size mismatch V=~A, R_nonneg=~A, R_free~A" (array-dimension V 0) (array-dimension R_nonneg 0) (array-dimension R_free 0))))
    ((and V R_nonneg) (setq R_free (make-matrix (array-dimension V 0) 0)))
    ((and V R_free) (setq R_nonneg (make-matrix (array-dimension V 0) 0)))
    ((and R_nonneg R_free) (setq V (make-matrix (array-dimension R_nonneg 0) 0)))
    (V (progn (setq R_nonneg (make-matrix (array-dimension V 0) 0))
              (setq R_free (make-matrix (array-dimension V 0) 0))))
    (R_nonneg (progn (setq V (make-matrix (array-dimension R_nonneg 0) 0))
                     (setq R_free (make-matrix (array-dimension R_nonneg 0) 0))))
    (R_free (progn (setq V (make-matrix (array-dimension R_free 0) 0))
                   (setq R_free (make-matrix (array-dimension R_free 0) 0))))
    (t (return-from cddlib-V-to-H (list (make-matrix 0 0) (make-matrix 0 0) (make-matrix 0 0) (make-matrix 0 0)))))
  (let ((m_eq (instantiate integer-vector 1))
        (m_ineq (instantiate integer-vector 1))
        (d (array-dimension V 0))
        )
    (if (= 0 (_cddlib-V-to-H
              (array-entity V)
              (array-entity R_nonneg)
              (array-entity R_free)
              d
              (array-dimension V 1)
              (array-dimension R_nonneg 1)
              (array-dimension R_free 1)
              m_eq
              m_ineq
              verbose))
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
        nil)
    )
  )

(_cddlib-initialize)
