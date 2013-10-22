
(require "nlopt-object.l")

(defclass test-nlopt-object
  :super nlopt-object
  :slots ())

(defmethod test-nlopt-object
  (:init
   (&rest
    args
    )
   (send-super*
    :init
    :x0 (float-vector 1 9)
    :x-min #F(0 0)
    :x-max #F(10 10)
    :m-x 2
    :m-g 1
    :m-h 1
    :ftol 1e-2
    :xtol 1e-2
    :eqthre 1e-1
    args
    ))
  (:f (v1 v2)
      (setf (aref v2 0) (sqrt (aref v1 1)))
      0)
  (:df (v1 v2)
       (setf (aref v2 0) 0)
       (setf (aref v2 1) (/ (sqrt (aref v1 1)) 2.0))
       0)
  (:g (v1 v2)
      (setf (aref v2 0) (+ (aref v1 0) (aref v1 1) -10))
      0)
  (:dg (v1 v2)
       (setf (aref v2 0) 1.0)
       (setf (aref v2 1) 1.0)
       0)
  (:h (v1 v2)
      (setf (aref v2 0)
	    (- (* (* 2 (aref v1 0)) (* 2 (aref v1 0)) (* 2 (aref v1 0)))
	       (aref v1 1)))
      0)
  (:dh (v1 v2)
       (setf (aref v2 0)
	     (* 3 2 (* 2 (aref v1 0)) (* 2 (aref v1 0))))
       (setf (aref v2 1) -1)
       0)
  )


(defun test
  (&optional
   (nlopt (instance test-nlopt-object :init)))
  (dolist
      (alg
       (list DIRECT
	     G_DIRECT
	     DIRECT_L
	     G_DIRECT_L
	     CRS
	     ;STOGO ;un-supported
	     ISRES
	     CCSA
	     SLSQP
	     L_BFGS
	     TN
	     SL_VM
	     COBYLA ;; beta
	     BOBYQA
	     NEWUOA
	     PRAXIS
	     ;NelderMeadSimplex ; never end
	     ;Sbplx
	     ))
    (send nlopt :optimize :alg alg)))
