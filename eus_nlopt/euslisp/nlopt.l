(defvar m (load-foreign "../lib/libnlopt_wrapper.so"))

;; algorithm
(defconstant DIRECT 0)
(defconstant G_DIRECT 1)
(defconstant DIRECT_L 2)
(defconstant G_DIRECT_L 3)
(defconstant CRS 4)
(defconstant STOGO 5)
(defconstant ISRES 6)
(defconstant CCSA 7)
(defconstant SLSQP 8)
(defconstant L_BFGS 9)
(defconstant TN 10)
(defconstant SL_VM 11)
(defconstant NUM_ALGORITHMS 12)

;; result status
(defconstant NLOPT_FAILURE 4)
(defconstant NLOPT_INVALID_ARGS 3)
(defconstant NLOPT_OUT_OF_MEMORY 2)
(defconstant NLOPT_ROUNDOFF_LIMITED 1)
(defconstant NLOPT_FORCED_STOP 0)
(defconstant NLOPT_SUCCESS 6)
(defconstant NLOPT_STOPVAL_REACHED 7)
(defconstant NLOPT_FTOL_REACHED 8)
(defconstant NLOPT_XTOL_REACHED 9)
(defconstant NLOPT_MAXEVAL_REACHED 10)
(defconstant NLOPT_MAXTIME_REACHED 11)

(defforeign optimize m "optimize" (:string :string :string ;; x E [min max]
					   :integer :integer ;; object
					   :integer :integer ;; eq const
					   :integer :integer ;; neq const
					   :integer :integer :integer ;; dim
					   :double :double :double ; thres
					   :integer ; log?
					   :integer ; alg
					   :string :string ;; buf 
					   :string :string
					   :string :string
					   ) :string)
(defforeign get-result m "get_result" () :integer)

(defun-c-callable my_print ((v1 :string) (v2 :string)) :integer
  (print v1) (print v2) 0)
(defun-c-callable nop ((v1 :string) (v2 :string)) :integer 0)

(defvar xbuf)
;;
(defvar fbuf)
(defvar dfbuf)
;;
(defvar gbuf)
(defvar dgbuf)
;;
(defvar hbuf)
(defvar dhbuf)

;; util
(defun my-copy-matrix (mat vec)
  (let ((buf (if (matrixp mat) (cdr (assoc 'entity (send mat :slots))) mat)))
    (dotimes (i (length vec))
      (setf (aref vec i) (aref buf i)))))

;; sample
(defun-c-callable sample-f ((v1 :string) (v2 :string)) :integer
  (setf (aref v2 0) (sqrt (aref v1 1)))
  0)

(defun-c-callable sample-df ((v1 :string) (v2 :string)) :integer
  (setf (aref v2 0) 0)
  (setf (aref v2 1) (/ (sqrt (aref v1 1)) 2.0))
  0)

(defun-c-callable sample-g ((v1 :string) (v2 :string)) :integer
  (setf (aref v2 0) (+ (aref v1 0) (aref v1 1) -10))
  0)

(defun-c-callable sample-dg ((v1 :string) (v2 :string)) :integer
  (setf (aref v2 0) 1.0)
  (setf (aref v2 1) 1.0)
  0)

(defun-c-callable sample-h ((v1 :string) (v2 :string)) :integer
  (setf (aref v2 0)
	(- (* (* 2 (aref v1 0)) (* 2 (aref v1 0)) (* 2 (aref v1 0)))
	   (aref v1 1)))
  0)

(defun-c-callable sample-dh ((v1 :string) (v2 :string)) :integer
  (setf (aref v2 0)
	(* 3 2 (* 2 (aref v1 0)) (* 2 (aref v1 0))))
  (setf (aref v2 1) -1)
  0)

(defun nlopt-optmize
  (&key
   (x0 (float-vector 5 5))
   (x-min #F(0 0 ))
   (x-max #F(10 10))
   (m-x (length x0))
   ;;
   (f 'sample-f) (df 'sample-df)
   (g 'sample-g) (dg 'sample-dg)
   (h 'sample-h) (dh 'sample-dh)
   (m-g 1) (m-h 1)
   ;;
   (alg 9)
   (ftol 1e-16) (xtol 1e-8) (eqthre 1e-4)
   (log? t)
   ;;
   (fbuf (instantiate float-vector 1))
   (dfbuf (instantiate float-vector (* 1 m-x)))
   (gbuf (instantiate float-vector m-g))
   (dgbuf (instantiate float-vector (* m-g m-x)))
   (hbuf (instantiate float-vector m-h))
   (dhbuf (instantiate float-vector (* m-h m-x)))
   ;;
   (timer (instance mtimer :init))
   (timebuf #F(0))
   ret)
  (cond
   (log?
    (format t "ALGORITHM: ~A~%"
	    (aref (vector "DIRECT" "G_DIRECT" "DIRECT_L" "G_DIRECT_L"
			  "CRS" "STOGO" "ISRES" "CCSA" "SLSQP" "L_BFGS"
			  "TN" "SL_VM")
		  alg))))
  (setq
   ret
   (optimize
    x0 x-min x-max
    (pod-address f) (pod-address df)
    (pod-address g) (pod-address dg)
    (pod-address h) (pod-address dh)
    m-x m-g m-h
    ftol xtol eqthre
    (if log? 1 0)
    alg
    fbuf dfbuf gbuf dgbuf hbuf dhbuf))
  (setf (aref timebuf 0) (send timer :stop))
  (cond
   (log? (format t "TIME: ~A sec~%" (aref timebuf 0))))
  ret
  )

(defun test nil
  (dolist
      (alg
       (list DIRECT
	     G_DIRECT
	     DIRECT_L
	     G_DIRECT_L
	     CRS
             ;STOGO un-supported
	     ISRES
	     CCSA
	     SLSQP
	     L_BFGS
	     TN
	     SL_VM))
    (nlopt-optmize :ftol 1e-2 :xtol 1e-2 :eqthre 1e-1 :alg alg)))

;(nlopt-optmize)