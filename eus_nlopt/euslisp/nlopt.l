;; (defvar *nlopt-plugin* (load-foreign "../lib/libnlopt_wrapper.so"))
(defvar *nlopt-plugin*
  (labels
      ((library_search
	(str &key
	     (depth 0)
	     colon-pos lib-path)
	(format t "  [~A] target=" depth)
	(cond
	 ((eq (length str) 0)
	  (format t "~% nlopt_plugin_not_found in nlopt.l~%")
	  (exit -1))
	 ((and (setq colon-pos (or (position #\: str) (length str)))
	       (setq lib-path (subseq str 0 colon-pos))
	       (setq lib-path
		     (if (eq (aref lib-path (- (length lib-path) 1)) #\/)
			 (subseq lib-path 0 (- (length lib-path) 1))
		       lib-path))
	       (probe-file (setq lib-path
				 (print (format nil "~A/libnlopt_wrapper.so" lib-path)))))
	  (load-foreign lib-path))
	 (t
	  (library_search (subseq str (min (length str) (+ colon-pos 1)))
			  :depth (+ depth 1))))))
    (library_search (format nil "~A:~A/lib"
			    (unix:getenv "LD_LIBRARY_PATH")
			    (read-line (piped-fork "rospack find eus_nlopt"))))))

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
;; no test
(defconstant COBYLA 12)
(defconstant BOBYQA 13)
(defconstant NEWUOA 14)
(defconstant PRAXIS 15)
(defconstant NelderMeadSimplex 16)
(defconstant Sbplx 17)
;;
(defconstant NUM_ALGORITHMS 18)

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

(defforeign _nlopt_optimize *nlopt-plugin*
  "optimize" (:string :string :string ;; x E [min max]
		      :integer :integer ;; object f
		      :integer :integer ;; eq const g = 0
		      :integer :integer ;; neq const  h <= 0
		      :integer :integer :integer ;; dim
		      :double :double :double ; thres
		      :integer :double ; max_eval, max_time
		      :integer ; log?
		      :integer ; alg
		      :string :string ;; buf 
		      :string :string
		      :string :string
		      ) :string)
(defforeign get-result *nlopt-plugin* "get_result" () :integer)
(defforeign stop *nlopt-plugin* "stop" () :integer)

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

;; (defun simple-jacobian
;;   (v1 v2 func)
;;   (let ((buf #f(0))
;; 	tmp
;; 	(org #F(0))
;; 	(delta 1e-10))
;;     (funcall func v1 org)
;;     (dotimes (i (length v1))
;;       (setq tmp (aref v1 i))
;;       (setf (aref v1 i) (+ (aref v1 i) delta))
;;       (funcall func v1 buf)
;;       (setf (aref v2 i) (/ (- (aref buf 0) (aref org 0)) delta))
;;       (setf (aref v1 i) tmp))))

(defun simple-jacobian
  (v1 v2 func &optional (w (/ (length v2) (length v1))))
  (let* ((h (length v1))
	 (buf (instantiate float-vector w))
	 tmp
	 (org (instantiate float-vector w))
	 (delta 1e-6))
    (funcall func v1 org)
    (dotimes (i h)
      (setq tmp (aref v1 i))
      (setf (aref v1 i) (+ (aref v1 i) delta))
      ;;
      (funcall func v1 buf)
      (dotimes (j w)
	(setf (aref v2 (+ j (* i w)))
	      (/ (- (aref buf j) (aref org j)) delta)))
      ;;
      (setf (aref v1 i) tmp))))

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

(defun nlopt-optimize
  (&key
   ((:initial-state x0) (float-vector 0 0))
   ((:state-min-vector x-min) #F(0 0 ))
   ((:state-max-vector x-max) #F(10 10))
   ((:state-dimension m-x) (length x0))
   ;;
   ((:evaluation-function f) 'sample-f)
   ((:evaluation-function-gradient df) 'sample-df)
   ((:equality-function g) 'sample-g) ;; g = 0
   ((:equality-function-gradient dg) 'sample-dg)
   ((:inequality-function h) 'sample-h) ;; h <= 0
   ((:inequality-function-gradient dh) 'sample-dh)
   ((:equality-dimension m-g) 1)
   ((:inequality-dimension m-h) 1)
   ;;
   (alg 9)
   (ftol 1e-16) (xtol 1e-8) (eqthre 1e-4)
   (max-eval -1)
   (max-time -1) ; sec
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
   ret
   &allow-other-keys)
  (cond
   (log?
    (format t "ALGORITHM: ~A(~A)~%"
	    (aref (vector "DIRECT" "G_DIRECT" "DIRECT_L" "G_DIRECT_L"
			  "CRS" "STOGO" "ISRES" "CCSA" "SLSQP" "L_BFGS"
			  "TN" "SL_VM"
			  "COBYLA" "BOBYQA" "NEWUOA" "PRAXIS" "NelderMeadSimplex" "Sbplx"
			  )
		  alg) alg)))
  (setq
   ret
   (_nlopt_optimize
    x0 x-min x-max
    (pod-address f) (pod-address df)
    (pod-address g) (pod-address dg)
    (pod-address h) (pod-address dh)
    m-x m-g m-h
    ftol xtol eqthre
    max-eval max-time
    (if log? 1 0)
    alg
    fbuf dfbuf gbuf dgbuf hbuf dhbuf))
  (setf (aref timebuf 0) (send timer :stop))
  (cond
   (log? (format t "TIME: ~A sec~%" (aref timebuf 0))))
  ret
  )

(defun test-nlopt nil
  (dolist
      (alg
       (list DIRECT
	     G_DIRECT
	     DIRECT_L
	     G_DIRECT_L
	     CRS
	     STOGO ;un-supported
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
	     NelderMeadSimplex
	     Sbplx
	     ))
    (nlopt-optimize :ftol 1e-10 :xtol 1e-10 :eqthre 1e-10
		    :max-time 1
		    :alg alg)))

;(nlopt-optmize)