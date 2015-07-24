#!/usr/bin/env roseus

(require "package://eus_caffe/euslisp/eus-caffe.l")

(defun linear-equation-test
  (&key
   (solver "linear_equation.prototxt")
   (size 3200)
   (dsize size)
   (isize (* 2 size))
   (idata)
   (ddata)
   (iclass (instantiate float-vector size)) ;; dummy
   (dclass iclass)
   )
  (cond
   ((or (not idata) (not ddata))
    (setq idata (instantiate float-vector isize))
    (setq ddata (instantiate float-vector dsize))
    (dotimes (i size)
      (let* ((x (* 10 (- (random 2.0) 1.0)))
	     (y (* 10 (- (random 2.0) 1.0))))
	(setf (aref idata (* i 2)) x)
	(setf (aref idata (+ 1 (* i 2))) y)
	(setf (aref ddata i) (+ (* 3 x) (* -2 y) 4))
	))))
  ;;
  (caffe::learn :solver solver
		:isize size
		:dsize size
		:idata idata
		:ddata ddata
		:iclass iclass
		:dclass dclass)
  ;;
  (format t " -- check net state~%  ~A->~A~%  ~A->~A~%  ~A->~A~%"
	  (float-vector 0 0)
	  (caffe::memory-calc-forward :input (float-vector 0 0) :osize 1)
	  (float-vector 0 2)
	  (caffe::memory-calc-forward :input (float-vector 0 2) :osize 1)
	  (float-vector 2 5)
	  (caffe::memory-calc-forward :input (float-vector 2 5) :osize 1))
  )
