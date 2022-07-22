#!/usr/bin/env roseus

(require "package://eus_caffe/euslisp/eus-caffe.l")

(defun gen+3x-2y+4-data
  (&key (size 3200) (isize (* size 2)) (dsize size))
  (let* ((idata (instantiate float-vector isize))
	 (ddata (instantiate float-vector dsize)))
    (dotimes (i size)
      (let* ((x (* 10 (- (random 2.0) 1.0)))
	     (y (* 10 (- (random 2.0) 1.0))))
	(setf (aref idata (* i 2)) x)
	(setf (aref idata (+ 1 (* i 2))) y)
	(setf (aref ddata i) (+ (* 3 x) (* -2 y) 4))
	))
    (list idata ddata)))

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
    (setq idata (gen+3x-2y+4-data))
    (setq ddata (nth 1 idata))
    (setq idata (nth 0 idata))))
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

(defun unix-command-line
  (cmd)
  (let* ((p (piped-fork cmd))
         (ret (read-line p nil)))
    (close p)
    ret))

(defun smart-initialize-net
  nil
  (let* ((caffemodel (unix-command-line "ls -t | grep -e \"caffemodel$\" | head -n 1")))
    (warning-message 6 "load caffe model ~A~%" caffemodel)
    (caffe::gen-test-net :netproto "predict_linear_equation_net.prototxt"
                         :caffemodel caffemodel))
  )
