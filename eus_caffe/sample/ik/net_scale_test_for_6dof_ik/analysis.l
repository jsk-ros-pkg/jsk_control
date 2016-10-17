#!/usr/bin/env roseus

(require "../learn.l")

(defun ik-analysis
  (&key (output "test")
	(func '(lambda (i) (ik-random-test :calc-forward 'caffe::calc-forward-double :draw? nil))))
  (let* ((size (/ (length *ef-coords-map*) 6))
	 (dif-max (float-vector 0 0 0 0 0 0))
	 (dif-sum (float-vector 0 0 0 0 0 0))
	 (dif-sum2 (float-vector 0 0 0 0 0 0))
	 (net-size (or (unix::getenv "NET_SIZE") "5x400"))
	 (iteration (or (unix::getenv "SOLVER_ITERATION") "5000000"))
	 (log (open (format nil "analysis_~A.log.~A" net-size output) :direction :output))
	 ret dif tm (avtm 0) dif-l tm-l
	 (variance (float-vector 0 0 0 0 0 0))
	 (variance2 (float-vector 0 0 0 0 0 0))
	 (tm-variance 0)
	 )
    ;;
    (caffe::gen-test-net
     :netproto (format nil "ik_net_~A_predict.prototxt" net-size)
     :caffemodel (format nil "ik_net_~A_iter_~A.caffemodel" net-size iteration))
    ;;
    (format log "~A~%" :raw)
    (dotimes (i size)
      (setq ret (funcall func i))
      ;; (ik-random-test :calc-forward 'caffe::calc-forward-double :draw? nil))
      (setq dif (cdr (assoc :diff ret)))
      (setq tm (cdr (assoc :time ret)))
      ;;
      (format log "~A" tm)
      (dotimes (j (length dif))
	(format log " ~A" (aref dif j))
	(setf (aref dif-max j)
	      (max (abs (aref dif j)) (aref dif-max j)))
	(setf (aref dif-sum j)
	      (+ (/ (aref dif j) size)
		 (aref dif-sum j)))
	(setf (aref dif-sum2 j)
	      (+ (/ (abs (aref dif j)) size)
		 (aref dif-sum2 j)))
	)
      (format log " ~%")
      ;;
      (setq avtm (+ avtm (/ tm size)))
      (format t "tm: ~A, maxd: ~A, sumd: ~A~%" avtm dif-max dif-sum)
      (push dif dif-l)
      (push tm tm-l)
      ;;
      )
    ;;
    (dolist (dif dif-l)
      (dotimes (j (length dif))
	(setf (aref variance j)
	      (+ (aref variance j)
		 (/ (expt (- (aref dif j) (aref dif-sum j)) 2) size)))))
    (dotimes (j (length dif))
      (setf (aref variance j) (sqrt (aref variance j))))
    ;;
    (dolist (dif dif-l)
      (dotimes (j (length dif))
	(setf (aref variance2 j)
	      (+ (aref variance2 j)
		 (/ (expt (- (aref dif j) (aref dif-sum2 j)) 2) size)))))
    (dotimes (j (length dif))
      (setf (aref variance2 j) (sqrt (aref variance2 j))))
    ;;
    (dolist (tm tm-l)
      (setq tm-variance (+ tm-variance (/ (expt (- tm avtm) 2) size))))
    (setq tm-variance (sqrt tm-variance))
    ;;
    (format log "~A~%" :average)
    (format log "~A" avtm)
    (dotimes (j (length dif))
      (format log " ~A" (aref dif-sum j)))
    (format log "~%")
    ;;
    (format log "~A~%" :abs_average)
    (format log "~A" avtm)
    (dotimes (j (length dif))
      (format log " ~A" (aref dif-sum2 j)))
    (format log "~%")
    ;;
    (format log "~A~%" :variance)
    (format log "~A" tm-variance)
    (dotimes (j (length variance))
      (format log " ~A" (aref variance j)))
    (format log "~%")
    ;;
    (format log "~A~%" :abs_variance)
    (format log "~A" tm-variance)
    (dotimes (j (length variance2))
      (format log " ~A" (aref variance2 j)))
    (format log "~%")
    ;;
    (close log)
    ;;
    (format t "tm: ~A, maxd: ~A, sumd: ~A, var: ~A~%" avtm dif-max dif-sum variance)
    (list avtm dif-max dif-sum)
    ))

(load-ik-learning-data)
(ik-analysis :output "test"
	     :func '(lambda (i) (ik-random-test :calc-forward 'caffe::calc-forward-double :draw? nil)))
(ik-analysis :output "train"
	     :func '(lambda (i) (ik-test i 'caffe::calc-forward-double nil)))
(exit 0)