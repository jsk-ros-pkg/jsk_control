#!/usr/bin/env roseus

(require "../learn.l")
(require "package://eus_nlopt/euslisp/nlopt.l")

(caffe::gen-test-net :netproto "ik_net_6x200_predict.prototxt"
		     :caffemodel "ik_net_6x200_NESTEROV_iter_1000000.caffemodel")
(caffe::output-log 0)
(random 1.0 (integer-vector (car (unix::gettimeofday)) 0))

(defvar *input*)

(defun-c-callable target-loss ((v1 :string) (v2 :string)) :integer
  ;; (print v1)
  (let* ((input (concatenate float-vector *input* v1))
	 (joint-list (butlast (send *robot* :rarm :joint-list)))
	 (output
	  (caffe::calc-forward-double
	   :isize (length input) :osize (length joint-list)
	   :num 1 :channels (length input) :width 1 :height 1
	   :input input))
	 c)
    ;;
    (map cons '(lambda (j av) (send j :joint-angle (- (rad2deg av) (send j :joint-angle))
				    :relative t))
	 joint-list output)
    (setq c (make-coords :pos (scale 1e+3 (subseq input 0 3))
			 :rot (matrix-exponent (subseq input 3 6))))
    (setq c (send (send (car (send *robot* :links)) :copy-worldcoords)
		  :transform c :local))
    ;;
    (setf
     (aref v2 0)
     (apply 'max
	    (map cons
		 'abs
		 (concatenate float-vector
			      (scale 1e-2
				     (send c :difference-position
					   (send *robot* :rarm :end-coords)))
			      (send c :difference-rotation
				    (send *robot* :rarm :end-coords))))))
    0
    ))

(defun-c-callable nop ((v1 :string) (v2 :string)) :integer
  0)

(defun redundancy-ik-search
  (&key (target
	 (setq *input*
	       (progn
		 (mapcar #'(lambda (j)
			     (send j :joint-angle
				   (+ (send j :min-angle)
				      (* (random 1.0)
					 (- (send j :max-angle) (send j :min-angle))))))
			 (butlast (send *robot* :rarm :joint-list)))
		 (send-all (send *robot* :links) :worldcoords)
		 (concatenate
		  float-vector
		  (scale 1e-3
			 (send (send (car (send *robot* :links)) :copy-worldcoords)
			       :difference-position (send *robot* :rarm :end-coords :copy-worldcoords)))
		  (send (send (car (send *robot* :links)) :copy-worldcoords)
			:difference-rotation (send *robot* :rarm :end-coords :copy-worldcoords))))))
	(time-buf (float-vector 0))
	(draw? t)
	)
  (setq *input* target)
  (let* ((c (make-coords :pos (scale 1e+3 (subseq *input* 0 3))
			 :rot (matrix-exponent (subseq *input* 3 6))))
	 ret)
    (setq c (send (send (car (send *robot* :links)) :copy-worldcoords)
		  :transform c :local))
    (setq
     ret
     (nlopt-optimize
      :initial-state (float-vector 0)
      :state-min-vector (float-vector 0)
      :state-max-vector (float-vector 1)
      :evaluation-function 'target-loss
      :evaluation-function-gradient 'nop
      :equality-function 'nop
      :equality-function-gradient 'nop
      :inequality-function 'nop
      :inequality-function-gradient 'nop
      :alg G_DIRECT
      :ftol 1e-3
      :xtol 1e-3
      :eqthre 1e-3
      :max-eval 50
      :max-time 1
      :timebuf time-buf
      :log? draw?
      ))
    (cond
     (draw?
      (send *viewer* :draw-objects :flush nil)
      (send c :draw-on :flush nil :color (float-vector 1 0 0) :size 100)
      (send *viewer* :viewsurface :flush)))
    ;;
    (list
     (cons :diff
	   (concatenate float-vector
			(scale 1e-3
			       (send c :difference-position
				     (send *robot* :rarm :end-coords)))
			(send c :difference-rotation
			      (send *robot* :rarm :end-coords))))
     (cons :time (aref time-buf 0)))
    ))

(defun redundancy-ik-analysis
  (&key (output "test")
	(func '(lambda (i) (redundancy-ik-search :draw? nil)))
	;;
	(size (expt 6 7))
	(dif-max (float-vector 0 0 0 0 0 0))
	(dif-sum (float-vector 0 0 0 0 0 0))
	(net-size (or (unix::getenv "NET_SIZE") "6x200"))
	(alg "NESTEROV")
	(log (open (format nil "analysis.~A.~A.log.~A" net-size alg output) :direction :output))
	ret dif tm (avtm 0) dif-l tm-l
	(variance (float-vector 0 0 0 0 0 0))
	(tm-variance 0)
	)
  ;;
  (caffe::gen-test-net
   :netproto (format nil "ik_net_~A_predict.prototxt" net-size)
   :caffemodel (format nil "ik_net_~A_~A_iter_1000000.caffemodel" net-size alg))
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
	    (+ (/ (abs (aref dif j)) size)
	       (aref dif-sum j))))
    (format log " ~%")
    ;;
    (setq avtm (+ avtm (/ tm size)))
    ;; (format t "tm: ~A, maxd: ~A, sumd: ~A~%" avtm dif-max dif-sum)
    (push dif dif-l)
    (push tm tm-l)
    ;;
    )
  (dolist (dif dif-l)
    (dotimes (j (length dif))
      (setf (aref variance j)
	    (+ (aref variance j)
	       (/ (expt (- (abs (aref dif j)) (aref dif-sum j)) 2) size)))))
  (dotimes (j (length dif))
    (setf (aref variance j) (sqrt (aref variance j))))
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
  (format log "~A~%" :variance)
  (format log "~A" tm-variance)
  (dotimes (j (length variance))
    (format log " ~A" (aref variance j)))
  (format log "~%")
  (close log)
  ;;
  (format t "tm: ~A, maxd: ~A, sumd: ~A, var: ~A~%" avtm dif-max dif-sum variance)
  (list avtm dif-max dif-sum)
  )

(redundancy-ik-analysis)




#|

(caffe::gen-test-net :netproto "redundancy_ik_net_predict.prototxt"
		     :caffemodel "redundancy_ik_net_x6_iter_60000000.caffemodel")
(let* ((id2 (random 1000)) (id 0.0) (step 0.0001) (max 0.1) (min 0.0) (d-cnt 0)
       (min-dif *inf*) (min-dif-id id) buf input output c (done nil)
       (joint-list (butlast (send *robot* :rarm :joint-list))))
  (caffe::db-open :path "target_lmdb_limb_coords")
  (caffe::db-read id2)
  (setq input (caffe::db-get-float-data))
  (setq input (concatenate float-vector input (float-vector 0)))
  (caffe::db-close)
  (do-until-key
   ;; (format t "[param] (db_id, rd_id) = (~A, ~A)~%" id2 id)
   ;; (format t "        (min_dif, rd_id) = (~A, ~A)~%" min-dif min-dif-id)
   (setq id (+ id step))
   (setf (aref input (- (length input) 1)) id)
   (setq output
	 (caffe::calc-forward-double
	  :isize (length input) :osize (length joint-list)
	  :num 1 :channels (length input) :width 1 :height 1
	  :input input))
   ;;
   (map cons '(lambda (j av) (send j :joint-angle (- (rad2deg av) (send j :joint-angle))
				   :relative t))
	joint-list output)
   (setq c (make-coords :pos (scale 1e+3 (subseq input 0 3))
			:rot (matrix-exponent (subseq input 3 6))))
   (setq c (send (send (car (send *robot* :links)) :copy-worldcoords)
		 :transform c :local))
   ;;
   (setq output
	 (concatenate float-vector
		      (scale 1e-2
			     (send c :difference-position
				   (send *robot* :rarm :end-coords)))
		      (send c :difference-rotation
			    (send *robot* :rarm :end-coords))))
   ;;
   (cond
    ((or (if (> min-dif (setq buf (apply 'max (map cons 'abs output))))
	     (setq min-dif buf min-dif-id id))
	 (> (incf d-cnt) (/ 0.1 step)))
     (setq d-cnt 0)
     (send *viewer* :draw-objects :flush nil)
     (send c :draw-on :flush nil :color (float-vector 1 0 0) :size 100)
     (send *viewer* :viewsurface :flush)))
   ;;
   (if done (return-from nil (print min-dif)))
   (if (or (> id max) (< id min)) (setq step (* -1 step)))
   (cond
    ((> id max) (setq done t) (setq id (- min-dif-id step))))
   ))
