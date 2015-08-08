#!/usr/bin/env roseus

(require "../learn.l")
(require "package://eus_nlopt/euslisp/nlopt.l")

(caffe::gen-test-net :netproto "ik_net_6x200_predict.prototxt"
		     :caffemodel "ik_net_6x200_NESTEROV_iter_1000000.caffemodel")
(caffe::output-log 0)
(random 1.0 (integer-vector (car (unix::gettimeofday)) 0))

(defvar *input*)

(defun _target-loss (v1 v2)
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
    (send-all (send *robot* :links) :worldcoords)
    (setq c (make-coords :pos (scale 1e+3 (subseq input 0 3))
			 :rot (matrix-exponent (subseq input 3 6))))
    (setq c (send (send (car (send *robot* :links)) :copy-worldcoords)
		  :transform c :local))
    ;;
    (concatenate float-vector
		 (scale 1e-2
			(send c :difference-position
			      (send *robot* :rarm :end-coords)))
		 (send c :difference-rotation
		       (send *robot* :rarm :end-coords)))
    ))

(defun _normal-target-loss (v1 v2)
  (let* ((joint-list (butlast (send *robot* :rarm :joint-list)))
	 c)
    ;;
    (map cons '(lambda (j av) (send j :joint-angle (- (rad2deg av) (send j :joint-angle))
				    :relative t))
	 joint-list v1)
    (send-all (send *robot* :links) :worldcoords)
    (setq c (make-coords :pos (scale 1e+3 (subseq *input* 0 3))
			 :rot (matrix-exponent (subseq *input* 3 6))))
    (setq c (send (send (car (send *robot* :links)) :copy-worldcoords)
		  :transform c :local))
    ;; (send *viewer*  :draw-objects)
    ;;
    (concatenate float-vector
		 (scale 1e-2
			(send c :difference-position
			      (send *robot* :rarm :end-coords)))
		 (send c :difference-rotation
		       (send *robot* :rarm :end-coords)))
    ))

(defun-c-callable target-loss ((v1 :string) (v2 :string)) :integer
  (let* ((dif (_target-loss v1 v2)))
    (setf (aref v2 0)
	  (apply 'max (map cons 'abs dif)))
    0
    ))

(defun-c-callable normal-target-loss ((v1 :string) (v2 :string)) :integer
  (let* ((dif (_normal-target-loss v1 v2)))
    (setf (aref v2 0)
	  (apply 'max (map cons 'abs dif)))
    0
    ))

(defun-c-callable torque-objective ((v1 :string) (v2 :string)) :integer
  (let* ((joint-list (butlast (send *robot* :rarm :joint-list))))
    (_target-loss v1 v2)
    ;;
    (send *robot* :calc-torque :force-list (list (float-vector 1000 0 0)) :moment-list (list (float-vector 0 0 0)) :target-coords (list (send *robot* :rarm :end-coords)))
    ;;
    (setf
     (aref v2 0)
     (* 1e-3 (apply 'max (mapcar 'abs (send-all joint-list :joint-torque)))))
    0
    ))

(defun-c-callable normal-torque-objective ((v1 :string) (v2 :string)) :integer
  (let* ((joint-list (butlast (send *robot* :rarm :joint-list))))
    (_normal-target-loss v1 v2)
    ;;
    (send *robot* :calc-torque :force-list (list (float-vector 1000 0 0)) :moment-list (list (float-vector 0 0 0)) :target-coords (list (send *robot* :rarm :end-coords)))
    ;;
    (setf
     (aref v2 0)
     (* 1e-3 (apply 'max (mapcar 'abs (send-all joint-list :joint-torque)))))
    0
    ))

(defun-c-callable nop ((v1 :string) (v2 :string)) :integer
  0)

(defun-c-callable check-ik-thre ((v1 :string) (v2 :string)) :integer
  (let* ((dif (_target-loss v1 v2))
	 (thre (float-vector 0.5 0.5 0.5 0.5 0.5 0.5)))
    (dotimes (i 6)
      (setf (aref v2 i) (- (abs (aref dif i)) (aref thre i))))
    )
  0)

(defun-c-callable normal-check-ik-thre ((v1 :string) (v2 :string)) :integer
  (let* ((dif (_normal-target-loss v1 v2))
	 (thre (float-vector 0.5 0.5 0.5 0.5 0.5 0.5)))
    (dotimes (i 6)
      (setf (aref v2 i) (- (abs (aref dif i)) (aref thre i))))
    )
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
	;;
	(objective 'target-loss) ;; torque-objective
	)
  (setq *input* target)
  (let* ((c (make-coords :pos (scale 1e+3 (subseq *input* 0 3))
			 :rot (matrix-exponent (subseq *input* 3 6))))
	 x (f (float-vector 0)))
    (setq c (send (send (car (send *robot* :links)) :copy-worldcoords)
		  :transform c :local))
    (setq
     x
     (nlopt-optimize
      :initial-state (float-vector 0)
      :state-min-vector (float-vector 0)
      :state-max-vector (float-vector 1)
      :evaluation-function objective
      :evaluation-function-gradient 'nop
      :equality-function 'nop
      :equality-function-gradient 'nop
      :inequality-function 'check-ik-thre ;;'nop
      :inequality-function-gradient 'nop
      :alg G_DIRECT
      :ftol 1e-3
      :xtol 1e-3
      :eqthre 1e-3
      :max-eval 50
      :max-time 1
      :timebuf time-buf
      :log? draw?
      :equality-dimension 0
      :inequality-dimension 6
      ))
    (funcall objective x f)
    (cond
     (draw?
      ;; (send *viewer* :draw-objects :flush nil)
      ;; (send c :draw-on :flush nil :color (float-vector 1 0 0) :size 100)
      (send *arrow* :newcoords (send c :copy-worldcoords))
      (send *viewer* :draw-objects :flush nil)
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
     (cons :x x) (cons :f f)
     (cons :time (aref time-buf 0)))
    ))

(defun normal-ik-search
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
	(joint-list (butlast (send *robot* :rarm :joint-list)))
	(time-buf (float-vector 0))
	(draw? t)
	;;
	(objective 'normal-target-loss) ;; 'normal-torque-objective
	)
  (setq *input* target)
  (let* ((c (make-coords :pos (scale 1e+3 (subseq *input* 0 3))
			 :rot (matrix-exponent (subseq *input* 3 6))))
	 x (f (float-vector 0)))
    (setq c (send (send (car (send *robot* :links)) :copy-worldcoords)
		  :transform c :local))
    (send *arrow* :newcoords (send c :copy-worldcoords))
    (setq
     x
     (nlopt-optimize
      :initial-state (map float-vector 'deg2rad (send-all joint-list :joint-angle))
      :state-min-vector (map float-vector 'deg2rad (send-all joint-list :min-angle))
      :state-max-vector (map float-vector 'deg2rad (send-all joint-list :max-angle))
      :evaluation-function objective
      :evaluation-function-gradient 'nop
      :equality-function 'nop
      :equality-function-gradient 'nop
      :inequality-function 'normal-check-ik-thre ;;'nop
      :inequality-function-gradient 'nop
      :alg G_DIRECT
      :ftol 1e-3
      :xtol 1e-3
      :eqthre 1e-3
      :max-eval 10000000
      :max-time (* 60 15)
      :timebuf time-buf
      :log? draw?
      :equality-dimension 0
      :inequality-dimension 6
      ))
    (funcall objective x f)
    (cond
     (draw?
      ;; (send *viewer* :draw-objects :flush nil)
      ;; (send c :draw-on :flush nil :color (float-vector 1 0 0) :size 100)
      (send *arrow* :newcoords (send c :copy-worldcoords))
      (send *viewer* :draw-objects :flush nil)
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
     (cons :x x) (cons :f f)
     (cons :time (aref time-buf 0)))
    ))

(defun torque-eval-once
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
	(joint-list (butlast (send *robot* :rarm :joint-list)))
	(time-buf (float-vector 0))
	(draw? t)
	;;
	(objective 'normal-target-loss) ;; torque-objective
	;; (ret (float-vector 0))
	(timer (instance mtimer :init))
	)
  (setq *input* target)
  (let* ((c (make-coords :pos (scale 1e+3 (subseq *input* 0 3))
			 :rot (matrix-exponent (subseq *input* 3 6)))))
    (setq c (send (send (car (send *robot* :links)) :copy-worldcoords)
		  :transform c :local))
    (send *arrow* :newcoords (send c :copy-worldcoords))
    (send *robot* :rarm :inverse-kinematics c
	  :thre 5 :rthre 0.5)
    (send *robot* :calc-torque :force-list (list (float-vector 1000 0 0)) :moment-list (list (float-vector 0 0 0)) :target-coords (list (send *robot* :rarm :end-coords)))
    ;;
    (list
     (cons :diff
	   (concatenate float-vector
			(scale 1e-3
			       (send c :difference-position
				     (send *robot* :rarm :end-coords)))
			(send c :difference-rotation
			      (send *robot* :rarm :end-coords))))
     (cons :x (map float-vector 'deg2rad (send-all joint-list :joint-angle)))
     (cons :f (* 1e-3 (apply 'max (mapcar 'abs (send-all joint-list :joint-torque)))))
     (cons :time (send timer :stop))
     )))

(defun torque-ik-analysis
  (&key (output "test")
	(size 100)
	;;
	(net-size (or (unix::getenv "NET_SIZE") "6x200"))
	(alg "NESTEROV")
	(log (open (format nil "torque_analysis.~A.~A.log.~A" net-size alg output) :direction :output))
	;;
	)
  (caffe::gen-test-net
   :netproto (format nil "ik_net_~A_predict.prototxt" net-size)
   :caffemodel (format nil "ik_net_~A_~A_iter_1000000.caffemodel" net-size alg))
  (dotimes (i size)
    ;; random target
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
		   :difference-rotation (send *robot* :rarm :end-coords :copy-worldcoords)))))
    ;;
    (format log "[~A] target=~A~%" i *input*)
    (format log ":ik ~A~%" (torque-eval-once :target *input*))
    (format log ":redundancy-ik ~A~%"
	    (redundancy-ik-search :target *input* :objective 'torque-objective))
    (format log ":direct ~A~%"
	    (normal-ik-search :target *input* :objective 'normal-torque-objective)))
  (close log)
  )


(defun redundancy-ik-analysis
  (&key (output "test")
	(func '(lambda (i) (redundancy-ik-search :draw? nil)))
	;;
	(size (expt 3 7))
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

(defun gather-candidates
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
	(cnt 0)
	(step 0.01)
	(draw-step 100)
	(draw-cnt 0)
	(thre (float-vector 0.05 0.05 0.05 0.5 0.5 0.5))
	(draw? t)
	)
  (setq *input* target)
  (let* ((_c (make-coords :pos (scale 1e+3 (subseq *input* 0 3))
			  :rot (matrix-exponent (subseq *input* 3 6))))
	 (c (send (send (car (send *robot* :links)) :copy-worldcoords)
		  :transform _c :local))
	 (joint-list (butlast (send *robot* :rarm :joint-list)))
	 ret)
    (while (< cnt 1.0)
      (let* ((input (concatenate float-vector *input* (float-vector cnt)))
	     (output
	      (caffe::calc-forward-double
	       :isize (length input) :osize (length joint-list)
	       :num 1 :channels (length input) :width 1 :height 1
	       :input input))
	     (update? t)
	     )
	;;
	(map cons '(lambda (j av) (send j :joint-angle (- (rad2deg av) (send j :joint-angle))
					:relative t))
	     joint-list output)
	(send-all (send *robot* :links) :worldcoords)
	(if (v<
	     (concatenate float-vector
			  (scale 1e-3
				 (send c :difference-position
				       (send *robot* :rarm :end-coords)))
			  (send c :difference-rotation
				(send *robot* :rarm :end-coords)))
	     thre)
	    (push output ret) (setq update? nil))
	;;
	(cond
	 ((and draw? (or update? (> draw-cnt draw-step)))
	  ;; (send *viewer* :draw-objects :flush nil)
	  ;; (send c :draw-on :flush nil :color (float-vector 1 0 0) :size 100)
	  (send *arrow* :newcoords (send c :copy-worldcoords))
	  (send *viewer* :draw-objects :flush nil)
	  (send *viewer* :viewsurface :flush)
	  (setq draw-cnt 0)
	  )))
      (setq cnt (+ cnt step))
      (setq draw-cnt (+ draw-cnt step)))
    ret))

;; (redundancy-ik-analysis)




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
     ;; (send *viewer* :draw-objects :flush nil)
     ;; (send c :draw-on :flush nil :color (float-vector 1 0 0) :size 100)
     (send *arrow* :newcoords (send c :copy-worldcoords))
     (send *viewer* :draw-objects :flush nil)
     (send *viewer* :viewsurface :flush)))
   ;;
   (if done (return-from nil (print min-dif)))
   (if (or (> id max) (< id min)) (setq step (* -1 step)))
   (cond
    ((> id max) (setq done t) (setq id (- min-dif-id step))))
   ))
