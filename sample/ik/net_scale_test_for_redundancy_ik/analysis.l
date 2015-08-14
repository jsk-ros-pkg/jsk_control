#!/usr/bin/env roseus

(require "../learn.l")
(require "package://eus_nlopt/euslisp/nlopt.l")

(caffe::gen-test-net :netproto "ik_net_5x200_predict.prototxt"
		     :caffemodel "ik_net_5x200_NESTEROV_iter_10000000.caffemodel")
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
#f(0.022274 0.02484 0.024909 0.176786 0.149207 0.171939)
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
	(inequality-dimension 6)
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
      :alg DIRECT
      :ftol 1e-3
      :xtol 1e-3
      :eqthre 1e-3
      :max-eval 50
      :max-time 1
      :timebuf time-buf
      :log? draw?
      :equality-dimension 0
      :inequality-dimension inequality-dimension
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
	(alg DIRECT)
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
      :alg alg
      :ftol 1e-32
      :xtol 1e-32
      :eqthre 1e-5
      :max-eval 10000000
      :max-time (* 60 3)
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
	(size 500)
	;;
	(net-size (or (unix::getenv "NET_SIZE") "6x200"))
	(alg "NESTEROV")
	(log (open (format nil "torque_analysis.~A.~A.log.~A.small" net-size alg output) :direction :output))
	;;
	)
  ;; (caffe::gen-test-net
  ;; :netproto (format nil "ik_net_~A_predict.prototxt" net-size)
  ;; :caffemodel (format nil "ik_net_~A_~A_iter_1000000.caffemodel" net-size alg))
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
	    (progn
	      (normal-ik-search :target *input* :objective 'normal-torque-objective
				:alg DIRECT)
	      ;; (normal-ik-search :target *input* :objective 'normal-torque-objective
	      ;; :alg COBYLA)
	      )))
  (close log)
  )

(defun redundancy-ik-analysis
  (&key (output "test")
	(func '(lambda (i) (redundancy-ik-search :draw? nil :inequality-dimension 0)))
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
  ;; (caffe::gen-test-net
  ;; :netproto (format nil "ik_net_~A_predict.prototxt" net-size)
  ;; :caffemodel (format nil "ik_net_~A_~A_iter_1000000.caffemodel" net-size alg))
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
	       (/ (expt (- (aref dif j) (aref dif-sum j)) 2) size)))))
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

(send *irtviewer* :change-background (float-vector 1 1 1))
(setq avl (gather-candidates))
;;
(unix:system "mkdir img")
(dotimes (i (length avl))
  (map cons '(lambda (j av) (send j :joint-angle (rad2deg av)))
       (butlast (send *robot* :rarm :joint-list))
       (nth i avl))
  (send-all (send *robot* :links) :worldcoords)
  (send *viewer* :draw-objects)
  (send *viewer* :viewsurface :write-to-image-file (format nil "img/~A.jpg" i)))

(setq
 *torque-log*
 (let* ((in (open "torque_analysis.6x200.NESTEROV.log.test.small" :direction :input))
	buf ret ret-buf id)
   (while (setq buf (read-line in nil))
     (cond
      ((< (length buf) 3) 'nop)
      ((let* (s d)
	 (and (setq s (position #\[ buf))
	      (setq d (position #\] buf))
	      (setq id (subseq buf (+ 1 s) d))
	      (> (length id) 0)
	      (numberp (setq id (read-from-string id)))))
       (if ret-buf (push ret-buf ret))
       (setq ret-buf (list id)))
      (t
       (setq ret-buf (append ret-buf (list (read-from-string (format nil "(~A)" buf))))))
      ))
   (if ret-buf (push ret-buf ret))
   (close in)
   (remove-if '(lambda (d) (not (eq (length d) 4))) ret))
 ;;
 flat-data
 (mapcar
  '(lambda (dl) (mapcar '(lambda (d) (cond ((vectorp d) (elt d 0)) (t d))) dl))
  (mapcar
   #'(lambda (dl) (mapcar '(lambda (d) (cdr (assoc :f (cadr d)))) (cdr dl)))
   *torque-log*))
 tm-data
 (mapcar
  '(lambda (dl) (mapcar '(lambda (d) (cond ((vectorp d) (elt d 0)) (t d))) dl))
  (mapcar
   #'(lambda (dl) (mapcar '(lambda (d) (cdr (assoc :time (cadr d)))) (cdr dl)))
   *torque-log*))
 rate-data
 (mapcar
  '(lambda (dl) (list (/ (nth 0 dl) (nth 2 dl)) (/ (nth 1 dl) (nth 2 dl))))
  flat-data)
 rate-data2
 (mapcar
  #'(lambda (dl)
      ;; (format t "~A vs ~A~%" (/ (nth 0 dl) (nth 2 dl)) (/ (nth 1 dl) (nth 2 dl)))
      (/ (nth 1 dl) (nth 0 dl))
      ) flat-data)
 average (/ (apply '+ rate-data2) (length rate-data2))
 variance (/ (apply '+
		    (mapcar
		     #'(lambda (d) (expt (- d average) 2))
		     rate-data2))
	     (length rate-data2)))

(sort
 (mapcar 'cons
	 (mapcar 'car rate-data)
	 (mapcar '(lambda (d) (car (last d))) tm-data))
 '(lambda (a b) (< (car a) (car b))))

(let* ((log (open "torque_analysis.objective.dat" :direction :output))
       (rate-data (mapcar '(lambda (l) (list (/ 100 (car l)) (/ 100 (cadr l)))) rate-data))
       (rate-data2 (mapcar '(lambda (d) (* 100 d)) rate-data2))
       (av (list (/ (apply '+ (mapcar 'car rate-data)) (length rate-data))
		 (/ (apply '+ (mapcar 'cadr rate-data)) (length rate-data))
		 (/ (apply '+ rate-data2) (length rate-data2))
		 ))
       (vr
	(append
	 (mapcar
	  #'(lambda (rate av)
	      (sqrt (/ (apply '+ (mapcar #'(lambda (d) (expt (- d av) 2)) rate))
		       (length rate-data))))
	  (list (mapcar 'car rate-data) (mapcar 'cadr rate-data))
	  av)
	 (list
	  (sqrt (/ (apply '+ (mapcar #'(lambda (d) (expt (- d (caddr av)) 2)) rate-data2))
		   (length rate-data2))))))
       )
  (format log ":raw~%")
  (mapcar
   #'(lambda (rate rate2) (format log "~A ~A ~A~%" (car rate) (cadr rate) rate2))
   rate-data rate-data2)
  (format log ":average~%")
  (format log "~A ~A~%" (car av) (cadr av) (caddr av))
  (format log ":variance~%")
  (format log "~A ~A~%" (car vr) (cadr vr) (caddr vr))
  (close log))


(let* ((log (open "/tmp/buf" :direction :output)))
  (mapcar #'(lambda (d) (format log "~A~%" d)) rate-data2) ;;(mapcar 'cadr rate-data))
  (close log))
