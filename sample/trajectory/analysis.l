
(require "learn.l")
(require "package://eus_nlopt/euslisp/nlopt.l")

(let* ((predict_net (read-line (piped-fork "ls -v | grep predict | grep -e \"prototxt$\" | tail -1")))
       (candidate (remove-if #'(lambda (p) (not (substringp "caffemodel" p))) (directory ".")))
       (iteration (mapcar
		   #'(lambda (p)
		       (let* ((p- (subseq p 0 (- (length p) (length ".caffemodel"))))
			      (id (- (length p-) 1))
			      ret)
			 (while (and (>= id 0)
				     (<= (aref p- id) #\9) (>= (aref p- id) #\0))
			   (push (aref p- id) ret)
			   (decf id))
			 (setq ret (coerce ret string))
			 (if (plusp (length ret)) (setq ret (read-from-string ret)))
			 (if (numberp ret) ret)))
		   candidate))
       (max-iteration (apply 'max iteration))
       (max-candidates
	(mapcar 'cdr (remove-if #'(lambda (dcan) (not (eq (car dcan) max-iteration)))
				(mapcar 'cons iteration candidate))))
       ;;
       (caffemodel (or (find-if #'(lambda (d) (substringp "NESTEROV" d)) max-candidates)
		       (car max-candidates)))
       )
  (caffe::gen-test-net :netproto predict_net
		       :caffemodel caffemodel)
  (format t "[auto gen-test-net]~% predict: ~A~% caffemodel: ~A~%" predict_net caffemodel)
  )
;; (caffe::gen-test-net :netproto "traj_net_predict.prototxt"
;; :caffemodel "traj_net_NESTEROV_iter_1000000.caffemodel")
(caffe::output-log 0)
(random 1.0 (integer-vector (car (unix::gettimeofday)) 0))

(defvar *input*)
(defvar *traj-bspline* (instance partition-spline-vector :init :dimension (length *rarm-joints1*) :id-max 8 :recursive-order (make-list (length *rarm-joints1*) :initial-element 3) :x-min 0.0 :x-max 1.0))
(defvar *traj-bspline-jerk-gain*
  (mapcar
   '(lambda (b) (send b :calc-integral-objective-coeff-matrix :n 3))
   (send *traj-bspline* :partition-spline-list)))

(defun _update_redundancy (redundancy
			   &key (update-models? nil))
  (let* ((input (concatenate float-vector *input* (float-vector redundancy)))
	 (output
	  (caffe::calc-forward-double
	   :isize (length input) :osize 32
	   :num 1 :channels (length input) :width 1 :height 1
	   :input input))
	 c)
    ;;
    (cond
     (update-models?
      (map cons
	   #'(lambda (j s)
	       (send j :joint-angle
		     (- (+ (send j :min-angle)
			   (* s (- (send j :max-angle) (send j :min-angle))))
			(send j :joint-angle))
		     :relative t))
	   *rarm-joints2* (subseq input 3))
      (send-all (send-all *rarm-joints2* :child-link) :worldcoords)
      ;;
      (setq c (vector2coords (subseq input 0 3)))
      (send *goal* :newcoords c)
      ))
    (send *traj-bspline* :convert-gain-vector-to-gain-matrix output)
    output))

(defun _update_traj (x)
  (let* ((ret))
    (map cons
	 #'(lambda (j av)
	     (send j :joint-angle
		   (- (+ (send j :min-angle)
			 (* av (- (send j :max-angle) (send j :min-angle))))
		      (send j :joint-angle))
		   :relative t))
	 *rarm-joints1* (setq ret (send *traj-bspline* :calc x)))
    (send-all (send-all *rarm-joints1* :child-link) :worldcoords)
    ret))

(defun _target-loss (v1 v2)
  (_update_redundancy (aref v1 0) :update-models? nil)
  (concatenate
   float-vector
   (progn
     (_update_traj 1.0)
     (scale
      1000.0
      (v- (coords2vector (send *robot* :rarm :end-coords :copy-worldcoords))
	  (coords2vector (send *goal* :copy-worldcoords)))))
   (progn
     (_update_traj 0.0)
     (v- (map float-vector 'deg2rad (send-all *rarm-joints1* :joint-angle))
	 (map float-vector 'deg2rad (send-all *rarm-joints2* :joint-angle)))))
  ;; (v-
  ;;  (concatenate float-vector
  ;; 		(progn (_update_traj 1.0)
  ;; 		       (coords2vector (send *robot* :rarm :end-coords
  ;; 					    :copy-worldcoords)))
  ;; 		(_update_traj 0))
  ;;  (traj-input-vector))
  )

(defun-c-callable target-loss ((v1 :string) (v2 :string)) :integer
  (let* ((dif (_target-loss v1 v2)))
    (setf (aref v2 0) (norm dif))
    0
    ))

(defvar *jerk-objective-scale* 1)
(defun-c-callable jerk-objective ((v1 :string) (v2 :string)) :integer
  (let* ((dif (_target-loss v1 v2)))
    (setf
     (aref v2 0)
     (*
      *jerk-objective-scale*
      (apply
       '+
       (let*((id -1) g)
	 (mapcar
	  #'(lambda(b m)
	      (setq g (matrix-row (send *traj-bspline* :gain-matrix) (incf id)))
	      (v. g (transform m g)))
	  (send *traj-bspline* :partition-spline-list)
	  *traj-bspline-jerk-gain*)))))))

(defun-c-callable nop ((v1 :string) (v2 :string)) :integer
  0)

(defun-c-callable check-loss-thre ((v1 :string) (v2 :string)) :integer
  (let* ((dif (_target-loss v1 v2))
	 (thre (concatenate float-vector
			    (scale (* 1000 0.1) (float-vector 1 1 1))
			    (scale (deg2rad 10) (float-vector 1 1 1 1)))))
    (dotimes (i (length v2))
      (setf (aref v2 i) (- (abs (aref dif i)) (aref thre i))))
    )
  0)

(defun redundancy-traj-search
  (&key (input (random-traj-input-vector))
	(time-buf (float-vector 0))
	(draw? t)
	(check-trajectory-args nil)
	;;
	(objective 'target-loss) ;; torque-objective
	(inequality-dimension 7) ;; 7
	)
  (setq *input* input)
  (_update_redundancy 0 :update-models? t)
  (if draw? (send *viewer* :draw-objects))
  (let* (x (f (float-vector 0)))
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
      :inequality-function 'check-loss-thre ;;'nop
      :inequality-function-gradient 'nop
      :alg DIRECT
      :ftol 1e-3
      :xtol 1e-3
      :eqthre 1e-3
      :max-eval 300
      :max-time 3.0
      :timebuf time-buf
      :log? draw?
      :equality-dimension 0
      :inequality-dimension inequality-dimension
      ))
    (funcall objective x f)
    (cond
     ((or draw? check-trajectory-args)
      (apply
       'check-trajectory
       (append
	(if (listp check-trajectory-args) check-trajectory-args)
	(list :gain (_update_redundancy (aref x 0) :update-models? nil))))))
    ;;
    (list
     (cons :diff (_target-loss x nil))
     (cons :x x) (cons :f (aref f 0))
     (cons :time (aref time-buf 0)))
    ))

(defun redundancy-traj-search-analysis
  (&key (output-path "redundancy_traj_search_analysis.log")
	(log (open output-path :direction :output))
	(func '(lambda (i) (redundancy-traj-search :draw? nil :inequality-dimension 0)))
	;;
	(size (expt 3 7))
	(dif-max (float-vector 0 0 0 0 0 0 0))
	(dif-sum (float-vector 0 0 0 0 0 0 0))
	ret dif tm (avtm 0) dif-l tm-l
	(variance (float-vector 0 0 0 0 0 0 0))
	(tm-variance 0)
	;;
	(print-step (/ size 100))
	(print-cnt 0)
	)
  (format log "~A~%" :raw)
  (dotimes (i size)
    ;;
    (cond
     ((> (incf print-cnt) print-step)
      (format t "~A/~A = ~A~%" i size (/ (* i 1.0) size))
      (setq print-cnt 0)))
    ;;
    (setq ret (funcall func i))
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

(defun redundancy-minjerk-traj-search-analysis-once
  (&rest
   args
   &key
   (input (random-traj-input-vector))
   ret1 ret2
   &allow-other-keys
   )
  ;;
  (setq *jerk-objective-scale* 1)
  (setq
   ret1
   (redundancy-traj-search
    :draw? nil :input input
    :objective 'jerk-objective
    :inequality-dimension 7))
  ;;
  (setq *jerk-objective-scale* -1)
  (setq
   ret2
   (redundancy-traj-search
    :draw? nil :input input
    :objective 'jerk-objective
    :inequality-dimension 7))
  (list
   (cons :diff
	 (float-vector (cdr (assoc :time ret1))
		       (cdr (assoc :time ret2))
		       (cdr (assoc :f ret1))
		       (cdr (assoc :f ret2))))
   (cons :x (cdr (assoc :x ret1)))
   (cons :time 0)))

(defun redundancy-minjerk-traj-search-analysis
  (&rest
   args
   &key (output-path "redundancy_minjerk_traj_search_analysis.log")
   ;; (log (open output-path :direction :output))
   (func '(lambda (i) (redundancy-minjerk-traj-search-analysis-once)))
   )
  (apply 'redundancy-traj-search-analysis
	 (append args
		 (list :func func :output-path output-path)))
  )
