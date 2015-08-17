
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
		 (scale 1e-1
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
		 (scale 1e-1
			(send c :difference-position
			      (send *robot* :rarm :end-coords)))
		 (send c :difference-rotation
		       (send *robot* :rarm :end-coords)))
    ))

(defun-c-callable target-loss ((v1 :string) (v2 :string)) :integer
  (let* ((dif (_target-loss v1 v2)))
    (setf (aref v2 0)
	  ;; (apply 'max (map cons 'abs dif))
	  (norm dif)
	  )
    0
    ))

(defun-c-callable normal-target-loss ((v1 :string) (v2 :string)) :integer
  (let* ((dif (_normal-target-loss v1 v2)))
    (setf (aref v2 0)
	  ;; (apply 'max (map cons 'abs dif))
	  (norm dif)
	  )
    0
    ))

(defun-c-callable torque-objective ((v1 :string) (v2 :string)) :integer
  (let* ((joint-list (butlast (send *robot* :rarm :joint-list))))
    (_target-loss v1 v2)
    ;;
    (send *robot* :calc-torque :force-list (list (float-vector 0 0 0)) :moment-list (list (float-vector 0 0 0)) :target-coords (list (send *robot* :rarm :end-coords)))
    ;;
    (setf
     (aref v2 0)
     ;; (* 1e-3 (apply 'max (mapcar 'abs (send-all joint-list :joint-torque))))
     (* 1e-3 (norm (coerce (send-all joint-list :joint-torque) float-vector)))
     )
    0
    ))

(defun-c-callable normal-torque-objective ((v1 :string) (v2 :string)) :integer
  (let* ((joint-list (butlast (send *robot* :rarm :joint-list))))
    (_normal-target-loss v1 v2)
    ;;
    (send *robot* :calc-torque :force-list (list (float-vector 0 0 0)) :moment-list (list (float-vector 0 0 0)) :target-coords (list (send *robot* :rarm :end-coords)))
    ;;
    (setf
     (aref v2 0)
     ;; (* 1e-3 (apply 'max (mapcar 'abs (send-all joint-list :joint-torque))))
     (* 1e-3 (norm (coerce (send-all joint-list :joint-torque) float-vector)))
     )
    0
    ))

(defun-c-callable nop ((v1 :string) (v2 :string)) :integer
  0)

(defun-c-callable check-ik-thre ((v1 :string) (v2 :string)) :integer
  (let* ((dif (_target-loss v1 v2))
	 (thre (float-vector 5 5 5 0.5 0.5 0.5)))
    (dotimes (i 6)
      (setf (aref v2 i) (- (abs (aref dif i)) (aref thre i))))
    )
  0)

(defun-c-callable normal-check-ik-thre ((v1 :string) (v2 :string)) :integer
  (let* ((dif (_normal-target-loss v1 v2))
	 (thre (float-vector 5 5 5 0.5 0.5 0.5)))
    (dotimes (i 6)
      (setf (aref v2 i) (- (abs (aref dif i)) (aref thre i))))
    )
  0)
