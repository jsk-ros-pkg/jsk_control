#!/usr/bin/env roseus

(require "package://eus_caffe/euslisp/eus-caffe.l")
(require "package://eus_caffe/euslisp/eus-caffe-db.l")
(require "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknt-interface.l")
(require "models/arrow-object.l")

(require "package://contact_behavior_generation/euslisp/util/partition-spline/partition-spline.lisp")

(defvar *start* (arrow))
(defvar *goal* (arrow))
(defvar *robot* (hrp2jsknt))
(defvar *rarm-joints*
  (remove (send *robot* :rarm :wrist-y)
	  (butlast (send *robot* :rarm :joint-list))))

(objects (append
	  (list *start* *goal*)
	  (send-all *rarm-joints* :child-link)
	  (send *robot* :hand :rarm :links)))

(defvar *sample-points* (expt 3 (* 3 6)))

(defun coords2vector
  (c)
  (let* ((org (send (car (send *robot* :links)) :copy-worldcoords)))
    (concatenate
     float-vector
     (scale 1e-3 (send org :difference-position c))
     (send org :difference-rotation c))))

(defun vector2coords
  (input)
  (let* ((c (make-coords :pos (scale 1e+3 (subseq input 0 3))
			 :rot (matrix-exponent (subseq input 3 6)))))
    (send (send (car (send *robot* :links)) :copy-worldcoords)
	  :transform c :local)))

;; (let* ((v (concatenate float-vector (random-vector) (random-vector))) (c (vector2coords v)) (v2 (coords2vector c))) (print (v- v v2)))

(defun traj-learn
  (&rest args &key (solver "traj_solver.prototxt") &allow-other-keys)
  (apply
   'caffe::learn
   (append
    args
    (list :solver solver
	  :size 0))
   ))

(defun gen-traj-learning-data
  (&key (sep 3)
	(id-max 3)
	(joint-list (butlast (send *robot* :rarm :joint-list)))
	(sample-points (expt sep (* id-max (length joint-list))))
	;;
	(recursive-order 1)
	(x-min 0.0)
	(x-max 1.0)
	(bspline (instance partition-spline-vector :init :dimension (length joint-list) :id-max id-max :recursive-order (make-list (length joint-list) :initial-element recursive-order) :x-min x-min :x-max x-max))
	;;
	(draw? nil)
	;;
	(input-path "input_lmdb_traj")
	(target-path "target_lmdb_coords")
	(i 0)
	(print-step 1000)
	;; (random-index-vector (random-index-vector sample-points))
	(zero-str-length (length (format nil "~A" sample-points)))
	)
  (caffe::db-set-id 0)
  (if (not (zerop (caffe::db-open :dtype "lmdb" :path target-path :mode #\n)))
      (return-from gen-redundancy-ik-learning-data (print 'db-open-failed)))
  (caffe::db-set-id 1)
  (if (not (zerop (caffe::db-open :dtype "lmdb" :path input-path :mode #\n)))
      (return-from gen-redundancy-ik-learning-data (print 'db-open-failed)))
  ;;
  (labels ((_draw
	    nil
	    (cond
	     (draw?
	      (send *viewer* :draw-objects :flush nil)
	      (send (send *robot* :rarm :end-coords :worldcoords) :draw-on :flush nil
		    :color (float-vector 1 0 0) :size 100)
	      (send *viewer* :viewsurface :flush)
	      (x::window-main-one)
	      (unix::usleep (* 100 1000)))))
	   (itter
	    (v depth)
	    (cond
	     ((>= depth (length v))
	      (let* (v1 v2)
		(send bspline :convert-gain-vector-to-gain-matrix v)
		;;
		(map cons
		     #'(lambda (j av)
			 (send j :joint-angle
			       (- (+ (send j :min-angle)
				     (* av (- (send j :max-angle)
					      (send j :min-angle))))
				  (send j :joint-angle))
			       :relative t
			       ))
		     joint-list (send bspline :calc 0))
		(send-all (send *robot* :links) :worldcoords)
		(setq v1 (coords2vector (send *robot* :rarm :end-coords :copy-worldcoords)))
		(_draw)
		;;
		(map cons
		     #'(lambda (j av)
			 (send j :joint-angle
			       (- (+ (send j :min-angle)
				     (* av (- (send j :max-angle)
					      (send j :min-angle))))
				  (send j :joint-angle))
			       :relative t))
		     joint-list (send bspline :calc 1.0))
		(send-all (send *robot* :links) :worldcoords)
		(setq v2 (coords2vector (send *robot* :rarm :end-coords :copy-worldcoords)))
		(_draw)
		;;
		(caffe::db-set-id 1)
		(caffe::db-put-double
		 :channels (length v) :width 1 :height 1 :label i
		 :id_str (caffe::zero-string i zero-str-length)
		 :data v)
		;;
		(caffe::db-set-id 0)
		(caffe::db-put-double
		 :channels (+ (length v1) (length v2)) :width 1 :height 1 :label i
		 :id_str (caffe::zero-string i zero-str-length)
		 :data (concatenate float-vector v1 v2))
		(setq i (+ i 1))
		(if (eq 0 (mod i print-step))
		    (format t "[~A/~A]~%" i sample-points))
		))
	     (t
	      (dotimes (j sep)
		(setf (aref v depth) (/ (* 1.0 j) (- sep 1)))
		(itter v (+ depth 1))))
	     )))
    (itter (instantiate float-vector (* id-max (length joint-list))) 0)
    ;;
    (caffe::db-set-id 0) (caffe::db-close)
    (caffe::db-set-id 1) (caffe::db-close)
    ;; swap
    (format t "start random swapping~%")
    (caffe::db-set-id 0)
    (caffe::db-open :dtype "lmdb" :path target-path :mode #\w)
    (caffe::db-set-id 1)
    (caffe::db-open :dtype "lmdb" :path input-path :mode #\w)
    (caffe::db-random-swap-fast (* 3 sample-points) :size sample-points
                                :id-list '(0 1) :step (min (/ sample-points 30) 10000))
    (caffe::db-set-id 0) (caffe::db-close)
    (caffe::db-set-id 1) (caffe::db-close)
    ))

(defun traj-db-test
  (&key
   (id (random 1000))
   (input-path "input_lmdb_traj")
   (target-path "target_lmdb_coords")
   input target c1 c2 ret)
  (caffe::db-open :path target-path)
  (caffe::db-read id)
  (setq input (caffe::db-get-float-data))
  (caffe::db-close)
  ;;
  (setq c1 (vector2coords (subseq input 0 6)))
  (setq c2 (vector2coords (subseq input 6 12)))
  ;;
  (caffe::db-open :path input-path)
  (caffe::db-read id)
  (setq output (caffe::db-get-float-data))
  (caffe::db-close)
  ;;
  (send *start* :newcoords c1)
  (send *goal* :newcoords c1)
  (setq ret (check-trajectory :gain output))
  ;;
  (v- input
      (concatenate float-vector (car (last ret)) (car ret)))
  )

(defun check-trajectory
  (&key
   (robot *robot*)
   (jlist *rarm-joints*)
   (id-max 3)
   (recursive-order 1)
   (x-min 0.0)
   (x-max 1.0)
   (bspline (instance partition-spline-vector :init :dimension (length jlist) :id-max id-max :recursive-order (make-list (length jlist) :initial-element recursive-order) :x-min x-min :x-max x-max))
   ;;
   (cnt 30)
   (x-step (/ (- x-max x-min) (* 1.0 cnt)))
   (x-buf x-min)
   (gain
    (let* ((gain-list
	    (flatten (mapcar
		      #'(lambda (j)
			  (mapcar
			   #'(lambda (id) (random 1.0))
			   (make-list id-max)))
		      jlist))))
      (coerce gain-list float-vector)))
   ret
   )
  (send bspline :convert-gain-vector-to-gain-matrix gain)
  (while (<= x-buf x-max)
    (map cons
	 #'(lambda (j av) (send j :joint-angle
				(+ (send j :min-angle)
				   (* av (- (send j :max-angle) (send j :min-angle))))))
	 jlist (print (send bspline :calc x-buf)))
    (send-all (send *robot* :links) :worldcoords)
    (push (coords2vector (send *robot* :rarm :end-coords :copy-worldcoords)) ret)
    (send *viewer* :draw-objects)
    (x::window-main-one)
    (unix:usleep (round (* 0.01 1000 1000)))
    (setq x-buf (+ x-buf x-step))
    )
  ret
  )

#|

(defun demo-bspline-interpole
  (&key
   (robot *robot*)
   (jlist *rarm-joints*)
   (start-av (send-all jlist :joint-angle))
   (end-av (mapcar #'(lambda (j)
		       (+ (send j :min-angle)
			  (random (- (send j :max-angle) (send j :min-angle)))))
		   jlist))
   (id-max 3)
   (recursive-order 1)
   (x-min 0.0)
   (x-max 1.0)
   (bspline
    (mapcar #'(lambda (k)
		(instance basic-spline :init
			  :id-max id-max :recursive-order recursive-order
			  :x-min x-min :x-max x-max))
	    jlist))
   (initial-state
    (instantiate float-vector (* id-max (length bspline))))
   (state-min-vector
    (apply #'concatenate
	   (cons float-vector
		 (mapcar
		  #'(lambda (j) (make-list id-max :initial-element (send j :min-angle)))
		  jlist))))
   (state-max-vector
    (apply #'concatenate
	   (cons float-vector
		 (mapcar
		  #'(lambda (j) (make-list id-max :initial-element (send j :max-angle)))
		  jlist))))
   (equality-matrix-for-start/end-pos
    (matrix-append
     (map cons
	  #'(lambda (bs st ed)
	      (send bs :calc-gain-vector-coeff-matrix-from-via-x-list (list 0.0 0.99)))
	  bspline start-av end-av)
     '(1 1)))
   (equality-coeff-for-start/end-pos
    (concatenate float-vector
		 (flatten (map cons #'list start-av end-av))))
   ;;
   (equality-matrix-for-start/end-vel
    (matrix-append
     (map cons
	  #'(lambda (bs st ed)
	      (make-matrix
	       2 id-max
	       (mapcar
		#'(lambda (x) (send bs :calc-delta-coeff-vector x :n 1))
		(list 0.0 0.99))))
	  bspline start-av end-av)
     '(1 1)))
   (equality-coeff-for-start/end-vel
    (scale 0 (concatenate float-vector
			  (flatten (map cons #'list start-av end-av)))))
   ;;
   (equality-matrix-for-start/end-acc
    (matrix-append
     (map cons
	  #'(lambda (bs st ed)
	      (make-matrix
	       2 id-max
	       (mapcar
		#'(lambda (x) (send bs :calc-delta-coeff-vector x :n 2))
		(list 0.0 0.99))))
	  bspline start-av end-av)
     '(1 1)))
   (equality-coeff-for-start/end-acc
    (scale 0 (concatenate float-vector
			  (flatten (map cons #'list start-av end-av)))))
   ;;
   (eval-weight-matrix
    (let* ((mat
	    (matrix-append
	     (mapcar
	      #'(lambda (rate)
		  (matrix-append
		   (mapcar
		    #'(lambda (bs)
			(make-matrix
			 1 id-max
			 (list
			  (scale
			   1e-3
			   (send bs :calc-delta-coeff-vector
				 (+ x-min (* rate (- x-max x-min)))
				 :n 1)))))
		    bspline)
		   '(1 1)))
	      '(0 0.1 0.2 0.3 0.4 0.5 0.6 0.7 0.8 0.9 1.0))
	     '(1 0))))
      (m* (transpose mat) mat)))
   (cnt 30)
   (x-step (/ (- x-max x-min) (* 1.0 cnt)))
   (x-buf x-min)
   (ret (solve-eiquadprog
	 :debug? t
	 :initial-state initial-state
	 :eval-weight-matrix eval-weight-matrix
	 :state-min-vector state-min-vector
	 :state-max-vector state-max-vector
	 :equality-vector
	 (concatenate float-vector
		      equality-coeff-for-start/end-pos
		      equality-coeff-for-start/end-vel
		      equality-coeff-for-start/end-acc)
	 :equality-matrix
	 (matrix-append
	  (list equality-matrix-for-start/end-pos
		equality-matrix-for-start/end-vel
		equality-matrix-for-start/end-acc)
	  '(1 0))
	 ))
   )
  (if (null ret) (setq ret initial-state))
  (format t "   --- ~A x ~A = ~A variables~%" id-max (length start-av) (length initial-state))
  (let* ((retl (list (cons :gain ret))) p dp ddp (id) tau)
    (setq x-buf x-min)
    (while (<= x-buf x-max)
      (setq id 0)
      (mapcar
       #'(lambda (bs js)
	   (list
	    (send js :joint-angle
		  (send bs :calc x-buf (subseq ret id (+ id id-max))))
	    (send js :put :p (send bs :calc x-buf (subseq ret id (+ id id-max))))
	    (send js :put :dp (send bs :calc-delta x-buf (subseq ret id (+ id id-max)) :n 1))
	    (send js :put :ddp (send bs :calc-delta x-buf (subseq ret id (setq id (+ id id-max))) :n 2))
	    ))
       bspline jlist)
      (push (send-all jlist :get :ddp) ddp)
      (push (send-all jlist :get :dp) dp)
      (push (send-all jlist :get :p) p)
      (send *robot* :calc-torque-from-vel-acc
	    :jvv (map float-vector
		      #'(lambda (j) (deg2rad (or (send j :get :dp) 0)))
		      (cdr (send robot :links)))
	    :jav (map float-vector
		      #'(lambda (j) (deg2rad (or (send j :get :ddp) 0)))
		      (cdr (send robot :links))))
      (push (send-all jlist :joint-torque) tau)
      (setq x-buf (+ x-buf x-step))
      (send *viewer* :draw-objects)
      (x::window-main-one)
      (unix:usleep (round (* 0.01 1000 1000))))
    (push (cons :p (reverse p)) retl)
    (push (cons :dp (reverse dp)) retl)
    (push (cons :ddp (reverse ddp)) retl)
    (push (cons :tau (reverse tau)) retl)
    (format t "  [dif] |~A| = ~A~%"
	    (map float-vector #'- end-av (send-all jlist :joint-angle))
	    (norm (map float-vector #'- end-av (send-all jlist :joint-angle))))
    retl
    )
  )
