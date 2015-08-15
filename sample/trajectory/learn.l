#!/usr/bin/env roseus

(require "package://eus_caffe/euslisp/eus-caffe.l")
(require "package://eus_caffe/euslisp/eus-caffe-db.l")
(require "package://eus_caffe/euslisp/eus-log.l")
(require "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknt-interface.l")
(require "models/arrow-object.l")

(require "package://contact_behavior_generation/euslisp/util/partition-spline/partition-spline.lisp")

(defvar *start* (arrow))
(defvar *goal* (arrow))
(defvar *robot* (hrp2jsknt))
(defvar *rarm-joints1*
  (subseq (send *robot* :rarm :joint-list) 0 4))
(defvar *rarm-joints2* (copy-object *rarm-joints1*))

(objects (append
	  (list *start* *goal*)
	  (send-all *rarm-joints1* :child-link)
	  (send-all *rarm-joints2* :child-link)
	  (send *robot* :hand :rarm :links)))

(defvar *sample-points* (expt 6 (+ (length *rarm-joints1*) (length *rarm-joints2*))))

(defun coords2vector
  (c)
  (let* ((org (send (car (send *robot* :links)) :copy-worldcoords)))
    ;; (concatenate
    ;; float-vector
    (scale 1e-3 (send org :difference-position c))
    ;; (send org :difference-rotation c))
    ))

(defun vector2coords
  (input)
  (let* ((c (make-coords :pos (scale 1e+3 (subseq input 0 3))
			 ;; :rot (matrix-exponent (subseq input 3 6))
			 )))
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

;; ;; deprecated
;; (defun gen-traj-learning-data
;;   (&key (sep 3)
;; 	(id-max 3)
;; 	(joint-list *rarm-joints1*)
;; 	(sample-points (expt sep (* id-max (length joint-list))))
;; 	;;
;; 	(recursive-order 1)
;; 	(x-min 0.0)
;; 	(x-max 1.0)
;; 	(bspline (instance partition-spline-vector :init :dimension (length joint-list) :id-max id-max :recursive-order (make-list (length joint-list) :initial-element recursive-order) :x-min x-min :x-max x-max))
;; 	;;
;; 	(draw? nil)
;; 	;;
;; 	(input-path "input_lmdb_traj")
;; 	(target-path "target_lmdb_coords")
;; 	(i 0)
;; 	(print-step 1000)
;; 	;; (random-index-vector (random-index-vector sample-points))
;; 	(zero-str-length (length (format nil "~A" sample-points)))
;; 	)
;;   (caffe::db-set-id 0)
;;   (if (not (zerop (caffe::db-open :dtype "lmdb" :path target-path :mode #\n)))
;;       (return-from gen-redundancy-ik-learning-data (print 'db-open-failed)))
;;   (caffe::db-set-id 1)
;;   (if (not (zerop (caffe::db-open :dtype "lmdb" :path input-path :mode #\n)))
;;       (return-from gen-redundancy-ik-learning-data (print 'db-open-failed)))
;;   ;;
;;   (labels ((_draw
;; 	    nil
;; 	    (cond
;; 	     (draw?
;; 	      (send *viewer* :draw-objects :flush nil)
;; 	      (send (send *robot* :rarm :end-coords :worldcoords) :draw-on :flush nil
;; 		    :color (float-vector 1 0 0) :size 100)
;; 	      (send *viewer* :viewsurface :flush)
;; 	      (x::window-main-one)
;; 	      (unix::usleep (* 100 1000)))))
;; 	   (itter
;; 	    (v depth)
;; 	    (cond
;; 	     ((>= depth (length v))
;; 	      (let* (v1 v2)
;; 		(send bspline :convert-gain-vector-to-gain-matrix v)
;; 		;;
;; 		(map cons
;; 		     #'(lambda (j av)
;; 			 (send j :joint-angle
;; 			       (- (+ (send j :min-angle)
;; 				     (* av (- (send j :max-angle)
;; 					      (send j :min-angle))))
;; 				  (send j :joint-angle))
;; 			       :relative t
;; 			       ))
;; 		     joint-list (send bspline :calc 0))
;; 		(send-all (send *robot* :links) :worldcoords)
;; 		(setq v1 (coords2vector (send *robot* :rarm :end-coords :copy-worldcoords)))
;; 		(_draw)
;; 		;;
;; 		(map cons
;; 		     #'(lambda (j av)
;; 			 (send j :joint-angle
;; 			       (- (+ (send j :min-angle)
;; 				     (* av (- (send j :max-angle)
;; 					      (send j :min-angle))))
;; 				  (send j :joint-angle))
;; 			       :relative t))
;; 		     joint-list (send bspline :calc 1.0))
;; 		(send-all (send *robot* :links) :worldcoords)
;; 		(setq v2 (coords2vector (send *robot* :rarm :end-coords :copy-worldcoords)))
;; 		(_draw)
;; 		;;
;; 		(caffe::db-set-id 1)
;; 		(caffe::db-put-double
;; 		 :channels (length v) :width 1 :height 1 :label i
;; 		 :id_str (caffe::zero-string i zero-str-length)
;; 		 :data v)
;; 		;;
;; 		(caffe::db-set-id 0)
;; 		(caffe::db-put-double
;; 		 :channels (+ (length v1) (length v2)) :width 1 :height 1 :label i
;; 		 :id_str (caffe::zero-string i zero-str-length)
;; 		 :data (concatenate float-vector v1 v2))
;; 		(setq i (+ i 1))
;; 		(if (eq 0 (mod i print-step))
;; 		    (format t "[~A/~A]~%" i sample-points))
;; 		))
;; 	     (t
;; 	      (dotimes (j sep)
;; 		(setf (aref v depth) (/ (* 1.0 j) (- sep 1)))
;; 		(itter v (+ depth 1))))
;; 	     )))
;;     (itter (instantiate float-vector (* id-max (length joint-list))) 0)
;;     ;;
;;     (caffe::db-set-id 0) (caffe::db-close)
;;     (caffe::db-set-id 1) (caffe::db-close)
;;     ;; swap
;;     (format t "start random swapping~%")
;;     (caffe::db-set-id 0)
;;     (caffe::db-open :dtype "lmdb" :path target-path :mode #\w)
;;     (caffe::db-set-id 1)
;;     (caffe::db-open :dtype "lmdb" :path input-path :mode #\w)
;;     (cpplog::change-output-stream "/tmp/traj_swap.log")
;;     (caffe::db-random-swap-fast (* 3 sample-points) :size sample-points
;;                                 :id-list '(0 1) :step (min (/ sample-points 30) 10000))
;;     (cpplog::change-output-stream "")
;;     (caffe::db-set-id 0) (caffe::db-close)
;;     (caffe::db-set-id 1) (caffe::db-close)
;;     ))

(defun gen-minjerk-traj-learning-data
  (&key (sep 6)
	(start-joint-list *rarm-joints2*)
	(end-joint-list *rarm-joints1*)
	(joint-list (append start-joint-list end-joint-list))
	;; (angle-vector
	;; (scale 0 (coerce (send-all joint-list :joint-angle) float-vector)))
	(sample-points (expt sep (length joint-list)))
	;;
	(draw? nil)
	;;
	(input-path "input_lmdb_traj")
	(target-path "target_lmdb_coords")
	(i 0)
	(print-step 100)
	(gc-step 10000)
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
		(map cons
		     #'(lambda (j av)
			 (send j :joint-angle
			       (- (+ (send j :min-angle)
				     (* av (- (send j :max-angle)
					      (send j :min-angle))))
				  (send j :joint-angle))
			       :relative t
			       ))
		     joint-list v)
		(send-all (send-all joint-list :child-link) :worldcoords)
		(setq v1 (concatenate float-vector (coords2vector (send *robot* :rarm :end-coords :copy-worldcoords)) (subseq v 0 (length start-joint-list))))
		(_draw)
		;;
		(setq v2
		      (send (gen-minjerk-gain :jlist end-joint-list  :start-av (subseq v 0 (length start-joint-list)) :end-av (subseq v (length start-joint-list))) :gain-vector))
		;;
		(caffe::db-set-id 1)
		(caffe::db-put-double
		 :channels (length v2) :width 1 :height 1 :label i
		 :id_str (caffe::zero-string i zero-str-length)
		 :data v2)
		;;
		(caffe::db-set-id 0)
		(caffe::db-put-double
		 :channels (length v1) :width 1 :height 1 :label i
		 :id_str (caffe::zero-string i zero-str-length)
		 :data v1)
		(setq i (+ i 1))
		(if (eq 0 (mod i print-step))
		    (format t "[~A/~A]~%" i sample-points))
		(if (eq 0 (mod i gc-step)) (sys::gc))
		))
	     (t
	      (dotimes (j sep)
		(setf (aref v depth) (/ (* 1.0 j) (- sep 1)))
		(itter v (+ depth 1))))
	     )))
    (itter (instantiate float-vector (length joint-list)) 0)
    ;;
    (caffe::db-set-id 0) (caffe::db-close)
    (caffe::db-set-id 1) (caffe::db-close)
    ;; swap
    (format t "start random swapping~%")
    (caffe::db-set-id 0)
    (caffe::db-open :dtype "lmdb" :path target-path :mode #\w)
    (caffe::db-set-id 1)
    (caffe::db-open :dtype "lmdb" :path input-path :mode #\w)
    (cpplog::change-output-stream "/tmp/traj_swap.log")
    (caffe::db-random-swap-fast (* 3 sample-points) :size sample-points
                                :id-list '(0 1) :step (min (/ sample-points 30) 10000))
    (cpplog::change-output-stream "")
    (caffe::db-set-id 0) (caffe::db-close)
    (caffe::db-set-id 1) (caffe::db-close)
    ))

(defun add-random-minjerk-traj-learning-data
  (&key (cnt 100)
	(start-joint-list *rarm-joints2*)
	(end-joint-list *rarm-joints1*)
	(joint-list (append start-joint-list end-joint-list))
	;;
	(draw? nil)
	;;
	(input-path "input_lmdb_traj")
	(target-path "target_lmdb_coords")
	;;
	(zero-str-length);; (length (format nil "~A" sample-points)))
	(sample-points)
	(print-step 1000)
	(gc-step 10000)
	(i -1)
	)
  (caffe::db-set-id 0)
  (if (not (zerop (caffe::db-open :dtype "lmdb" :path target-path :mode #\w)))
      (return-from gen-redundancy-ik-learning-data (print 'db-open-failed)))
  (caffe::db-set-id 1)
  (if (not (zerop (caffe::db-open :dtype "lmdb" :path input-path :mode #\w)))
      (return-from gen-redundancy-ik-learning-data (print 'db-open-failed)))
  (cond
   ((minusp i)
    (caffe::db-set-id 0)
    (setq i (caffe::db-get-size))
    (caffe::db-set-id 1)
    (if (not (eq  i (caffe::db-get-size)))
	(format t "-- db size mismatch ~A vs ~A~%" i (caffe::db-get-size)))))
  (setq zero-str-length (length (format nil "~A" (setq sample-points (+ i cnt)))))
  (format t " -- size : ~A~%" i)
  ;;
  (random 1.0 (integer-vector (car (unix::gettimeofday)) 0))
  (let ((v (instantiate float-vector (length joint-list))))
    (dotimes (j cnt)
      (dotimes (k (length v)) (setf (aref v k) (random 1.0)))
      (let* (v1 v2)
	(map cons
	     #'(lambda (j av)
		 (send j :joint-angle
		       (- (+ (send j :min-angle)
			     (* av (- (send j :max-angle)
				      (send j :min-angle))))
			  (send j :joint-angle))
		       :relative t
		       ))
	     joint-list v)
	(send-all (send-all joint-list :child-link) :worldcoords)
	(setq v1 (concatenate float-vector (coords2vector (send *robot* :rarm :end-coords :copy-worldcoords)) (subseq v 0 (length start-joint-list))))
	;;
	(setq v2
	      (send (gen-minjerk-gain :jlist end-joint-list  :start-av (subseq v 0 (length start-joint-list)) :end-av (subseq v (length start-joint-list))) :gain-vector))
	(if draw? (check-trajectory :gain v2))
	;;
	(caffe::db-set-id 1)
	(caffe::db-put-double
	 :channels (length v2) :width 1 :height 1 :label i
	 :id_str (caffe::zero-string i zero-str-length)
	 :data v2)
	;;
	(caffe::db-set-id 0)
	(caffe::db-put-double
	 :channels (length v1) :width 1 :height 1 :label i
	 :id_str (caffe::zero-string i zero-str-length)
	 :data v1)
	(setq i (+ i 1))
	(if (eq 0 (mod i print-step))
	    (format t "[~A/~A]~%" i sample-points))
	(if (eq 0 (mod i gc-step)) (sys::gc))
	))
    ;;
    (caffe::db-set-id 0) (caffe::db-close)
    (caffe::db-set-id 1) (caffe::db-close)
    ))

(defun traj-db-test
  (&key
   (id (random 1000))
   (input-path "input_lmdb_traj")
   (target-path "target_lmdb_coords")
   input target c ret)
  (caffe::db-open :path target-path)
  (caffe::db-read id)
  (setq input (caffe::db-get-float-data))
  (format t " -- size : ~A~%" (caffe::db-get-size))
  (caffe::db-close)
  ;;
  (setq c (vector2coords (subseq input 0 3)))
  (map cons
       #'(lambda (j s)
	   (send j :joint-angle
		 (- (+ (send j :min-angle)
		       (* s (- (send j :max-angle) (send j :min-angle))))
		    (send j :joint-angle))))
       *rarm-joints2* (subseq input 3))
  ;;
  (caffe::db-open :path input-path)
  (caffe::db-read id)
  (setq output (caffe::db-get-float-data))
  (format t " -- size : ~A~%" (caffe::db-get-size))
  (caffe::db-close)
  ;;
  (send *goal* :newcoords c)
  (setq ret (check-trajectory :gain output))
  ;;
  (v- input
      (concatenate float-vector (caar ret) (cadr (car (last ret)))))
  )

(defun gen-minjerk-gain
  (&key
   (jlist *rarm-joints1*)
   (start-av (map float-vector '(lambda (j) (random 1.0)) jlist))
   (end-av (map float-vector '(lambda (j) (random 1.0)) jlist))
   )
  (minjerk-interpole-partition-spline-vector :dimension (length start-av) :start-pos start-av :end-pos end-av :debug? nil :id-max 8))

(defun check-trajectory
  (&key
   (robot *robot*)
   (jlist *rarm-joints1*)
   (id-max 8)
   (recursive-order 3)
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
   ret tmp
   )
  (send bspline :convert-gain-vector-to-gain-matrix gain)
  (while (<= x-buf x-max)
    (map cons
	 #'(lambda (j av) (send j :joint-angle
				(+ (send j :min-angle)
				   (* av (- (send j :max-angle) (send j :min-angle))))))
	 jlist (setq tmp (send bspline :calc x-buf)))
    (send-all (send *robot* :links) :worldcoords)
    (push (list (coords2vector (send *robot* :rarm :end-coords :copy-worldcoords))
		(copy-seq tmp)) ret)
    (send *viewer* :draw-objects)
    (x::window-main-one)
    (unix:usleep (round (* 0.01 1000 1000)))
    (setq x-buf (+ x-buf x-step))
    )
  ret
  )

#|

(do-until-key (add-random-minjerk-traj-learning-data :cnt 10000 :input-path "input_lmdb_traj.cygnus" :target-path "target_lmdb_coords.cygnus"))
