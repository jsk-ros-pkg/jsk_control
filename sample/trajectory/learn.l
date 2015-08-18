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
		    (send j :joint-angle))
		 :relative t))
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
   (x-step (/ (- x-max x-min) (* 1.0 (or cnt 2))))
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
   (draw? t)
   (draw-stick? nil)
   &allow-other-keys
   )
  (send bspline :convert-gain-vector-to-gain-matrix gain)
  (while (<= x-buf x-max)
    (map cons
	 #'(lambda (j av)
	     (send j :joint-angle
		   (- (+ (send j :min-angle)
			 (* av (- (send j :max-angle) (send j :min-angle))))
		      (send j :joint-angle))
		   :relative t))
	 jlist (setq tmp (send bspline :calc x-buf)))
    (send-all (send *robot* :links) :worldcoords)
    (push (list (coords2vector (send *robot* :rarm :end-coords :copy-worldcoords))
		(copy-seq tmp)) ret)
    (setq x-buf (+ x-buf x-step))
    (cond
     (draw-stick?
      (apply 'draw-arm-line (if (listp draw-stick?) draw-stick? nil))
      )
     (draw?
      (send *viewer* :draw-objects)
      (x::window-main-one)
      (if (<= x-buf x-max) (unix:usleep (round (* 0.05 1000 1000))))
      )
     )
    ;; (if (eq x-min x-buf) (read-line))
    )
  ret
  )

(defun traj-input-vector
  nil
  (concatenate
   float-vector
   (coords2vector (send *goal* :copy-worldcoords))
   (map float-vector
	#'(lambda (j)
	    (/ (- (send j :joint-angle) (send j :min-angle))
	       (- (send j :max-angle) (send j :min-angle))))
	*rarm-joints2*)))

(defun random-traj-input-vector
  nil
  (mapcar
   #'(lambda (j) (send j :joint-angle
		       (+ (send j :min-angle)
			  (* (random 1.0)
			     (- (send j :max-angle) (send j :min-angle))))))
   (append *rarm-joints1* *rarm-joints2*))
  (send-all (send-all (append *rarm-joints1* *rarm-joints2*) :child-link) :worldcoords)
  (send *goal* :newcoords (send *robot* :rarm :end-coords :copy-worldcoords))
  (traj-input-vector))

(defun traj-random-test
  (&rest args)
  (apply 'traj-test (append args (list :input (random-traj-input-vector)))))

(defun traj-test
  (&rest
   args
   &key
   (id (random 10000))
   (input-path "input_lmdb_traj")
   (target-path "target_lmdb_coords")
   ;;
   (initialize-net? nil)
   (redundancy 0.0)
   (input
    (let* ((ret))
      (if initialize-net?
	  (caffe::gen-test-net :netproto "traj_net_predict.prototxt"
			       :caffemodel "traj_net_NESTEROV_iter_5000000.caffemodel"))
      ;;
      (caffe::db-open :path target-path)
      (caffe::db-read id)
      (setq ret (caffe::db-get-float-data))
      (format t " -- size : ~A~%" (caffe::db-get-size))
      (caffe::db-close)
      ret))
   target c ret
   ;;
   (draw? t)
   (draw-last? nil)
   &allow-other-keys
   )
  ;;
  (setq c (vector2coords (subseq input 0 3)))
  (map cons
       #'(lambda (j s)
	   (send j :joint-angle
		 (- (+ (send j :min-angle)
		       (* s (- (send j :max-angle) (send j :min-angle))))
		    (send j :joint-angle))
		 :relative t))
       *rarm-joints2* (subseq input 3))
  ;;
  ;; (caffe::db-open :path input-path)
  ;; (caffe::db-read id)
  ;; (setq output (caffe::db-get-float-data))
  ;; (format t " -- size : ~A~%" (caffe::db-get-size))
  ;; (caffe::db-close)
  (setq input (concatenate float-vector input (float-vector redundancy)))
  (setq output
	(caffe::calc-forward-double
	 :isize (length input) :osize 32
	 :num 1 :channels (length input) :width 1 :height 1
	 :input input))
  ;;
  (send *goal* :newcoords c)
  (setq ret (apply
	     'check-trajectory
	     (append
	      args
	      (list
	       :gain output
	       :cnt (cond
		     ((numberp draw?) draw?)
		     (draw? 10)
		     (t nil))
	       :x-buf (if draw-last? 1.0 0.0)
	       :draw? draw?))))
  ;;
  (list
   (cons :diff
	 (v- (subseq input 0 (- (length input) 1))
	     (concatenate float-vector (caar ret) (cadr (car (last ret))))))
   (cons :redundancy redundancy)
   (cons :input input)
   (cons :output output))
  )

(defun gather-traj-test
  (&key
   (id (random 10000))
   (input-path "input_lmdb_traj")
   (target-path "target_lmdb_coords")
   ;;
   (initialize-net? nil)
   (redundancy 0.0)
   (input
    (let* ((ret))
      (if initialize-net?
	  (caffe::gen-test-net :netproto "traj_net_predict.prototxt"
			       :caffemodel "traj_net_NESTEROV_iter_5000000.caffemodel"))
      ;;
      (caffe::db-open :path target-path)
      (caffe::db-read id)
      (setq ret (caffe::db-get-float-data))
      (format t " -- size : ~A~%" (caffe::db-get-size))
      (caffe::db-close)
      ret))
   (thre (float-vector 0.1 0.1 0.1
		       0.15 0.15 0.15 0.15))
   target c
   ;;
   (step 0.05)
   (cnt 0.0)
   ret buf
   ;;
   (draw? t)
   (draw-last? nil)
   (draw-step (/ 1 step))
   (draw-cnt 0)
   ;;
   prev-output
   (output-thre 0.3)
   )
  (dotimes (i (round (/ 1.0 step)))
    (setq buf (traj-test :id id :input input :redundancy (* i step)
			 :draw-last? draw-last?
			 :draw? (if (> (incf draw-cnt) draw-step)
				    (+ (if (numberp draw?) draw? 10)
				       (setq draw-cnt 0)) nil)))
    (cond
     ((v< (map float-vector 'abs (cdr (assoc :diff buf))) thre)
      (cond
       ((or (not prev-output)
	    (> (norm (v- prev-output (cdr (assoc :output buf))))
	       output-thre))
	(setq prev-output (cdr (assoc :output buf)))
	(if draw? (traj-test :id id :input input :redundancy (* i step)
			     :draw-last? draw-last? :draw? draw?))
	(push buf ret))))))
  (setq ret (sort ret #'(lambda (a b) (< (norm (Cdr (assoc :diff a)))
					 (norm (cdr (assoc :diff b)))))))
  (print (car ret))
  (if ret (traj-test :id id :input input :redundancy (cdr (assoc :redundancy (car ret)))))
  ret)



#|

(do-until-key (add-random-minjerk-traj-learning-data :cnt 10000 :input-path "input_lmdb_traj.cygnus" :target-path "target_lmdb_coords.cygnus"))

(caffe::gen-test-net :netproto "traj_net_predict.prototxt"
		     :caffemodel "traj_net_NESTEROV_iter_1000000.caffemodel")

(traj-test :initialize-net? t)

(caffe::output-log 0)
(mapcar '(lambda (id) (print (length (gather-traj-test :input (random-traj-input-vector))))) (make-list 1000))
(8 15 14 7 3 11 5 15 4 10 9 19 9 14 3 5 3 13 3 12 15 13 8 11 10 14 13 11 13 14 3 15 3 10 18 11 15 18 17 16 17 10 7 5 3 9 4 13 20 2 11 14 14 6 13 8 13 12 10 13 13 14 12 11 13 14 8 4 14 14 11 6 13 15 9 17 6 12 15 5 6 7 3 11 5 9 9 17 17 18 15 9 16 18 8 7 4 3 15 19 5 19 3 13 12 11 16 15 6 12 16 14 3 13 4 13 12 8 4 9 4 9 15 8 7 4 12 10 14 2 10 11 13 16 13 8 10 11 11 12 12 8 12 2 11 14 14 6 4 11 12 7 11 18 8 5 11 11 19 15 17 2 14 3 8 15 14 2 12 12 13 12 9 13 3 4 3 4 11 12 5 10 13 12 8 11 14 10 3 11 12 11 3 17 17 11 1 6 14 9 15 7 3 5 10 8 16 13 13 2 3 2 13 15 15 13 16 12 11 10 7 10 13 7 13 17 5 18 19 5 10 10 13 7 12 14 10 19 3 9 12 2 4 2 12 10 11 7 14 2 12 2 11 13 12 19 6 14 5 8 16 7 8 12 18 4 13 8 13 5 19 4 7 11 17 12 17 2 7 11 13 1 12 3 18 11 5 10 11 5 15 5 7 6 8 4 14 3 14 4 11 12 11 16 9 10 12 18 3 2 13 14 11 8 14 15 17 7 6 4 13 18 4 4 15 10 11 7 4 17 8 4 5 3 6 3 6 6 4 4 16 9 13 2 13 9 16 14 6 9 13 10 11 18 17 5 11 8 13 14 4 13 10 14 18 3 11 6 4 10 6 8 3 9 9 11 4 4 14 8 11 10 14 13 10 8 13 12 5 7 14 11 10 17 10 5 15 10 2 12 3 11 9 15 14 19 5 13 9 5 6 11 13 13 11 12 12 3 15 17 4 12 17 2 12 17 14 12 13 7 7 12 4 5 12 19 20 12 18 9 2 10 11 16 5 11 5 13 3 5 10 8 18 15 4 8 8 18 17 13 8 11 8 5 10 7 15 4 15 7 18 13 16 12 10 14 7 17 13 18 11 12 2 7 7 12 4 14 13 4 5 3 8 13 11 8 9 14 3 8 6 8 16 4 9 9 14 4 2 9 5 5 11 12 16 14 7 7 11 17 6 10 9 8 18 3 11 1 10 13 20 12 12 15 9 12 14 3 10 2 10 2 19 14 2 10 16 12 14 8 17 10 11 14 13 11 5 13 15 17 15 2 16 2 14 15 1 5 5 11 4 7 6 12 9 12 15 5 9 7 5 7 9 7 14 14 17 9 15 14 17 11 11 11 16 17 13 1 4 12 3 12 6 3 2 15 16 15 15 14 19 1 3 15 12 9 7 6 14 10 6 16 10 9 8 6 3 6 4 11 7 1 5 10 11 11 14 10 13 12 14 19 4 8 13 2 10 19 8 18 13 4 16 16 12 15 1 18 15 13 13 13 16 15 8 4 10 12 2 12 6 6 3 11 3 4 1 13 12 18 3 17 6 4 15 16 18 14 18 4 2 14 3 18 9 9 15 4 16 19 3 4 15 13 3 2 10 9 16 7 5 9 6 12 8 15 16 6 18 11 4 7 3 16 2 9 8 6 7 17 10 17 13 15 9 2 18 17 13 6 18 6 15 4 13 11 6 1 12 8 14 17 10 11 4 18 13 12 8 18 5 17 5 7 18 9 10 8 10 19 17 12 19 12 8 18 7 12 14 19 11 6 10 2 16 13 11 13 7 12 2 3 18 0 6 14 16 9 6 3 3 11 3 9 13 15 10 10 17 8 2 10 10 14 8 8 6 12 13 2 4 5 15 4 15 8 11 13 11 5 4 13 7 12 15 3 7 17 13 10 19 19 8 19 3 5 15 4 8 19 10 10 15 0 6 8 9 7 11 4 4 3 14 9 13 12 12 15 14 15 14 10 14 18 5 18 17 9 13 9 13 6 3 13 5 3 10 16 17 2 19 4 3 4 9 8 19 15 13 4 10 18 6 7 3 5 17 9 7 12 11 3 18 4 12 15 16 4 8 15 12 14 4 13 18 12 10 17 15 8 14 4 5 4 17 14 18 15 6 1 4 5 12 19 4 10 19 13 11 11 9 5 14 11 15 6 12 14 7 5 16 15 7 18 15 14 3 10 10 13 1 4 10 12 9 10 11 8 3 14 15 4 4 1 12 8 5 14 5 11 15 3 5 18 11 16 8 10 14 15)