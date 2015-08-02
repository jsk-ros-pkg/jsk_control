#!/usr/bin/env roseus

(require "package://eus_caffe/euslisp/eus-caffe.l")
(require "package://eus_caffe/euslisp/eus-caffe-db.l")
(require "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknt-interface.l")

(defvar *robot* (hrp2jsknt))
(defvar *rarm-links* (send *robot* :rarm :links))
(defvar *rarm-joints*
  (remove (send *robot* :rarm :wrist-y)
	  (send-all (butlast *rarm-links*) :joint)))

(objects (append
	  *rarm-links*
	  (send *robot* :hand :rarm :links)))

(defvar *sample-points* (expt 6 6))
(defvar *angle-vector-map*)
(defvar *ef-coords-map*)

(defun shuffle-random-vector-map
  nil
  (labels ((swap (a i j)
		 (let* ((c (aref a i)))
		   (setf (aref a i) (aref a j))
		   (setf (aref a j) c))))
    (dotimes (i *sample-points*)
      (let* ((r (random *sample-points*)))
	(dotimes (j (length *rarm-joints*))
	  (swap *angle-vector-map*
		(+ j (* (length *rarm-joints*) i))
		(+ j (* (length *rarm-joints*) r))))
	(dotimes (j 6)
	  (swap *ef-coords-map*
		(+ j (* 6 i)) (+ j (* 6 r))))))))

(defun gen-ik-learning-data
  (&key (sep 6)
	(sample-points (expt sep (length *rarm-joints*)))
	(i 0)
	(draw? nil))
  (setq *sample-points* sample-points)
  (if (not (eq (length *angle-vector-map*)
	       (* (length *rarm-joints*) sample-points)))
      (setq *angle-vector-map*
	    (instantiate float-vector (* (length *rarm-joints*) sample-points))))
  (if (not (eq (length *ef-coords-map*) (* 6 sample-points)))
      (setq *ef-coords-map*
	    (instantiate float-vector (* 6 sample-points))))
  (labels ((itter
	    (jlist)
	    (cond
	     ((null jlist)
	      (let* (pos rot)
		(send-all *rarm-links* :worldcoords)
		(send *robot* :rarm :end-coords :worldcoords)
		(cond
		 (draw?
		  (send *viewer* :draw-objects :flush nil)
		  (send (send *robot* :rarm :end-coords :worldcoords) :draw-on :flush nil
			:color (float-vector 1 0 0) :size 100)
		  (send *viewer* :viewsurface :flush)))
		(setq pos (send (send (car (send *robot* :links)) :worldcoords)
		      :difference-position
		      (send *robot* :rarm :end-coords)))
		(setq rot (send (send (car (send *robot* :links)) :worldcoords)
				:difference-rotation
				(send *robot* :rarm :end-coords)))
		(dotimes (j (length *rarm-joints*))
		  (setf (aref *angle-vector-map* (+ j (* i (length *rarm-joints*))))
			(deg2rad (send (nth j *rarm-joints*) :joint-angle))))
		(dotimes (j 3)
		  (setf (aref *ef-coords-map* (+ j (* i 6))) (* 1e-3 (aref pos j))))
		(dotimes (j 3)
		  (setf (aref *ef-coords-map* (+ 3 j (* i 6))) (aref rot j)))
		(setq i (+ i 1))))
	     (t
	      (dotimes (j sep)
		(send (car jlist) :joint-angle
		      (+ (send (car jlist) :min-angle)
			 (* (/ j (- sep 1.0))
			    (- (send (car jlist) :max-angle)
			       (send (car jlist) :min-angle)))))
		(itter (cdr jlist))))
	     )))
    (itter *rarm-joints*)
    (shuffle-random-vector-map)
    (dump-ik-learning-data)
    ))

(defun gen-random-ik-learning-data
  nil
  (if (not (eq (length *angle-vector-map*)
	       (* (length *rarm-joints*) *sample-points*)))
      (setq *angle-vector-map*
	    (instantiate float-vector (* (length *rarm-joints*) *sample-points*))))
  (if (not (eq (length *ef-coords-map*) (* 6 *sample-points*)))
      (setq *ef-coords-map*
	    (instantiate float-vector (* 6 *sample-points*))))
  (let* (pos rot)
    (dotimes (i *sample-points*)
      (dolist (j *rarm-joints*)
	(send j :joint-angle
	      (+ (send j :min-angle)
		 (* (random 1.0)
		    (- (send j :max-angle) (send j :min-angle))))))
      (send-all *rarm-links* :worldcoords)
      (setq pos (send (send (car (send *robot* :links)) :worldcoords)
		      :difference-position
		      (send *robot* :rarm :end-coords)))
      (setq rot (send (send (car (send *robot* :links)) :worldcoords)
		      :difference-rotation
		      (send *robot* :rarm :end-coords)))
      (dotimes (j (length *rarm-joints*))
	(setf (aref *angle-vector-map* (+ j (* i (length *rarm-joints*))))
	      (deg2rad (send (nth j *rarm-joints*) :joint-angle))))
      (dotimes (j 3)
	(setf (aref *ef-coords-map* (+ j (* i 6))) (* 1e-3 (aref pos j))))
      (dotimes (j 3)
	(setf (aref *ef-coords-map* (+ 3 j (* i 6))) (aref rot j)))
      )
    ;; (dump-loadable-structure "ef-coords-map.l" *ef-coords-map*)
    ;; (dump-loadable-structure "angle-vector-map.l" *angle-vector-map*)
    (dump-ik-learning-data)
    ))

(defun load-ik-learning-data
  nil
  (if (probe-file "ik_learning.dat")
      (let* ((op (open "ik_learning.dat" :direction :input)) buf)
	(if (not (setq buf (read-line op nil))) (throw :load-ik-learning-data-error nil))
	(setq buf (read-from-string buf))
	(if (not (eq (length *angle-vector-map*) buf))
	    (setq *angle-vector-map* (instantiate float-vector buf)))
	(dotimes (i (length *angle-vector-map*))
	  (setq buf (read-from-string (read-line op nil)))
	  (setf (aref *angle-vector-map* i) buf))
	;;
	(if (not (setq buf (read-line op nil))) (throw :load-ik-learning-data-error nil))
	(setq buf (read-from-string buf))
	(if (not (eq (length *ef-coords-map*) buf))
	    (setq *ef-coords-map* (instantiate float-vector buf)))
	(dotimes (i (length *ef-coords-map*))
	  (setq buf (read-from-string (read-line op nil)))
	  (setf (aref *ef-coords-map* i) buf))
	(close op))))

(defun dump-ik-learning-data
  nil
  (let* ((op (open "ik_learning.dat" :direction :output)))
    (format op "~A~%" (length *angle-vector-map*))
    (dotimes (i (length *angle-vector-map*))
      (format op "~A~%" (aref *angle-vector-map* i)))
    (format op "~A~%" (length *ef-coords-map*))
    (dotimes (i (length *ef-coords-map*))
      (format op "~A~%" (aref *ef-coords-map* i)))
    (close op)))

;; (load-ik-learning-data)

(defun ik-learn
  (&key (solver "ik_solver.prototxt"))
  (if (or (zerop (length *ef-coords-map*))
	  (zerop (length *angle-vector-map*)))
      (load-ik-learning-data))
  (caffe::learn
   :solver solver
   :size *sample-points*
   :idata *ef-coords-map*
   :ddata *angle-vector-map*
   ))

(defun check-test-data
  (&optional
   (i 0)
   (input (instantiate float-vector 6))
   (output (instantiate float-vector 6))
   c
   )
  (dotimes (j 6) (setf (aref input j) (aref *ef-coords-map* (+ (* i 6) j))))
  (dotimes (j (length *rarm-joints*))
    (setf (aref output j)
	  (aref *angle-vector-map* (+ (* i (length *rarm-joints*)) j))))
  (setq c (make-coords :pos (scale 1e+3 (subseq input 0 3))
		       :rot (matrix-exponent (subseq input 3 6))))
  (setq c (send (send (car (send *robot* :links)) :copy-worldcoords)
		:transform c :local))
  (map cons #'(lambda (j a) (send j :joint-angle (rad2deg a)))
       *rarm-joints* output)
  (send *viewer* :draw-objects :flush nil)
  (send c :draw-on :flush nil :color (float-vector 1 0 0) :width 100)
  (send *viewer* :viewsurface :flush)
  (print (concatenate
	  float-vector
	  (send c :difference-position (send *robot* :rarm :end-coords))
	  (send c :difference-rotation (send *robot* :rarm :end-coords))))
  )

(defun ik-test
  (&optional
   (i 0)
   (input (instantiate float-vector 6))
   (output (instantiate float-vector 6))
   c
   )
  (dotimes (j 6) (setf (aref input j) (aref *ef-coords-map* (+ (* i 6) j))))
  (setq c (make-coords :pos (scale 1e+3 (subseq input 0 3))
		       :rot (matrix-exponent (subseq input 3 6))))
  (setq c (send (send (car (send *robot* :links)) :copy-worldcoords)
		:transform c :local))
  (caffe::memory-calc-forward
   :input input :osize (length output) :output output)
  (map cons #'(lambda (j a) (send j :joint-angle (rad2deg a)))
       *rarm-joints* output)
  (send *viewer* :draw-objects :flush nil)
  (send c :draw-on :flush nil :color (float-vector 1 0 0) :width 100)
  (send *viewer* :viewsurface :flush)
  (print (concatenate
	  float-vector
	  (send c :difference-position (send *robot* :rarm :end-coords))
	  (send c :difference-rotation (send *robot* :rarm :end-coords))))
  )

(defun ik-random-test
  nil
  (let* ((c (make-coords :pos (v+ (float-vector 436.244 -210.8 766.991)
				  (random-vector 500.0))
			 :rpy (random-vector 3.14)))
	 (b (send (car (send *robot* :links)) :copy-worldcoords))
	 (av (instantiate float-vector (length *rarm-joints*)))
	 (joint-list (butlast (send *robot* :rarm :joint-list)))
	 )
    (send *robot* :rarm :inverse-kinematics c
	  :link-list (send-all joint-list :child-link)
	  :revert-if-fail nil)
    (send c :newcoords (send *robot* :rarm :end-coords :copy-worldcoords))
    (bench
     (caffe::memory-calc-forward
      :input (concatenate float-vector
			  (scale 1e-3 (send b :difference-position c))
			  (send b :difference-rotation c))
      :osize (length av)
      :output av))
    (map cons #'(lambda (j a) (send j :joint-angle (rad2deg a))) *rarm-joints* av)
    (send *viewer* :draw-objects :flush nil)
    (send c :draw-on :flush nil :color (float-vector 1 0 0) :width 100)
    (send *viewer* :viewsurface :flush)
    (print (concatenate float-vector
			(send c :difference-position (send *robot* :rarm :end-coords))
			(send c :difference-rotation (send *robot* :rarm :end-coords))))
    ))

;;

(defun random-index-vector
  (size
   &key
   (ret (instantiate integer-vector size))
   (cnt (* 10 size)))
  (dotimes (i size) (setf (aref ret i) i))
  (dotimes (i cnt)
    (let* ((x (random size))
	   (y (random size))
	   (buf (aref ret x)))
      (setf (aref ret x) (aref ret y))
      (setf (aref ret y) buf)))
  ret)

(defun gen-redundancy-ik-learning-data
  (&key (sep 10)
	(joint-list (butlast (send *robot* :rarm :joint-list)))
	(sample-points (expt sep (length joint-list)))
	(draw? nil)
	;;
	(input-path "input_lmdb_potentio")
	(target-path "target_lmdb_limb_coords")
	(random-index-vector (random-index-vector sample-points))
	(i 0)
	(zero-str-length (length (format nil "~A" sample-points)))
	)
  (caffe::db-set-id 0)
  (if (not (zerop (caffe::db-open :dtype "lmdb" :path target-path :mode #\n)))
      (return-from gen-redundancy-ik-learning-data (print 'db-open-failed)))
  (caffe::db-set-id 1)
  (if (not (zerop (caffe::db-open :dtype "lmdb" :path input-path :mode #\n)))
      (return-from gen-redundancy-ik-learning-data (print 'db-open-failed)))
  ;;
  (labels ((itter
	    (jlist)
	    (cond
	     ((null jlist)
	      (let* (pos rot)
		(send-all (send *robot* :links) :worldcoords)
		(send *robot* :rarm :end-coords :worldcoords)
		(cond
		 (draw?
		  (send *viewer* :draw-objects :flush nil)
		  (send (send *robot* :rarm :end-coords :worldcoords) :draw-on :flush nil
			:color (float-vector 1 0 0) :size 100)
		  (send *viewer* :viewsurface :flush)))
		(setq pos (send (send (car (send *robot* :links)) :worldcoords)
				:difference-position
				(send *robot* :rarm :end-coords)))
		(setq rot (send (send (car (send *robot* :links)) :worldcoords)
				:difference-rotation
				(send *robot* :rarm :end-coords)))
		;;
		(caffe::db-set-id 1)
		(caffe::db-put-double
		 :channels (length joint-list) :width 1 :height 1 :label i
		 :id_str (caffe::zero-string (aref random-index-vector i) zero-str-length)
		 :data (map float-vector 'deg2rad (send-all joint-list :joint-angle)))
		;;
		(caffe::db-set-id 0)
		(caffe::db-put-double
		 :channels 6 :width 1 :height 1 :label i
		 :id_str (caffe::zero-string (aref random-index-vector i) zero-str-length)
		 :data (concatenate float-vector (scale 1e-3 pos) rot))
		(setq i (+ i 1))))
	     (t
	      (dotimes (j sep)
		(send (car jlist) :joint-angle
		      (+ (send (car jlist) :min-angle)
			 (* (/ j (- sep 1.0))
			    (- (send (car jlist) :max-angle)
			       (send (car jlist) :min-angle)))))
		(itter (cdr jlist))))
	     )))
    (itter joint-list)
    (caffe::db-set-id 0) (caffe::db-close)
    (caffe::db-set-id 1) (caffe::db-close)
    ))

(defun gen-redundancy-ik-learning-data2-once
  (&key (sep 3)
	(joint-list (butlast (send *robot* :rarm :joint-list)))
	(sample-points (expt sep (length joint-list)))
	(draw? nil)
	;;
	(i 0)
	(zero-str-length (length (format nil "~A" sample-points)))
	(target-coords (make-coords :pos (float-vector 230.357 -298.274 1024.051)
				    :rpy '(0.066 -0.021 -0.084)))
	)
  (labels ((itter
	    (jlist)
	    (cond
	     ((null jlist)
	      (let* (pos rot prev)
		(send-all (send *robot* :links) :worldcoords)
		(setq prev (send-all joint-list :joint-angle))
		(cond
		 ((send *robot* :rarm :inverse-kinematics target-coords :warnp nil)
		  (send *robot* :rarm :end-coords :worldcoords)
		  (cond
		   (draw?
		    (send *viewer* :draw-objects :flush nil)
		    (send (send *robot* :rarm :end-coords :worldcoords) :draw-on :flush nil
			  :color (float-vector 1 0 0) :size 100)
		    (send *viewer* :viewsurface :flush)))
		  (setq pos (send (send (car (send *robot* :links)) :worldcoords)
				  :difference-position
				  target-coords))
		  (setq rot (send (send (car (send *robot* :links)) :worldcoords)
				  :difference-rotation
				  target-coords))
		  ;;
		  (caffe::db-set-id 1)
		  (caffe::db-put-double
		   :channels (length joint-list) :width 1 :height 1 :label i
		   :id_str (caffe::zero-string i zero-str-length)
		   :data (map float-vector 'deg2rad (send-all joint-list :joint-angle)))
		  ;;
		  (caffe::db-set-id 0)
		  (caffe::db-put-double
		   :channels 6 :width 1 :height 1 :label i
		   :id_str (caffe::zero-string i zero-str-length)
		   :data (concatenate float-vector (scale 1e-3 pos) rot))
		  (setq i (+ i 1))
		  (mapcar '(lambda (j av) (send j :joint-angle av))
			  joint-list prev)
		  ))))
	     (t
	      (dotimes (j sep)
		(send (car jlist) :joint-angle
		      (+ (send (car jlist) :min-angle)
			 (* (/ j (- sep 1.0))
			    (- (send (car jlist) :max-angle)
			       (send (car jlist) :min-angle)))))
		(itter (cdr jlist))))
	     )))
    (itter joint-list)
    i
    ))

(defun gen-redundancy-ik-learning-data2
  (&key (sep 3)
	(joint-list (butlast (send *robot* :rarm :joint-list)))
	(sample-points (* (expt sep (length joint-list))
			  (expt sep (length joint-list))))
	(draw? nil)
	;;
	(input-path "input_lmdb_potentio")
	(target-path "target_lmdb_limb_coords")
	(i 0)
	(zero-str-length (length (format nil "~A" sample-points)))
	)
  (caffe::db-set-id 0)
  (if (not (zerop (caffe::db-open :dtype "lmdb" :path target-path :mode #\n)))
      (return-from gen-redundancy-ik-learning-data2 (print 'db-open-failed)))
  (caffe::db-set-id 1)
  (if (not (zerop (caffe::db-open :dtype "lmdb" :path input-path :mode #\n)))
      (return-from gen-redundancy-ik-learning-data2 (print 'db-open-failed)))
  ;;
  (labels ((itter
	    (jlist)
	    (cond
	     ((null jlist)
	      (send-all (send *robot* :links) :worldcoords)
	      (send *robot* :rarm :end-coords :worldcoords)
	      (setq i (apply
		       'gen-redundancy-ik-learning-data2-once
		       (append (list :i i
				     :sample-points (expt sep (length joint-list))
				     :sep sep
				     :joint-list joint-list
				     :zero-str-length zero-str-length
				     :draw? draw?
				     :target-coords (send *robot* :rarm :end-coords :copy-worldcoords)
				     )
			       nil)))
	      )
	     (t
	      (dotimes (j sep)
		(send (car jlist) :joint-angle
		      (+ (send (car jlist) :min-angle)
			 (* (/ j (- sep 1.0))
			    (- (send (car jlist) :max-angle)
			       (send (car jlist) :min-angle)))))
		(itter (cdr jlist))))
	     )))
    (itter joint-list)
    (caffe::db-set-id 0) (caffe::db-close)
    (caffe::db-set-id 1) (caffe::db-close)
    ;;
    ;; swap
    (caffe::db-set-id 0)
    (caffe::db-open :dtype "lmdb" :path target-path :mode #\w)
    (caffe::db-set-id 1)
    (caffe::db-open :dtype "lmdb" :path input-path :mode #\w)
    (caffe::db-random-swap (* 7 i) :size i)
    (caffe::db-set-id 0) (caffe::db-close)
    (caffe::db-set-id 1) (caffe::db-close)
    ))

(defun redundancy-ik-learn
  nil
  (caffe::learn
   :solver "redundancy_ik_solver.prototxt"
   :size 0
   ))

(defun ik-db-test
  (&key
   (id (random 1000))
   (input-path "input_lmdb_potentio")
   (target-path "target_lmdb_limb_coords")
   input target c)
  (caffe::db-open :path target-path)
  (caffe::db-read id)
  (setq input (caffe::db-get-float-data))
  (caffe::db-close)
  ;;
  (setq c (make-coords :pos (scale 1e+3 (subseq input 0 3))
		       :rot (matrix-exponent (subseq input 3 6))))
  (setq c (send (send (car (send *robot* :links)) :copy-worldcoords)
		:transform c :local))
  ;;
  (caffe::db-open :path input-path)
  (caffe::db-read id)
  (setq output (caffe::db-get-float-data))
  (caffe::db-close)
  (map cons '(lambda (av j) (send j :joint-angle (rad2deg av)))
       output (send *robot* :rarm :joint-list))
  ;;
  (send *viewer* :draw-objects :flush nil)
  (send c :draw-on :flush nil :color (float-vector 1 0 0) :size 100)
  (send *viewer* :viewsurface :flush)
  (v- (subseq input 0 6)
      (concatenate float-vector
		   (scale 1e-3 (send (send (car (send *robot* :links)) :worldcoords)
				     :difference-position
				     (send *robot* :rarm :end-coords)))
		   (send (send (car (send *robot* :links)) :worldcoords)
				:difference-rotation
				(send *robot* :rarm :end-coords))))
  )

(defun test-redundancy-ik
  (&key
   (id (random 10000))
   (joint-list (butlast (send *robot* :rarm :joint-list)))
   (initialize-net? nil)
   (redundancy (random 1.0))
   input output c)
  (if initialize-net?
      (caffe::gen-test-net :netproto "redundancy_ik_net_predict.prototxt"
			   :caffemodel "redundancy_ik_net_iter_50000000.caffemodel"))
  (caffe::db-open :path "target_lmdb_limb_coords")
  (caffe::db-read id)
  (setq input (caffe::db-get-float-data))
  (caffe::db-close)
  ;;
  (setq input (concatenate float-vector input (float-vector redundancy)))
  (setq output
	(caffe::calc-forward-double
	 :isize (length input) :osize (length joint-list)
	 :num 1 :channels (length input) :width 1 :height 1
	 :input input))
  ;; ;; test
  ;; (caffe::db-open :path "input_lmdb_potentio")
  ;; (caffe::db-read id)
  ;; (setq output (caffe::db-get-float-data))
  ;; (caffe::db-close)
  ;; ;;
  (map cons '(lambda (j av) (send j :joint-angle (rad2deg av)))
       joint-list output)
  (setq c (make-coords :pos (scale 1e+3 (subseq input 0 3))
		       :rot (matrix-exponent (subseq input 3 6))))
  (setq c (send (send (car (send *robot* :links)) :copy-worldcoords)
		:transform c :local))
  (send *viewer* :draw-objects :flush nil)
  (send c :draw-on :flush nil :color (float-vector 1 0 0) :size 100)
  (send *viewer* :viewsurface :flush)
  (print
   (concatenate float-vector
		(scale 1e-3
		       (send c :difference-position
			     (send *robot* :rarm :end-coords)))
		(send c :difference-rotation
		      (send *robot* :rarm :end-coords))))
  )

#|

(caffe::gen-test-net :netproto "redundancy_ik_net_predict.prototxt"
		     :caffemodel "redundancy_ik_net_x4_iter_60000000.caffemodel")
(let* ((id2 (random 1000)) (id 0.0) (step 0.01) (max 1.0) (min 0.0)
       (min-dif *inf*) (min-dif-id id) buf)
  (do-until-key
   (format t "[param] (db_id, rd_id) = (~A, ~A)~%" id2 id)
   (format t "        (min_dif, rd_id) = (~A, ~A)~%" min-dif min-dif-id)
   (setq id (+ id step))
   (if (> min-dif
	  (setq buf (apply 'max (map cons 'abs (test-redundancy-ik :id id2 :redundancy id)))))
       (setq min-dif buf min-dif-id id))
   (if (> id max) (return-from nil (print min-dif)))
   (if (or (> id max) (< id min)) (setq step (* -1 step)))))

(caffe::gen-test-net :netproto "redundancy_ik_net_predict.prototxt"
		     :caffemodel "redundancy_ik_net_iter_50000000.caffemodel")

(caffe::calc-forward-double
 :isize 7 :osize 7
 :num 1 :channels 7 :width 1 :height 1
 :input (float-vector -0.105524 0.0936122 0.250577 -0.603332 0.946547 1.00249
		      0))