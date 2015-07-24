#!/usr/bin/env roseus

(require "package://eus_caffe/euslisp/eus-caffe.l")
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
  nil
  (if (or (zerop (length *ef-coords-map*))
	  (zerop (length *angle-vector-map*)))
      (load-ik-learning-data))
  (caffe::learn
   :solver "ik_solver.prototxt"
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
	 (av (instantiate float-vector (length *rarm-joints*))))
    (caffe::memory-calc-forward
     :input (concatenate float-vector
			 (scale 1e-3 (send b :difference-position c))
			 (send b :difference-rotation c))
     :osize (length av)
     :output av)
    (map cons #'(lambda (j a) (send j :joint-angle (rad2deg a))) *rarm-joints* av)
    (send *viewer* :draw-objects :flush nil)
    (send c :draw-on :flush nil :color (float-vector 1 0 0) :width 100)
    (send *viewer* :viewsurface :flush)
    (print (concatenate float-vector
			(send c :difference-position (send *robot* :rarm :end-coords))
			(send c :difference-rotation (send *robot* :rarm :end-coords))))
    ))


#|

(caffe::gen-net-image :net-path "ik_net.prototxt")

(caffe::initialize-solver :solver "ik_solver.prototxt" :solverstate "_iter_5000000.solverstate")
(caffe::initialize-solver :solver "ik_solver.prototxt" :solverstate "ik_net_iter_5000000.solverstate")