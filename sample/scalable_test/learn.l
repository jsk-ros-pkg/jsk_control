#!/usr/bin/env roseus

(require "package://eus_caffe/euslisp/eus-caffe.l")
(require "package://eus_caffe/euslisp/eus-caffe-db.l")
(require "package://eus_caffe/euslisp/eus-log.l")
(require "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknt-interface.l")
(require "models/arrow-object.l")

(defvar *arrow* (arrow))
(defvar *robot* (hrp2jsknt))
(defvar *joint-list1* (append (send *robot* :rarm :joint-list)
			      (send *robot* :torso :joint-list)))
(defvar *joint-list2* (append (send *robot* :rarm :joint-list)
			      (send *robot* :larm :joint-list)
			      (send *robot* :torso :joint-list)))
(defvar *joint-list3* (append (send *robot* :rarm :joint-list)
			      (send *robot* :larm :joint-list)
			      (send *robot* :rleg :joint-list)
			      (send *robot* :lleg :joint-list)
			      (send *robot* :torso :joint-list)))
(objects (flatten (list (car (send *robot* :links))
			(reverse (send-all *joint-list3* :child-link))
			*arrow*)))

(defvar *ef-coords* (list (send *robot* :rarm :end-coords)))

(defvar *sample-points*
  (expt (max
	 (min (round (expt 1e+8 (/ 1.0 (length *joint-list1*))))
	      5) 2) (length *joint-list1*)))

(defvar *angle-vector-map*)
(defvar *ef-coords-map*)

(defun shuffle-random-vector-map
  (&key
   (sample-points *sample-points*)
   (joint-list *joint-list1*)
   (ef-coords *ef-coords*)
   (av-map *angle-vector-map*)
   (cc-map *ef-coords-map*)
   )
  (labels
   ((swap (a i j)
	  (let* ((c (aref a i)))
	    (setf (aref a i) (aref a j))
	    (setf (aref a j) c))))
   (dotimes (i sample-points)
     (let* ((r (random sample-points)))
       (dotimes (j (length joint-list))
	 (swap av-map
	       (+ j (* (length joint-list) i))
	       (+ j (* (length joint-list) r))))
       (dotimes (j (length ef-coords))
	 (dotimes (k 6)
	   (swap cc-map
		 (+ k (* 6 (length ef-coords) i))
		 (+ k (* 6 (length ef-coords) r)))))))))

(defun gen-ik-learning-data
  (&key (sample-points *sample-points*)
	(joint-list *joint-list1*)
	(ef-coords *ef-coords*)
	(sep (round (expt sample-points (/ 1.0 (length joint-list)))))
	(root-link (car (send *robot* :links)))
	(i 0)
	(draw? nil))
  (setq *sample-points* sample-points)
  (if (not (eq (length *angle-vector-map*)
	       (* (length joint-list) sample-points)))
      (setq *angle-vector-map*
	    (instantiate float-vector
			 (* (length joint-list) sample-points))))
  (if (not (eq (length *ef-coords-map*)
	       (* 6 (length ef-coords) sample-points)))
      (setq *ef-coords-map*
	    (instantiate float-vector
			 (* 6 (length ef-coords) sample-points))))
  (labels ((itter
	    (jlist)
	    (cond
	     ((null jlist)
	      (let* (pos rot)
		(send-all (send-all joint-list :child-link) :worldcoords)
		(send-all ef-coords :worldcoords)
		(cond
		 (draw?
		  (send *viewer* :draw-objects :flush nil)
		  (send-all ef-coords :draw-on :flush nil
			    :color (float-vector 1 0 0) :size 100)
		  (send *viewer* :viewsurface :flush)))
		(dotimes (j (length joint-list))
		  (setf (aref *angle-vector-map* (+ j (* i (length joint-list))))
			(deg2rad (send (nth j joint-list) :joint-angle))))
		(dotimes (j (length ef-coords))
		  (setq pos (send (send root-link :worldcoords)
				  :difference-position
				  (send (nth j ef-coords) :worldcoords)))
		  (setq rot (send (send root-link :worldcoords)
				  :difference-rotation
				  (send (nth j ef-coords) :worldcoords)))
		  (dotimes (k 3)
		    (setf (aref *ef-coords-map*
				(+ k (* j 6) (* i (length ef-coords) 6)))
			  (* 1e-3 (aref pos k))))
		  (dotimes (k 3)
		    (setf (aref *ef-coords-map*
				(+ 3 k (* j 6) (* i (length ef-coords) 6)))
			  (aref rot j)))
		  )
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
	  (shuffle-random-vector-map)
	  (dump-ik-learning-data)
	  ))

(defun gen-random-ik-learning-data
  (&key (sample-points *sample-points*)
	(joint-list *joint-list1*)
	(ef-coords *ef-coords*)
	(sep (round (expt sample-points (/ 1.0 (length joint-list)))))
	(root-link (car (send *robot* :links)))
	(i 0)
	(draw? nil))
  (setq *sample-points* sample-points)
  (if (not (eq (length *angle-vector-map*)
	       (* (length joint-list) sample-points)))
      (setq *angle-vector-map*
	    (instantiate float-vector
			 (* (length joint-list) sample-points))))
  (if (not (eq (length *ef-coords-map*)
	       (* 6 (length ef-coords) sample-points)))
      (setq *ef-coords-map*
	    (instantiate float-vector
			 (* 6 (length ef-coords) sample-points))))
  ;;
  (let* (pos rot)
    (dotimes (i sample-points)
      (dolist (j joint-list)
	(send j :joint-angle
	      (+ (send j :min-angle)
		 (* (random 1.0)
		    (- (send j :max-angle) (send j :min-angle))))))
      (send-all (send-all joint-list :child-link) :worldcoords)
      (send-all ef-coords :worldcoords)
      (dotimes (j (length joint-list))
	(setf (aref *angle-vector-map* (+ j (* i (length joint-list))))
	      (deg2rad (send (nth j joint-list) :joint-angle))))
      (dotimes (j (length ef-coords))
	(setq pos (send (send root-link :worldcoords)
			:difference-position
			(send (nth j ef-coords) :worldcoords)))
	(setq rot (send (send root-link :worldcoords)
			:difference-rotation
			(send (nth j ef-coords) :worldcoords)))
	(dotimes (k 3)
	  (setf (aref *ef-coords-map*
		      (+ k (* j 6) (* i (length ef-coords) 6)))
		(* 1e-3 (aref pos j))))
	(dotimes (k 3)
	  (setf (aref *ef-coords-map*
		      (+ 3 k (* j 6) (* i (length ef-coords) 6)))
		(aref rot j)))
	))
    (dump-ik-learning-data)
    ))

(defun load-ik-learning-data
  (&key (root "teacher"))
  (setq *angle-vector-map* (cpplog::read-vector (format nil "~A/angle-vector-map.dat" root)))
  (setq *ef-coords-map* (cpplog::read-vector (format nil "~A/ef-coords-map.dat" root)))
  (length *angle-vector-map*))

(defun dump-ik-learning-data
  (&key (root "teacher"))
  (unix::system (format nil "mkdir -p ~A" root))
  (cpplog::write-vector (format nil "~A/angle-vector-map.dat" root)
			*angle-vector-map*)
  (cpplog::write-vector (format nil "~A/ef-coords-map.dat" root)
			*ef-coords-map*))

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
  ;; (send *viewer* :draw-objects :flush nil)
  ;; (send c :draw-on :flush nil :color (float-vector 1 0 0) :width 100)
  (send *arrow* :newcoords (send c :copy-worldcoords))
  (send *viewer* :draw-objects :flush nil)
  (send *viewer* :viewsurface :flush)
  (print (concatenate
	  float-vector
	  (send c :difference-position (send *robot* :rarm :end-coords))
	  (send c :difference-rotation (send *robot* :rarm :end-coords))))
  )

(defun ik-test
  (&optional
   (i 0)
   (calc-forward 'caffe::calc-forward-double)
   (draw? t)
   ;;
   (input (instantiate float-vector 6))
   (output (instantiate float-vector 6))
   c tm
   )
  (dotimes (j 6) (setf (aref input j) (aref *ef-coords-map* (+ (* i 6) j))))
  (setq c (make-coords :pos (scale 1e+3 (subseq input 0 3))
		       :rot (matrix-exponent (subseq input 3 6))))
  (setq c (send (send (car (send *robot* :links)) :copy-worldcoords)
		:transform c :local))
  (setq
   tm
   (bench2
    (bench
     (apply calc-forward
	    (list :isize (length input) :input input
		  :osize (length output) :output output)))))
  (map cons #'(lambda (j a) (send j :joint-angle (rad2deg a)))
       *rarm-joints* output)
  (cond
   (draw?
    ;; (send *viewer* :draw-objects :flush nil)
    ;; (send c :draw-on :flush nil :color (float-vector 1 0 0) :width 100)
    (send *arrow* :newcoords (send c :copy-worldcoords))
    (send *viewer* :draw-objects :flush nil)
    (send *viewer* :viewsurface :flush)))
  (list
   (cons :time tm)
   (cons :diff
	 (print (concatenate
		 float-vector
		 (send c :difference-position (send *robot* :rarm :end-coords))
		 (send c :difference-rotation (send *robot* :rarm :end-coords))))))
  )

(defun ik-random-test
  (&key (calc-forward 'caffe::calc-forward-double) (draw? t) tm)
  (let* ((joint-list (butlast (send *robot* :rarm :joint-list)))
	 (random-angle
	  (mapcar '(lambda (j) (send j :joint-angle
				     (+ (send j :min-angle)
					(* (random 1.0)
					   (- (send j :max-angle)
					      (send j :min-angle))))))
		  joint-list))
	 (lc (send-all (send *robot* :links) :worldcoords))
	 (c (send *robot* :rarm :end-coords :copy-worldcoords))
	 ;; (make-coords :pos (v+ (float-vector 436.244 -210.8 766.991)
	  ;; (random-vector 500.0))
	 ;;:rpy (random-vector 3.14)))
	 (b (send (car (send *robot* :links)) :copy-worldcoords))
	 (av (instantiate float-vector (length *rarm-joints*)))
	 (input (concatenate float-vector
			     (scale 1e-3 (send b :difference-position c))
			     (send b :difference-rotation c)))
	 )
    ;; (send *robot* :rarm :inverse-kinematics c
    ;; :link-list (send-all joint-list :child-link)
    ;; :revert-if-fail nil)
    ;; (send c :newcoords (send *robot* :rarm :end-coords :copy-worldcoords))
    (setq
     tm
     (bench2
      (bench
       (apply calc-forward
	      (list :isize (length input) :input input
		    :osize (length av) :output av))
       )))
    (map cons #'(lambda (j a) (send j :joint-angle (rad2deg a))) *rarm-joints* av)
    (cond
     (draw?
      ;; (send *viewer* :draw-objects :flush nil)
      ;; (send c :draw-on :flush nil :color (float-vector 1 0 0) :width 100)
      (send *arrow* :newcoords (send c :copy-worldcoords))
      (send *viewer* :draw-objects :flush nil)
      (send *viewer* :viewsurface :flush)))
    (list
     (cons :time tm)
     (cons :diff
	   (print (concatenate float-vector
			       (send c :difference-position (send *robot* :rarm :end-coords))
			       (send c :difference-rotation (send *robot* :rarm :end-coords))))
	   ))))

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

#|

(caffe::gen-test-net :netproto "redundancy_ik_net_predict.prototxt"
		     :caffemodel "redundancy_ik_net_x6_iter_60000000.caffemodel")
(let* ((id2 (random 1000)) (id 0.0) (step 0.01) (max 1.0) (min 0.0)
       (min-dif *inf*) (min-dif-id id) buf)
  (do-until-key
   (format t "[param] (db_id, rd_id) = (~A, ~A)~%" id2 id)
   (format t "        (min_dif, rd_id) = (~A, ~A)~%" min-dif min-dif-id)
   (setq id (+ id step))
   (if (> min-dif
	  (setq buf (apply 'max (map cons 'abs (test-redundancy-ik :id id2 :redundancy id)))))
       (setq min-dif buf min-dif-id id))
   (cond
    ((> id max)
     (test-redundancy-ik :id id2 :redundancy min-dif-id)
     (return-from nil (print min-dif))))
   (if (or (> id max) (< id min)) (setq step (* -1 step)))))

(caffe::gen-test-net :netproto "redundancy_ik_net_predict.prototxt"
		     :caffemodel "redundancy_ik_net_iter_50000000.caffemodel")

(caffe::calc-forward-double
 :isize 7 :osize 7
 :num 1 :channels 7 :width 1 :height 1
 :input (float-vector -0.105524 0.0936122 0.250577 -0.603332 0.946547 1.00249
		      0))