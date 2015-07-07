#!/usr/bin/env roseus

(require "eus-caffe.l")
(require "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknt-interface.l")

(defvar *robot* (hrp2jsknt))
(defvar *rarm-links* (send *robot* :rarm :links))
(defvar *rarm-joints*
  (remove (send *robot* :rarm :wrist-y)
	  (send-all (butlast *rarm-links*) :joint)))

(objects (append
	  *rarm-links*
	  (send *robot* :hand :rarm :links)))

(defvar *sample-points* 32000)
(defvar *angle-vector-map*)
(defvar *ef-coords-map*)

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

(defun gen-ik-learning-data
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
	      (send (nth j *rarm-joints*) :joint-angle)))
      (dotimes (j 3)
	(setf (aref *ef-coords-map* (+ j (* i 6))) (aref pos j)))
      (dotimes (j 3)
	(setf (aref *ef-coords-map* (+ 3 j (* i 6))) (aref rot j)))
      )
    ;; (dump-loadable-structure "ef-coords-map.l" *ef-coords-map*)
    ;; (dump-loadable-structure "angle-vector-map.l" *angle-vector-map*)
    (dump-ik-learning-data)
    ))

(load-ik-learning-data)

#|

(caffe::learn
 :solver "ik_solver.prototxt"
 :dsize (length *ef-coords-map*)
 :isize (length *angle-vector-map*)
 :idata *angle-vector-map*
 :ddata *ef-coords-map*
 )

(caffe::learn
 :solver "ik_solver.prototxt"
 :dsize (/ (length *ef-coords-map*) 10)
 :isize (/ (length *angle-vector-map*) 10)
 :idata *angle-vector-map*
 :ddata *ef-coords-map*
 )
