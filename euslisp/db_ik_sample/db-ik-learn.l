(require "../eus-caffe.l")
(require "../eus-caffe-db.l")

(require "irteus/demo/sample-robot-model.l")

(defvar *robot* (instance sample-robot :init))
(objects (list *robot*))
(send *irtviewer* :change-background (float-vector 1 1 1))
(send *viewer* :draw-objects)

(defun get-gl-mono-image
  (&key (x 0) (y 0) z
        (view (send *viewer* :viewsurface))
        (width (send view :get-val 'x::width))
        (height (send view :get-val 'x::height))
        (worg (send view :get-val 'x::width))
        (horg (send view :get-val 'x::height))
        (wscale  (/ (* 1.0 worg) width))
        (hscale (/ (* 1.0 horg) height))
        ((:imagebuf imgbuf) (make-string (* worg horg 3))))
  (let ()
    (send view :makecurrent)
    (gl::glReadBuffer gl::GL_BACK)
    (gl::glPixelStorei gl::GL_PACK_ALIGNMENT 1)
    (gl::glReadPixels x y worg horg gl::GL_RGB gl::GL_UNSIGNED_BYTE imgbuf)
    ;; transpose
    (let ((b (make-string (* width height))) zv)
      (dotimes (_x width)
        (dotimes (_y height)
	  (setq zv 0)
	  (setq x (floor (* wscale _x)))
	  (setq y (floor (* hscale _y)))
          (dotimes (_z 3)
	    (setq zv (+ zv (elt imgbuf (+ (* y worg 3) (* x 3) _z)))))
	  (setf (elt b (+ (* (- height _y 1) width) _x))
		(round (/ zv 3)))
	  ))
      (instance image::grayscale-image :init width height b))
    ))
;; (img::write-image-file "test.jpg" (get-gl-mono-image :width 320 :height 320))

(defun gen-random-ik-learning-data
  (&key
   (sample-points 100)
   (joint-list (send *robot* :joint-list))
   ;;
   (target-path "target_lmdb_joint_angle")
   (input-path "input_lmdb_robot_image")
   (image-path "image")
   (width 32) (height 32)
   img
   )
  (caffe::db-set-id 0)
  (if (not (zerop (caffe::db-open :dtype "lmdb" :path target-path :mode #\n)))
      (return-from gen-linear-equation-db (print 'db-open-failed)))
  (caffe::db-set-id 1)
  (if (not (zerop (caffe::db-open :dtype "lmdb" :path input-path :mode #\n)))
      (return-from gen-linear-equation-db (print 'db-open-failed)))
  (if (not (probe-file image-path))
      (unix::system (format nil "mkdir ~A" image-path)))
  ;;
  (dotimes (i sample-points)
    (dolist (j joint-list)
      (send j :joint-angle
	    (+ (send j :min-angle)
	       (* (random 1.0)
		  (- (send j :max-angle) (send j :min-angle))))))
    (send *viewer* :draw-objects)
    (setq img (get-gl-mono-image :width width :height height))
    (img::write-image-file (format nil "~A/~A.jpg" image-path i) img)
    ;;
    (caffe::db-set-id 0)
    (caffe::db-put-double
     :channels (length joint-list) :width 1 :height 1 :label 0
     :id_str (caffe::zero-string i)
     :data (coerce (send-all joint-list :joint-angle) float-vector))
    ;;
    (caffe::db-set-id 1)
    (caffe::db-put
     :channels 1 :width width :height height :label 0
     :id_str (caffe::zero-string i) :data (send img :entity))
    )
  ;;
  (caffe::db-set-id 0) (caffe::db-close)
  (caffe::db-set-id 1) (caffe::db-close)
  )

(defun db-learn
  nil
  ;; (caffe::_create_solver "linear_equation_db.prototxt" "")
  ;; (caffe::_learn)
  (caffe::learn :solver "cnn_solver.prototxt" :size 0)
  ;; test
  ;; (caffe::gen-test-net :netproto "cnn_predict.prototxt"
  ;; 		       ;; :caffemodel "sample_iter_100000.caffemodel"
  ;; 		       )
  ;; (format t " -- check net state~%  ~A->~A~%  ~A->~A~%  ~A->~A~%"
  ;; 	  (float-vector 0 0)
  ;; 	  (caffe::calc-forward :input (float-vector 0 0) :isize 2 :osize 1)
  ;; 	  (float-vector 0 2)
  ;; 	  (caffe::calc-forward :input (float-vector 0 2) :isize 2 :osize 1)
  ;; 	  (float-vector 2 5)
  ;; 	  (caffe::calc-forward :input (float-vector 2 5) :isize 2 :osize 1))
  )

#|

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

