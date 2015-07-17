(require "../eus-caffe.l")
(require "../eus-caffe-db.l")

(defvar *robot*
  (cond
   ((probe-file (ros::resolve-ros-path "package://peppereus/pepper.l"))
    (require :hrp2jsk-utils "package://peppereus/pepper.l")
    (pepper))
   ((probe-file (ros::resolve-ros-path "package://euslisp/jskeus/irteus/demo/sample-robot-model.l"))
    (require "package://euslisp/jskeus/irteus/demo/sample-robot-model.l")
    (instance sample-robot :init))
   (t (throw :robot-not-found nil))))

(objects (list *robot*))
(send *robot* :transform (send (send *robot* :link "Tibia") :transformation (make-coords)) :local)
(send *irtviewer* :change-background (float-vector 0 0 0))
(send *irtviewer* :viewer :viewing :look #f(4000 -10 -80) #F(0 0 300))
(send *viewer* :draw-objects)

(send-all (send *robot* :torso :joint-list) :min-angle -10)
(send-all (send *robot* :torso :joint-list) :max-angle +10)

(defun read-mono-image
  (path)
  (let* ((img (img::read-image-file path))
	 (buf (instantiate string (* (send img :width)
				     (send img :height)))))
    (dotimes (x (send img :width))
      (dotimes (y (send img :height))
	(setf (elt buf (+ x (* y (send img :width))))
	      (/
	       (+
		(elt (send img :entity)
		     (+ 0 (* 3 (+ x (* y (send img :width))))))
		(elt (send img :entity)
		     (+ 1 (* 3 (+ x (* y (send img :width))))))
		(elt (send img :entity)
		     (+ 2 (* 3 (+ x (* y (send img :width)))))))
	       3)
	      )))
    (instance image::grayscale-image :init
	      (send img :width) (send img :height) buf)))

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

(defvar *mean-mono-image*
  (if (probe-file "image/mean.jpg") (read-mono-image "image/mean.jpg")
    (progn
      (unix:system "mkdir image")
      (img::write-image-file
       "image/mean.jpg" (get-gl-mono-image :width 32 :height 32))
      (read-mono-image "image/mean.jpg"))))

(defun image-minus
  (&optional
   (img1 (get-gl-mono-image :width 32 :height 32))
   (img2 *mean-mono-image*))
  (scale 1 ;;(/ 1.0 255)
	 (map float-vector '- (send img1 :entity) (send img2 :entity))))

(defun float-vector2image-string
  (data)
  (let* ((min *inf*) (max *-inf*))
    (dotimes (i (length data))
      (if (< (aref data i) min) (setq min (aref data i)))
      (if (> (aref data i) max) (setq max (aref data i))))
    (map string
	 #'(lambda (d) (round (/ (* 255.0 (- d min)) (- max min))))
	 data)))

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
    (send-all (send *robot* :links) :worldcoords)
    (send *robot* :newcoords (make-coords))
    (send *robot* :transform (send (send (send *robot* :link "Tibia") :worldcoords) :transformation (make-coords)) :local)
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
    (caffe::db-put-double
     :channels 1 :width width :height height :label 0
     :id_str (caffe::zero-string i) :data (image-minus img *mean-mono-image*)) ;;(send img :entity))
    )
  ;;
  (caffe::db-set-id 0) (caffe::db-close)
  (caffe::db-set-id 1) (caffe::db-close)
  )

(defun db-image-ik-learn
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

(caffe::gen-test-net :netproto "cnn_predict.prototxt" :caffemodel "cnn_iter_1000.caffemodel")

(caffe::db-open :path "target_lmdb_joint_angle")
(setq a (list (caffe::db-get-float-data)
	      (progn (caffe::db-read 1) (caffe::db-get-float-data))
	      (progn (caffe::db-read 1) (caffe::db-get-float-data))))
(caffe::db-close)

(caffe::db-open :path "input_lmdb_robot_image")
(setq b_ (list (caffe::db-get-data)
	       (progn (caffe::db-read 1) (caffe::db-get-data))
	       (progn (caffe::db-read 1) (caffe::db-get-data))))
(caffe::db-close)

(setq b
      (mapcar
       #'(lambda (av)
	   (send *robot* :angle-vector (copy-seq av))
	   (send *robot* :newcoords (make-coords))
	   (send *robot* :transform (send (send (send *robot* :link "Tibia") :worldcoords) :transformation (make-coords)) :local)
	   (send *viewer* :draw-objects)
	   (let* ((img (get-gl-mono-image :width 32 :height 32))
		  (ret
		   (caffe::calc-forward-double
		    :isize (* 32 32) :osize 15
		    :num 1 :channels 1 :width 32 :height 32
		    :input ;; (map float-vector 'identity (send img :entity))
		    (image-minus)
		    )))
	     (setq ret (map float-vector 'rad2deg ret))
	     (format t "vs ---------~%~A~%~A~%" av ret)
	     (read-line)
	     ret))
       a))

(caffe::get-blob-data :name "conv1" :ret (instantiate float-vector (* 32 32)))

(defun gen-blob-image
  (&key
   (name "conv1")
   (width 32)
   (height 32)
   (count 64)
   (data (caffe::get-blob-data :name name
			       :ret (instantiate float-vector (* width height count))))
   (sdata ;;(map string '(lambda (v) (round (* v 255))) data)))
    (float-vector2image-string data)))
  (unix:system "mkdir /tmp/caffe")
  (dotimes (i count)
    (setq img (instance image::grayscale-image :init width height
			(subseq sdata (* i width height) (* (+ 1 i) width height))))
    (img::write-image-file (format nil "/tmp/caffe/~A~A.jpg" name (caffe::zero-string i)) img)
    ))

(caffe::get-blob-data :name "pool3" :ret (instantiate float-vector 32))
(caffe::gen-net-image :net-path "cnn.prototxt")
