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
   (sample-points 3)
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

(caffe::gen-test-net :netproto "cnn_predict.prototxt" :caffemodel "cnn_iter_5000.caffemodel")

(caffe::db-open :path "target_lmdb_joint_angle")
(setq a (list (caffe::db-get-float-data)
	      (progn (caffe::db-read 1) (caffe::db-get-float-data))
	      (progn (caffe::db-read 1) (caffe::db-get-float-data))))
(caffe::db-close)

(setq b
      (mapcar
       #'(lambda (av)
	   (send *robot* :angle-vector (copy-seq av))
	   (send *viewer* :draw-objects)
	   (let* ((img (get-gl-mono-image :width 32 :height 32))
		  (ret
		   (caffe::calc-forward
		    :isize (* 32 32) :osize 30
		    :num 1 :channels 1 :width 32 :height 32
		    :input (send img :entity))))
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
   (count 32)
   (data (caffe::get-blob-data :name name
			       :ret (instantiate float-vector (* width height count))))
   (sdata (map string '(lambda (v) (round (* v 255))) data)))
  (unix:system "mkdir /tmp/caffe")
  (dotimes (i count)
    (setq img (instance image::grayscale-image :init width height
			(subseq sdata (* i width height) (* (+ 1 i) width height))))
    (img::write-image-file (format nil "/tmp/caffe/~A~A.jpg" name (caffe::zero-string i)) img)
    ))

(caffe::get-blob-data :name "pool3" :ret (instantiate float-vector 32))
