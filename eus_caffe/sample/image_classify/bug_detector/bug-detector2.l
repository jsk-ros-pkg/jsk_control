#!/usr/bin/env roseus

(require "../learn.l")

(defun gen-small-image
  (img
   &key (x 0) (y 0) z
   (width (send *mean-image* :width))
   (height (send *mean-image* :height))
   (worg (send img :width))
   (horg (send img :height))
   (wscale  (/ (* 1.0 worg) width))
   (hscale (/ (* 1.0 horg) height))
   ((:imagebuf imgbuf) (send img :entity)))
  (let ((b (make-string (* 3 width height))) zv)
    (dotimes (_x width)
      (dotimes (_y height)
	(setq zv 0)
	(setq x (floor (* wscale _x)))
	(setq y (floor (* hscale _y)))
	(dotimes (_z 3)
	  (setf (elt b (+ _z (* 3 (+ (* (- height _y 1) width) _x))))
		(elt imgbuf (+ (* y worg 3) (* x 3) _z))))
	))
    (instance image::color-image24 :init width height b))
  )

(defun gen-small-image-list
  (img
   &key
   (spilit-width (/ (send img :width) 3))
   (spilit-height (/ (send img :height) 3))
   (width (send *mean-image* :width))
   (height (send *mean-image* :height))
   ret)
  (print 'start-gen-small-image-list)
  (dotimes (w (/ (send img :width) spilit-width))
    (dotimes (h (/ (send img :height) spilit-height))
      (push (send img :subimage (* w spilit-width) (* h spilit-height) spilit-width spilit-height) ret)))
  (print 'done-gen-small-image-list)
  ret)

(defun gen-random-small-image
  (img
   &key
   (_width (/ (send img :width) 2))
   (width_ (/ (send *mean-image* :width) 2))
   (width (+ (send *mean-image* :width)
	     (floor (+ width_ (* (random 1.0) (- _width width_))))))
   (_height (/ (send img :height) 2))
   (height_ (/ (send *mean-image* :height) 2))
   (height (+ (send *mean-image* :height)
	      (floor (+ height_ (* (random 1.0) (- _height height_))))))
   (x (random (- (send img :width) width)))
   (y (random (- (send img :height) height))))
  (format t ":gen-random-small-image ~A x ~A at ~A x ~A~%"
	  width height x y)
  (send img :subimage x y width height))

(defun gen-random-small-image-list
  (img &rest args)
  (mapcar
   #'(lambda (a)
       (apply 'gen-random-small-image (cons img args)))
   (make-list 10)))

(defvar *tag* (list "cockroach" "spider" "human" "rat" "mosquito" "cat"))

(defun image-callback-spec
  (img
   &key
   (class-id '(0 1 2 3 4 5))
   (class-id-vector
    (mapcar #'(lambda (id)
		(let* ((v (instantiate float-vector (length class-id))))
		  (setf (aref v id) 1) v))
	    class-id))
   (tags *tag*))
  (let* ((img_caffe (convert-to-caffe-image (setq img (gen-small-image img))))
	 (ret (predict :img img_caffe :init-net? nil))
	 (ret-id (posision-min ret class-id-vector))
	 )
    (list img ret ret-id)))

(defun image-callback-publish
  (img ret ret-id
       &key
       (class-id '(0 1 2 3 4 5))
       (class-id-vector
	(mapcar #'(lambda (id)
		    (let* ((v (instantiate float-vector (length class-id))))
		      (setf (aref v id) 1) v))
		class-id))
       (tags *tag*))
  (ros::publish "/bug_detector/highest_tag"
		(instance std_msgs::string :init :data (nth ret-id tags)))
  (dolist (id class-id)
    (ros::publish (format nil "/bug_detector/~A/score"
			  (nth id tags))
		  (instance std_msgs::float32 :init :data (aref ret id))))
  (if (and (eq ret-id 0)
	   (> (aref ret 0) 0.9))
      (ros::publish "/bug_detector/cockroach/image"
		    (instance sensor_msgs::image :init :width (send img :width)
			      :height (send img :height)
			      :step (* 3 (send img :width))
			      :encoding "rgb8" :data (send img :entity))))
  )

(defun image-callback
  (msg &rest args)
  (let* ((ret
	  (mapcar
	   #'(lambda (img)
	       (apply 'image-callback-spec (append (list img) args)))
	   ;; (gen-small-image-list
	   (gen-random-small-image-list
	    (instance image::color-image24 :init
		      (send msg :width)
		      (send msg :height)
		      (send msg :data)))))
	 (max-cockroach-ret
	  (let* ((max (car ret)))
	    (dolist (r (cdr ret))
	      (if (> (aref (cadr r) 0)
		     (aref (cadr max) 0))
		  (setq max r)))
	    max)))
    (apply 'image-callback-publish max-cockroach-ret)
    ))

(ros::roseus "bug_detector")
(ros::subscribe "/usb_cam/image_raw"
		sensor_msgs::Image
		#'image-callback 1)
(ros::advertise "/bug_detector/highest_tag"
		std_msgs::string)
(dolist (tag *tag*)
  (ros::advertise (format nil "/bug_detector/~A/score" tag)
		  std_msgs::float32))
(ros::advertise "/bug_detector/cockroach/image"
		sensor_msgs::Image)

(caffe::gen-test-net
 :netproto
 (format nil "~A/sample/image_classify/image_classify_predict.prototxt"
	 (ros::rospack-find "eus_caffe"))
 :caffemodel
 (format nil "~A/sample/image_classify/bug_detector/bug.caffemodel"
	 (ros::rospack-find "eus_caffe")))

(ros::rate 10)
(do-until-key
 (if (not (ros::ok)) (return-from nil))
 (ros::spin-once)
 ;; (ros::sleep)
 (unix:usleep (* (+ 50 (random 50) 1000)))
 )

#|

rosrun usb_cam usb_cam_node
rosrun image_view image_view  image:=/bug_detector/cockroach/image

(send *robot* :reset-pose)
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
  (send *robot* :angle-vector (setq ret (map float-vector 'rad2deg ret)))
  (send *robot* :newcoords (make-coords))
  (send *robot* :transform (send (send (send *robot* :link "Tibia") :worldcoords) :transformation (make-coords)) :local)
  (send *viewer* :draw-objects)
  ret)
