#!/usr/bin/env roseus

(require "learn.l")

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
   (width (send *mean-image* :width))
   (height (send *mean-image* :height))
   ret)
  (dotimes (w (/ (send img :width) width))
    (dotimes (h (/ (send img :height) height))
      (push (send img :subimage (* w width) (* h height) width height)
	    ret)))
  ret)

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
  (let* ((img_caffe (convert-to-caffe-image (gen-small-image img)))
	 (ret (predict :img img_caffe :init-net? nil))
	 (ret-id (posision-min ret class-id-vector))
	 )
    (ros::publish "/bug_detector/highest_tag"
		  (instance std_msgs::string :init :data (nth ret-id tags)))
    (dolist (id class-id)
      (ros::publish (format nil "/bug_detector/~A/score"
			    (nth id tags))
		    (instance std_msgs::float32 :init :data (aref ret id))))
    (if (and (eq ret-id 0)
	     (> (aref ret 0) 0.99))
	(ros::publish "/bug_detector/cockroach/image"
		      (instance sensor_msgs::image :init :width (send img :width)
				:height (send img :height)
				:step (* 3 (send img :width))
				:encoding "rgb8" :data (send img :entity))))
    ))

(defun image-callback
  (msg &rest args)
  (dolist (img (gen-small-image-list
		(instance image::color-image24 :init
			  (send msg :width)
			  (send msg :height)
			  (send msg :data))))
    (apply 'image-callback-spec (append (list img) args))
    ))

(ros::roseus "bug_detector")
(ros::subscribe "/usb_cam/image_raw"
		sensor_msgs::Image
		#'image-callback)
(ros::advertise "/bug_detector/highest_tag"
		std_msgs::string)
(dolist (tag *tag*)
  (ros::advertise (format nil "/bug_detector/~A/score" tag)
		  std_msgs::float32))
(ros::advertise "/bug_detector/cockroach/image"
		sensor_msgs::Image)

(caffe::gen-test-net :netproto
		     (format nil "~A/euslisp/db_image_sample/cnn_small_predict.prototxt"
			     (ros::rospack-find "eus_caffe"))
		     :caffemodel
		     (format nil "~A/euslisp/db_image_sample/bug.caffemodel"
			     (ros::rospack-find "eus_caffe")))

(ros::rate 3)
(do-until-key
 (if (not (ros::ok)) (return-from nil))
 (ros::spin-once)
 (ros::sleep))

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
