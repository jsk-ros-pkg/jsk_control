#!/usr/bin/env roseus

(require "db-ik-learn.l")

(defun gen-small-mono-image
  (img
   &key (x 0) (y 0) z
   (width 32)
   (height 32)
   (worg (send img :width))
   (horg (send img :height))
   (wscale  (/ (* 1.0 worg) width))
   (hscale (/ (* 1.0 horg) height))
   ((:imagebuf imgbuf) (send img :entity)))
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
  )


(defun image-callback
  (msg)
  (setq a msg)
  (let* ((img (instance image::color-image24 :init
                        (send msg :width)
                        (send msg :height)
                        (send msg :data)))
	 (mimg (gen-small-mono-image img))
	 (ret
	  (caffe::calc-forward-double
	   :isize (* 32 32) :osize 15
	   :num 1 :channels 1 :width 32 :height 32
	   :input (image-minus mimg)
	   )))
    (ros::publish "/mimic_pepper/input_image"
		  (instance sensor_msgs::image :init :width 32 :height 32
			    :step 32 :encoding "mono8" :data (send mimg :entity)))
    (setq ret (map float-vector 'rad2deg ret))
    (send *robot* :angle-vector ret)
    (send-all (send *robot* :links) :worldcoords)
    (send *robot* :newcoords (make-coords))
    (send *robot* :transform (send (send (send *robot* :link "Tibia") :worldcoords) :transformation (make-coords)) :local)
    (send *viewer* :draw-objects)
    (x::window-main-one)
    ))

(ros::roseus "mimic_pepper")
(ros::subscribe "/usb_cam/image_raw" ;;"mimic_pepper/camera_image"
		sensor_msgs::Image
		#'image-callback)
(ros::advertise "/mimic_pepper/input_image" ;;"mimic_pepper/camera_image"
		sensor_msgs::Image)

(caffe::gen-test-net
 :netproto "cnn_predict.prototxt"
 :caffemodel "cnn_iter_5000.caffemodel")

(ros::rate 3)
(do-until-key
 (if (not (ros::ok)) (return-from nil))
 (ros::spin-once)
 (ros::sleep))

#|

rosrun usb_cam usb_cam_node
rosrun image_view image_viewimage:=/mimic_pepper/input_image
