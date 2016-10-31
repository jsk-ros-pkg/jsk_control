#!/usr/bin/env roseus

(defvar *img-root-path* "/tmp/image_logger")
(if (not (probe-file *img-root-path*))
    (unix:system (format nil "mkdir -p ~A" *img-root-path*)))

(defvar *expire-time* (* 60 60 24))

(defun remove-expired-images
  nil
  (dolist (p (remove-if '(lambda (str) (not (substringp "jpg" str)))
			(directory *img-root-path*)))
    (if (> (- (car (unix::gettimeofday)) (file-write-date (format nil "~A/~A" *img-root-path* p))) *expire-time*) (unix::system (format nil "rm -rf ~A/~A" *img-root-path* p)))))

(defun image-callback
  (msg &rest args)
  (let* ((img (instance image::color-image24 :init
			(send msg :width)
			(send msg :height)
			(send msg :data))))
    (img::write-image-file (format nil "~A/~A.jpg" *img-root-path*
				   (car (unix::gettimeofday)))
			   img)
    ))

(ros::roseus "image_logger")
(ros::subscribe "/bug_detector/cockroach/image"
		sensor_msgs::Image
		#'image-callback)

(ros::rate 10)
(let* ((cnt *expire-time*))
  (do-until-key
   (if (not (ros::ok)) (return-from nil))
   (ros::spin-once)
   (ros::sleep)
   (if (minusp (decf cnt))
       (progn (remove-expired-images)
	      (setq cnt *expire-time*)))
   ))
