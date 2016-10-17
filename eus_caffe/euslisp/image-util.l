#!/usr/bin/env roseus

(defun convert-from-caffe-image
  (img
   &key
   (width (send img :width))
   (height (send img :height))
   ;; (str-buf (instantiate string (length (send img :entity))))
   (ret-img (instance image::color-image24 :init width height))
   (str-buf (send ret-img :entity))
   )
  (dotimes (h height)
    (dotimes (w width)
      (setf (aref str-buf (+ (* (+ (* h width) w) 3) 0))
	    (aref (send img :entity) (+ (* h width) w)))
      (setf (aref str-buf (+ (* (+ (* h width) w) 3) 1))
	    (aref (send img :entity) (+ (* width height) (* h width) w)))
      (setf (aref str-buf (+ (* (+ (* h width) w) 3) 2))
	    (aref (send img :entity) (+ (* 2 width height) (* h width) w)))
      ))
  (or ret-img str-buf)
  )

(defun convert-to-caffe-image
  (img
   &key
   (width (send img :width))
   (height (send img :height))
   ;; (str-buf (instantiate string (length (send img :entity))))
   (ret-img (instance image::color-image24 :init width height))
   (str-buf (send ret-img :entity))
   )
  (let* ((did -1) (buf (send img :entity)))
    (dotimes (c (send img :byte-depth))
      (dotimes (i (* (send img :width) (send img :height)))
	(setf (aref str-buf (incf did))
	      (aref buf (+ (* i (send img :byte-depth)) c))))))
  (or ret-img str-buf))

(defun check-color-image24-size
  (img)
  (cond
   ((eq (length (send img :entity))
	(* (send img :width) (send img :height)
	   (send img :byte-depth)))
    img)
   ((eq (length (send img :entity))
	(* (send img :width) (send img :height)))
    (format t "[check-color-image24-size] monochronized color image detected~%")
    (instance image::color-image24 :init
	      (send img :width) (send img :height)
	      (let* ((str-buf (instantiate string
					   (* 3 (send img :width)
					      (send img :height)))))
		(dotimes (i (* (send img :width) (send img :height)))
		  (dotimes (j 3)
		    (setf (aref str-buf (+ j (* 3 i)))
			  (aref (send img :entity) i))))
		str-buf)))
   (t
    (format t "[check-color-image24-size] invalid image ~A x ~A x ~A~%"
	    (send img :byte-depth) (send img :width) (send img :height))
    nil)
   ))

(defun test-caffe-image
  nil
  (let* ((img0 (check-color-image24-size
		(img::read-image-file "test.png")))
	 (img-caffe (convert-to-caffe-image img0))
	 (img (convert-from-caffe-image img-caffe)))
    (img::write-image-file "img_to_caffe.png" img-caffe)
    (img::write-image-file "img_from_caffe.png" img)
    (list img0 img-caffe img)))

#|

(setq img0 (instance image::color-image24 :init 32 32 (caffe::db-get-data)))
(setq str-buf (instantiate string (* 32 32 3)))
(img::write-image-file "test0.png" img0)
(dotimes (h 32)
  (dotimes (w 32)
    ;; (dotimes (c 3)
    (setf (aref str-buf (+ (* (+ (* h 32) w) 3) 0))
	  (aref (send img0 :entity) (+ (* h 32) w)))
    (setf (aref str-buf (+ (* (+ (* h 32) w) 3) 1))
	  (aref (send img0 :entity) (+ (* 32 32) (* h 32) w)))
    (setf (aref str-buf (+ (* (+ (* h 32) w) 3) 2))
	  (aref (send img0 :entity) (+ (* 2 32 32) (* h 32) w)))
    ))
(img::write-image-file "test.png" (setq img (instance image::color-image24 :init 32 32 (Copy-seq str-buf))))
(img::write-image-file "test?.png" (setq img? (instance image::color-image24 :init 32 32 (concatenate string (send (send img :blue) :entity) (send (send img :green) :entity) (send (send img :red) :entity)))))
;;
;; (let* ((r (send img :red)) (g (send img :green)) (b (send img :blue))
;; 	 (id 0))
;;   (dotimes (i (length (send r :entity)))
;;     (setf (aref str-buf id) (aref (send r :entity) i))
;;     (incf id))
;;   (dotimes (i (length (send g :entity)))
;;     (setf (aref str-buf id) (aref (send g :entity) i))
;;     (incf id))
;;   (dotimes (i (length (send b :entity)))
;;     (setf (aref str-buf id) (aref (send b :entity) i))
;;     (incf id))
;;   )
(let* ((did -1) (buf (send img :entity)))
  (dotimes (c (send img :byte-depth))
    (dotimes (i (* (send img :width) (send img :height)))
      (setf (aref str-buf (incf did))
	    (aref buf (+ (* i (send img :byte-depth)) c))))))
;; (dotimes (h (send img :height))
;;   (dotimes (w (send img :width))
;;     (dotimes (c (send img :byte-depth))
;;       (setf (aref str-buf
;; 		    (+ (* c (send img :width) (send img :height))
;; 		       (* h (send img :width)) w))
;;             (aref (send img :entity)
;; 		    (+ (* (+ (* h (send img :width)) w)
;; 			  (send img :byte-depth)) c))))))
(img::write-image-file "test2.png" (setq img2 (instance image::color-image24 :init 32 32 (copy-seq str-buf))))
;;
