(require "../eus-caffe.l")
(require "../eus-caffe-db.l")

(defvar *mean-image*
  (if (probe-file "image/mean.jpg") (image::read-image-file "image/mean.jpg")
    (progn
      (unix:system "mkdir image")
      (img::write-image-file
       "image/mean.jpg" (instance image::color-image24 :init 128 128))
      (image::read-image-file "image/mean.jpg"))))

(defun image-minus
  (&optional
   (img1) ;;  (get-gl-mono-image :width 32 :height 32))
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

(defun gen-learning-data
  (&key
   (target-path "target_lmdb_class_id")
   (input-path "input_lmdb_image")
   (image-path "image")
   (width 128) (height 128)
   img
   ;;
   (class-id '(0 1))
   (root-path (list "image/else" "image/cockroach"))
   (i 0)
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
  (mapcar
   #'(lambda (id path)
       (dolist (p (remove-if '(lambda (str) (not (substringp "jpg" str)))
			     (directory path)))
	 (setq img (image::read-image-file (format nil "~A/~A" path p)))
	 (caffe::db-set-id 0)
	 (caffe::db-put-double
	  :channels 1 :width 1 :height 1 :label 0
	  :id_str (caffe::zero-string i)
	  :data (float-vector id))
	 ;;
	 (caffe::db-set-id 1)
	 (caffe::db-put-double
	  :channels 3 :width width :height height :label id
	  :id_str (caffe::zero-string i)
	  :data (image-minus img *mean-image*))
	 (incf i)
	 ))
   class-id root-path)
  ;;
  (caffe::db-set-id 0) (caffe::db-close)
  (caffe::db-set-id 1) (caffe::db-close)
  )

(defun db-image-learn
  nil
  (caffe::learn :solver "cnn_solver.prototxt" :size 0)
  )

(defun mem-image-learn
  (&key
   (width 128) (height 128)
   (size 1792)
   (input-size (* size 3 width height))
   (output-size (* size 1))
   (input-data (instantiate float-vector input-size))
   (output-data (instantiate float-vector output-size))
   img
   ;;
   (class-id '(0 1))
   (root-path (list "image/else" "image/cockroach"))
   (i 0)
   )
  (mapcar
   #'(lambda (id path)
       (if (< i size)
	   (dolist (p (remove-if '(lambda (str) (not (substringp "jpg" str)))
				 (directory path)))
	     (setq img (image::read-image-file (format nil "~A/~A" path p)))
	     (setf (aref output-data i) id)
	     (dotimes (j (length (send img :entity)))
	       (setf (aref input-data (+ j (* i width height 3)))
		     (aref (send img :entity) j)))
	     (incf i)))
       )
   class-id root-path)
  ;;
  (caffe::learn :solver "cnn_mem_solver.prototxt"
		:size size :isize input-size :dsize output-size
		:idata input-data :ddata output-data)
  )

