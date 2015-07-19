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
  (scale (/ 1.0 255)
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
   (test-width 100)
   img
   ;;
   (class-id '(1 2 3))
   (root-path (list "image/else" "image/cockroach" "image/spider"))
   (train-path
    (mapcar
     #'(lambda (r)
	 (subseq (remove-if '(lambda (str) (not (substringp "jpg" str)))
			    (directory r)) test-width))
     root-path))
   (test-path
    (mapcar
     #'(lambda (r)
	 (subseq (remove-if '(lambda (str) (not (substringp "jpg" str)))
			    (directory r)) 0 test-width))
     root-path))
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
   #'(lambda (id pl root)
       (dolist (p pl)
	 (setq img (image::read-image-file (format nil "~A/~A" root p)))
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
   class-id train-path root-path)
  ;;
  (caffe::db-set-id 0) (caffe::db-close)
  (caffe::db-set-id 1) (caffe::db-close)
  )

(defun db-image-learn
  (&key
   (test-width 100)
   (class-id '(1 2 3))
   (root-path (list "image/else" "image/cockroach" "image/spider"))
   (train-path
    (mapcar
     #'(lambda (r)
	 (subseq (remove-if '(lambda (str) (not (substringp "jpg" str)))
			    (directory r)) test-width))
     root-path))
   (test-path
    (mapcar
     #'(lambda (r)
	 (subseq (remove-if '(lambda (str) (not (substringp "jpg" str)))
			    (directory r)) 0 test-width))
     root-path))
   train-ret test-ret
   )
  ;;
  ;; learn
  (caffe::learn :solver "cnn_solver.prototxt" :size 0)
  ;;
  ;; test
  (predict :init-net? t)
  (setq
   test-ret
   (mapcar
    #'(lambda (id pl root)
	(mapcar #'(lambda (p) (format t "~A/~A(~A): " root p id)
		    (print (predict :path (format nil "~A/~A" root p) :init-net? nil))) pl))
    class-id test-path root-path))
  (setq
   train-ret
   (mapcar
    #'(lambda (id pl root)
	(mapcar #'(lambda (p) (format t "~A/~A(~A): " root p id)
		    (print (predict :path (format nil "~A/~A" root p) :init-net? nil))) pl))
    class-id train-path root-path))
  ;;
  (format t ":train hit rate: ~A/~A~%"
	  (apply '+ (mapcar '(lambda (id ret) (apply '+ (mapcar #'(lambda (id?) (if (< (abs (- id id?)) 0.5) 1 0)) ret))) class-id train-ret)) (length (flatten train-ret)))
  (format t ":test hit rate: ~A/~A~%"
	  (apply '+ (mapcar '(lambda (id ret) (apply '+ (mapcar #'(lambda (id?) (if (< (abs (- id id?)) 0.5) 1 0)) ret))) class-id test-ret)) (length (flatten test-ret)))
  )

(defun mem-image-learn
  (&key
   (width 128) (height 128)
   (size 1800)
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
       (dolist (p (remove-if '(lambda (str) (not (substringp "jpg" str)))
			     (directory path)))
	 (cond
	  ((< i size)
	   (setq img (image::read-image-file (format nil "~A/~A" path p)))
	   (setf (aref output-data i) id)
	   (dotimes (j (length (send img :entity)))
	     (setf (aref input-data (+ j (* i width height 3)))
		   (aref (send img :entity) j)))
	   (format t "~A/~A~%" (incf i) size)))))
   class-id root-path)
  ;;
  (caffe::learn :solver "cnn_mem_solver.prototxt"
		:size size :isize input-size :dsize output-size
		:idata input-data :ddata output-data)
  )

(defun predict
  (&key (path "image/cockroach/1.jpg")
	(img (image::read-image-file path))
	(init-net? t))
  (if init-net? (caffe::gen-test-net :netproto "cnn_predict.prototxt"
				     :caffemodel "cnn_iter_5000.caffemodel"))
  (caffe::calc-forward-double
   :isize (* 3 (send img :width) (send img :height)) :osize 1
   :num 1 :channels 3 :width (send img :width) :height (send img :height)
   :input(image-minus img *mean-image*)))
