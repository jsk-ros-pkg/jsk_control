(require "../eus-caffe.l")
(require "../eus-caffe-db.l")
(require "../image-util.l")

(defvar *image-root* "image_small")
(defvar *mean-image*
  (if (probe-file (format nil "~A/mean.jpg" *image-root*))
      (image::read-image-file (format nil "~A/mean.jpg" *image-root*))
    (progn
      (unix:system (format nil "mkdir ~A" *image-root*))
      (img::write-image-file
       (format nil "~A/mean.jpg" *image-root*)
       (instance image::color-image24 :init 64 64
		 (coerce (make-list (* 3 64 64) :initial-element 122) string))
       )
      (image::read-image-file (format nil "~A/mean.jpg" *image-root*)))))

;; (defun image-minus
;;   (&optional
;;    (img1) ;;  (get-gl-mono-image :width 32 :height 32))
;;    (img2 *mean-image*)
;;    (width (send img1 :width))
;;    (height (send img1 :height))
;;    (channels (send img1 :byte-depth))
;;    (channels1 (/ (length (send img1 :entity)) (* width height)))
;;    (channels2 (/ (length (send img2 :entity)) (* width height)))
;;    (ret (instantiate float-vector (* width height channels))))
;;   ;;
;;   (cond
;;    ((or (not (eq channels1 channels))
;; 	(not (eq channels2 channels)))
;;     (format t "[image-minus] channels conflict ~A vs ~A, ~A~%"
;; 	    channels channels1 channels2)
;;     (format t " image: ~A~%" (send img1 :name))))
;;   ;;
;;   (let* ((did -1) (buf1 (send img1 :entity)) (buf2 (send img2 :entity)))
;;     (dotimes (c channels)
;;       (dotimes (i (* width height))
;; 	(setf (aref ret (incf did))
;; 	      (- (aref buf1 (+ (* i channels1) (mod c channels1)))
;; 		 (aref buf2 (+ (* i channels2) (mod c channels2))))))))
;;   ret)

(defun image-minus
  (&optional
   (img1) ;;  (get-gl-mono-image :width 32 :height 32))
   (img2 *mean-image*)
   )
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

(defun random-index-vector
  (size
   &key
   (ret (instantiate integer-vector size))
   (cnt (* 10 size)))
  (dotimes (i size) (setf (aref ret i) i))
  (dotimes (i cnt)
    (let* ((x (random size))
	   (y (random size))
	   (buf (aref ret x)))
      (setf (aref ret x) (aref ret y))
      (setf (aref ret y) buf)))
  ret)

(defun gen-learning-data
  (&key
   (target-path "target_lmdb_class_id_small")
   (input-path "input_lmdb_image_small")
   (image-path "caffe_image")
   (width (send *mean-image* :width))
   (height (send *mean-image* :height))
   (test-width 50)
   img
   ;;
   (class-id '(0 1 2))
   (root-path (mapcar #'(lambda (str) (format nil str *image-root*))
		      (list "~A/else" "~A/cockroach" "~A/spider")))
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
   (random-index-vector (random-index-vector (length (flatten train-path))))
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
	 (setq img (check-color-image24-size img))
	 (setq img (convert-to-caffe-image img))
	 (img::write-image-file (format nil "~A/~A.jpg" image-path (caffe::zero-string (aref random-index-vector i))) img)
	 (format t "~A: ~A ~A x ~A x ~A (~A)~%"
		 (caffe::zero-string (aref random-index-vector i))
		 (format nil "~A/~A" root p)
		 3 (send img :width) (send img :height)
		 id)
	 (caffe::db-set-id 0)
	 (caffe::db-put-double
	  :channels 1 :width 1 :height 1 :label id
	  :id_str (caffe::zero-string (aref random-index-vector i))
	  :data (float-vector id))
	 ;;
	 (caffe::db-set-id 1)
	 (caffe::db-put-double
	  :channels 3 :width width :height height :label id
	  :id_str (caffe::zero-string (aref random-index-vector i))
	  :data (image-minus img *mean-image*)
	  )
	 (incf i)
	 ))
   class-id train-path root-path)
  ;;
  (caffe::db-set-id 0) (caffe::db-close)
  (caffe::db-set-id 1) (caffe::db-close)
  )

(defun db-image-learn
  (&key
   (test-width 50)
   (class-id '(0 1 2))
   (root-path (mapcar #'(lambda (str) (format nil str *image-root*))
		      (list "~A/else" "~A/cockroach" "~A/spider")))
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
  (caffe::learn :solver "cnn_small_solver.prototxt" :size 0)
  ;;
  ;; test
  (predict :init-net? t)
  (setq
   test-ret
   (mapcar
    #'(lambda (id pl root)
	(mapcar #'(lambda (p) (format t "~A/~A(~A): " root p id)
		    (print (aref (predict :path (format nil "~A/~A" root p) :init-net? nil) 0))) pl))
    class-id test-path root-path))
  (setq
   train-ret
   (mapcar
    #'(lambda (id pl root)
	(mapcar #'(lambda (p) (format t "~A/~A(~A): " root p id)
		    (print (aref (predict :path (format nil "~A/~A" root p) :init-net? nil) 0))) pl))
    class-id train-path root-path))
  ;;
  (format t ":train hit rate: ~A/~A~%"
	  (apply '+ (mapcar '(lambda (id ret) (apply '+ (mapcar #'(lambda (id?) (if (< (abs (- id id?)) 0.5) 1 0)) ret))) class-id train-ret)) (length (flatten train-ret)))
  (format t ":test hit rate: ~A/~A~%"
	  (apply '+ (mapcar '(lambda (id ret) (apply '+ (mapcar #'(lambda (id?) (if (< (abs (- id id?)) 0.5) 1 0)) ret))) class-id test-ret)) (length (flatten test-ret)))
  )

(defun mem-image-learn
  (&key
   (width (send *mean-image* :width))
   (height (send *mean-image* :height))
   (size 1800)
   (input-size (* size 3 width height))
   (output-size (* size 1))
   (input-data (instantiate float-vector input-size))
   (output-data (instantiate float-vector output-size))
   img
   ;;
   (class-id '(0 1 2))
   (root-path (mapcar #'(lambda (str) (format nil str *image-root*))
		      (list "~A/else" "~A/cockroach" "~A/spider")))
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
  (&key (path "image_small/cockroach/1.jpg")
	(img (let* ((i (image::read-image-file path)))
	       (setq i (check-color-image24-size i))
	       (setq i (convert-to-caffe-image i))))
	(init-net? t) ret)
  (if init-net? (caffe::gen-test-net :netproto "cnn_small_predict.prototxt"
				     :caffemodel "cnn_small_iter_1000.caffemodel"))
  (print (subseq (image-minus img *mean-image*) 0 4))
  (setq ret
	(caffe::calc-forward-double
	 :isize (* 3 (send img :width) (send img :height)) :osize 3
	 :num 1 :channels 3 :width (send img :width) :height (send img :height)
	 :input (image-minus img *mean-image*))
	)
  (print ret)
  (let* ((max (aref ret 0)) (pos 0))
    (dotimes (i (- (length ret) 1))
      (cond
       ((> (aref ret (+ 1 i)) max)
	(setq max (aref ret (+ 1 i)))
	(setq pos (+ 1 i)))))
    pos))

(defun gen-blob-image
  (&key
   (name "conv1")
   (width (send *mean-image* :width))
   (height (send *mean-image* :height))
   (count 64)
   (data (caffe::get-blob-data :name name
                               :ret (instantiate float-vector (* 3 width height count))))
   (sdata
    (float-vector2image-string data)))
  (unix:system "mkdir /tmp/caffe")
  (dotimes (i count)
    (setq img (instance image::color-image24 :init width height
                        (subseq sdata (* 3 i width height) (* 3 (+ 1 i) width height))))
    (img::write-image-file (format nil "/tmp/caffe/~A~A.jpg" name (caffe::zero-string i)) img)
    ))

