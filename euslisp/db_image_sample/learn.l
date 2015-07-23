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
   (class-id '(0 1 2 3 4 5))
   (class-id-vector
    (mapcar #'(lambda (id)
		(let* ((v (instantiate float-vector (length class-id))))
		  (setf (aref v id) 1) v))
	    class-id))
   (root-path (mapcar #'(lambda (str) (format nil str *image-root*))
		      (list "~A/cockroach" "~A/spider" "~A/human"
			    "~A/rat" "~A/mosquito" "~A/cat")))
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
   #'(lambda (id idv pl root)
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
	  :channels (length idv) :width 1 :height 1 :label id
	  :id_str (caffe::zero-string (aref random-index-vector i))
	  :data idv)
	 ;;
	 (caffe::db-set-id 1)
	 (caffe::db-put-double
	  :channels 3 :width width :height height :label id
	  :id_str (caffe::zero-string (aref random-index-vector i))
	  :data (image-minus img *mean-image*)
	  )
	 (incf i)
	 ))
   class-id class-id-vector train-path root-path)
  ;;
  (caffe::db-set-id 0) (caffe::db-close)
  (caffe::db-set-id 1) (caffe::db-close)
  )

(defun posision-min
  (v vl
     &key
     (comp #'(lambda (v1 v2) (norm (v- v1 v2))))
     (min-dist *inf*)
     (ret 0) (id 0) buf)
  (dolist (v? vl)
    (cond
     ((< (setq buf (funcall comp v v?)) min-dist)
      (setq min-dist buf)
      (setq ret id)))
    (incf id))
  ret)

(defun calc-hit-rate
  (&key
   (class-id '(0 1 2 3 4 5))
   (class-id-vector
    (mapcar #'(lambda (id)
		(let* ((v (instantiate float-vector (length class-id))))
		  (setf (aref v id) 1) v))
	    class-id))
   (root-path (mapcar #'(lambda (str) (format nil str *image-root*))
		      (list "~A/cockroach" "~A/spider" "~A/human"
			    "~A/rat" "~A/mosquito" "~A/cat")))
   (check-path
    (mapcar
     #'(lambda (r)
	 (subseq (remove-if '(lambda (str) (not (substringp "jpg" str)))
			    (directory r)) 0 100))
     root-path))
   (check-full-path
    (mapcar
     #'(lambda (pl root)
	 (mapcar #'(lambda (p) (format nil "~A/~A" root p)) pl))
     check-path root-path))
   ;;
   (ret
    (mapcar
     #'(lambda (pl id idv)
	 (mapcar #'(lambda (p) (format t "~A (~A): " p id)
		     (cond
		      ((eq id
			   (posision-min
			    (predict :path p :init-net? nil)
			    class-id-vector))
		       (print 'yes)
		       1)
		      (t (print 'no) 0)))
		 pl))
     check-full-path class-id class-id-vector))
   ;;
   )
  (mapcar #'(lambda (tag ret)
	      (format t "~A: ~A~%" tag (/ (apply '+ ret) (* 1.0 (length ret)))))
	  root-path ret)
  (format t "average: ~A~%" (/ (* 1.0 (apply '+ (flatten ret))) (length (flatten ret))))
  (list (apply '+ (flatten ret)) (length (flatten ret))))

(defun db-image-learn
  (&key
   (test-width 50)
   (class-id '(0 1 2 3 4 5))
   (class-id-vector
    (mapcar #'(lambda (id)
		(let* ((v (instantiate float-vector (length class-id))))
		  (setf (aref v id) 1) v))
	    class-id))
   (root-path (mapcar #'(lambda (str) (format nil str *image-root*))
		      (list "~A/cockroach" "~A/spider" "~A/human"
			    "~A/rat" "~A/mosquito" "~A/cat")))
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
  (let* ((test-rate (calc-hit-rate :class-id class-id :class-id-vector class-id-vector :root-path root-path :check-path test-path))
	 (train-rate (calc-hit-rate :class-id class-id :class-id-vector class-id-vector :root-path root-path :check-path train-path)))
    (format t ":train hit rate: ~A/~A~%" (car train-rate) (cadr train-rate))
    (format t ":test hit rate: ~A/~A~%" (car test-rate) (cadr test-rate)))
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
				     :caffemodel "cnn_small_iter_30000.caffemodel"))
  (print (subseq (image-minus img *mean-image*) 0 4))
  (setq ret
	(caffe::calc-forward-double
	 :isize (* 3 (send img :width) (send img :height)) :osize 6
	 :num 1 :channels 3 :width (send img :width) :height (send img :height)
	 :input (image-minus img *mean-image*))
	)
  (print ret))

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

#|

(caffe::gen-test-net :netproto "cnn_small_predict.prototxt"
		     :caffemodel "cnn_small_iter_30000.caffemodel")
(calc-hit-rate :root-path (list "image_small/cockroach") :check-path (list (list "782.jpg" "546.jpg" "713.jpg" "869.jpg" "949.jpg" "49.jpg" "738.jpg" "746.jpg" "813.jpg" "152.jpg" "968.jpg" "184.jpg" "123.jpg" "822.jpg" "79.jpg" "414.jpg" "220.jpg" "714.jpg" "932.jpg" "718.jpg" "864.jpg" "603.jpg" "161.jpg" "842.jpg" "624.jpg" "467.jpg" "784.jpg" "177.jpg" "307.jpg" "304.jpg" "541.jpg" "270.jpg" "646.jpg" "504.jpg" "379.jpg" "213.jpg" "229.jpg" "880.jpg" "557.jpg" "480.jpg" "587.jpg" "702.jpg" "314.jpg" "734.jpg" "688.jpg" "853.jpg" "954.jpg" "570.jpg" "271.jpg" "793.jpg" "723.jpg" "313.jpg" "565.jpg" "888.jpg" "310.jpg" "787.jpg" "894.jpg" "815.jpg" "301.jpg" "36.jpg" "918.jpg" "816.jpg" "488.jpg" "500.jpg" "199.jpg" "437.jpg" "300.jpg" "6.jpg" "625.jpg" "792.jpg" "297.jpg" "915.jpg" "491.jpg" "986.jpg" "522.jpg" "879.jpg" "439.jpg" "306.jpg" "181.jpg" "507.jpg" "964.jpg" "214.jpg" "303.jpg" "855.jpg" "65.jpg" "117.jpg" "665.jpg" "97.jpg" "25.jpg" "375.jpg" "241.jpg" "678.jpg" "249.jpg" "610.jpg" "697.jpg" "274.jpg" "244.jpg" "495.jpg" "928.jpg" "67.jpg")))