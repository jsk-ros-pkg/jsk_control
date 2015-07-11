
(require "../eus-caffe.l")
(require "../eus-caffe-db.l")

(defun zero-string
  (i &optional
     (length 5)
     (ret (instantiate string length))
     (istr (format nil "~A" i)))
  (dotimes (j length)
    (setf (aref ret j)
	  (if (>= j (- length (length istr)))
	      (aref istr (min (length istr)
			      (- j (- length (length istr)))))
	    #\0)))
  ret)

(defun gen-linear-equation-db
  (&key
   (target-path "target_lmdb_linear_equation")
   (input-path "input_lmdb_linear_equation")
   (size 3200)
   (dscale 1)
   (iscale 2)
   (dsize (* dscale size))
   (isize (* iscale size))
   idata ddata
   )
  (cond
   ((or (not idata) (not ddata))
    (setq idata (instantiate float-vector isize))
    (setq ddata (instantiate float-vector dsize))
    (dotimes (i size)
      (let* ((x (* 10 (- (random 2.0) 1.0)))
	     (y (* 10 (- (random 2.0) 1.0))))
	(setf (aref idata (* i 2)) x)
	(setf (aref idata (+ 1 (* i 2))) y)
	(setf (aref ddata i) (+ (* 3 x) (* -2 y) 4))
	))))
  ;; gen target data
  (if (not (zerop (caffe::db-open :dtype "lmdb" :path target-path :mode #\n)))
      (return-from gen-linear-equation-db (print 'db-open-failed)))
  (let* ((buf (instantiate float-vector dscale)))
    (dotimes (i size)
      (dotimes (j dscale) (setf (aref buf j) (aref ddata (+ j (* i dscale)))))
      (caffe::db-put-double
       :channels dscale :width 1 :height 1 :label 0
       :id_str (zero-string i) :data buf)))
  (caffe::db-close)
  ;;
  (if (not (zerop (caffe::db-open :dtype "lmdb" :path input-path :mode #\n)))
      (return-from gen-linear-equation-db (print 'db-open-failed)))
  (let* ((buf (instantiate float-vector iscale)))
    (dotimes (i size)
      (dotimes (j iscale) (setf (aref buf j) (aref idata (+ j (* i iscale)))))
      (caffe::db-put-double
       :channels iscale :width 1 :height 1 :label 0
       :id_str (zero-string i) :data buf)))
  (caffe::db-close)
  ;;
  )

(defun db-learn
  nil
  ;; (caffe::_create_solver "linear_equation_db.prototxt" "")
  ;; (caffe::_learn)
  (caffe::learn :solver "linear_equation_db.prototxt" :size 0)
  ;; test
  (caffe::gen-test-net :netproto "linear_equation_db_net_predict.prototxt"
		       ;; :caffemodel "sample_iter_100000.caffemodel"
		       )
  (format t " -- check net state~%  ~A->~A~%  ~A->~A~%  ~A->~A~%"
	  (float-vector 0 0)
	  (caffe::calc-forward :input (float-vector 0 0) :isize 2 :osize 1)
	  (float-vector 0 2)
	  (caffe::calc-forward :input (float-vector 0 2) :isize 2 :osize 1)
	  (float-vector 2 5)
	  (caffe::calc-forward :input (float-vector 2 5) :isize 2 :osize 1))
  )
