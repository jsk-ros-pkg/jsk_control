
(require "package://eus_caffe/euslisp/eus-caffe.l")
(require "package://eus_caffe/euslisp/eus-caffe-db.l")

(defun gen-linear-equation-db
  (&key
   (target-path "target_lmdb_linear_equation")
   (input-path "input_lmdb_linear_equation")
   (size 3200)
   (dscale 1)
   (iscale 2)
   (dsize (* dscale size))
   (isize (* iscale size))
   )
  (caffe::db-set-id 0)
  (if (not (zerop (caffe::db-open :dtype "lmdb" :path target-path :mode #\n)))
      (return-from gen-linear-equation-db (print 'db-open-failed)))
  (caffe::db-set-id 1)
  (if (not (zerop (caffe::db-open :dtype "lmdb" :path input-path :mode #\n)))
      (return-from gen-linear-equation-db (print 'db-open-failed)))
  ;;
  (let* ((dbuf (instantiate float-vector dscale))
	 (ibuf (instantiate float-vector iscale)))
    (dotimes (i size)
      (let* ((x (* 10 (- (random 2.0) 1.0)))
	     (y (* 10 (- (random 2.0) 1.0))))
	(setf (aref ibuf 0) x)
	(setf (aref ibuf 1) y)
	(setf (aref dbuf 0) (+ (* 3 x) (* -2 y) 4))
	;;
	(caffe::db-set-id 0)
	(caffe::db-put-double
	 :channels dscale :width 1 :height 1 :label 0
	 :id_str (caffe::zero-string i) :data dbuf)
	;;
	(caffe::db-set-id 1)
	(caffe::db-put-double
	 :channels iscale :width 1 :height 1 :label 0
	 :id_str (caffe::zero-string i) :data ibuf)
	)))
  (caffe::db-set-id 0) (caffe::db-close)
  (caffe::db-set-id 1) (caffe::db-close)
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
	  (caffe::calc-forward-double :input (float-vector 0 0) :isize 2 :osize 1)
	  (float-vector 0 2)
	  (caffe::calc-forward-double :input (float-vector 0 2) :isize 2 :osize 1)
	  (float-vector 2 5)
	  (caffe::calc-forward-double :input (float-vector 2 5) :isize 2 :osize 1))
  )
