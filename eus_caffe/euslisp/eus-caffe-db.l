#!/usr/bin/env roseus

(require "eus-plugin-util.l")

(if (not (find-package "CAFFE")) (make-package "CAFFE"))
(In-package "CAFFE")

(defvar *eus-caffe-db-plugin* (user::gen-plugin-obj "libeus_caffe_db.so"))

(defforeign db-set-id *eus-caffe-db-plugin* "eus_caffe_db_set_id" (:integer) :integer)
(defforeign _db-open *eus-caffe-db-plugin* "eus_caffe_db_open" (:string :string :integer) :integer)
(defforeign _db-put *eus-caffe-db-plugin* "eus_caffe_db_put" (:integer :integer :integer :integer :string :string :integer) :integer)
(defforeign _db-put-double *eus-caffe-db-plugin* "eus_caffe_db_put_double" (:integer :integer :integer :integer :string :string :integer) :integer)
(defforeign db-commit *eus-caffe-db-plugin* "eus_caffe_db_commit" () :integer)
(defforeign db-close *eus-caffe-db-plugin* "eus_caffe_db_close" () :integer)
(defforeign db-read *eus-caffe-db-plugin* "eus_caffe_db_read" (:integer) :integer)
(defforeign db-read-pos *eus-caffe-db-plugin* "eus_caffe_db_read_pos" (:integer) :integer)
(defforeign db-dump *eus-caffe-db-plugin* "eus_caffe_db_dump" () :integer)
(defforeign db-get-size *eus-caffe-db-plugin* "eus_caffe_db_get_size" () :integer)
(defforeign _db-get-shape *eus-caffe-db-plugin* "eus_caffe_db_get_shape" (:string) :integer)
(defforeign _db-get-key *eus-caffe-db-plugin* "eus_caffe_db_get_key" (:string) :integer)
(defforeign db-get-label *eus-caffe-db-plugin* "eus_caffe_db_get_label" () :integer)
(defforeign _db-get-data *eus-caffe-db-plugin* "eus_caffe_db_get_data" (:string) :integer)
(defforeign _db-get-float-data *eus-caffe-db-plugin* "eus_caffe_db_get_float_data" (:string) :integer)
(defforeign db-get-key-size *eus-caffe-db-plugin* "eus_caffe_db_get_key_size" () :integer)
(defforeign db-get-data-size *eus-caffe-db-plugin* "eus_caffe_db_get_data_size" () :integer)
(defforeign db-get-float-data-size *eus-caffe-db-plugin* "eus_caffe_db_get_float_data_size" () :integer)

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

(defun db-open
  (&key (dtype "lmdb") (path "test") (mode #\r))
  (_db-open dtype path mode))

(defun db-put
  (&key (channels 1) (width 1) (height 1) (label 1) (id_str "000") (data ""))
  (_db-put channels width height label id_str data (length data)))

(defun db-put-double
  (&key (channels 1) (width 1) (height 1) (label 1) (id_str "000") (data (float-vector 0)))
  (_db-put-double channels width height label id_str data (length data)))

(defun db-get-key
  (&key (ret (instantiate string (db-get-key-size))))
  (_db-get-key ret)
  ret)

(defun db-get-data
  (&key (ret (instantiate string (db-get-data-size))))
  (_db-get-data ret)
  ret)

(defun db-get-float-data
  (&key (ret (instantiate float-vector (db-get-float-data-size))))
  (_db-get-float-data ret)
  ret)

(defun db-get-shape
  (&key (ret (instantiate float-vector 3)))
  (_db-get-shape ret)
  ret)

(defun db-swap
  (id1 id2 &key
   key1 val1 cval1 shape1 label1
   key2 val2 cval2 shape2 label2)
  ;;
  (db-read-pos id1)
  (setq key1 (copy-seq (db-get-key)))
  (setq cval1 (copy-seq (db-get-data)))
  (setq val1 (copy-seq (db-get-float-data)))
  (setq shape1 (coerce (copy-seq (db-get-shape)) integer-vector))
  (setq label1 (db-get-label))
  ;;
  (db-read-pos id2)
  (setq key2 (copy-seq (db-get-key)))
  (setq cval2 (copy-seq (db-get-data)))
  (setq val2 (copy-seq (db-get-float-data)))
  (setq shape2 (coerce (copy-seq (db-get-shape)) integer-vector))
  (setq label2 (db-get-label))
  ;;
  (apply
   (if (and cval2 (plusp (length cval2))) 'db-put 'db-put-double)
   (list :channels (aref shape2 0) :width (aref shape2 1) :height (aref shape2 2)
	 :label label2 :id_str key1
	 :data
	 (if (and cval2 (plusp (length cval2))) cval2 val2)))
  (apply
   (if (and cval1 (plusp (length cval1))) 'db-put 'db-put-double)
   (list :channels (aref shape1 0) :width (aref shape1 1) :height (aref shape1 2)
	 :label label1 :id_str key2
	 :data
	 (if (and cval1 (plusp (length cval1))) cval1 val1))))

(defun db-random-swap
  (_cnt
   &key
   (size (db-get-size))
   (id-list '(0 1)))
  (labels ((_db-random-swap
	    (_cnt
	     &key
	     (size (db-get-size))
	     (cnt (min _cnt 1000 (/ size 2)))
	     (id-vector
	      (let* ((l (user::random-sample-indices (* cnt 2) :max size))
		     (v (coerce l integer-vector)))
		(user::random-vector-swap v :cnt (* 2 cnt))))
	     (id-list '(0 1))
	     rand1 rand2
	     key1 val1 cval1 shape1 label1
	     key2 val2 cval2 shape2 label2)
	    (dotimes (i cnt)
	      (setq rand1 (aref id-vector (+ 0 (* 2 i))))
	      (setq rand2 (aref id-vector (+ 1 (* 2 i))))
	      (dolist (id id-list)
		(db-set-id id)
		(db-swap rand1 rand2)
		))
	    cnt))
    (while (> (setq _cnt (- _cnt (_db-random-swap _cnt :size size :id-list id-list))) 0) 'nop)))


#|

(unix::system "rm -rf hoge fuga")
(caffe::db-set-id 0)
(caffe::db-open :path "hoge" :mode #\n)
(caffe::db-set-id 1)
(caffe::db-open :path "fuga" :mode #\n)
(dotimes (i 1000)
  (caffe::db-set-id 0)
  (caffe::db-put
   :channels i :width i :height i :label i :id_str (format nil "~A" i)
   :data (format nil "hoge~A" i))
  (caffe::db-set-id 1)
  (caffe::db-put-double
   :channels i :width i :height i :label i :id_str (format nil "~A" i)
   :data (float-vector i)))
(caffe::db-set-id 0)
(caffe::db-close)
(caffe::db-set-id 1)
(caffe::db-close)

(caffe::db-set-id 0)
(caffe::db-open :path "hoge" :mode #\w)
(caffe::db-set-id 1)
(caffe::db-open :path "fuga" :mode #\w)
(let* ((ret) buf1 buf2)
  (dotimes (i 10)
    (caffe::db-set-id 0)
    (caffe::db-read 1)
    (setq buf1 (caffe::db-get-data))
    (caffe::db-set-id 1)
    (caffe::db-read 1)
    (setq buf2 (aref (caffe::db-get-float-data) 0))
    (push (list buf2 buf1) ret)
    )
  (print ret)
  (print (mapcar
	  #'(lambda (val)
	      (count-if #'(lambda (val2) (eq (car val) (car val2))) ret))
	  ret))
  )
(caffe::db-set-id 0)
(caffe::db-close)
(caffe::db-set-id 1)
(caffe::db-close)

(caffe::db-set-id 0)
(caffe::db-open :path "hoge" :mode #\w)
(caffe::db-set-id 1)
(caffe::db-open :path "fuga" :mode #\w)
(caffe::db-random-swap 10000 :id-list '(0 1))
(caffe::db-set-id 0)
(caffe::db-close)
(caffe::db-set-id 1)
(caffe::db-close)

(caffe::db-set-id 0)
(caffe::db-open :path "hoge" :mode #\r)
(caffe::db-set-id 1)
(caffe::db-open :path "fuga" :mode #\r)
(let* ((ret) buf1 buf2)
  (dotimes (i 1000)
    (caffe::db-set-id 0)
    (caffe::db-read 1)
    (setq buf1 (caffe::db-get-data))
    (caffe::db-set-id 1)
    (caffe::db-read 1)
    (setq buf2 (aref (caffe::db-get-float-data) 0))
    (push (list buf2 buf1) ret)
    )
  (print ret)
  (print (mapcar
	  #'(lambda (val)
	      (count-if #'(lambda (val2) (eq (car val) (car val2))) ret))
	  ret))
  )
(caffe::db-set-id 0)
(caffe::db-close)
(caffe::db-set-id 1)
(caffe::db-close)
