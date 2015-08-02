#!/usr/bin/env roseus

(if (not (find-package "CAFFE")) (make-package "CAFFE"))
(In-package "CAFFE")

(require "eus-plugin-util.l")
(defvar *eus-caffe-db-plugin* (gen-plugin-obj "libeus_caffe_db.so"))

(defforeign db-set-id *eus-caffe-db-plugin* "eus_caffe_db_set_id" (:integer) :integer)
(defforeign _db-open *eus-caffe-db-plugin* "eus_caffe_db_open" (:string :string :integer) :integer)
(defforeign _db-put *eus-caffe-db-plugin* "eus_caffe_db_put" (:integer :integer :integer :integer :string :string :integer) :integer)
(defforeign _db-put-double *eus-caffe-db-plugin* "eus_caffe_db_put_double" (:integer :integer :integer :integer :string :string :integer) :integer)
(defforeign db-close *eus-caffe-db-plugin* "eus_caffe_db_close" () :integer)
(defforeign db-read *eus-caffe-db-plugin* "eus_caffe_db_read" (:integer) :integer)
(defforeign db-dump *eus-caffe-db-plugin* "eus_caffe_db_dump" () :integer)
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

(defun db-random-swap
  (cnt
   &key
   (size 1000)
   (id-list '(0 1))
   rand1 rand2 key val1 val2 shape label)
  (dotimes (i cnt)
    (setq rand1 (random size))
    (setq rand2 (random size))
    (dolist (id id-list)
      (db-set-id id)
      (db-read rand1)
      (setq key (db-get-key))
      (setq val1 (db-get-data))
      (setq val2 (db-get-float-data))
      (setq shape (coerce (db-get-shape) integer-vector))
      (setq label (db-get-label))
      ;;
      (db-read rand2)
      (let ((s (coerce (db-get-shape) integer-vector)))
	(apply
	 (if (plusp (db-get-data-size)) 'db-put 'db-put-double)
	 (list :channels (aref s 0) :width (aref s 1) :height (aref s 2)
	       :label (db-get-label) :id_str key
	       :data
	       (if (plusp (db-get-data-size)) (db-get-data) (db-get-float-data))
	       )))
      (apply
       (if (and val1 (plusp (length val1))) 'db-put 'db-put-double)
       (list :channels (aref shape 0) :width (aref shape 1) :height (aref shape 2)
	     :label label :id_str (db-get-key)
	     :data
	     (if (and val1 (plusp (length val1))) val1 val2)))
      )
    ))

#|

(caffe::db-set-id 0)
(caffe::db-open :path "hoge" :mode #\n)
(caffe::db-set-id 1)
(caffe::db-open :path "fuga" :mode #\n)
(dotimes (i 10)
  (caffe::db-set-id 0)
  (caffe::db-put-double
   :channels i :width i :height i :label i :id_str (format nil "~A" i)
   :data (float-vector i))
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
(dotimes (i 10)
  (caffe::db-set-id 0)
  (caffe::db-read 1) (caffe::db-dump)
  (caffe::db-set-id 1)
  (caffe::db-read 1) (caffe::db-dump)
  (print '----------------)
  )

(caffe::db-random-swap 10 :size 10 :id-list '(0 1))

(dotimes (i 10)
  (caffe::db-set-id 0)
  (caffe::db-read 1) (caffe::db-dump)
  (caffe::db-set-id 1)
  (caffe::db-read 1) (caffe::db-dump)
  (print '----------------)
  )
(caffe::db-set-id 0)
(caffe::db-close)
(caffe::db-set-id 1)
(caffe::db-close)
