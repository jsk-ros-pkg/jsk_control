#!/usr/bin/env roseus

(if (not (find-package "CAFFE")) (make-package "CAFFE"))
(In-package "CAFFE")

(defvar *eus-caffe-db-plugin*
  (labels
      ((library_search
	(str &key
	     (depth 0)
	     colon-pos lib-path)
	(format t "  [~A] target=" depth)
	(cond
	 ((eq (length str) 0)
	  (format t "~% caffe_db_plugin_not_found in eus_caffe_db.l~%")
	  (exit -1))
	 ((and (setq colon-pos (or (position #\: str) (length str)))
	       (setq lib-path (subseq str 0 colon-pos))
	       (setq lib-path
		     (if (eq (aref lib-path (- (length lib-path) 1)) #\/)
			 (subseq lib-path 0 (- (length lib-path) 1))
		       lib-path))
	       (probe-file (setq lib-path
				 (print (format nil "~A/libeus_caffe_db.so" lib-path)))))
	  (load-foreign lib-path))
	 (t
	  (library_search (subseq str (min (length str) (+ colon-pos 1)))
			  :depth (+ depth 1))))))
    (library_search (format nil "~A:~A/lib"
			    (unix:getenv "LD_LIBRARY_PATH")
			    (read-line (piped-fork "rospack find eus_caffe"))))))


(defforeign _db-open *eus-caffe-db-plugin* "eus_db_open" (:string :string :integer) :integer)
(defforeign _db-put *eus-caffe-db-plugin* "eus_db_put" (:integer :integer :integer :integer :string :string :integer) :integer)
(defforeign _db-put-double *eus-caffe-db-plugin* "eus_db_put_double" (:integer :integer :integer :integer :string :string :integer) :integer)
(defforeign db-close *eus-caffe-db-plugin* "eus_db_close" () :integer)
(defforeign db-read *eus-caffe-db-plugin* "eus_db_read" (:integer) :integer)
(defforeign db-dump *eus-caffe-db-plugin* "eus_db_dump" () :integer)
(defforeign _db-get-shape *eus-caffe-db-plugin* "eus_db_get_shape" (:string) :integer)
(defforeign _db-get-key *eus-caffe-db-plugin* "eus_db_get_key" (:string) :integer)
(defforeign _db-get-data *eus-caffe-db-plugin* "eus_db_get_data" (:string) :integer)
(defforeign _db-get-float-data *eus-caffe-db-plugin* "eus_db_get_float_data" (:string) :integer)
(defforeign db-get-key-size *eus-caffe-db-plugin* "eus_db_get_key_size" () :integer)
(defforeign db-get-data-size *eus-caffe-db-plugin* "eus_db_get_data_size" () :integer)
(defforeign db-get-float-data-size *eus-caffe-db-plugin* "eus_db_get_float_data_size" () :integer)

(defun db-open
  (&key (dtype "lmdb") (path "test") (mode #\w))
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
