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
(defforeign db-close *eus-caffe-db-plugin* "eus_db_close" (:string :string) :integer)
(defforeign db-read *eus-caffe-db-plugin* "eus_db_read" (:integer) :integer)
(defforeign db-dump *eus-caffe-db-plugin* "eus_db_dump" () :integer)
(defforeign db-get-shape *eus-caffe-db-plugin* "eus_db_get_shape" (:string) :integer)

(defun db-open
  (&key (dtype "lmdb") (path "test") (mode #\w))
  (_db-open dtype path mode))

(defun db-put
  (&key (channel 1) (width 1) (height 1) (label 1) (id_str "000") (data ""))
  (_db-put channel width height label id_str data (length data)))
