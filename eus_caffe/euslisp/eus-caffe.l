#!/usr/bin/env roseus

(defvar *eus-caffe-plugin*
  (labels
      ((library_search
	(str &key
	     (depth 0)
	     colon-pos lib-path)
	(format t "  [~A] target=" depth)
	(cond
	 ((eq (length str) 0)
	  (format t "~% caffe_plugin_not_found in eus_caffe.l~%")
	  (exit -1))
	 ((and (setq colon-pos (or (position #\: str) (length str)))
	       (setq lib-path (subseq str 0 colon-pos))
	       (setq lib-path
		     (if (eq (aref lib-path (- (length lib-path) 1)) #\/)
			 (subseq lib-path 0 (- (length lib-path) 1))
		       lib-path))
	       (probe-file (setq lib-path
				 (print (format nil "~A/libeus_caffe.so" lib-path)))))
	  (load-foreign lib-path))
	 (t
	  (library_search (subseq str (min (length str) (+ colon-pos 1)))
			  :depth (+ depth 1))))))
    (library_search (format nil "~A:~A/lib"
			    (unix:getenv "LD_LIBRARY_PATH")
			    (read-line (piped-fork "rospack find eus_caffe"))))))

(defforeign _eus-caffe-test
  *eus-caffe-plugin*
  "deep_learning_test"
  (:integer :integer :string :string :string :string)
  :float)

(defforeign _get_ip_blob_by_id
  *eus-caffe-plugin*
  "get_ip_blob_by_id"
  (:integer :string)
  :integer)

(defforeign _calc_learning_data
  *eus-caffe-plugin*
  "calc_learning_data"
  (:integer :string :string :string)
  :integer)

(defun calc-learning-test
  (&key
   (isize 32)
   (input (instantiate float-vector isize))
   (output (instantiate float-vector isize))
   (idummy (instantiate float-vector (length input))))
  (_calc_learning_data isize input output idummy)
  output)

(defun get-ip-blob-by-id
  (&key
   (blob_id 0)
   (ret (instantiate float-vector 32)))
  (_get_ip_blob_by_id blob_id ret)
  ret)

(defun eus-caffe-test
  (&key
   (dsize 32)
   (isize (* 2 dsize))
   (idata)
   (ddata)
   (idummy)
   (ddummy)
   )
  (cond
   ((or (not idata) (not ddata))
    (setq idata (instantiate float-vector isize))
    (setq ddata (instantiate float-vector dsize))
    (dotimes (i dsize)
      (let* ((x (* 10 (- (random 2.0) 1.0)))
	     (y (* 10 (- (random 2.0) 1.0))))
	(setf (aref idata (* i 2)) x)
	(setf (aref idata (+ 1 (* i 2))) y)
	(setf (aref ddata i) (+ (* 3 x) (* -2 y) 4))
	))))
  (cond
   ((not idummy) (setq idummy (instantiate float-vector isize))))
  (cond
   ((not ddummy) (setq ddummy (instantiate float-vector dsize))))
  ;;
  (_eus-caffe-test isize dsize idata ddata idummy ddummy)
  )
