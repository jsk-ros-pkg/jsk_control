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
  :integer)

(defforeign _get_ip_blobs
  *eus-caffe-plugin*
  "get_ip_blobs"
  (:integer :string)
  :integer)

(defun eus-caffe-test
  (&key
   (isize (* 2 32))
   (dsize 32)
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
      (let* ((x (- (random 2.0) 1.0))
	     (y (- (random 2.0) 1.0)))
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
