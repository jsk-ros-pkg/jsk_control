#!/usr/bin/env roseus

(if (not (find-package "CAFFE")) (make-package "CAFFE"))
(In-package "CAFFE")

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

(defforeign _get_layer_blob_data *eus-caffe-plugin* "eus_caffe_get_layer_blob_data" (:string :integer :string :integer) :integer)
(defforeign _get_blob_data *eus-caffe-plugin* "eus_caffe_get_blob_data" (:string :integer :string :integer) :integer)
;;
(defforeign _create_solver *eus-caffe-plugin* "eus_caffe_create_solver" (:string :string) :integer)
(defforeign _initialize_solver *eus-caffe-plugin* "eus_caffe_initialize_solver" (:integer :integer :string :string :string :string) :integer)
(defforeign _learn *eus-caffe-plugin* "eus_caffe_learn" () :float)
(defforeign _calc_forward *eus-caffe-plugin* "eus_caffe_calc_forward" (:integer :integer :string :string :string) :integer)

(defun gen-net-image
  (&key
   (caffe-root
    (format nil "~A/3rdparty/caffe" (ros::rospack-find "eus_caffe")))
   (net-path "net2.prototxt")
   (out-path "/tmp/net.png")
   (eog-open? t)
   )
  ;; PYTHONPATH=$PYTHONPATH:`pwd` python draw_net.py train_val.prototxt net.png --rankdir "TB"
  (unix:system
   (format nil "PYTHONPATH=$PYTHONPATH:~A/python python ~A/python/draw_net.py ~A ~A --rankdir \"TB\""
	   caffe-root caffe-root net-path out-path))
  (if eog-open?
      (unix:system (format nil "eog ~A &" out-path)))
  out-path)

(defun get-layer-blob-data
  (&key
   (name "ip")
   (blob_id 0)
   (ret (instantiate float-vector 32)))
  (_get_layer_blob_data name blob_id ret (length ret))
  ret)

(defun get-blob-data
  (&key
   (name "ip")
   (ret (instantiate float-vector 32)))
  (_get_layer_blob_data name ret (length ret))
  ret)

(defun calc-forward
  (&key
   (isize 32)
   (osize 1)
   (input (instantiate float-vector isize))
   (output (instantiate float-vector osize))
   (idummy (instantiate float-vector (length input))))
  (_calc_forward isize osize input output idummy)
  output)

(defun create-solver
  (&key
   (solver "solver.prototxt")
   (solverstate ""))
  (_create_solver solver solverstate))

(defun initialize-solver
  (&key
   (solver "solver.prototxt")
   (solverstate "")
   (size 3200)
   (dsize size)
   (isize size)
   (idata (instantiate float-vector isize))
   (ddata (instantiate float-vector dsize))
   (iclass (instantiate float-vector size))
   (dclass (instantiate float-vector size))
   )
  (_create_solver solver solverstate)
  (_initialize_solver (length iclass) (length dclass)
		      idata ddata iclass dclass)
  ;; (_learn)
  )

(defun learn
  (&rest
   args
   &key
   (initialize? t)
   &allow-other-keys
   )
  (if initialize? (apply 'initialize-solver args))
  (_learn))

