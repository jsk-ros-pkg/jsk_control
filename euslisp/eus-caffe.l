#!/usr/bin/env roseus

;; (require "eus-log.l")
(require "eus-plugin-util.l")

(if (not (find-package "CAFFE")) (make-package "CAFFE"))
(In-package "CAFFE")

(defvar *eus-caffe-plugin* (user::gen-plugin-obj "libeus_caffe.so"))

(defforeign _get_layer_blob_data *eus-caffe-plugin* "eus_caffe_get_layer_blob_data" (:string :integer :string :integer) :integer)
(defforeign _get_train_net_layer_blob_data *eus-caffe-plugin* "eus_caffe_get_train_net_layer_blob_data" (:string :integer :string :integer) :integer)
(defforeign _get_blob_data *eus-caffe-plugin* "eus_caffe_get_blob_data" (:string :string :integer) :integer)
(defforeign _get_train_net_blob_data *eus-caffe-plugin* "eus_caffe_get_train_net_blob_data" (:string :string :integer) :integer)
(defforeign _get_input_blob_data *eus-caffe-plugin* "eus_caffe_get_input_blob_data" (:integer :string :integer) :integer)
;;
(defforeign _create_solver *eus-caffe-plugin* "eus_caffe_create_solver" (:string :string) :integer)
(defforeign _reset_memory_layer *eus-caffe-plugin* "eus_caffe_reset_memory_layer" (:string :integer :string :string) :integer)
(defforeign _initialize_solver *eus-caffe-plugin* "eus_caffe_initialize_solver" (:integer :integer :string :string :string :string) :integer)
(defforeign _learn *eus-caffe-plugin* "eus_caffe_learn" () :float)
(defforeign _calc_forward *eus-caffe-plugin* "eus_caffe_calc_forward" (:integer :integer :integer :integer :integer :string :integer :string) :integer)
(defforeign _calc_forward_double *eus-caffe-plugin* "eus_caffe_calc_forward_double" (:integer :integer :integer :integer :integer :string :integer :string) :integer)
(defforeign _memory_calc_forward *eus-caffe-plugin* "eus_caffe_memory_calc_forward" (:integer :integer :string :string :string) :integer)
(defforeign _gen_test_net *eus-caffe-plugin* "eus_caffe_gen_test_net" (:string :string) :integer)

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

(defun get-blob-data
  (&key
   (name "ip")
   (ret (instantiate float-vector 32)))
  (_get_blob_data name ret (length ret))
  ret)

(defun get-train-net-blob-data
  (&key
   (name "ip")
   (ret (instantiate float-vector 32)))
  (_get_train_net_blob_data name ret (length ret))
  ret)

(defun get-layer-blob-data
  (&key
   (name "ip")
   (blob_id 0)
   (ret (instantiate float-vector 32)))
  (_get_layer_blob_data name blob_id ret (length ret))
  ret)

(defun get-train-net-layer-blob-data
  (&key
   (name "ip")
   (blob_id 0)
   (ret (instantiate float-vector 32)))
  (_get_train_net_layer_blob_data name blob_id ret (length ret))
  ret)

(defun get-input-blob-data
  (&key
   (id 0)
   (ret (instantiate float-vector 32)))
  (_get_input_blob_data id ret (length ret))
  ret)

(defun calc-forward
  (&key
   (isize 2)
   (osize 1)
   (num 1)
   (channels isize)
   (width 1)
   (height 1)
   (input (instantiate string isize))
   (output (instantiate float-vector osize)))
  (_calc_forward num channels width height
		 (length input) input (length output) output)
  output)

(defun calc-forward-double
  (&key
   (isize 2)
   (osize 1)
   (num 1)
   (channels isize)
   (width 1)
   (height 1)
   (input (instantiate float-vector isize))
   (output (instantiate float-vector osize)))
  (_calc_forward_double
   num channels width height
   (length input) input (length output) output)
  output)

(defun memory-calc-forward
  (&key
   (isize 32)
   (osize 1)
   (input (instantiate float-vector isize))
   (output (instantiate float-vector osize))
   (idummy (instantiate float-vector (length input))))
  (_memory_calc_forward isize osize input output idummy)
  output)

(defun create-solver
  (&key
   (solver "solver.prototxt")
   (solverstate ""))
  (_create_solver solver solverstate))

(defun reset-memory-layer
  (&key
   (name "input")
   (size 3200)
   (data (instantiate float-vector (* 1 size)))
   (label (instantiate float-vector size)))
  (_reset_memory_layer name size data label))

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
  (create-solver :solver solver :solverstate solverstate)
  (_initialize_solver (length iclass) (length dclass)
		      idata ddata iclass dclass)
  ;; (_learn)
  )

(defun learn
  (&rest
   args
   &key
   (initialize? t)
   (timer (instance user::mtimer :init))
   (log (format nil "caffe_~A.log" (car (unix::gettimeofday))))
   ret
   &allow-other-keys
   )
  ;; (if log (cpplog::change-output-stream log))
  (if initialize? (apply 'initialize-solver args))
  (setq ret (_learn))
  (if timer (format t "time: ~A~%" (setq timer (send timer :stop))))
  ;; (if log (cpplog::change-output-stream ""))
  ret)

(defun gen-test-net
  (&key (netproto "") (caffemodel ""))
  (_gen_test_net netproto caffemodel))
