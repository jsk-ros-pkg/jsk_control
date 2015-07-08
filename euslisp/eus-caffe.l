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

(defforeign _create_solver
  *eus-caffe-plugin*
  "create_solver"
  (:string :string)
  :integer)

(defforeign _initialize_solver
  *eus-caffe-plugin*
  "initialize_solver"
  (:integer :integer :string :string :string :string)
  :integer)

(defforeign _learn
  *eus-caffe-plugin*
  "caffe_learn"
  ()
  :float)

(defforeign _get_ip_layer_blob
  *eus-caffe-plugin*
  "get_ip_layer_blob"
  (:integer :string :integer)
  :integer)

(defforeign _calc_forward
  *eus-caffe-plugin*
  "calc_forward"
  (:integer :integer :string :string :string)
  :integer)

;; (defforeign string-test
;;   *eus-caffe-plugin*
;;   "string_test"
;;   (:string)
;;   :integer)

(defun calc-forward
  (&key
   (isize 32)
   (osize 1)
   (input (instantiate float-vector isize))
   (output (instantiate float-vector osize))
   (idummy (instantiate float-vector (length input))))
  (_calc_forward isize osize input output idummy)
  output)

(defun get-ip-layer-blob
  (&key
   (blob_id 0)
   (ret (instantiate float-vector 32)))
  (_get_ip_layer_blob blob_id ret (length ret))
  ret)

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

(defun linear-equation-test
  (&key
   (solver "solver.prototxt")
   (size 3200)
   (dsize size)
   (isize (* 2 size))
   (idata)
   (ddata)
   (iclass (instantiate float-vector size)) ;; dummy
   (dclass iclass)
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
  ;;
  (learn :solver solver
	 :isize size
	 :dsize size
	 :idata idata
	 :ddata ddata
	 :iclass iclass
	 :dclass dclass)
  ;;
  (format t " -- check net state~%  ~A->~A~%  ~A->~A~%  ~A->~A~%"
	  (float-vector 0 0) (calc-forward :input (float-vector 0 0) :osize 1)
	  (float-vector 0 2) (calc-forward :input (float-vector 0 2) :osize 1)
	  (float-vector 2 5) (calc-forward :input (float-vector 2 5) :osize 1))
  )

;; (defun linear-equation-test2
;;   (&key
;;    (solver "solver.prototxt")
;;    (dsize 3200)
;;    (isize dsize)
;;    (idata)
;;    (ddata)
;;    (idummy)
;;    (ddummy)
;;    )
;;   (cond
;;    ((or (not idata) (not ddata))
;;     (setq idata (instantiate float-vector isize))
;;     (setq ddata (instantiate float-vector dsize))
;;     (dotimes (i (/ dsize 2))
;;       (let* ((x (* 1 (- (random 2.0) 1.0)))
;; 	     (y (* 1 (- (random 2.0) 1.0))))
;; 	(setf (aref idata (* 2 i)) x)
;; 	(setf (aref idata (+ 1 (* 2 i))) y)
;; 	(setf (aref ddata (* 2 i)) (+ x y))
;; 	(setf (aref ddata (+ 1 (* 2 i))) (+ x y))
;; 	))))
;;   (cond
;;    ((not idummy) (setq idummy (instantiate float-vector isize))))
;;   (cond
;;    ((not ddummy) (setq ddummy (instantiate float-vector dsize))))
;;   ;;
;;   (_learn solver isize dsize idata ddata idummy ddummy)
;;   )
