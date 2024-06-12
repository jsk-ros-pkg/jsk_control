#!/usr/bin/env roseus

(require "eus-plugin-util.l")

(if (not (find-package "CPPLOG")) (make-package "CPPLOG"))
(In-package "CPPLOG")

(defvar *eus-log-plugin* (user::gen-plugin-obj "libeus_log.so"))

(defforeign _change_output_stream *eus-log-plugin* "log_change_output_stream" (:string :integer) :integer)
(defforeign echo *eus-log-plugin* "log_test_echo" (:string) :integer)
(defforeign size-vector *eus-log-plugin* "size_vector" (:string) :integer)
(defforeign _write-vector *eus-log-plugin* "write_vector" (:string :string :integer) :integer)
(defforeign _read-vector *eus-log-plugin* "read_vector" (:string :string :integer) :integer)

(defun change-output-stream (str)
  (if (null str) (setq str ""))
  (format t "cpplog::change-output-stream -> ~A~%"
	  (if (eq (length str) 0) "std::cout" str))
  (_change_output_stream str (length str)))

(defun write-vector (path vec) (_write-vector path vec (length vec)))
(defun read-vector
  (path
   &key
   (size (size-vector path))
   (vec (instantiate float-vector size)))
  (_read-vector path vec (length vec))
  vec)
