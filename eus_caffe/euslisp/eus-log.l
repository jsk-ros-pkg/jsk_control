#!/usr/bin/env roseus

(require "eus-plugin-util.l")

(if (not (find-package "CPPLOG")) (make-package "CPPLOG"))
(In-package "CPPLOG")

(defvar *eus-log-plugin* (user::gen-plugin-obj "libeus_log.so"))

(defforeign _change_output_stream *eus-log-plugin* "log_change_output_stream" (:string :integer) :integer)
(defforeign echo *eus-log-plugin* "log_test_echo" (:string) :integer)

(defun change-output-stream (str)
  (if (null str) (setq str ""))
  (format t "cpplog::change-output-stream -> ~A~%"
	  (if (eq (length str) 0) "std::cout" str))
  (_change_output_stream str (length str)))
