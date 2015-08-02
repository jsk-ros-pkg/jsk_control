#!/usr/bin/env roseus

(if (not (find-package "CPPLOG")) (make-package "CPPLOG"))
(In-package "CPPLOG")

(require "eus-plugin-util.l")

(defvar *eus-log-plugin* (gen-plugin-obj "libeus_log.so"))

(defforeign _change_output_stream *eus-log-plugin* "log_change_output_stream" (:string :integer) :integer)
(defforeign echo *eus-log-plugin* "log_test_echo" (:string) :integer)

(defun change-output-stream (str)
  (if (null str) (setq str ""))
  (_change_output_stream str (length str)))
