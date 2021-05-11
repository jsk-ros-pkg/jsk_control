#!/usr/bin/env roseus

(require :unittest "lib/llib/unittest.l")


(init-unit-test)

(deftest test-load-euslisp-files
  (let ((base-dir (ros::resolve-ros-path "package://eus_qp/optmotiongen/euslisp"))
        (full-file-name))
    (dolist (file (unix::directory base-dir))
      (setq full-file-name (namestring (make-pathname :directory (list base-dir) :name file)))
      (when (eq (probe-file full-file-name) :file)
        (setq *loader-current-directory* nil)
        (load full-file-name)))
    ))


(eval-when
 (load eval)
 (run-all-tests)
 (exit)
 )
