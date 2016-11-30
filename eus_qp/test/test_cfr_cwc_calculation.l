#!/usr/bin/env roseus

(require "package://eus_qp/euslisp/test-cfr-cwc-calculation.l")

(require :unittest "lib/llib/unittest.l")

(init-unit-test)

(defun get-jskeus-version-num
  ()
  (let ((str (car (piped-fork-returns-list "rosversion jskeus"))))
    (read-from-string (format nil "(~A)" (substitute (elt " " 0) (elt "." 0) str)))))

(defun check-jskeus-version-num
  (aversion)
  (let ((current-version (get-jskeus-version-num)))
    (every #'identity (mapcar #'(lambda (x y) (<= x y)) aversion current-version))))

(defun check-jskeus-version-validity
  ()
#+:ros (check-jskeus-version-num '(1 0 13))
#-:ros t)

(if (check-jskeus-version-validity)
(deftest test-cfr-cwc-calculation
  (assert (progn (setup) t))
  (assert (demo-calc-cfr-all :press-enter-p nil))
  (assert (compare-calc-scfr-lp-qp-all :debug-view nil))
  ;;(assert (compare-calc-dcfr-all :debug-view nil))
  (assert (compare-calc-dcfr-all :debug-view nil :ref-cog-z 700.0))))

(run-all-tests)
(exit 0)
