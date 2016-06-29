#!/usr/bin/env roseus

(require :unittest "lib/llib/unittest.l")
(init-unit-test)

(require "package://eus_qpoases/euslisp/test-eus-qpoases.l")

(deftest test-normal-qp/lp
  (assert (demo-eus-qpOASES-all)))

(deftest test-hotstart-sqp/slp
  (assert (demo-eus-qpOASES-all :mode :sqp/slp-with-hotstart)))

(run-all-tests)
(exit 0)
