#!/usr/bin/env roseus

(require :unittest "lib/llib/unittest.l")
(init-unit-test)

(require "package://eus_osqp/euslisp/test-eus-osqp.l")

(deftest test-eus-osqp-all
  (assert (demo-eus-osqp-all :qp-mode :qp)))

(deftest test-eus-osqp-all-sqp
  (assert (demo-eus-osqp-all :qp-mode :sqp-with-hotstart)))

(run-all-tests)
(exit 0)
