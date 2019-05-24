#!/usr/bin/env roseus

(require :unittest "lib/llib/unittest.l")
(init-unit-test)

(require "package://eus_osqp/euslisp/test-eus-osqp.l")

(deftest test-eus-osqp-all
  (assert (demo-eus-osqp-all)))

(run-all-tests)
(exit 0)
