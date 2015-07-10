#!/usr/bin/env roseus

(require :unittest "lib/llib/unittest.l")
(init-unit-test)

(require "package://eus_qp/euslisp/test-contact-wrench-opt.l")
(setup)

(deftest test-all
  (assert (and (test-distribute-all :func #'test-wrench-torque-distribute-by-qp :press-enter-p nil)
               (test-distribute-all :func #'test-wrench-distribute-by-pseudo-inverse :press-enter-p nil)
               (test-distribute-all :func #'test-wrench-distribute-by-qp :press-enter-p nil)
               (demo-cbox-wrench-calc-1)
               (demo-cbox-wrench-calc-2))))

(run-all-tests)
(exit 0)
