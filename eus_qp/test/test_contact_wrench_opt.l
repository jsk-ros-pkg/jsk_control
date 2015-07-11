#!/usr/bin/env roseus

(require :unittest "lib/llib/unittest.l")
(init-unit-test)

(require "package://eus_qp/euslisp/test-contact-wrench-opt.l")
(setup)

(deftest test-samplerobot-distribute
  (assert (and (test-distribute-all :func #'test-wrench-torque-distribute-by-qp :press-enter-p nil)
               (test-distribute-all :func #'test-wrench-distribute-by-pseudo-inverse :press-enter-p nil)
               (test-distribute-all :func #'test-wrench-distribute-by-qp :press-enter-p nil))))
(deftest test-cbox-distribute
  (assert (demo-cbox-wrench-calc-all :press-enter-p nil)))

(run-all-tests)
(exit 0)
