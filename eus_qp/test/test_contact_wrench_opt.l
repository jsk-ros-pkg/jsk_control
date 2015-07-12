#!/usr/bin/env roseus

(require :unittest "lib/llib/unittest.l")
(init-unit-test)

(require "package://eus_qp/euslisp/test-contact-wrench-opt.l")
(setup)

(deftest test-samplerobot-distribute
  (assert (demo-robot-contact-wrench-distribution-all :press-enter-p nil)))
(deftest test-cbox-distribute
  (assert (demo-cbox-wrench-calc-all :press-enter-p nil)))

(run-all-tests)
(exit 0)
