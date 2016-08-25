#!/usr/bin/env roseus

(require :unittest "lib/llib/unittest.l")
(init-unit-test)

(require "package://eus_qp/euslisp/test-contact-wrench-opt.l")
(setup)

(deftest test-samplerobot-distribute
  (assert (demo-robot-contact-wrench-distribution-all :press-enter-p nil)))
(deftest test-samplerobot-distribute-use-equality-error-weight
  (assert (demo-robot-contact-wrench-distribution-all :press-enter-p nil :force-use-equality-error-weight t)))
(deftest test-samplerobot-distribute-with-given-wrench
  (assert (demo-robot-wrench-distribute-from-total-wrench-and-given-wrench)))
(deftest test-cbox-distribute
  (assert (demo-cbox-wrench-calc-all :press-enter-p nil)))
(deftest test-cop-polygon-friction-polycone-constraint
  (assert
   (and (test-cop-polygon-constraint) (test-friction-polycone-constraint))))

(run-all-tests)
(exit 0)
