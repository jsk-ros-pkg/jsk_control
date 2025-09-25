#!/usr/bin/env roseus

(require :unittest "lib/llib/unittest.l")
(init-unit-test)

(require "package://eus_qp/euslisp/test-contact-wrench-opt.l")
(require "package://eus_osqp/euslisp/eus-osqp-compiled.l")
(setup)

(deftest test-osqp-samplerobot-distribute
  (assert (demo-robot-contact-wrench-distribution-all :press-enter-p nil :qp-solver #'solve-osqp-qp :solve-qp-mode :sqp-with-hotstart)))
(deftest test-osqp-samplerobot-distribute-use-equality-error-weight
  (assert (demo-robot-contact-wrench-distribution-all :press-enter-p nil :force-use-equality-error-weight t :qp-solver #'solve-osqp-qp :solve-qp-mode :sqp-with-hotstart)))
(deftest test-osqp-samplerobot-distribute-with-given-wrench
  (assert (demo-robot-wrench-distribute-from-total-wrench-and-given-wrench :qp-solver #'solve-osqp-qp :solve-qp-mode :sqp-with-hotstart)))
(deftest test-osqp-cbox-distribute
  (assert (demo-cbox-wrench-calc-all :press-enter-p nil :qp-solver #'solve-osqp-qp :solve-qp-mode :sqp-with-hotstart)))

(run-all-tests)
(exit 0)
