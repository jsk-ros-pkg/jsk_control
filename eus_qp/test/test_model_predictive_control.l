#!/usr/bin/env roseus

(require :unittest "lib/llib/unittest.l")
(init-unit-test)

(require "package://eus_qp/euslisp/test-model-predictive-control.l")
(setup)

(deftest test-demo-mpc-gen-motion-all
  (assert (every #'identity (demo-mpc-gen-motion-all))))
(deftest test-demo-mpc-gen-motion-all-2
  (assert (every #'identity (demo-mpc-gen-motion-all :receding-horizon-proc-count 2))))
(deftest test-test-predictive-matrices-all
  (assert (every #'identity (test-predictive-matrices-all))))

(run-all-tests)
(exit 0)
