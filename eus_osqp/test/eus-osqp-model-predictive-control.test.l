#!/usr/bin/env roseus

(require :unittest "lib/llib/unittest.l")
(init-unit-test)

(require "package://eus_qp/euslisp/test-model-predictive-control.l")
(require "package://eus_osqp/euslisp/eus-osqp-compiled.l")
(setup)

(deftest test-osqp-demo-mpc-gen-motion-all
  (assert (every #'identity (demo-mpc-gen-motion-all :qp-solver #'solve-osqp-qp))))
(deftest test-osqp-demo-mpc-gen-motion-all-2
  (assert (every #'identity (demo-mpc-gen-motion-all :receding-horizon-proc-count 2 :qp-solver #'solve-osqp-qp))))
(deftest test-osqp-demo-mpc-gen-motion-all-3
  (assert (every #'identity (demo-mpc-gen-motion-all :solve-qp-mode :sqp-with-hotstart :qp-solver #'solve-osqp-qp))))
(deftest test-osqp-demo-mpc-gen-motion-all-4
  (assert (every #'identity (demo-mpc-gen-motion-all :receding-horizon-proc-count 2 :solve-qp-mode :sqp-with-hotstart :qp-solver #'solve-osqp-qp))))

(run-all-tests)
(exit 0)
