#!/usr/bin/env roseus

(require "package://eus_qp/euslisp/test-cfr-cwc-calculation.l")

(require :unittest "lib/llib/unittest.l")

(init-unit-test)

(deftest test-cfr-cwc-calculation
  (assert (progn (setup) t))
  (assert (demo-calc-cfr-all :press-enter-p nil))
  (assert (compare-calc-scfr-lp-qp-all :debug-view nil))
  ;;(assert (compare-calc-dcfr-all :debug-view nil))
  (assert (compare-calc-dcfr-all :debug-view nil :ref-cog-z 700.0)))

(run-all-tests)
(exit 0)
