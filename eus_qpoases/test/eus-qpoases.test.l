#!/usr/bin/env roseus

(require :unittest "lib/llib/unittest.l")
(init-unit-test)

(require "package://eus_qpoases/euslisp/test-eus-qpoases.l")

(deftest test-soft
  (if (and (demo-eus-qpOASES1)
	   (demo-eus-qpOASES2)
	   (demo-eus-qpOASES3)
           (demo-eus-qpOASES4)
           (demo-eus-qpOASES5)
           (demo-eus-qpOASES6)
           (demo-eus-qpOASES7))
      (assert t)
    (throw :eus_qpoases_fail nil)))

(run-all-tests)
(exit 0)
