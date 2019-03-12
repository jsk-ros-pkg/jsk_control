#!/usr/bin/env roseus

(require :unittest "lib/llib/unittest.l")
(load "../euslisp/sample/sample-torque-gradient.l")


(init-unit-test)

(deftest test-sample-torque-gradient
  (let* ((ret (sample-torque-gradient))
         (contact-torque-jacobian-error (elt ret 0))
         (gravity-torque-jacobian-error (elt ret 1))
         (torque-jacobian-error (elt ret 2))
         )
    (assert (eps= (norm contact-torque-jacobian-error) 0.0 1e-3))
    (assert (eps= (norm gravity-torque-jacobian-error) 0.0 1e-3))
    (assert (eps= (norm torque-jacobian-error) 0.0 1e-3))
    )
  )


(eval-when
 (load eval)
 (run-all-tests)
 (exit)
 )
