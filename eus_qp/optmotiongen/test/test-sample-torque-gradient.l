#!/usr/bin/env roseus

(require :unittest "lib/llib/unittest.l")
(load "../euslisp/sample/sample-torque-gradient.l")


(init-unit-test)

(defun test-sample-torque-gradient-common
    (ret)
  (let* ((contact-torque-error
          (cadr (assoc :contact-torque-error ret)))
         (gravity-torque-error
          (cadr (assoc :gravity-torque-error ret)))
         (contact-torque-jacobian-error
          (cadr (assoc :contact-torque-jacobian-error ret)))
         (gravity-torque-jacobian-error
          (cadr (assoc :gravity-torque-jacobian-error ret)))
         (torque-jacobian-error
          (cadr (assoc :torque-jacobian-error ret)))
         )
    (assert (eps= (norm contact-torque-error) 0.0 1e-3))
    (assert (eps= (norm gravity-torque-error) 0.0 1e-3))
    (assert (eps= (norm contact-torque-jacobian-error) 0.0 1e-3))
    (assert (eps= (norm gravity-torque-jacobian-error) 0.0 1e-3))
    (assert (eps= (norm torque-jacobian-error) 0.0 1e-3))
    )
  )

(deftest test-sample-torque-gradient-sample-arm
  (test-sample-torque-gradient-common
   (sample-torque-gradient-sample-arm))
  )

(deftest test-sample-torque-gradient-sample-robot
  (test-sample-torque-gradient-common
   (sample-torque-gradient-sample-robot))
  )

(deftest test-sample-torque-gradient-sample-robot-with-root-virtual
  (test-sample-torque-gradient-common
   (sample-torque-gradient-sample-robot :root-virtual-mode :6dof))
  )

(deftest test-sample-torque-gradient-sample-robot-with-root-virtual-drive-root-virtual
  (test-sample-torque-gradient-common
   (sample-torque-gradient-sample-robot :root-virtual-mode :6dof :drive-root-virtual? t))
  )


(eval-when
 (load eval)
 (run-all-tests)
 (exit)
 )
