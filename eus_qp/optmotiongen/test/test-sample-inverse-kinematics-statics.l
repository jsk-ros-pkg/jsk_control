#!/usr/bin/env roseus

(require :unittest "lib/llib/unittest.l")
(load "../euslisp/sample/sample-inverse-kinematics-statics.l")


(init-unit-test)

(deftest test-sample-robot-iks
  (assert (null-output (sample-robot-reach-iks-raw :optimize-torque? nil)))
  (assert (null-output (sample-robot-reach-iks-raw :optimize-torque? t)))
  (assert (null-output (float-vector-p (sample-robot-reach-iks :optimize-torque? nil))))
  (assert (null-output (float-vector-p (sample-robot-reach-iks :optimize-torque? t))))
  (assert (null-output (null (sample-robot-reach-iks :optimize-torque? nil :target-coords (make-coords :pos (float-vector 10000 0 0)))))) ;; intentional failure
  (assert (null-output (float-vector-p (sample-robot-reach-iks-face :optimize-torque? nil))))
  (assert (null-output (float-vector-p (sample-robot-reach-iks-face :optimize-torque? t))))
  )

(deftest test-sample-robot-trajectory-iks
  (assert (null-output (sample-robot-reach-trajectory-iks-raw :optimize-start-end-torque? nil)))
  (assert (null-output (sample-robot-reach-trajectory-iks-raw :optimize-start-end-torque? t)))
  (assert (null-output (consp (sample-robot-reach-trajectory-iks :optimize-start-end-torque? nil))))
  (assert (null-output (consp (sample-robot-reach-trajectory-iks :optimize-start-end-torque? t))))
  (assert (null-output (null (sample-robot-reach-trajectory-iks :optimize-start-end-torque? nil :target-coords (make-coords :pos (float-vector 2000 0 0)))))) ;; intentional failure
  (assert (null-output (consp (sample-robot-reach-trajectory-iks-face :optimize-start-end-torque? nil))))
  (assert (null-output (consp (sample-robot-reach-trajectory-iks-face :optimize-start-end-torque? t))))
  )


(eval-when
 (load eval)
 (run-all-tests)
 (exit)
 )
