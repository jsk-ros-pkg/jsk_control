#!/usr/bin/env roseus

(require :unittest "lib/llib/unittest.l")
(load "../euslisp/sample/sample-inverse-kinematics-statics.l")


(init-unit-test)

(deftest test-sample-robot-iks
  (assert (null-output (sample-robot-reach-iks-raw :optimize-torque? nil :loop-num 10)))
  (assert (null-output (sample-robot-reach-iks-raw :optimize-torque? t :loop-num 10)))
  (assert (null-output (float-vector-p (sample-robot-reach-iks :optimize-torque? nil :stop 50))))
  (assert (null-output (float-vector-p (sample-robot-reach-iks :optimize-torque? t :stop 50))))
  (assert (null-output (null (sample-robot-reach-iks :optimize-torque? nil :target-coords (make-coords :pos (float-vector 10000 0 0)) :stop 50)))) ;; intentional failure
  (assert (null-output (float-vector-p (sample-robot-reach-iks-face :optimize-torque? nil :stop 50))))
  ;; (assert (null-output (float-vector-p (sample-robot-reach-iks-face :optimize-torque? t :stop 50))))
  )

(deftest test-sample-robot-trajectory-iks
  (assert (null-output (sample-robot-reach-trajectory-iks-raw :optimize-start-end-torque? nil :loop-num 10)))
  (assert (null-output (sample-robot-reach-trajectory-iks-raw :optimize-start-end-torque? t :loop-num 10)))
  (assert (null-output (consp (sample-robot-reach-trajectory-iks :optimize-start-end-torque? nil :stop 50))))
  (assert (null-output (consp (sample-robot-reach-trajectory-iks :optimize-start-end-torque? t :stop 50))))
  (assert (null-output (null (sample-robot-reach-trajectory-iks :optimize-start-end-torque? nil :target-coords (make-coords :pos (float-vector 5000 0 0)) :stop 50)))) ;; intentional failure
  (assert (null-output (consp (sample-robot-reach-trajectory-iks-face :optimize-start-end-torque? nil :stop 50))))
  ;; (assert (null-output (consp (sample-robot-reach-trajectory-iks-face :optimize-start-end-torque? t :stop 100))))
  )


(eval-when
 (load eval)
 (run-all-tests)
 (exit)
 )
