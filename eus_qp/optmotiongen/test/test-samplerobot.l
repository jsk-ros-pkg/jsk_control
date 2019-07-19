#!/usr/bin/env roseus

(require :unittest "lib/llib/unittest.l")
(load "../euslisp/sample/sample-samplerobot.l")


(init-unit-test)

(deftest test-sample-robot-sqp-instant-manip-config-task
  (null-output (sample-robot-sqp-instant-manip-config-task :loop-num 5 :fix-obj? nil :optimize-torque? nil))
  (assert (< (norm (send *instant-manip-config-task* :task-value :update? nil)) 10.0))

  (null-output (sample-robot-sqp-instant-manip-config-task :loop-num 5 :fix-obj? t :optimize-torque? nil))
  (assert (< (norm (send *instant-manip-config-task* :task-value :update? nil)) 10.0))

  (null-output (sample-robot-sqp-instant-manip-config-task :loop-num 5 :fix-obj? t :optimize-torque? t))
  (assert (< (norm (send *instant-manip-config-task* :task-value :update? nil)) 10.0))
  )

(deftest test-sample-arm-sqp-bspline-config-task
  (null-output (sample-arm-sqp-bspline-config-task :loop-num 5 :graph-filename nil))
  (assert (< (norm (send *bspline-config-task* :task-value :update? nil)) 1.0))
  )

(deftest test-sample-robot-sqp-bspline-dynamic-config-task
  (null-output (sample-robot-sqp-bspline-dynamic-config-task
                :kin-task-time-divide-num 5
                :eom-task-time-divide-num 5
                :centroid-task-time-divide-num 5
                :posture-task-time-divide-num 5
                :joint-num-control-point 8
                :centroid-num-control-point 8
                :bspline-order 3
                :cog-bspline-order 3
                :loop-num 5
                :graph-filename nil
                ))
  (assert (< (norm (send *bspline-dynamic-config-task* :task-value :update? nil)) 100.0))
  )


(eval-when
 (load eval)
 (run-all-tests)
 (exit)
 )
