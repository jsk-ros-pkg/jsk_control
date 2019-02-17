#!/usr/bin/env roseus

(require :unittest "lib/llib/unittest.l")
(load "../euslisp/sample/sample-samplerobot.l")


(init-unit-test)

(deftest test-sample-robot-sqp-instant-manip-config-task
  (sample-robot-sqp-instant-manip-config-task :fix-obj? nil :optimize-torque? nil)
  (assert (< (norm (send *instant-manip-config-task* :task-value :update? nil)) 10.0))

  (sample-robot-sqp-instant-manip-config-task :fix-obj? t :optimize-torque? nil)
  (assert (< (norm (send *instant-manip-config-task* :task-value :update? nil)) 10.0))

  (sample-robot-sqp-instant-manip-config-task :fix-obj? nil :optimize-torque? t)
  (assert (< (norm (send *instant-manip-config-task* :task-value :update? nil)) 10.0))

  (sample-robot-sqp-instant-manip-config-task :fix-obj? t :optimize-torque? t)
  (assert (< (norm (send *instant-manip-config-task* :task-value :update? nil)) 10.0))
  )

(deftest test-sample-arm-sqp-bspline-config-task
  (sample-arm-sqp-bspline-config-task :graph-filename nil)
  (assert (< (norm (send *bspline-config-task* :task-value :update? nil)) 1.0))
  )


(eval-when
 (load eval)
 (run-all-tests)
 (exit)
 )
