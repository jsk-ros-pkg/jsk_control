#!/usr/bin/env roseus

(require :unittest "lib/llib/unittest.l")


(init-unit-test)

(deftest test-load-euslisp-files
  (assert (= (unix:system (ros::resolve-ros-path "package://eus_qp/optmotiongen/test/test-load-euslisp-files.bash")) 0))
  )


(eval-when
 (load eval)
 (run-all-tests)
 (exit)
 )
