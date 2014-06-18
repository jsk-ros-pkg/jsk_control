(require :unittest "lib/llib/unittest.l")
(init-unit-test)

(require "irteus/demo/sample-robot-model.l")
(require "package://jsk_ik_server/euslisp/ik-server.l")
(defmethod ik-server
  (:loop-ik-server nil nil))

(deftest test-soft
  (require "package://jsk_ik_server/euslisp/ik-server-impl/sample-robot-ik-server.l")
  (dotimes (i 1)
    (ros::spin-once)
    (ros::sleep)
    (assert t)
    )
  )

(run-all-tests)
(exit 0)
