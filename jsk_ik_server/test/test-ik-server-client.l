(require :unittest "lib/llib/unittest.l")
(init-unit-test)


(require "package://jsk_ik_server/test/fullbody-ik-client-test.l")
(require "irteus/demo/sample-robot-model.l")

(defmethod fullbody-ik-client
  (:solvable-ik-server-call
   (&rest
    args
    &key
    (robot *robot*)
    (target-limbs '(:rarm :larm :rleg :lleg))
    (move-target (mapcar #'(lambda (k) (send robot k :end-coords)) target-limbs))
    (target-coords (send-all move-target :copy-worldcoords))
    &allow-other-keys)
   (apply
    #'send self :ik-server-call
    (append
     (list
      :robot robot
      :target-limbs target-limbs
      :move-target move-target
      :target-coords target-coords
      :root-link-virtual-joint-weight #F(1 1 1 1 1 1)
      ;;:fix-limbs '(:rleg :lleg)
      ;;:centroid-thre 100
      :stop 50
      :use-toes '(nil nil)
      )
     args))))

(deftest test-soft
  (let* ((ik-client-test (instance fullbody-ik-client :init))
	 (robot (instance sample-robot :init))
	 ret
	 )
    (send robot :fix-leg-to-coords (make-coords))
    (ros::roseus "fullbody_ik_client_test")
    (objects (list robot))
    (send robot :reset-pose)
    (send robot :fix-leg-to-coords (make-coords))
    (setq
     ret
     (send ik-client-test :solvable-ik-server-call :robot robot))
    (assert
     (eq (send ret :error_code :val)
	 moveit_msgs::MoveItErrorCodes::*SUCCESS*))
    (dotimes (i 1)
      (send robot :reset-pose)
      (send robot :fix-leg-to-coords (make-coords))
      (setq ret
	    (send ik-client-test :test-fullbody-ik :robot robot))
      (assert t)
      )
    ))

(run-all-tests)
(exit)
