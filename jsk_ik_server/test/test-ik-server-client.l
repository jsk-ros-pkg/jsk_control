#!/usr/bin/env roseus

(require :unittest "lib/llib/unittest.l")
(init-unit-test)


(require "package://jsk_ik_server/test/fullbody-ik-client-test.l")
(require :samplerobot "package://hrpsys_ros_bridge_tutorials/models/samplerobot.l")

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
      :fix-limbs '(:rleg :lleg)
      ;;:centroid-thre 100
      :stop 50
      :use-toes '(nil nil)
      )
     args))))

(defun wait-for-ik-server
  (&key in str ret (max 10) (cnt max) (warn-id 3))
  (warning-message warn-id " ... wait for ik-server~%")
  (while (null ret)
    (setq in (piped-fork "rosnode list"))
    (warning-message warn-id "[dump node] ~A/~A > 0~%" cnt max)
    (while (setq str (read-line in nil))
      (if (and (substringp "ik_server" str)
	       (not (substringp "client" str)))
	  (setq ret str))
      (warning-message warn-id " : ~A~%" str))
    (close in)
    (if (not (plusp (decf cnt))) (return-from nil nil))
    (unix:sleep 1)))

(deftest test-soft
  (let* ((ik-client-test (instance fullbody-ik-client :init))
	 (robot (instance samplerobot-robot :init))
	 ret
	 )
    (send robot :fix-leg-to-coords (make-coords))
    (ros::roseus "fullbody_ik_client_test")
    (wait-for-ik-server)
    (unless (or (null x::*display*) (= x::*display* 0)) (objects (list robot)))
    (send robot :reset-pose)
    (send robot :fix-leg-to-coords (make-coords))
    (setq
     ret
     (send ik-client-test :solvable-ik-server-call :robot robot))
    (cond
     ((eq (send ret :error_code :val)
	  moveit_msgs::MoveItErrorCodes::*SUCCESS*)
      (assert t))
     (t
      (wait-for-ik-server :max 1 :warn-id 1)
      (throw :ik-failed nil)))
    ;; (dotimes (i 1)
    ;;   (send robot :reset-pose)
    ;;   (send robot :fix-leg-to-coords (make-coords))
    ;;   (setq ret
    ;; 	    (send ik-client-test :test-fullbody-ik :robot robot))
    ;;   (assert t)
    ;;   )
    ))

(run-all-tests)
(exit 0)
