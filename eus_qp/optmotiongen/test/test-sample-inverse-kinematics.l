#!/usr/bin/env roseus

(require :unittest "lib/llib/unittest.l")
(load "../euslisp/sample/sample-inverse-kinematics.l")


(init-unit-test)

(deftest test-sample-arm-ik
  (assert (null-output (sample-arm-reach-ik-raw)))
  (assert (null-output (sample-arm-reach-ik-raw-msc)))
  (assert (null-output (sample-arm-reach-ik-discrete-raw)))
  (assert (null-output (float-vector-p (sample-arm-reach-ik :irteus-ik? nil))))
  (assert (null-output (float-vector-p (sample-arm-reach-ik :irteus-ik? t))))
  (assert (null-output (null (sample-arm-reach-ik :target-coords (make-coords :pos (float-vector 10000 0 0)) :irteus-ik? nil)))) ;; intentional failure
  (assert (null-output (null (sample-arm-reach-ik :target-coords (make-coords :pos (float-vector 10000 0 0)) :irteus-ik? t)))) ;; intentional failure
  (assert (null-output (float-vector-p (sample-arm-reach-ik-obstacle :irteus-ik? nil))))
  (assert (null-output (progn (sample-arm-reach-ik-obstacle :irteus-ik? t) t))) ;; irteus ik fails
  (assert (null-output (float-vector-p (sample-arm-reach-ik-with-root-virtual-joint))))
  (assert (null-output (float-vector-p (sample-arm-reach-ik-face))))
  (assert (null-output (float-vector-p (sample-arm-reach-ik-line))))
  )

(deftest test-sample-robot-ik
  (assert (null-output (sample-robot-reach-ik-raw)))
  (assert (null-output (sample-robot-reach-ik-raw :set-posture-joint? t :look-at? t)))
  (assert (null-output (float-vector-p (sample-robot-reach-ik :irteus-ik? nil))))
  (assert (null-output (float-vector-p (sample-robot-reach-ik :irteus-ik? t))))
  (assert (null-output (float-vector-p (sample-robot-reach-ik-limb :irteus-ik? nil))))
  (assert (null-output (float-vector-p (sample-robot-reach-ik-limb :irteus-ik? t))))
  (assert (null-output (float-vector-p (sample-robot-reach-ik-dual-arm :irteus-ik? nil))))
  (assert (null-output (float-vector-p (sample-robot-reach-ik-dual-arm :irteus-ik? t))))
  (assert (null-output (float-vector-p (sample-robot-reach-ik-dual-arm-with-torso :irteus-ik? nil))))
  (assert (null-output (float-vector-p (sample-robot-reach-ik-dual-arm-with-torso :irteus-ik? t))))
  (assert (null-output (float-vector-p (sample-robot-reach-ik-fullbody :irteus-ik? nil))))
  (assert (null-output (float-vector-p (sample-robot-reach-ik-fullbody :irteus-ik? t))))
  )

(deftest test-sample-arm-trajectory-ik
  (assert (null-output (sample-arm-reach-trajectory-ik-raw)))
  (assert (null-output (consp (sample-arm-reach-trajectory-ik))))
  (assert (null-output (consp (sample-arm-reach-trajectory-ik-with-root-virtual-joint))))
  (assert (null-output (null (sample-arm-reach-trajectory-ik-with-root-virtual-joint
                              :target-coords (make-coords :pos (float-vector 10000 0 0)))))) ;; intentional failure
  (assert (null-output (null (sample-arm-reach-trajectory-ik-with-root-virtual-joint
                              :root-virtual-joint-min-angle-list (list 0 0 0) :root-virtual-joint-max-angle-list (list 0 0 0))))) ;; intentional failure
  (assert (null-output (consp (sample-arm-reach-trajectory-ik-with-root-virtual-joint
                               :rotation-axis-list (list nil nil nil)
                               :root-virtual-joint-min-angle-list (list -1e10 0 0) :root-virtual-joint-max-angle-list (list 1e10 0 0)))))
  (assert (null-output (consp (sample-arm-reach-trajectory-ik-with-root-virtual-joint-obstacle))))
  )

(deftest test-sample-robot-trajectory-ik
  (assert (null-output (consp (sample-robot-reach-trajectory-ik-dual-arm-with-torso))))
  (assert (null-output (consp (sample-robot-reach-trajectory-ik-dual-arm-with-torso-no-mid-constraint))))
  (assert (null-output (consp (sample-robot-reach-trajectory-ik-fullbody))))
  )


(eval-when
 (load eval)
 (run-all-tests)
 (exit)
 )
