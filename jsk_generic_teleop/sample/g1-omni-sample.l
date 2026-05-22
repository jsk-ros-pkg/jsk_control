#!/usr/bin/env roseus
(ros::roseus "jsk_generic_g1_omni_teleop")
(require "package://g1eus/g1-interface.l")
(require "package://jsk_generic_teleop/euslisp/jsk-generic-teleop.l")

(g1-init)

(jsk-generic-teleop
 :robot *g1*
 :arms '(:larm :rarm)
 :device-type :omni
 :init-pose :reset-pose
 :revert-if-fail nil
 :check-self-collision nil
 :origin-offsets nil
 :ratio 0.5
 :send-time 200
 :rate 30
 :debug-view t)
