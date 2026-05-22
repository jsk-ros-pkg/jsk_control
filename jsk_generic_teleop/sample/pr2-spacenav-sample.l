#!/usr/bin/env roseus
(ros::roseus "jsk_generic_pr2_spacenav_teleop")
(require "package://pr2eus/pr2-interface.l")
(require "package://jsk_generic_teleop/euslisp/jsk-generic-teleop.l")

(pr2-init)

(jsk-generic-teleop
 :robot *pr2*
 :arms '(:larm :rarm)
 :device-type :spacenav
 :init-pose t
 :origin-offsets nil
 :active-arm :larm
 :ratio 0.5
 :relative-translation-scale 30.0
 :relative-rotation-scale (/ pi 24.0)
 :relative-deadzone 0.2
 :spacenav-topic "/spacenav/joy"
 :send-time 200
 :rate 30
 :debug-view t)
