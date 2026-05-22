#!/usr/bin/env roseus
(ros::roseus "jsk_generic_ps4_teleop")
(require "package://jsk_generic_teleop/euslisp/jsk-generic-teleop.l")
(require "package://g1eus/g1-interface.l")

(g1-init)
(jsk-generic-teleop
 :robot *g1*
 :arms '(:larm :rarm)
 :device-type :joy
 :joy-topic "/joy_rarm"
 :init-pose :reset-pose
 :active-arm :rarm
 :ratio 0.5
 :relative-translation-scale 30.0
 :relative-rotation-scale (/ pi 24.0)
 :relative-deadzone 0.15
 :send-time 200
 :rate 30
 :debug-view t
 )
