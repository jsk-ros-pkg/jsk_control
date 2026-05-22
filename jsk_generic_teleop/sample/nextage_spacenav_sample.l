#!/usr/bin/env roseus
(ros::roseus "jsk_generic_nextage_spacenav_teleop")
(require "package://nextage_tutorials/euslisp/nextage-interface.l")
(require "package://jsk_generic_teleop/euslisp/jsk-generic-teleop.l")

(nextage-init)

(jsk-generic-teleop
 :robot *nextage*
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
