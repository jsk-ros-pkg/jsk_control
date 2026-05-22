#!/usr/bin/env roseus
(ros::roseus "jsk_nextage_touch_teleop")
(require "package://nextage_tutorials/euslisp/nextage-utils.l")
(require "package://jsk_generic_teleop/euslisp/jsk-generic-teleop.l")

(nextage-init)
(jsk-generic-teleop
 :robot *nextage*
 :arms '(:larm :rarm)
 :device-type :omni
 :init-pose nil
 :origin-offsets '((:larm . (:pos #f(400 100 50)
                             :rpy-deg #f(0 0 0)))
                   (:rarm . (:pos #f(400 -100 50)
                             :rpy-deg #f(0 0 0))))
 :ratio 0.5
 :send-time 200
 :rate 30
 :debug-view t)
