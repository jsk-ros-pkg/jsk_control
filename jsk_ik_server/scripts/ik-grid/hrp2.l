#!/usr/bin/env roseus

(load "package://jsk_ik_server/euslisp/ik-evaluation.l")

(warning-message 2 "It takes much time! Be patient!~%")

(require "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknt-interface.l")
(setq *robot* (make-robot-model-from-name "hrp2jsknt"))
(setq *xrange* '(0.0 . 1000.0))
(setq *yrange* '(-1000.0 . 2200.0))
(setq *zrange* '(800.0 . 2000.0))
(setq *output-directory* (ros::resolve-ros-path "package://jsk_ik_server/data/"))

(generate-ik-grid-for-robot *robot* "hrp2-zup"
                            *xrange* *yrange* *zrange*
                            100
                            '(:rotation-axis :z)
                            (unit-matrix)
                            *output-directory*)
(generate-ik-grid-for-robot *robot* "hrp2-yup"
                            *xrange* *yrange* *zrange*
                            100
                            '(:rotation-axis :z)
                            (send (make-coords :rpy (list 0 0 pi/2)) :worldrot)
                            *output-directory*)
(generate-ik-grid-for-robot *robot* "hrp2-xup"
                            *xrange* *yrange* *zrange*
                            100
                            '(:rotation-axis :z)
                            (send (make-coords :rpy (list 0 -pi/2 0)) :worldrot)
                            *output-directory*)

(sys::exit 0)
