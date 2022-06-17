#!/usr/bin/env roseus

(load "package://jsk_ik_server/euslisp/ik-evaluation.l")

(warning-message 2 "It takes much time! Be patient!~%")

(require "package://hrpsys_ros_bridge_tutorials/euslisp/tablis-interface.l")
(setq *robot* (make-robot-model-from-name "tablis"))
(setq *xrange* '(-700.0 . 900.0))
(setq *yrange* '(-300.0 . 1300.0))
(setq *zrange* '(-400.0 . 1000.0))
(setq *output-directory* (ros::resolve-ros-path "package://jsk_ik_server/data/"))

(generate-ik-grid-for-robot *robot* "tablis-zup"
                            *xrange* *yrange* *zrange*
                            100
                            '(:rotation-axis :z)
                            (unit-matrix)
                            *output-directory*
                            :min-distance 400)
(generate-ik-grid-for-robot *robot* "tablis-yup"
                            *xrange* *yrange* *zrange*
                            100
                            '(:rotation-axis :z)
                            (send (make-coords :rpy (list 0 0 pi/2)) :worldrot)
                            *output-directory*
                            :min-distance 400)
(generate-ik-grid-for-robot *robot* "tablis-xup"
                            *xrange* *yrange* *zrange*
                            100
                            '(:rotation-axis :z)
                            (send (make-coords :rpy (list 0 -pi/2 0)) :worldrot)
                            *output-directory*
                            :min-distance 400)

(sys::exit 0)
