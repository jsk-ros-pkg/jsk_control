#!/usr/bin/env roseus

(load "package://jsk_ik_server/euslisp/ik-evaluation.l")

(warning-message 2 "It takes much time! Be patient!~%")

(require "package://fetcheus/fetch-interface.l")
(setq *robot* (make-robot-model-from-name "fetch"))
(setq *xrange* '(0.0 . 1000.0))
(setq *yrange* '(-2000.0 . 2000.0))
(setq *zrange* '(0.0 . 2000.0))
(setq *output-directory* (ros::resolve-ros-path "package://jsk_ik_server/data/"))

(generate-ik-grid-for-robot *robot* "fetch-zup"
                            *xrange* *yrange* *zrange*
                            100
                            '(:rotation-axis :z :use-torso t)
                            (unit-matrix)
                            *output-directory*
                            :arm :rarm)
(generate-ik-grid-for-robot *robot* "fetch-yup"
                            *xrange* *yrange* *zrange*
                            100
                            '(:rotation-axis :z :use-torso t)
                            (send (make-coords :rpy (list 0 0 pi/2)) :worldrot)
                            *output-directory*
                            :arm :rarm)
(generate-ik-grid-for-robot *robot* "fetch-xup"
                            *xrange* *yrange* *zrange*
                            100
                            '(:rotation-axis :z :use-torso t)
                            (send (make-coords :rpy (list 0 -pi/2 0)) :worldrot)
                            *output-directory*
                            :arm :rarm)

(sys::exit 0)
