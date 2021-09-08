#!/usr/bin/env roseus

(load "package://hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l")
(ros::roseus "base_controller")

(setq *ri* (instance rtm-ros-robot-interface :define-all-ROSBridge-srv-methods))

(ros::rate 5)
(send *ri* :cmd-vel-mode)
