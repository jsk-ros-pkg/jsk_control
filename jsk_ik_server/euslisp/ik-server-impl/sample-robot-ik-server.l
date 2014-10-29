#!/usr/bin/env roseus

(ros::load-ros-manifest "jsk_ik_server")

(require :samplerobot "package://hrpsys_ros_bridge_tutorials/models/samplerobot.l")
(require "package://jsk_ik_server/euslisp/ik-server.l")

(ros::roseus "sample_robot_ik_server")

(defvar *sample-robot-ik-server* (instance ik-server :init :robot (instance samplerobot-robot :init) :ik-server-name "sample_robot_ik_server"))
(send *sample-robot-ik-server* :start-ik-server)
