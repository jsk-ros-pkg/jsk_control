#!/usr/bin/env roseus

(ros::load-ros-manifest "jsk_ik_server")

;;(require "package://pr2eus/pr2-interface.l")
(require :pr2 "package://pr2eus/pr2.l")
(require :pr2-utils "package://pr2eus/pr2-utils.l")

(require :ik-server "package://jsk_ik_server/euslisp/ik-server.l")

(ros::roseus "pr2_ik_server")

(defvar *pr2-ik-server* (instance ik-server :init :robot (pr2) :root-link-frame-id "/base_link"))
(send *pr2-ik-server* :start-ik-server)


