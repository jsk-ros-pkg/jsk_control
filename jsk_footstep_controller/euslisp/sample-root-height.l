#!/usr/bin/env roseus

(load "package://jsk_footstep_controller/euslisp/root-height.l")
(load "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknt-interface.l")
(setq *robot* (instance hrp2jsknt-robot :init))
(objects (list *robot*))
(test-root-height-init)
(while (ros::ok)
  (test-root-height-all nil))

