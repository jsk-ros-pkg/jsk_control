
#!/usr/bin/env roseus

(ros::load-ros-manifest "jsk_ik_server")

(require "irteus/demo/sample-robot-model.l")
(require "package://jsk_ik_server/euslisp/ik-server.l")

(ros::roseus "sample_robot_ik_server")

(defvar *sample-robot-ik-server* (instance ik-server :init :robot (instance sample-robot :init)))
(send *sample-robot-ik-server* :start-ik-server)
