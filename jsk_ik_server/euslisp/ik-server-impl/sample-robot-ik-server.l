
#!/usr/bin/env roseus

(ros::load-ros-manifest "jsk_ik_server")

(require "irteus/demo/sample-robot-model.l")
(require "package://jsk_ik_server/euslisp/ik-server.l")

(ros::roseus "sample_robot_ik_server")

(defclass sample-robot-ik-server
  :super ik-server
  )

(defmethod sample-robot-ik-server
  (:init
   (&rest args)
   (setq robot (instance sample-robot :init))
   (send robot :fix-leg-to-coords (make-coords))
   (send-super* :init
		:robot robot
		:ik-server-name "sample_robot_ik_server"
		:ik-server-service-name "/solve_ik"
		:root-link-frame-id nil
		args)
   (send self :update-support-links '(:rleg :lleg))
   (send self :make-foot-convex :force? t)
   (send self :make-centroid-object)
   )
  )

(defvar *sample-robot-ik-server* (instance sample-robot-ik-server :init))
(send *sample-robot-ik-server* :start-ik-server)
