
#!/usr/bin/env roseus

(ros::load-ros-manifest "jsk_ik_server")

;;(require "package://hrpsys_ros_bridge_tutorials/euslisp/staro-interface.l")
(require :staro "package://hrpsys_ros_bridge_tutorials/models/staro.l")
(when (probe-file (ros::resolve-ros-path "package://hrpsys_ros_bridge_tutorials/euslisp/staro-utils.l"))
  (require :staro-utils "package://hrpsys_ros_bridge_tutorials/euslisp/staro-utils.l"))
(require "package://jsk_ik_server/euslisp/ik-server.l")

(ros::roseus "staro_ik_server")

;; (defclass staro-ik-server
;;   :super ik-server
;;   )

;; (defmethod staro-ik-server
;;   (:init
;;    (&rest args)
;;    (setq robot (staro))
;;    (send-super* :init
;; 		:robot robot
;; 		:ik-server-name "staro_ik_server"
;; 		:ik-server-service-name "/solve_ik"
;; 		:root-link-frame-id nil
;; 		args)
;;    (send self :update-support-links '(:rleg :lleg))
;;    (send self :make-foot-convex :force? t)
;;    (send self :make-centroid-object)
;;    )
;;   )

(defvar *staro-ik-server* (instance ik-server :init :robot (staro)))
(send *staro-ik-server* :start-ik-server)
