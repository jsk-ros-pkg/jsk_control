#!/usr/bin/env roseus

;; forostep-controller.l
(ros::roseus-add-msgs "jsk_footstep_msgs")

(defun read-pose-from-rosparam (param-name)
  ;; read a pose parameter from ros param specified by `param-name',
  ;; and that parameter should be a 7 length array:
  ;;   (x, y, z, xx, yy, zz, ww)
  ;; where xx, yy, zz and ww menas orientation of the pose.
  (let ((param (ros::get-param param-name)))
    (let ((coords (make-coords :pos (float-vector
                                     (* 1000.0 (elt param 0))
                                     (* 1000.0 (elt param 1))
                                     (* 1000.0 (elt param 2)))
                               :rot (quaternion2matrix
                                     (float-vector (elt param 6)
                                                   (elt param 3)
                                                   (elt param 4)
                                                   (elt param 5))))))
      coords)))

(defun execute-cb (server goal)
  ;; coordinate transformation:
  ;;  1. convert all the coordinates to end-coords
  ;;     the retrieved coordinates are written in center-foot.
  ;;  2. convert all the coordinates relative to the coords
  ;;     used in rtm-ros-robot-interface.l (`rtm-ros-coords')
  (let ((rtm-ros-coords
         (midcoords 0.5
                    (send *ri* :abc-footstep->eus-footstep (send (send *ri* :get-foot-step-param) :rleg_coords))
                    (send *ri* :abc-footstep->eus-footstep (send (send *ri* :get-foot-step-param) :lleg_coords)))))
    (let* ((footstep (send goal :goal :footstep))
           (frame-id (send footstep :header :frame_id)))
      (let ((footstep-coords
             (mapcar #'(lambda (footstep)      ;no tf transformation
                         (let ((coords (ros::tf-pose->coords
                                         (send footstep :pose))))
                           ;; force to set z=0
                           (let ((pos (send coords :worldpos)))
                             (setf (elt pos 2) 0.0)
                             (send coords :locate pos :world))
                           (send coords :put :l/r
                                 (if (= (send footstep :leg)
                                        jsk_footstep_msgs::Footstep::*LEFT*)
                                     :lleg
                                   :rleg))
                           coords))
                     (send footstep :footsteps))))
        (send *ri* :set-foot-steps footstep-coords))))
  (let ((result (send server :result)))
    (send server :set-succeeded result)))

(defun main()
  (ros::roseus "footstep_controller")
  ;; read params
  (setq *lfoot-end-coords-offset*
        (read-pose-from-rosparam "~lfoot_endcoords_offset"))
  (setq *rfoot-end-coords-offset*
        (read-pose-from-rosparam "~rfoot_endcoords_offset"))
  ;; *ri*
  (let ((interface-file
         (ros::get-param "~interface_file"
           "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknt-interface.l"))
        (interface-class
         (ros::get-param "~interface_class" "hrp2jsknt-interface")))
    (load interface-file)
    (setq *ri* (instance (eval (read-from-string interface-class)) :init))
    (send *ri* :start-st)
    )
  ;; interface
  (setq *tf* (instance ros::transform-listener :init))
  (let ((s (instance ros::simple-action-server :init
                     "footstep_controller"
                     jsk_footstep_msgs::ExecFootstepsAction
                     :execute-cb 'execute-cb)))
    (ros::rate 10)
    (while (ros::ok)
      (send s :worker)
      (ros::spin-once)
      (ros::sleep))))

(main)
