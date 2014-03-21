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


(defun foostep->coords (footstep frame-id)
  ;; convert jsk_footstep_msgs/Footstep to euslisp coodinates
  ;; where `frame-id' is the frame-id of `footstep'
  (let ((coords (ros::tf-pose->coords (send footstep :pose))))
    ;; in order to specify :lleg or :rleg, we need to set property list
    ;; of coordinates object, it will be read by robot-interface.
    (send coords :put :l/r
          (if (= (send footstep :leg) jsk_footstep_msgs::Footstep::*LEFT*)
              :lleg
            :rleg))
    coords))
  
(defun execute-cb (server goal)
  ;; coordinate transformation:
  ;;  1. convert all the coordinates to end-coords
  ;;     the retrieved coordinates are written in center-foot.
  ;;  2. convert all the coordinates relative to the coords
  ;;     used in rtm-ros-robot-interface.l (`rtm-ros-coords')
  (let* ((rtm-ros-coords
          (midcoords
           0.5
           (send *ri* :abc-footstep->eus-footstep
                 (send (send *ri* :get-foot-step-param) :rleg_coords))
           (send *ri* :abc-footstep->eus-footstep
                 (send (send *ri* :get-foot-step-param) :lleg_coords))))
         (footstep (send goal :goal :footstep)) ;jsk_footstep_msgs/FootstepArray
         (frame-id (send footstep :header :frame_id)))
    (let ((footstep-coords            ; convert footstep to euslisp coords
           (mapcar #'(lambda (footstep)
                       (footstep->coords footstep frame-id))
                   (send footstep :footsteps))))
      (let ((footstep-coords-w/-offset ;apply offset
             (mapcar #'(lambda (coords)
                         (let ((offset
                                (if (eq (get coords :l/r) :lleg)
                                    *lfoot-end-coords-offset-from-center*
                                  *rfoot-end-coords-offset-from-center*)))
                           (send coords :transform offset)))
                     footstep-coords)))
        (let ((footstep-coords-from-rtm-ros
               (mapcar #'(lambda (coords)
                           (send rtm-ros-coords :transformation coords))
                       footstep-coords-w/-offset)))
          (send *ri* :set-foot-steps footstep-coords-from-rtm-ros)))))
  (let ((result (send server :result)))
    (send server :set-succeeded result)))


  
(defun main()
  (ros::roseus "footstep_controller")
  ;; read params
  ;; offset from link coordinates to end-coords
  (setq *lfoot-end-coords-offset*
        (read-pose-from-rosparam "~lfoot_endcoords_offset"))
  (setq *rfoot-end-coords-offset*
        (read-pose-from-rosparam "~rfoot_endcoords_offset"))
  ;; offset from link coordinates to center of foot
  (setq *lfoot-offset* (read-pose-from-rosparam "~lfoot_offset"))
  (setq *rfoot-offset* (read-pose-from-rosparam "~rfoot_offset"))
  ;; calc offset from center of foot to end-coords
  (setq *lfoot-end-coords-offset-from-center*
        (send *lfoot-offset* :transformation *lfoot-end-coords-offset*))
  (setq *rfoot-end-coords-offset-from-center*
        (send *rfoot-offset* :transformation *rfoot-end-coords-offset*))
  
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
