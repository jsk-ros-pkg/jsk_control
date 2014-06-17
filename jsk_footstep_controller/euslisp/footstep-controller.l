#!/usr/bin/env roseus

;; forostep-controller.l
(ros::roseus-add-msgs "jsk_footstep_msgs")
(ros::roseus-add-srvs "jsk_footstep_controller")

(defvar *all-the-results* nil)
;; ((footstep footstep ....) (footstep footstep ....) ...)

(defun footstep->coords (footstep frame-id)
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

(defun hrpsys-init-coords ()
  (let ((param (send *ri* :get-foot-step-param)))
    (midcoords
     0.5
     (send *ri* :abc-footstep->eus-footstep (send param :rleg_coords))
     (send *ri* :abc-footstep->eus-footstep (send param :lleg_coords)))))

(defun execute-cb (server goal)
  ;; coordinate transformation:
  ;; 1. calc transformation from hrpsys-coords to footstep-coords[0].
  ;; 2. apply the inverse transformation to footstep-ocorods[1:]
  ;; 3. convert the footstep respected to hrpsys-mid-coords
  (let* ((footstep (send goal :goal :footstep)) ;jsk_footstep_msgs/FootstepArray
         (frame-id (send footstep :header :frame_id)))
    (let* ((footstep-coords            ; convert footstep to euslisp coords
            (mapcar #'(lambda (footstep)
                        (footstep->coords footstep frame-id))
                    (send footstep :footsteps)))
           (footstep-coords0 (car footstep-coords))
           (footstep-coords-rest (cdr footstep-coords))
           (hrpsys-coords (send *ri* :abc-footstep->eus-footstep
                                (send (send *ri* :get-foot-step-param)
                                      (if (eq (get footstep-coords0 :l/r) :lleg)
                                          :lleg_coords :rleg_coords))))
           (hrpsys-mid-coords (hrpsys-init-coords)))
      (let* ((hrpsys->ros (send hrpsys-coords :transformation footstep-coords0))
             (ros->hrpsys (send hrpsys->ros :inverse-transformation)))
        (let ((transformed-footstep
               (mapcar
                #'(lambda (footstep)
                    (let ((coords (send (send ros->hrpsys :copy-worldcoords)
                                        :transform footstep)))
                      (setf (get coords :l/r) (get footstep :l/r))
                      coords))
                footstep-coords-rest)))
          (let ((final-footstep
                 (mapcar
                  #'(lambda (footstep)
                      (let ((c (send hrpsys-mid-coords
                                     :transformation footstep)))
                        (setf (get c :l/r) (get footstep :l/r))
                        c))
                  transformed-footstep)))
            (push final-footstep *all-the-results*)
            (send *ri* :set-foot-steps final-footstep))))))
    (let ((result (send server :result)))
      (send server :set-succeeded result)))

(defun require-log-service-callback (req)
  (let ((res (send req :response)))
    (let ((*print-object* t) (*print-circle* t)
          (*print-level* nil) (*print-length* nil))
      (send res :sexp (format nil "~A" *all-the-results*))
      res)))

(defun main()
  (ros::roseus "footstep_controller")
  ;; *ri*
  (let ((interface-file
         (ros::get-param "~interface_file"
           "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknt-interface.l"))
        (interface-class
         (ros::get-param "~interface_class" "hrp2jsknt-interface")))
    (load interface-file)
    (setq *ri* (instance (eval (read-from-string interface-class)) :init
                         :controller-timeout nil))
    (send *ri* :start-st)
    )
  ;; interface
  (setq *tf* (instance ros::transform-listener :init))
  (ros::advertise-service "require_log"
    jsk_footstep_controller::RequireLog
    #'require-log-service-callback)
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
