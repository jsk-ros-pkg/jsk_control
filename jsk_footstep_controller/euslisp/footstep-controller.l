#!/usr/bin/env roseus

;; forostep-controller.l
(ros::roseus-add-msgs "jsk_footstep_msgs")
(ros::roseus-add-srvs "jsk_footstep_controller")

(defvar *all-the-results* nil)
(setq *footstep-list* nil)
(setq *goal-id* nil)
(setq *footstep-list-bak* nil)
(setq *goal-id-bak* nil)
(setq *interruptible-mode* nil)

;; ((footstep footstep ....) (footstep footstep ....) ...)

(defun footstep->coords (footstep frame-id)
  ;; convert jsk_footstep_msgs/Footstep to euslisp coodinates
  ;; where `frame-id' is the frame-id of `footstep'
  (let ((coords (ros::tf-pose->coords (send footstep :pose))))
    ;; in order to specify :lleg or :rleg, we need to set property list
    ;; of coordinates object, it will be read by robot-interface.
    (send coords :name
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

(defun new-goal-cb (server goal)
  ;; coordinate transformation:
  ;; 1. calc transformation from hrpsys-coords to footstep-coords[0].
  ;; 2. apply the inverse transformation to footstep-ocorods[1:]
  ;; 3. convert the footstep respected to hrpsys-mid-coords
  (send *hrp2jsknts* :reset-manip-pose)
  (send *hrp2jsknts* :legs :move-end-pos (float-vector 0 0 50))
  (send *ri* :angle-vector (send *hrp2jsknts* :angle-vector) 5000)
  (send *ri* :wait-interpolation)
  (cond ((= (send goal :goal :strategy) jsk_footstep_msgs::ExecFootstepsGoal::*RESUME*)
         (progn
           (print 'resuming)
           (setq *footstep-list* *footstep-list-bak*)
           (setq *footstep-list-bak* nil)
           (setq *goal-id* (send goal :goal_id :id))))
        ((= (send goal :goal :strategy) jsk_footstep_msgs::ExecFootstepsGoal::*NEW_TARGET*)
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
                                             (if (eq (send footstep-coords0 :name) :lleg)
                                                 :lleg_coords :rleg_coords))))
                  (hrpsys-mid-coords (hrpsys-init-coords)))
             (let* ((hrpsys->ros (send hrpsys-coords :transformation footstep-coords0))
                    (ros->hrpsys (send hrpsys->ros :inverse-transformation)))
               (let ((transformed-footstep
                      (mapcar
                       #'(lambda (footstep)
                           (let ((coords (send (send ros->hrpsys :copy-worldcoords)
                                               :transform footstep)))
                             (send coords :name (send footstep :name))
                             coords))
                       footstep-coords-rest)))
                 (let ((final-footstep
                        (mapcar
                         #'(lambda (footstep)
                             (let ((c (send hrpsys-mid-coords
                                            :transformation footstep)))
                               (send c :name (send footstep :name))
                               c))
                         transformed-footstep)))
                   (push final-footstep *all-the-results*)
                   (setq *footstep-list* final-footstep)
                   (setq *goal-id* (send goal :goal_id :id))
                   ))))))))

(defun proc-goal-cb (server goal)
  (if *interruptible-mode*
      (progn
        (if (<= (length *footstep-list*) 1) ;finish
            (progn
              (send *hrp2jsknts* :reset-manip-pose)
              (send *ri* :angle-vector (send *hrp2jsknts* :angle-vector) 5000)
              (send *ri* :wait-interpolation)
              (let ((result (send server :result)))
                (send server :set-succeeded result)))
          (let ((first-step (pop *footstep-list*))
                (second-step (car *footstep-list*)))
            (send *ri* :set-foot-steps (list first-step second-step))
            )))
    (progn
      (print (send-all *footstep-list* :name))
      (send *ri* :set-foot-steps *footstep-list*)
      (setq *footstep-list* nil)
      (send *hrp2jsknts* :reset-manip-pose)
      (send *ri* :angle-vector (send *hrp2jsknts* :angle-vector) 5000)
      (send *ri* :wait-interpolation)
      (send server :set-succeeded (send server :result))
      )
    ))

(defun preempt-cb (server goal)
  (print 'preempt-cb)
  (setq *footstep-list-bak* *footstep-list*)
  (setq *footstep-list* nil)
  (send server :set-preempted)
  (setq *goal-id-bak* *goal-id*)
  (setq *goal-id* nil)
  )

(defun execute-cb (server goal)
  (print 'execute-cb)
  (if (or (null *goal-id*)
          (not (string= *goal-id* (send goal :goal_id :id))))
      (new-goal-cb server goal)
    (proc-goal-cb server goal)))

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
    (setq *interruptible-mode* (ros::get-param "~interruptible_mode" nil))
    (if *interruptible-mode*
        (ros::ros-info "footstep controller running in interruptible mode"))
    (load interface-file)
    (setq *ri* (instance (eval (read-from-string interface-class)) :init
                         :controller-timeout nil))
    ;; 
    (hrp2jsknts-init)
    (send *ri* :start-st)
    (send *ri* :start-auto-balancer)
    )
  ;; interface
  (setq *tf* (instance ros::transform-listener :init))
  (ros::advertise-service "require_log"
    jsk_footstep_controller::RequireLog
    #'require-log-service-callback)
  (let ((s (instance ros::simple-action-server :init
                     "footstep_controller"
                     jsk_footstep_msgs::ExecFootstepsAction
                     :execute-cb 'execute-cb
                     :preempt-cb 'preempt-cb)))
    (ros::rate 10)
    (while (ros::ok)
      (send s :worker)
      (ros::spin-once)
      (ros::sleep))))

(main)


#|
(setq aa '((#J(coordinates ((:name . :rleg)) #J(array nil #f(0.99987 0.002702 0.01588 -0.002847 0.999954 0.009123 -0.015855 -0.009167 0.999832) 2 nil 0 3 3 nil nil nil nil nil) #f(-2.5818 -134.762 -11.945)) #J(coordinates ((:name . :lleg)) #J(array nil #f(0.999716 0.000819 0.02382 -0.000794 0.999999 -0.001046 -0.023821 0.001027 0.999716) 2 nil 0 3 3 nil nil nil nil nil) #f(94.3242 52.2552 -16.8706)) #J(coordinates ((:name . :rleg)) #J(array nil #f(0.999716 0.000819 0.02382 -0.000794 0.999999 -0.001046 -0.023821 0.001027 0.999716) 2 nil 0 3 3 nil nil nil nil nil) #f(94.1522 -157.745 -17.0863)))))
(dotimes (i (1- (length footsteps)))
  (print (rad2deg (acos (v. (normalize-vector (send (elt footsteps i) :z-axis)) (normalize-vector (send (elt footsteps (1+ i)) :z-axis)))))
  )
|#
