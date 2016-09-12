;;(ros::roseus-add-msgs "jsk_footstep_msgs")
(ros::roseus-add-msgs "jsk_footstep_controller")
(load "package://hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l")

(ros::roseus "go_pos_client" :anonymous nil)

(defclass footstep-simple-action-client
  :super ros::simple-action-client
  :slots (footstep-feedback)
  )
(defmethod footstep-simple-action-client
  (:action-feedback-cb
   (msg)
   (ros::ros-info "get footstep server feedback")
   (setq footstep-feedback (send msg :feedback))
   )
  (:plan-status
   ()
   (let ((retval nil))
     (when footstep-feedback
       (setq retval (send footstep-feedback :status))
       )
     retval)
   )
  (:remaining-steps
   ()
   (let ((retval nil))
     (when footstep-feedback
       (setq retval (send footstep-feedback :remaining_steps))
       )
     retval)
   )
  )

(defclass go-pos-client
  :super propertied-object
  :slots (client ri
          ;; initial-coords
          ;; finished-goal-id
          ;; target-x target-y target-th
         )
  )

(defmethod go-pos-client
  (:init
   (&key (wait t))
   (setq client
         (instance footstep-simple-action-client :init
                   "go_pos_server" jsk_footstep_controller::GoPosAction))
   (setq ri (instance rtm-ros-robot-interface :define-all-ROSBridge-srv-methods))
   (when wait (send client :wait-for-server))
   self)
  (:wait-client
   (&optional (timeout 0))
   (let ((ret (send client :wait-for-result :timeout timeout)))
     (when (or ret
               (= (client . ros::simple-state) ros::*simple-goal-state-done*))
       (setq ret (send client :get-result)))
     ret))
  (:go-pos
   (x y th &key (overwrite nil) (wait t))
   (let ((walking? (send self :get-remaining-steps-and-index)))
     (cond
      ((and walking? (not overwrite))
       (warn "robot is walking now!~%")
       (return-from :go-pos))
      ((and walking? overwrite)
       (when (< (length (car walking?)) 5) ;; few remaining steps
         (warn "few remaining steps ~A~%" walking?)
         (return-from :go-pos)))
      ((and (not walking?) overwrite)
       (warn "overwritting steps was required, but robot is not stepping!~%")
       (return-from :go-pos))
      )
     (let ((goal (instance jsk_footstep_controller::GoPosActionGoal :init)))
       (send goal :goal :x x)
       (send goal :goal :y y)
       (send goal :goal :theta th)
       (send goal :goal :action
             (if (not walking?)
                 jsk_footstep_controller::GoPosGoal::*NEW_TARGET*
               jsk_footstep_controller::GoPosGoal::*OVER_WRITE*))
       (send client :send-goal goal)
       ))
   (if wait (send self :wait-client) t)
   )
  (:go-pos-no-wait
   (x y th &key (overwrite nil))
   (send self :go-pos x y th :overwrite overwrite :wait nil))
  (:get-remaining-steps-and-index
   ()
   (let ((remaining-steps (send ri :get-remaining-foot-step-sequence-current-index)))
     (if (car remaining-steps)
         remaining-steps nil))
   )
  (:is-walking
   ()
   (send client :spin-once)
   (eq (send client :plan-status) jsk_footstep_controller::GoPosFeedback::*WALKING*)
   )
  (:is-planning
   ()
   (send client :spin-once)
   (eq (send client :plan-status) jsk_footstep_controller::GoPosFeedback::*PLANNING*)
   )
  ;;(:can-overwrite () )
  )

(warn "(setq *gp* (instance go-pos-client :init))~%(send *gp* :go-pos-no-wait 2 0 0)~%(send *gp* :go-pos-no-wait 2 1 0 :overwrite t)~%")
