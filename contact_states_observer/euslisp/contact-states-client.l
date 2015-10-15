(ros::load-ros-manifest "contact_states_observer")
(ros::load-ros-manifest "roseus")

(defclass contact-state-client
  :super propertied-object
  :slots (result)
  )

(defmethod contact-state-client
  (:init
   ()
   (ros::create-nodehandle "ore_contact_states")
   (ros::subscribe
    "/ore_contact_states"
    contact_states_observer::OREContactStates
    #'send self :ore-contact-states-cb
    :groupname "ore_contact_states")
   )
  (:ore-contact-states-cb
   (msg)
   (let ((ret))
     (dolist (cs (send msg :contact_states))
       (push (read-from-string (send cs :name)) ret)
       (push (send cs :state) ret))
     (setq result (reverse ret))
     ))
  (:get-contact-states
   (names)
   (ros::spin-once "ore_contact_states")
   (mapcar #'(lambda (l) (cadr (memq l result))) names))
  (:tmp-set-state-param
   (service-name d)
   (if (ros::service-exists service-name)
       (ros::service-call service-name
                          (instance roseus::StringStringRequest :init :str (format nil "~A" d))))
   )
  ;; for grasp
  (:set-grasp-state-param
   (d)
   (send self :tmp-set-state-param "/grasp_state_set_param" d)
   (unix:usleep 100000))
  (:set-obj-env-state-param
   (d)
   (send self :tmp-set-state-param "/obj_env_state_set_param" d)
   (unix:usleep 100000))
  )

