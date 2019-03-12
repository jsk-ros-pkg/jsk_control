(load "../torque-gradient.l")
(load "../validate-jacobian.l")
(require "irteus/demo/sample-arm-model.l")
(require "irteus/demo/sample-robot-model.l")


(defclass torque-validate-config-task
  :super propertied-object
  :slots (_robot
          _joint-list
          _drive-joint-list
          _wrench-list
          _contact-target-coords-list
          _contact-attention-coords-list
          _gravity-link-list
          )
  )

(defmethod torque-validate-config-task
  (:init
   (&key
    (robot)
    (joint-list)
    (drive-joint-list)
    (wrench-list)
    (contact-target-coords-list)
    (contact-attention-coords-list)
    (gravity-link-list (send robot :links))
    )
   (setq _robot robot)
   (setq _joint-list joint-list)
   (setq _drive-joint-list drive-joint-list)
   (setq _wrench-list wrench-list)
   (setq _contact-target-coords-list contact-target-coords-list)
   (setq _contact-attention-coords-list contact-attention-coords-list)
   (setq _gravity-link-list gravity-link-list)
   )
  ;; config
  (:theta
   ()
   (mapcar
    #'(lambda (j) (send j :angle-to-speed (send j :joint-angle ))) ;; [m] [rad]
    _joint-list)
   )
  (:set-theta
   (theta)
   (map
    nil
    #'(lambda (j a) (send j :joint-angle (send j :speed-to-angle a))) ;; [mm] [deg]
    _joint-list
    theta)
   )
  ;; torque
  (:get-torque
   ()
   (v+ (send self :get-contact-torque)
       (send self :get-gravity-torque))
   )
  (:get-torque-jacobian
   ()
   (m+ (send self :get-contact-torque-jacobian)
       (send self :get-gravity-torque-jacobian))
   )
  ;; contact torque
  (:get-contact-torque
   ()
   (get-contact-torque
    :robot _robot
    :drive-joint-list _drive-joint-list
    :wrench-list _wrench-list
    :contact-target-coords-list _contact-target-coords-list
    :contact-attention-coords-list _contact-attention-coords-list
    ))
  (:get-contact-torque-jacobian
   ()
   (get-contact-torque-jacobian
    :robot _robot
    :joint-list _joint-list
    :drive-joint-list _drive-joint-list
    :wrench-list _wrench-list
    :contact-target-coords-list _contact-target-coords-list
    :contact-attention-coords-list _contact-attention-coords-list
    ))
  ;; gravity torque
  (:get-gravity-torque
   ()
   (get-gravity-torque
    :robot _robot
    :drive-joint-list _drive-joint-list
    :gravity-link-list _gravity-link-list
    ))
  (:get-gravity-torque-jacobian
   ()
   (get-gravity-torque-jacobian
    :robot _robot
    :joint-list _joint-list
    :drive-joint-list _drive-joint-list
    :gravity-link-list _gravity-link-list
    ))
  )


(defun sample-torque-gradient
    ()
  (setq *robot* (instance sarmclass :init))
  (setq *config-task*
        (instance torque-validate-config-task :init
                  :robot *robot*
                  :joint-list (butlast (send *robot* :joint-list) 2)
                  :drive-joint-list (butlast (send *robot* :joint-list) 2)
                  :wrench-list (list (float-vector 10 -15 20 1 2 -3))
                  :contact-target-coords-list (list (send (send (send *robot* :end-coords) :copy-worldcoords) :translate #f(100 0 0)))
                  :contact-attention-coords-list (list (send *robot* :end-coords))
                  ))

  (setq *contact-torque-jacobian-error*
        (calc-jacobian-error-with-diff
         :config-task *config-task*
         :config-method :theta
         :set-config-method :set-theta
         :task-value-method :get-contact-torque
         :jacobian-method :get-contact-torque-jacobian
         :delta 1e-3
         ))
  (warning-message 2 "contact-torque-jacobian-error: ~a~%"
                   (norm *contact-torque-jacobian-error*))

  (setq *gravity-torque-jacobian-error*
        (calc-jacobian-error-with-diff
         :config-task *config-task*
         :config-method :theta
         :set-config-method :set-theta
         :task-value-method :get-gravity-torque
         :jacobian-method :get-gravity-torque-jacobian
         :delta 1e-3
         ))
  (warning-message 2 "gravity-torque-jacobian-error: ~a~%"
                   (norm *gravity-torque-jacobian-error*))

  (setq *torque-jacobian-error*
        (calc-jacobian-error-with-diff
         :config-task *config-task*
         :config-method :theta
         :set-config-method :set-theta
         :task-value-method :get-torque
         :jacobian-method :get-torque-jacobian
         :delta 1e-3
         ))
  (warning-message 2 "torque-jacobian-error: ~a~%"
                   (norm *torque-jacobian-error*))
  (norm *torque-jacobian-error*)

  (list *contact-torque-jacobian-error*
        *gravity-torque-jacobian-error*
        *torque-jacobian-error*)
  )
