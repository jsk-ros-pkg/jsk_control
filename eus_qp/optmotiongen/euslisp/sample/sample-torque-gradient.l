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


(defun sample-torque-gradient-common
    (&key
     (robot *robot*)
     (joint-list (send robot :joint-list))
     (drive-joint-list (send robot :joint-list))
     (wrench-list)
     (contact-target-coords-list)
     (contact-attention-coords-list)
     )
  (let* ((target-coords-list-for-irtdyna
          (mapcar
           #'(lambda (trg-coords att-coords)
               (make-cascoords :parent (send att-coords :parent)
                               :coords (send trg-coords :copy-worldcoords))
               )
           contact-target-coords-list
           contact-attention-coords-list
           ))
         )
    (setq *contact-torque-error*
          (v-
           (get-contact-torque
            :robot robot
            :drive-joint-list (send robot :joint-list) ;; use all joints for irtdyna
            :wrench-list wrench-list
            :contact-target-coords-list
            contact-target-coords-list
            :contact-attention-coords-list
            contact-attention-coords-list
            )
           (send robot :calc-torque-from-ext-wrenches
                 :force-list (mapcar #'(lambda (w) (subseq w 0 3)) wrench-list)
                 :moment-list (mapcar #'(lambda (w) (subseq w 3 6)) wrench-list)
                 :target-coords target-coords-list-for-irtdyna
                 )
           ))
    (dolist (trg-coords target-coords-list-for-irtdyna)
      (send (send trg-coords :parent) :dissoc trg-coords))
    )
  (warning-message 2 "contact-torque-error: ~a~%" (norm *contact-torque-error*))

  (setq *gravity-torque-error*
        (v-
         (get-gravity-torque
          :robot robot
          :drive-joint-list (send robot :joint-list) ;; use all joints for irtdyna
          :gravity-link-list (send robot :links)
          )
         (send robot :calc-torque)
         ))
  (warning-message 2 "gravity-torque-error: ~a~%" (norm *gravity-torque-error*))

  (setq *config-task*
        (instance torque-validate-config-task :init
                  :robot robot
                  :joint-list joint-list
                  :drive-joint-list drive-joint-list
                  :wrench-list wrench-list
                  :contact-target-coords-list contact-target-coords-list
                  :contact-attention-coords-list contact-attention-coords-list
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

  (list (list :contact-torque-error *contact-torque-error*)
        (list :gravity-torque-error *gravity-torque-error*)
        (list :contact-torque-jacobian-error *contact-torque-jacobian-error*)
        (list :gravity-torque-jacobian-error *gravity-torque-jacobian-error*)
        (list :torque-jacobian-error *torque-jacobian-error*)
        )
  )

(defun sample-torque-gradient-sample-arm
    ()
  (setq *robot* (instance sarmclass :init))

  (sample-torque-gradient-common
   :robot *robot*
   :joint-list (butlast (send *robot* :joint-list) 2)
   :drive-joint-list (butlast (send *robot* :joint-list) 2)
   :wrench-list (list (float-vector 10 -15 20 1 2 -3))
   :contact-target-coords-list (list (send (send (send *robot* :end-coords) :copy-worldcoords) :translate #f(100 0 0)))
   :contact-attention-coords-list (list (send *robot* :end-coords))
   )
  )
(warn "(sample-torque-gradient-sample-arm)~%")

(defun sample-torque-gradient-sample-robot
    (&key
     (root-virtual-mode nil)
     (drive-root-virtual? nil)
     )
  (setq *robot* (instance sample-robot :init))
  (when root-virtual-mode
    (setq *robot-env* (instance robot-environment :init :robot *robot* :root-virtual-mode root-virtual-mode)))
  (send *robot* :reset-pose)

  (sample-torque-gradient-common
   :robot
   (if root-virtual-mode (send *robot-env* :robot-with-root-virtual) *robot*)
   :joint-list
   (if root-virtual-mode
       (send *robot-env* :robot-with-root-virtual :joint-list)
     (send *robot* :joint-list))
   :drive-joint-list
   (if drive-root-virtual?
       (send *robot-env* :robot-with-root-virtual :joint-list)
     (send *robot* :joint-list))
   :wrench-list (list (float-vector 10 -15 20 1 2 -3))
   :contact-target-coords-list (list (send *robot* :rarm :end-coords))
   :contact-attention-coords-list (list (send *robot* :larm :end-coords))
   )
  )
(warn "(sample-torque-gradient-sample-robot)~%")
(warn "(sample-torque-gradient-sample-robot :root-virtual-mode :6dof)~%")
(warn "(sample-torque-gradient-sample-robot :root-virtual-mode :6dof :drive-root-virtual? t)~%")
