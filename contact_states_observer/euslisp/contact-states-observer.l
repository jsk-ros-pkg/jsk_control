#!/usr/bin/env roseus
;;#-:jsk (jsk)
;;#-:rbrain-basic (rbrain)

(ros::load-ros-manifest "contact_states_observer")
(ros::load-ros-manifest "roseus")
(ros::load-ros-manifest "hrpsys_ros_bridge")
(ros::roseus "hrp2_contact")

(ros::advertise "ore_contact_states" contact_states_observer::OREContactStates 1)
(ros::advertise "grasp_states" contact_states_observer::GraspStates 1)
(ros::advertise "contact_grasp_states" hrpsys_ros_bridge::ContactStatesStamped  1)

(defclass grasp-state
  :super propertied-object
  :slots (check-jname-list reference-datas current-av result)
  )

(defmethod grasp-state
  (:init
   (nm)
   (send self :name nm)
   self)
  (:set-reference-datas
   (rd)
   (setq reference-datas rd)
   )
  (:set-current-av
   (cav)
   (setq current-av cav)
   )
  (:set-check-jname-list
   (cjl)
   (setq check-jname-list cjl)
   )
  (:pose-similarity
   (jlist &key (debug-view nil))
   (let* ((ret)
          (idx-list
           (mapcar #'(lambda (j) (position (send (send (send *robot* :hand (send self :name)) j) :name) (send-all jlist :name))) check-jname-list))
          (cur-ja-list
           (mapcar #'(lambda (idx) (elt current-av idx)) idx-list)))
     (dolist (rav reference-datas)
       (let ((err (mapcar #'(lambda (rj cj) (abs (- rj cj)))
                          (mapcar #'(lambda (idx) (elt (cadr rav) idx)) idx-list)
                          cur-ja-list)))
         (if debug-view (format t ";; error for ~A ~A = ~A~%" (car rav) check-jname-list err))
         (push (list (car rav) err) ret)))
     (setq result (car (find-extream (reverse ret) #'(lambda (x) (reduce #'+ (cadr x) :initial-value 0)) #'<)))
     ))
  (:get-result
   ()
   "return t if result is one of :success, :success0, and so on"
   (substringp (string :success) (string result))
   )
  )

(defclass contact-force-state
  :super propertied-object
  :slots (ref-force-thre result)
  )

(defmethod contact-force-state
  (:init
   (nm)
   (send self :name nm)
   (setq ref-force-thre 20) ;; [N]
   self)
  (:set-ref-force-thre
   (tmp-thre)
   (setq ref-force-thre tmp-thre)
   )
  (:check-contact-force
   (con-force)
   (setq result (> (norm con-force) ref-force-thre))
   )
  (:get-result
   ()
   result
   )
  )

(defclass ore-contact-observer
  :super propertied-object
  :slots (mode result)
  )
(defmethod ore-contact-observer
  (:init
   (nm tmp-mode)
   (send self :name nm)
   (setq mode tmp-mode)
   self)
  (:get-result () (error ";; ~A requires :get-result~%" (send self :name)))
  (:mode () mode)
  )

(defclass robot-contact-observer
  :super ore-contact-observer
  :slots (contact-observer-list)
  )

(defmethod robot-contact-observer
  (:init
   (nm tmp-mode
    &key (observer-class-list (list grasp-state contact-force-state)))
   (setq contact-observer-list
         (mapcar #'(lambda (x) (instance x :init nm))
                 observer-class-list))
   (send-super :init nm tmp-mode))
  (:get-result
   ()
   (let* ((co (find-if #'(lambda (x) (string= (format nil "~A-state" mode) (string-downcase (send (class x) :name))))
                       contact-observer-list))
          (st
           (cond
            (co (send co :get-result))
            (t
             (every #'identity (send-all co :get-result))
             ))
           ))
     (if st "On" "Off")
     ))
  (:get-one-state
   (nm)
   (if (string= mode nm)
       (find-if #'(lambda (x) (string= (format nil "~A-state" nm) (string-downcase (send (class x) :name))))
                contact-observer-list)))
  (:get-grasp-state () (send self :get-one-state "grasp"))
  (:get-contact-force-state () (send self :get-one-state "contact-force"))
  )

(defclass obj-env-contact-observer
  :super ore-contact-observer
  :slots ()
  )

(defmethod obj-env-contact-observer
  (:init
   (nm tmp-mode)
   (setq result "face-contact")
   (send-super :init nm tmp-mode))
  (:set-state
   (st)
   (setq result st))
  (:get-result () result)
  )

(defun setup-grasp-states ()
  (let ((limbs
         (mapcar #'(lambda (x) (find-if #'(lambda (l) (member x (send *robot* l :force-sensors))) '(:rleg :lleg :rarm :larm)))
                 (send *robot* :force-sensors))))
    (unless (boundp '*rcol*)
      (setq *rcol* (mapcar #'(lambda (a)
                            (instance robot-contact-observer
                                      :init a
                                      (if (substringp "arm" (string-downcase a))
                                          "grasp" "contact-force")
                                      ))
                        limbs)))
    (unless (boundp '*col*)
      (setq *col* (append *rcol* (list (instance obj-env-contact-observer
                                                 :init :obj-env "force")))))
    ))

(defun publish-contact-states ()
  (let* ((stamp (ros::time-now))
         (cs (instance contact_states_observer::OREContactStates
                       :init
                       :contact_states
                       (mapcar #'(lambda (co)
                                   (instance contact_states_observer::OREContactState
                                             :init
                                             :name (format nil "~A" (send co :name))
                                             :state (send co :get-result)
                                             :mode (send co :mode)))
                               *col*
                               ))))
    (send-all (send cs :contact_states) :header :stamp stamp)
    (ros::publish "ore_contact_states" cs)
    ))

(defun get-limbs-in-force-sensor-order (robot)
  (mapcar #'(lambda (x) (find-if #'(lambda (l) (member x (send robot l :force-sensors))) '(:rleg :lleg :rarm :larm)))
          (send robot :force-sensors)))

(defun publish-hrpsysrosbridge-contact-states ()
  (let* ((stamp (ros::time-now))
         (force-cs (null-output (send *ri* :state :act-contact-states)))
         (co-list (remove-if-not #'(lambda (x) (memq (send x :name) (get-limbs-in-force-sensor-order *robot*))) *col*)))
    (unless (= (length co-list) (length force-cs))
      (warn ";; [contact-states-observer.l] Invalid length in publish-hrpsysrosbridge-contact-states ~A, ~A~%"
            (length co-list) (length force-cs))
      (return-from publish-hrpsysrosbridge-contact-states nil))
    (let ((cs (instance hrpsys_ros_bridge::ContactStatesStamped
                        :init
                        :states
                        (mapcar #'(lambda (co fcs)
                                    (warn ";; HOGE~%")
                                    (let ((ret (instance hrpsys_ros_bridge::ContactStateStamped
                                                         :init
                                                         :state (instance hrpsys_ros_bridge::ContactState
                                                                          :init
                                                                          :state (if (string= "grasp" (send co :mode))
                                                                                     (if (string= "Off" (send co :get-result))
                                                                                         hrpsys_ros_bridge::contactstate::*off*
                                                                                       hrpsys_ros_bridge::contactstate::*on*)
                                                                                   (if (= fcs 0)
                                                                                       hrpsys_ros_bridge::contactstate::*off*
                                                                                     hrpsys_ros_bridge::contactstate::*on*))))))
                                      (send (send ret :header) :frame_id (send (send (send *robot* (send co :name) :end-coords) :parent) :name))
                                      ret))
                                co-list force-cs)
                        )))
      (send cs :header :stamp stamp)
      (ros::publish "contact_grasp_states" cs)
      )))

(defun publish-grasp-states
  ()
  (let* ((rgs (mapcar #'(lambda (gs)
                          (instance contact_states_observer::GraspState
                                   :init
                                   :check_jname_list (mapcar #'(lambda (x) (format nil "~A" x)) (gs . check-jname-list))
                                   :result (format nil "~A" (if (gs . result) (gs . result) ""))
                                   :reference_datas
                                   (mapcar #'(lambda (x)
                                               (instance contact_states_observer::GraspReferenceData
                                                         :init
                                                         :name (format nil "~A" (car x))
                                                         :av (cadr x)))
                                           (gs . reference-datas))
                                   :current_av (gs . current-av)))
                      (remove nil (send-all *rcol* :get-grasp-state))
                      ))
         (rgss (instance contact_states_observer::GraspStates
                         :init :grasp_states rgs)))
    (send rgss :header :stamp (ros::time-now))
    (ros::publish "grasp_states" rgss)
    ))

(defun grasp-state-set-param-cb (req)
  (let* ((res (send req :response))
         (str (send req :str))
         (dd (read-from-string str)))
    (mapcar #'(lambda (x)
                (let ((gs (find-if #'(lambda (gs) (eq (send gs :name) (car x)))
                                   (remove nil (send-all *rcol* :get-grasp-state)))))
                  (when gs
                    (if (cdr x)
                        (progn
                          (send gs :set-reference-datas (cadr x))
                          (send gs :set-check-jname-list  (caddr x)))
                      (progn
                        (send gs :set-reference-datas nil)
                        (send gs :set-check-jname-list nil)
                        )))))
            dd)
    res))

(defun obj-env-state-set-param-cb (req)
  (let* ((res (send req :response))
         (str (send req :str))
         (oe-co (find-if #'(lambda (x) (derivedp x obj-env-contact-observer)) *col*)))
    (send oe-co :set-state str)
    res))

(ros::advertise-service
 "/grasp_state_set_param"
 roseus::stringstring
 #'grasp-state-set-param-cb)

(ros::advertise-service
 "/obj_env_state_set_param"
 roseus::stringstring
 #'obj-env-state-set-param-cb)

(load "package://pr2eus/robot-interface.l")
(robot-init)
(setq *robot* (eval (read-from-string (format nil "*~A*" (ros::get-param "robot/type")))))
(ros::rate 20)
;;(setup-sensor-viewer)
(setup-grasp-states)
(do-until-key
 (let ((hav (or (and (find-method *ri* :hand-state) (send (send *ri* :hand-state) :potentio))
                (send *ri* :state :gripper :arms :angle-vector))))
   (when hav
     (cond
      ((substringp "HRP2" (send *robot* :name))
       (null-output (send *robot* :hand-angle-vector hav)))
      ((substringp "JAXON" (send *robot* :name))
       (mapcar #'(lambda (a av)
                   (send (send *robot* :hand a) :angle-vector av))
               '(:rarm :larm) hav)))
     (mapcar #'(lambda (gs hd)
                 (send gs :set-current-av
                       (send hd :angle-vector))
                 (send gs :pose-similarity (send hd :joint-list)))
             (remove nil (send-all *rcol* :get-grasp-state))
             (send *robot* :hand :arms))
     )
   (publish-contact-states)
   (publish-hrpsysrosbridge-contact-states)
   (publish-grasp-states)
   (ros::spin-once)
   (ros::sleep)
   ))


