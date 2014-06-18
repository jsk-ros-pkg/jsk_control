#!/usr/bin/env roseus

(require "package://jsk_ik_server/euslisp/fullbody-ik-client.l")
(ros::load-ros-manifest "sensor_msgs")
(ros::roseus "robot_controller_with_ik_server")

(let ((ik-client-test (instance fullbody-ik-client :init))
      (robot-name (unix::getenv "ROBOT_CONTROLLER_ROBOT_NAME"))
      (foot? (unix::getenv "ROBOT_CONTROLLER_WITH_LEG"))
      (sim? (unix::getenv "ROBOT_CONTROLLER_SIM_MODE"))
      )
  (if (and (stringp foot?) (plusp (length foot?)))
      (setq foot? (string-equal foot? "true")))
  (if (and (stringp sim?) (plusp (length sim?)))
      (setq sim? (string-equal sim? "true")))
  (if (not sim?)
      (setq *ri*
	    (progn
	      (defclass dummy-ri :super object :slots nil)
	      (defmethod dummy-ri (:nomethod (&rest args) nil))
	      (instance dummy-ri :init))))
  (cond
   ((substringp "staro" robot-name)
    ;; (require "package://hrpsys_ros_bridge_tutorials/models/staro.l")
    ;; (defvar *robot* (staro))
    ;; (objects (list *robot*))
    (require "package://jsk_hrpsys_ros_bridge/euslisp/staro-interface.l")
    (cond
     (sim?
      (staro-init)
      (setq *robot* *staro*))
     (t
      (setq *robot* (staro))
      (objects (list *robot*)))
     )
    )
   (t ;;(substringp "hrp2jsknt" robot-name)
    (require "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknt-interface.l")
    (cond
     (sim?
      (hrp2jsknt-init)
      (setq *robot* *hrp2jsknt*))
     (t
      (setq *robot* (hrp2jsknt))
      (objects (list *robot*))))
    )
   )
  (objects (list *robot*))
  )

(defvar *updated* nil)
(defvar *target-limb* :rarm)

(defvar *target-cascoords*
  (mapcar #'(lambda (k) (send *robot* k :end-coords))
	  '(:rarm :larm :rleg :lleg)))
(defvar *target-coords*
  (send-all *target-cascoords* :copy-worldcoords))

(defun model2real
  (&key (robot *robot*)
	(sleep-time 5000)
	(max-vel 50) ;; deg/s
	(diff (norm (v- (send robot :angle-vector)
			(or (send *ri* :state :potentio-vector)
			    (send robot :angle-vector))
			)))
	wait?
	)
  (setq sleep-time
	(round
	 (max
	  (* 1000.0 (/ diff max-vel))
	  sleep-time)))
  (send *ri* :angle-vector
	(send robot :angle-vector) sleep-time)
  (if wait? (send *ri* :wait-interpolation))
  sleep-time)

;; (defun model2real (&rest args &key (sleep-time 1000) &allow-other-keys) sleep-time)

(defun real2model
  nil
  (if (and (boundp '*ri*) *ri*)
      (send *robot* :angle-vector (send *ri* :state :potentio-vector))))

(defun update-pose
  nil
  (let* ((mt *target-cascoords*)
	 (tc *target-coords*)
	 (ll
	  (mapcar
	   #'(lambda (mt)
	       (send *robot* :link-list (send mt :parent)))
	   mt))
	 buf
	 )
    (setq buf (mapcar #'list '(:rarm :larm :rleg :lleg) mt tc ll))
    (setq buf (cons
	       (find-if #'(lambda (l) (eq *target-limb* (car l))) buf)
	       (remove-if #'(lambda (l) (eq *target-limb* (car l))) buf)))
    (ik-server-call
     :target-coords (mapcar #'caddr buf)
     :move-target (mapcar #'cadr buf)
     :link-list (mapcar #'cadddr buf)
     :all-links
     (remove-if
      #'(lambda (l)
	  (not (and (find-method l :joint)
		    (send l :joint))))
      (send *robot* :links))
     :target-cendtroid-pos
     (scale 0.5 (apply #'v+ (send-all (subseq *target-coords* 2 4) :worldpos)))
     :centroid-thre 10
     :cog-gain 1.0
     :debug-view :no-message
     :ik-server-service-name "/solve_ik"
     )
    ))

(defun string-command-callback
  (msg)
  (let* ((com (send msg :data))
	 (parsed (if (and com (plusp (length com)))
		     (read-from-string com))))
    (cond
     ((null com)
      (format t "[string-command-callback] null command~%"))
     ((and (listp parsed)
	   (functionp (car parsed)))
      (eval parsed)
      (setq *updated* t)
      (send *robot* :fix-leg-to-coords (make-coords))
      (send-all (send *robot* :links) :worldcoords)
      (setq *target-coords*
	    (append
	     (send-all (subseq *target-cascoords* 0 2) :copy-worldcoords)
	     (subseq *target-coords*  2 4)))
      )
     ))
  )

(defun limb-posestamped-callback
  (msg
   &key
   (limb :rarm)
   (id (cdr (assoc limb '((:rarm . 0) (:larm . 1) (:rleg . 2) (:lleg . 3)))))
   (flag :absolute)
   (target-coords (nth id *target-coords*))
   (newcoords
    (make-coords
     :pos (scale 1e+3 (float-vector (send (send msg :pose) :position :x)
				    (send (send msg :pose) :position :y)
				    (send (send msg :pose) :position :z)))
     :rot (quaternion2matrix
	   (float-vector
	    (send (send msg :pose) :orientation :w)
	    (send (send msg :pose) :orientation :x)
	    (send (send msg :pose) :orientation :y)
	    (send (send msg :pose) :orientation :z))))))
  (setq *target-limb* limb)
  (setq *updated* t)
  (cond
   ((eq flag :absolute)
    (send
     target-coords
     :newcoords
     (send (send (car (send *robot* :links)) :copy-worldcoords)
	   :transform newcoords)))
   ((eq flag :relative)
    (send target-coords :transform newcoords :local))
   (t
    (send target-coords
	  :newcoords
	  (make-coords
	   :pos
	   (v+ (send target-coords :worldpos)
	       (send newcoords :worldpos))
	   :rot
	   (m*
	    (send newcoords :worldrot)
	    (send target-coords :worldrot)))))))

(defun absolute-rarm-posestamped-callback
  (msg) (limb-posestamped-callback msg :limb :rarm :flag :absolute))
(defun absolute-larm-posestamped-callback
  (msg) (limb-posestamped-callback msg :limb :larm :flag :absolute))
(defun absolute-rleg-posestamped-callback
  (msg) (limb-posestamped-callback msg :limb :rleg :flag :absolute))
(defun absolute-lleg-posestamped-callback
  (msg) (limb-posestamped-callback msg :limb :lleg :flag :absolute))

(defun relative-rarm-posestamped-callback
  (msg) (limb-posestamped-callback msg :limb :rarm :flag :relative))
(defun relative-larm-posestamped-callback
  (msg) (limb-posestamped-callback msg :limb :larm :flag :relative))
(defun relative-rleg-posestamped-callback
  (msg) (limb-posestamped-callback msg :limb :rleg :flag :relative))
(defun relative-lleg-posestamped-callback
  (msg) (limb-posestamped-callback msg :limb :lleg :flag :relative))

(defun relative2-rarm-posestamped-callback
  (msg) (limb-posestamped-callback msg :limb :rarm :flag :relative2))
(defun relative2-larm-posestamped-callback
  (msg) (limb-posestamped-callback msg :limb :larm :flag :relative2))
(defun relative2-rleg-posestamped-callback
  (msg) (limb-posestamped-callback msg :limb :rleg :flag :relative2))
(defun relative2-lleg-posestamped-callback
  (msg) (limb-posestamped-callback msg :limb :lleg :flag :relative2))

(defvar *prev-q*)
(defun head-posestamped-callback
  (msg &key (gain 0) relative)
  (let* ((ori (send (send msg :pose) :orientation))
	 (q (coerce
	     (mapcar #'(lambda (k) (send ori k)) '(:w :x :y :z))
	     float-vector))
	 (rpy
	  (matrix-log
	   (quaternion2matrix q))))
    (if (null *prev-q*) (setq *prev-q* q))
    (cond
     ((> (norm (v- *prev-q* q)) 0.1)
      (setq *updated* t)
      (format t "[cyborg2head] q=~A~%" q)
      (send *robot* :head :neck-p :joint-angle
	    (rad2deg (* gain (aref rpy 1)))
	    :relative relative)
      (send *robot* :head :neck-y :joint-angle
	    (rad2deg (* gain (aref rpy 2)))
	    :relative relative)))
    (setq *prev-q* q)))

(defun relative-head-posestamped-callback
  (msg) (head-posestamped-callback msg :gain 0.1 :relative t))
(defun absolute-head-posestamped-callback
  (msg) (head-posestamped-callback msg :gain 0.5 :relative nil))

(defun joint-state-callback
  (msg)
  (let ((names (send msg :name))
	(positions (send msg :position))
	(velocitys (send msg :velocity))
	key j
	(link-list
	 (remove-if #'(lambda (l) (not (send l :joint))) (send *robot* :links))))
    (cond
     ((not (eq (length positions) (length names)))
      (format t "[robot-controller] position skip~%")
      (setq positions (make-list (length names))))
     ((not (eq (length velocitys) (length names)))
      (format t "[robot-controller] velocity skip~%")
      (setq velocitys (make-list (length names)))))
    (map cons
	 #'(lambda (n pos vel)
	     (format t "[robot-controller] ~A ~A ~A~%" n pos vel)
	     (cond
	      ((setq j
		     (find-if #'(lambda (j)
				  (substringp n (format nil "~A" (send j :name))))
			      (send-all link-list :joint)))
	       (setq link-list (remove (send j :child-link) link-list))
	       (send j :joint-angle
		     (cond
		      (pos (rad2deg pos))
		      (vel (+ (send j :joint-angle)
			      (rad2deg vel)))
		      (t (send j :joint-angle))))
	       (setq *updated* t))
	      ((setq j
		     (find-if #'(lambda (j)
				  (substringp n (format nil "~A" (send j :name))))
			      link-list))
	       (setq link-list (remove j link-list))
	       (setq j (send j :joint))
	       (send j :joint-angle
		     (cond
		      (pos (rad2deg pos))
		      (vel (+ (send j :joint-angle)
			      (rad2deg vel)))
		      (t (send j :joint-angle))))
	       (setq *updated* t)
	       )))
	 names positions velocitys)
    (send *robot* :fix-leg-to-coords (make-coords))
    (setq *target-coords*
	  (append
	   (send-all (subseq *target-cascoords* 0 2) :copy-worldcoords)
	   (subseq *target-coords*  2 4)))
    ))

(defun joint-state-fix-leg-callback
  (msg)
  (let* ((leg-coords
	  (midcoords 0.5 (send *robot* :rleg :end-coords :worldcoords)
		     (send *robot* :lleg :end-coords :worldcoords)))
	 (leg-joint-angles
	  (append
	   (send-all (send *robot* :rleg :joint-list) :joint-angle)
	   (send-all (send *robot* :lleg :joint-list) :joint-angle))))
    (joint-state-callback msg)
    (mapcar
     #'(lambda (j av) (send j :joint-angle av))
     (append (send *robot* :rleg :joint-list)
	     (send *robot* :lleg :joint-list))
     leg-joint-angles)
    (send *robot* :fix-leg-to-coords leg-coords)))

(ros::subscribe
 "/robot_joint_command/string_command"
 std_msgs::string
 #'string-command-callback 1)

(ros::subscribe
 "/robot_joint_command/absolute_head_posestamped"
 geometry_msgs::posestamped
 #'absolute-head-posestamped-callback 1)

(ros::subscribe
 "/robot_joint_command/relative_head_posestamped"
 geometry_msgs::posestamped
 #'relative-head-posestamped-callback 1)

(ros::subscribe
 "/robot_joint_command/joint_state"
 sensor_msgs::JointState
 #'joint-state-callback 1)
(ros::subscribe
 "/robot_joint_command/joint_state_fix_leg"
 sensor_msgs::JointState
 #'joint-state-fix-leg-callback 1)

(ros::subscribe
 "/robot_joint_command/relative_rarm_posestamped"
 geometry_msgs::posestamped
 #'relative-rarm-posestamped-callback 1)
(ros::subscribe
 "/robot_joint_command/relative_larm_posestamped"
 geometry_msgs::posestamped
 #'relative-larm-posestamped-callback 1)
(ros::subscribe
 "/robot_joint_command/relative_rleg_posestamped"
 geometry_msgs::posestamped
 #'relative-rleg-posestamped-callback 1)
(ros::subscribe
 "/robot_joint_command/relative_lleg_posestamped"
 geometry_msgs::posestamped
 #'relative-lleg-posestamped-callback 1)

(ros::subscribe
 "/robot_joint_command/relative2_rarm_posestamped"
 geometry_msgs::posestamped
 #'relative2-rarm-posestamped-callback 1)
(ros::subscribe
 "/robot_joint_command/relative2_larm_posestamped"
 geometry_msgs::posestamped
 #'relative2-larm-posestamped-callback 1)
(ros::subscribe
 "/robot_joint_command/relative2_rleg_posestamped"
 geometry_msgs::posestamped
 #'relative2-rleg-posestamped-callback 1)
(ros::subscribe
 "/robot_joint_command/relative2_lleg_posestamped"
 geometry_msgs::posestamped
 #'relative2-lleg-posestamped-callback 1)

(ros::subscribe
 "/robot_joint_command/absolute_rarm_posestamped"
 geometry_msgs::posestamped
 #'absolute-rarm-posestamped-callback 1)
(ros::subscribe
 "/robot_joint_command/absolute_larm_posestamped"
 geometry_msgs::posestamped
 #'absolute-larm-posestamped-callback 1)
(ros::subscribe
 "/robot_joint_command/absolute_rleg_posestamped"
 geometry_msgs::posestamped
 #'absolute-rleg-posestamped-callback 1)
(ros::subscribe
 "/robot_joint_command/absolute_lleg_posestamped"
 geometry_msgs::posestamped
 #'absolute-lleg-posestamped-callback 1)


(send *viewer* :viewsurface :bg-color #F(1 1 1 0))
(real2model)
(send *robot* :fix-leg-to-coords (make-coords))
(setq *target-coords*
      (send-all *target-cascoords* :copy-worldcoords))

(defvar *ros-rate* 3)
(ros::rate *ros-rate*)
(do-until-key
 (let (tmp
       (timer (instance mtimer :init))
       (sleep-time (/ 1.0 *ros-rate*)))
   (if (not (ros::ok)) (return-from nil 'ros-dead))
   (ros::spin-once)
   (send *viewer* :draw-objects)
   (x::window-main-one)
   (cond
    (*updated*
     (update-pose)
     (format t "[robot-controller] model2real ")
     (setq
      sleep-time
      (model2real :sleep-time (* 1.0 1000 sleep-time)))
     (print sleep-time)
     (setq *updated* nil)
     )
    (t
     (real2model)
     (send *robot* :fix-leg-to-coords (make-coords))
     (send-all *target-cascoords* :worldcoords)
     (setq *target-coords*
	   (send-all *target-cascoords* :copy-worldcoords))
     ))
   (setq timer (send timer :stop))
   (cond
    ((> timer (* 1e-3 sleep-time ))
     (format t "[robot-controller] overslept ~A~%" timer))
    (t
     (unix::usleep (max 0 (round (* (- (* 1e-3 sleep-time) timer) 1000 1000))))
     ))
   ))
