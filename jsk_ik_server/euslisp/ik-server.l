#!/usr/bin/env roseus

(ros::load-ros-manifest "jsk_ik_server")
(ros::load-ros-manifest "arm_navigation_msgs")
(ros::load-ros-manifest "moveit_msgs")

(require :rtm-ros-robotics-interface
	 "package://hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l")
(require "models/arrow-object.l")
(require "package://pr2eus/pr2-interface.l")

(ros::roseus "ik_server")

(defclass ik-server
  :slots
  (*robot*
   *arrow*
   *centroid-object*
   *foot-convex*
   *root-link*
   *success-state-collection*
   *fix-limb-coords*
   ))

(defmethod ik-server
  (:init
   nil
   (unless (boundp '*do-not-start-ik-server*)
     (send self :start-ik-server)))
  (:ik-service-cb
   (req)
   (let* ((res (send req :response))
	  (arm (if (or (substringp "larm" (send req :ik_request :group_name))
		       (substringp "lhand" (send req :ik_request :group_name)))
		   :larm
		 :rarm))
	  (torsop (substringp "torso" (send req :ik_request :group_name)))
	  (torso2p (substringp "torso2" (send req :ik_request :group_name)))
	  (torso3p (substringp "torso3" (send req :ik_request :group_name)))
	  (fullbodyp (substringp "fullbody" (send req :ik_request :group_name)))
	  (opt-str (send (send (send req :ik_request) :constraints) :name))
	  (opt
	   (cond
	    ((and (stringp opt-str)
		  (plusp (length opt-str))
		  (listp (setq opt-str (read-from-string opt-str))))
	     opt-str)
	    (t (list :update-convex t)))))
     ;; taking joint state into account
     (cond
      ((and (send req :ik_request :robot_state)
	    (send req :ik_request :robot_state :joint_state)
	    (send req :ik_request :robot_state :joint_state :name)
	    (send req :ik_request :robot_state :joint_state :position)
	    (= (length (send req :ik_request :robot_state :joint_state :name))
	       (length (send req :ik_request :robot_state :joint_state :position))))
       (dotimes (i (length (send req :ik_request :robot_state :joint_state :name)))
	 (let ((joint-name (elt (send req :ik_request :robot_state :joint_state :name) i)))
	   (let ((j (find joint-name (send *robot* :joint-list)
			  :test #'string= :key #'(lambda (x) (send x :name)))))
	     (if j
		 (send j :joint-angle
		       (rad2deg
			(elt
			 (send req :ik_request :robot_state :joint_state :position)
			 i)))
	       (warn "we cannot find ~A~%" joint-name)
	       )))))
      (t (send *robot* :reset-manip-pose)))
     (format t "[ik-server] options: ~A~%" opt)
     (format t "            target: limb=~A, torso=~A(~A), fullbody=~A~%"
	     arm torsop torso2p fullbodyp)

     (send *robot* :fix-leg-to-coords (make-coords) :both)

     (cond
      ((cadr (member :update-convex opt))

       (send self :make-foot-convex :force? t)
       (send *irtviewer* :objects
	     (list *robot* *centroid-object* *foot-convex* *arrow*))))
     ;;(make-foot-convex)
     ;;(send *irtviewer* :objects
     ;;(list *robot* *foot-convex* *centroid-object* *arrow*))
     (send *viewer* :draw-objects)
     (let ((goal (send req :ik_request :pose_stamped))
	   ;; check that goal is malformed or not
	   (q (float-vector (send req :ik_request :pose_stamped :pose :orientation :w)
			    (send req :ik_request :pose_stamped :pose :orientation :x)
			    (send req :ik_request :pose_stamped :pose :orientation :y)
			    (send req :ik_request :pose_stamped :pose :orientation :z))))
       (send res :error_code :val moveit_msgs::MoveItErrorCodes::*FAILURE*)
       (cond
	((not (eps= (norm q) 1.0))
	 (format t "invalid quotanion detected ~A~%" q)
	 res)
	((let* ((goal-in-pelvis (send *tf* :transform-pose *root-link* goal))
		(goal-in-world
		 (send (send *robot* :copy-worldcoords)
		       :transform goal-in-pelvis))
		(draw-arrow
		 (send *arrow* :newcoords goal-in-world))
		(ik-ret (cond
			 (fullbodyp
			  (apply
			   #'send self :fullbody-ik-with-collision
			   (append
			    (list arm goal-in-world)
			    opt))
			  )
			 (torso2p
			  (apply
			   #'send self :torso-ik-with-collision
			   (append
			    (list arm goal-in-world)
			    (list :root-link-virtual-joint-weight
				  (float-vector 0.02 0.02 0.1 0.0 0.0 0.04))
			    opt)))
			 (torso3p
			  (apply
			   #'send self :torso-ik-with-collision
			   (append
			    (list arm goal-in-world)
			    (list :waist-gain '(0 0 1))
			    opt)))
			 (torsop
			  (apply
			   #'send self :torso-ik-with-collision
			   (append
			    (list arm goal-in-world)
			    opt)))
			 (t
			  (apply
			   #'send self :normal-ik-with-collision
			   (append
			    (list arm goal-in-world)
			    opt))))))
	   ;;(print (list 'original goal))
	   ;; (print (list 'original
	   ;;              (send goal :header :frame_id)
	   ;;              (send goal :pose :position :x)
	   ;;              (send goal :pose :position :y)
	   ;;              (send goal :pose :position :z)
	   ;;              (send goal :pose :orientation :x)
	   ;;              (send goal :pose :orientation :y)
	   ;;              (send goal :pose :orientation :z)
	   ;;              (send goal :pose :orientation :w)))
	   ;; (print (list 'goal-in-pelvis goal-in-pelvis))
	   ;; (print (list 'goal-in-world goal-in-world))
	   (vectorp ik-ret))
	 ;;(format t "[ik-server] SUCCESS!!~%")
	 (send res :error_code :val moveit_msgs::MoveItErrorCodes::*SUCCESS*)
	 (send *viewer* :draw-objects)
	 (send res :solution :joint_state :name
	       (send-all (send *robot* :joint-list) :name))
	 (send res :solution :joint_state :position
	       (map float-vector #'deg2rad (send *robot* :angle-vector)))
	 res)
	(t res))))
   )
  (:start-ik-server
   nil
   (send self :init-robot)
   ;; initialization
   (setq *arrow* (arrow))

   (cond
    ((not (and (boundp '*irtviewer*) *irtviewer*))
     (make-irtviewer :width 640 :height 640)
     (send *irtviewer* :change-background #f(0.9 1.0 0.9))
     (send *irtviewer* :title "IK server")))
   (send *irtviewer*
	 :objects
	 (list *robot* *arrow*))

   (unless *foot-convex* (setq *foot-convex* (make-coords)))
   (unless *centroid-object* (setq *centroid-object* (make-coords)))
   (send *irtviewer*
	 :objects
	 (list *robot* *foot-convex* *arrow*))
   (send *irtviewer* :look-all)
   (send *viewer* :draw-objects)
   ;;
   (setq *tf* (instance ros::transform-listener :init))
   ;;
   (send *irtviewer*
	 :objects
	 (list *robot* *foot-convex* *arrow* *centroid-object*))

   (ros::advertise-service "/solve_ik" moveit_msgs::GetPositionIK #'send self :ik-service-cb)
   (ros::rate 10)
   (send self :loop-ik-server)
   )
  (:loop-ik-server
   nil
   (do-until-key
    (if (not (ros::ok)) (return-from nil nil))
    (unless (or (null x::*display*) (= x::*display* 0))
      (x::window-main-one))
    (ros::sleep)
    (ros::spin-once))
   )
  
  (:init-robot
   nil
   (defvar *robot-name* (ros::get-param "~robot" "PR2"))
   (cond
    ;;pr2
    ((equal (string-upcase *robot-name*) "PR2")
     (require "package://pr2eus/pr2-interface.l")
     (setq *robot* (pr2))
     (setq *root-link* "/base_link")
     )
    ;;atlas
    ((equal (string-upcase *robot-name*) "ATLAS")
     (require "package://hrpsys_gazebo_atlas/euslisp/atlas-interface.l")
     (setq *atlas* (atlas-with-hand))

     (send *atlas* :reset-manip-pose)
     (send *atlas* :fix-leg-to-coords (make-coords) :both)
     (send-all (send *atlas* :joint-list) :max-joint-velocity 0.3)

     (send *atlas* :torso :waist-r :max-angle 10)
     (send *atlas* :torso :waist-r :min-angle -10)
     ;;
     (send *atlas* :torso :waist-p :max-angle 10)
     (send *atlas* :torso :waist-p :min-angle -10)
     (setq *robot* *atlas*)
     
     (send self :make-foot-convex)
     (send self :make-centroid-object)

     (send self :init-fullbody-ik-main)
     (setq *root-link* "/pelvis")

     )

    (t
     (ros::ros-error "undefined robot: ~A" *robot-name*)
     (ros::ros-error "use PR2")
     (require "package://pr2eus/pr2-interface.l")
     (setq *robot* (pr2))
     (setq *root-link* "/base_link")
     )
    )
   )


  )

;; IK methods definition
(require "package://jsk_ik_server/euslisp/eus-fullbody-ik-ex.l")

(setq *ik-server* (instance ik-server :init))