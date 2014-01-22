#!/usr/bin/env roseus

(ros::load-ros-manifest "hrpsys_gazebo_atlas")
(ros::load-ros-manifest "arm_navigation_msgs")
(ros::load-ros-manifest "moveit_msgs")

(ros::roseus "atlas_eus_ik_client")
(ros::load-ros-manifest "hrpsys_gazebo_atlas")
(ros::load-ros-manifest "moveit_msgs")

(defun random-select (lst)
  (let ((len (length lst)))
    (let ((r (random 10.0)))
      (dotimes (i len)
        (if (< r (* (1+ i) (/ 10.0 len)))
            (return-from random-select (elt lst i)))))))

(setq *prev-js* nil)

(do-until-key
 (let ((req (instance moveit_msgs::GetPositionIKRequest :init))
       (D 0.2))
   (send req :ik_request :group_name (random-select
                                      '("larm" "larm_torso" "larm_fullbody"
                                        "rarm" "rarm_torso" "rarm_fullbody")))
   (print (send req :ik_request :group_name))
   (let ((c (make-coords)))
     (if (or (string= (subseq (send req :ik_request :group_name) 0 4) "larm"))
         (send c :rotate (deg2rad 90) :x)
       (progn
         (send c :rotate (deg2rad 180) :z)
         (send c :rotate (deg2rad 90) :x)))
     (if *prev-js*
         (send req :ik_request :robot_state :joint_state *prev-js*))
     ;;(send c :rotate (deg2rad 90) :x))
     (let ((q (matrix2quaternion (send c :rot))))
       (send req :ik_request :pose_stamped (instance geometry_msgs::PoseStamped :init
                 :header (instance std_msgs::Header :init
                       :stamp (ros::time 0.0)
                       :frame_id "/pelvis")
                 :pose (instance geometry_msgs::Pose :init
                     :position (instance geometry_msgs::Point :init
                                         :x (+ 0.6 (- (random D) (/ D 2.0)))
                                         ;;:x (+ 0.4 (- (random D) (/ D 2.0)))
                                         :y (- (random (* D 2)) (/ (* 2 D) 2.0))
                                         :z (+ 0.3 (- (random D) (/ D 2.0))))
                     :orientation
                     (instance geometry_msgs::Quaternion :init
                         :x (elt q 1)
                         :y (elt q 2)
                         :z (elt q 3)
                         :w (elt q 0))
                     )))
       (let ((res (ros::service-call "/atlas_eus_ik" req)))
   (if (= (send res :error_code :val) moveit_msgs::MoveItErrorCodes::*SUCCESS*)
       (progn
         (print "successed!")
         ;;(setq *prev-js* (send res :solution :joint_state))
         (print (send res :solution :joint_state :position))
         )
     (print "failed!")
     )))
     )))
;(exit)

