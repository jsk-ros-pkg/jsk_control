#!/usr/bin/env roseus

(ros::load-ros-manifest "hrpsys_gazebo_atlas")
;(ros::load-ros-manifest "arm_navigation_msgs")
(ros::load-ros-manifest "moveit_msgs")

(ros::roseus "ik_client_test")

(defun random-select (lst)
  (let ((len (length lst)))
    (let ((r (random 10.0)))
      (dotimes (i len)
        (if (< r (* (1+ i) (/ 10.0 len)))
            (return-from random-select (elt lst i)))))))

(setq *prev-js* nil)
(setq *root-link* (ros::get-param "~root_link" "/pelvis"))

(do-until-key
 (let ((req (instance moveit_msgs::GetPositionIKRequest :init))
       (D 0.2))
   (send req :ik_request :group_name (random-select
                                      '("larm" "larm_torso" "larm_fullbody"
                                        "rarm" "rarm_torso" "rarm_fullbody")))
   (let ((c (make-coords)))
     (if (or (string= (subseq (send req :ik_request :group_name) 0 4) "larm"))
         (send c :rotate (deg2rad 90) :x)
       (progn
         (send c :rotate (deg2rad 180) :z)
         (send c :rotate (deg2rad 90) :x)))
     (if *prev-js*
         (send req :ik_request :robot_state :joint_state *prev-js*))
     (ros::ros-info "send reqeust ~12A ~A" (send req :ik_request :group_name) c)
     ;;(send c :rotate (deg2rad 90) :x))
     (let ((q (matrix2quaternion (send c :rot))))
       (send req :ik_request :pose_stamped (instance geometry_msgs::PoseStamped :init
                 :header (instance std_msgs::Header :init
                       :stamp (ros::time 0.0)
                       :frame_id *root-link*)
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
                     ))))
     (let ((res (ros::service-call "/solve_ik" req)))
       (if (= (send res :error_code :val) moveit_msgs::MoveItErrorCodes::*SUCCESS*)
	   (ros::ros-info " success ~A"  (send res :solution :joint_state :position))
	 (ros::ros-error " failed ")
	 ))
     )))
;(exit)

