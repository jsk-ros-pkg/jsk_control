#!/usr/bin/env roseus

(load "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknts-interface.l")
(ros::roseus-add-msgs "jsk_footstep_msgs")
(ros::roseus-add-msgs "jsk_footstep_controller")

(hrp2jsknts-init)
(setq *robot* *hrp2jsknts*)
(setq *hrp2* *robot*)


(defun fullbody-lookat
  (lp)
  (send *robot* :reset-manip-pose)
  (send *robot* :torso :waist-p :joint-angle 90)
  (send *robot* :fix-leg-to-coords (make-coords) :both)
  (let* ((mt (mapcar
              #'(lambda (k) (send *robot* k :end-coords))
              '(:rarm :larm :rleg :lleg :head)))
         (ll (mapcar
              #'(lambda (mt)
                  (remove-if
                   #'(lambda (l) (find l (list (send *robot* :rleg
:toe-p :child-link)
                                               (send *robot* :lleg
:toe-p :child-link))))
                   (send *robot* :link-list (send mt :parent))))
              mt))
         (tc (send-all mt :copy-worldcoords))
         ;; (lp #F(0 0 0))
         (vd (normalize-vector (v- lp (send (send *robot* :head
:end-coords) :worldpos))))
         (z (transform (send *robot* :head :end-coords :worldrot) #F(0 0 1)))
         )
    (send (car (last tc)) :newcoords
          (make-coords
           :pos (copy-seq (send (car (last tc)) :worldpos))
           :rot (m*
                 (matrix-exponent
                  (v* z vd) (acos (v. z vd)))
                 (send (car (last tc)) :worldrot))))
    (send *robot* :fullbody-inverse-kinematics
          tc
          :move-target mt
          :link-list ll
          :translation-axis (list nil nil t t :z)
          :rotation-axis (list :z nil t t t)
          :thre (list 1000 1000 5 5 100)
          :rthre (list 1000 1000 (deg2rad 3) (deg2rad 3) (deg2rad 3))
          :root-link-virtual-joint-weight (scale 10 #F(1 1 1 1 1 1))
          :centroid-thre 25
          :cog-gain 10.0
          :min #F(-1000 -1000 -1000 -400 -400 -400)
          :max #F(1000 1000 1000 400 400 400)
          :stop 100
          :debug-view :no-message
)))

(ros::roseus "lookaround_ground")

(defun call-empty-service (srv)
  (let ((req (instance std_srvs::EmptyRequest :init)))
    (ros::service-call srv req)))

(defun do-lookaround ()
  (send *hrp2* :reset-manip-pose)
  (fullbody-lookat (float-vector 0 0 0))
  (send *ri* :angle-vector (send *hrp2* :angle-vector) 5000)
  (send *ri* :wait-interpolation)
  (call-empty-service "/env_server/start_building")
  (send *hrp2* :head :neck-p :joint-angle 10)
  (send *ri* :angle-vector (send *hrp2* :angle-vector) 5000)
  (send *ri* :wait-interpolation)
  (call-empty-service "/env_server/stop_building")
  (send *hrp2* :reset-manip-pose)
  (send *ri* :angle-vector (send *hrp2* :angle-vector) 5000)
  (send *ri* :wait-interpolation)
  )

(defun look-around-callback (req)
  (do-lookaround)
  (send req :response))

(defun execute-cb (server msg)
  (do-lookaround)
  (let ((result (send server :result)))
    (send server :set-succeeded result))
  )

(setq *action-server* (instance ros::simple-action-server :init
                                "lookaround_ground" jsk_footstep_controller::LookAroundGroundAction
                                :execute-cb 'execute-cb)))
(ros::advertise-service "lookaround_ground" std_srvs::Empty #'look-around-callback)
(ros::spin)
