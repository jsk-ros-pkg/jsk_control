#!/usr/bin/env roseus

(ros::roseus "lookaround_ground")

(setq *robot-name* (ros::get-param "~robot" (string-downcase (unix::getenv "ROBOT"))))

(ros::roseus-add-msgs "jsk_footstep_msgs")
(ros::roseus-add-srvs "std_srvs")
(ros::roseus-add-msgs "std_msgs")
(ros::roseus-add-msgs "jsk_footstep_controller")

(load (format nil "package://hrpsys_ros_bridge_tutorials/euslisp/~A-interface.l" 
              (string-downcase *robot-name*)))
(load "package://drc_task_common/euslisp/test-drc-terrain-walk.l")
(init-robot-from-name *robot-name*)


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

(defun call-empty-service (srv)
  (let ((req (instance std_srvs::EmptyRequest :init)))
    (ros::service-call srv req)))

(defun do-lookaround-old ()
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
  (send *ri* :angle-vector (send *hrp2* :angle-vector) 10000)
  (send *ri* :wait-interpolation)
  )

(defun do-lookaround-jaxon ()
  (walking-pose *robot*
                :default-pose-method :reset-pose
                :root-link-height-offset -80
                :root-link-pitch-offset 20
                :chest-link-pitch-offset 20)
  ;; (walking-pose *robot*
  ;;               :default-pose-method :narrow-width-pose
  ;;               :root-link-height-offset -40
  ;;               :root-link-pitch-offset 20
  ;;               :chest-link-pitch-offset 20)

  (send *robot* :head :neck-p :joint-angle 35)
  ;;(send *robot* :torso :waist-p :joint-angle 35) ; it's not working because of model bug
  (send *ri* :angle-vector (send *robot* :angle-vector) 5000)
  (send *ri* :wait-interpolation)
  (if (ros::service-exists "/tilt_laser_listener/clear_cache")
      (call-empty-service "/tilt_laser_listener/clear_cache")
    )
  ;; Call /locomotion/input/request
  (call-empty-service "/locomotion/input/request")
  ;; Wait for next publish
  (wait-for-snapshot-publish)
  (send *robot* :reset-pose)
  ;; (walking-pose *robot*
  ;;               :default-pose-method :narrow-width-pose
  ;;               :root-link-height-offset -140
  ;;               :root-link-pitch-offset 20
  ;;               :chest-link-pitch-offset 20)

  (send *ri* :angle-vector (send *robot* :angle-vector) 5000)
  (send *ri* :wait-interpolation)
  )

#|
(walking-pose *jaxon*
              :default-pose-method :reset-pose
              :root-link-height-offset 0
              :root-link-pitch-offset 20
              :chest-link-pitch-offset 20)
|#

(defun do-lookaround ()
  ;; (send *robot* :reset-pose)
  ;; (send *ri* :angle-vector (send *robot* :angle-vector) 5000)
  ;; (send *ri* :wait-interpolation)
  (cond ((or (string= *robot-name* "jaxon")
             (string= *robot-name* "JAXON")
             (string= *robot-name* "JAXON_RED")
             (string= *robot-name* "jaxon_red"))
         (do-lookaround-jaxon))
        (t
         (send *robot* :torso :waist-p :joint-angle 40)
         (send *robot* :head :neck-y :joint-angle 0)
         (send *robot* :head :neck-p :joint-angle 40)
         (if (or (string= *robot-name* "HRP2JSK")
                 (string= *robot-name* "hrp2jsk"))
             (send *robot* :legs :move-end-pos (float-vector 0 0 -40)))
         (send *robot* :move-centroid-on-foot :both '(:lleg :rleg))
         (send *ri* :angle-vector (send *robot* :angle-vector) 5000)
         (send *ri* :wait-interpolation)
         ;; (unix:sleep 5)
         ;; Call /tilt_laser_listener/clear_cache
         (if (ros::service-exists "/tilt_laser_listener/clear_cache")
             (call-empty-service "/tilt_laser_listener/clear_cache")
           )
         ;; Call /locomotion/input/request
         (call-empty-service "/locomotion/input/request")
         ;; Wait for next publish
         (wait-for-snapshot-publish)
         ;;(read-line)
         (send *robot* :reset-pose)
         (send *ri* :angle-vector (send *robot* :angle-vector) 5000)
         (send *ri* :wait-interpolation)
         )))

(defun look-around-callback (req)
  (do-lookaround)
  (send req :response))

(defun execute-cb (server msg)
  (do-lookaround)
  (let ((result (send server :result)))
    (send server :set-succeeded result))
  )

(defun timestamp-callback (msg)
  (setq *latest-timestamp* msg))

(defun wait-for-snapshot-publish ()
  (ros::spin-once)
  (setq *latest-timestamp* nil)
  (while (and (null *latest-timestamp*) (ros::ok))
    (ros::spin-once)
    (ros::sleep))
  )

(ros::subscribe "/locomotion/input/output/stamp" std_msgs::Time #'timestamp-callback)
(setq *action-server* (instance ros::simple-action-server :init
                                "lookaround_ground" jsk_footstep_controller::LookAroundGroundAction
                                :execute-cb 'execute-cb)))
(ros::advertise-service "lookaround_ground" std_srvs::Empty #'look-around-callback)
(ros::spin)
