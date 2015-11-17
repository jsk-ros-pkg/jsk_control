#!/usr/bin/env roseus

(ros::roseus "gui_client")
(ros::roseus-add-srvs "std_srvs")
(load "package://jsk_footstep_controller/euslisp/util.l")
(setq *robot-name* (ros::get-param "/robot/type" (unix::getenv "ROBOT")))
(load (robot-interface-file *robot-name*))
(init-robot-from-name *robot-name*)

(defun walking-pose
  (robot
   &key (root-link-height-offset 0)
        (root-link-pitch-offset 0)
        (head-link-pitch-offset 0)
        (root-link-roll-offset 0)
        (chest-link-pitch-offset 0)
        (chest-link-roll-offset 0)
        (fix-coords (make-coords))
        (default-pose-method :reset-manip-pose))
  "Generate and set walking pose.
   default-pose-method is initial pose, reset-manip-pose by default.
   Generated pose is near from default-pose-method pose.
   root-link-height-offset is root height offset [mm] from default-pose-method.
   root-link-pitch-offset and root-link-roll-offset are root pitch and roll offset [deg] from default-pose-method."
  (send robot default-pose-method)
  (send robot :fix-leg-to-coords fix-coords)
  (let ((lc (mapcar #'(lambda (l)
                        (send robot l :end-coords :copy-worldcoords))
                    '(:rleg :lleg))))
    (send robot :move-coords
          (send
           (send
            (send (send (car (send robot :links)) :copy-worldcoords)
                  :translate (float-vector 0 0 root-link-height-offset))
            :rotate (deg2rad root-link-pitch-offset) :y)
           :rotate (deg2rad root-link-roll-offset) :x)
          (car (send robot :links)))
    (if (find-method robot :torso-waist-p) (send robot :torso-waist-p :joint-angle chest-link-pitch-offset))
    (if (find-method robot :torso-waist-r) (send robot :torso-waist-r :joint-angle chest-link-roll-offset))
    (mapcar #'(lambda (l c)
                (send robot l :inverse-kinematics c))
            '(:rleg :lleg) lc)
    (send robot :move-centroid-on-foot :both '(:rleg :lleg))
    (if (find-method robot :head-neck-p) (send robot :head-neck-p :joint-angle head-link-pitch-offset))
    (send robot :angle-vector)
    ))


(defun reset-pose-callback (req)
  (send *robot* :reset-pose)
  (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
  (send req :response))

(defun reset-walking-pose-callback (req)
  (send *robot* :reset-pose)
  (walking-pose *robot* :root-link-height-offset -40
                :head-link-pitch-offset 35)
  (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
  (send req :response))

(defun start-abc-callback (req)
  (send *ri* :start-auto-balancer)
  (send req :response))

(ros::advertise-service "/footstep_command/reset_walking_pose" std_srvs::Empty #'reset-walking-pose-callback)
(ros::advertise-service "/footstep_command/reset_pose" std_srvs::Empty #'reset-pose-callback)
(ros::advertise-service "/footstep_command/start_abc" std_srvs::Empty #'start-abc-callback)
(ros::spin)
