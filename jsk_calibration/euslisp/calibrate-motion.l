#!/usr/bin/env runeus

;; for old version
(when (and (boundp '*use-hrpsys-ros-bridge*)
           (not *use-hrpsys-ros-bridge*))
  #-:jsk
  (jsk)
  #-:rbrain-basic
  (rbrain))

;; for new version
(defvar *use-hrpsys-ros-bridge* t)

;;(unix::putenv "ROBOT=HRP2JSK")
;;(unix::putenv "HRP2NO=7")
;;(unix::putenv "ROS_MASTER_URI=http://hrp2007v:10007")

;;(unix::putenv "ROBOT=HRP2JSKNT")
;;(unix::putenv "HRP2NO=16")
;;(unix::putenv "ROS_MASTER_URI=http://hrp2016v:10016")

;; (unix::putenv "ROBOT=HRP2JSKNTS")
;; (unix::putenv "HRP2NO=17")
;; (unix::putenv "ROS_MASTER_URI=http://hrp2017v:10017")

;;(unix::putenv "ROBOT=HRP2W")
;;(unix::putenv "HRP2NO=8")

;;(setq ros::*compile-message* t)
;;(ros::roseus-add-msgs "roslib")
(ros::roseus-add-msgs "std_msgs")
(ros::roseus-add-msgs "geometry_msgs")
(ros::roseus-add-msgs "posedetection_msgs")
;;(ros::roseus-add-srvs "checkerboard_pose")

(setq *use-camera-idx* 0)
;;;

(defun vector-mean (lst)
  (let ((ret (instantiate float-vector (length (car lst)))))
    (dolist (l lst)
      (v+ ret l ret))
    (scale (/ 1.0 (length lst)) ret)))

(defun setup-ros ()
  (ros::roseus "call_pose")
  (ros::subscribe "/checkerdetector/ObjectDetection"
		  posedetection_msgs::ObjectDetection
		  #'object-detection-cb)
  )
  ;;(ros::wait-for-service "get_checkerboard_pose"))

;; (setq *req* (instance checkerboard_pose::GetCheckerboardPoseRequest :init))
;; (send *req* :corners_x 5)
;; (send *req* :corners_y 4)
;;(send *req* :spacing_x 0.025)
;;(send *req* :spacing_y 0.025)
;; (send *req* :spacing_x 0.030)
;; (send *req* :spacing_y 0.030)

;; normal board
;;(send *req* :corners_x 8)
;;(send *req* :corners_y 6)
;;(send *req* :spacing_x 0.108)
;;(send *req* :spacing_y 0.108)

;;
;; ToDo: add snapshotter for making bugfile of calibration process
;;

;;
;; HRP2 settings
;;
(defun setup-model ()
  (unless *use-hrpsys-ros-bridge*
    (unless (boundp '*pickview*) (pickview :no-menu t))
    (unless (boundp '*hrp2*)
      (cond
       ((string= "HRP2JSKNTS" (unix:getenv "ROBOT")) (hrp2jsknts-simple))
       ((string= "HRP2JSKNT" (unix:getenv "ROBOT")) (hrp2jsknt-simple))
       ((substringp "HRP2W"(unix:getenv "ROBOT"))
	(cond ((string= "8" (unix:getenv "HRP2NO"))
	       (hrp2vz-simple))
	      (t (hrp2v-80-simple))))
       ((string= "HRP2JSK" (unix:getenv "ROBOT")) (hrp2jsk-simple))
       )
      (objects (list *hrp2*)))
    )
  (when *use-hrpsys-ros-bridge*
    (cond
     ((string= "HRP2JSKNTS" (unix:getenv "ROBOT"))
      (load "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknts-interface.l")
      (hrp2jsknts-init)
      (setq *hrp2* *hrp2jsknts*))
     ((string= "HRP2JSKNT" (unix:getenv "ROBOT"))
      (load "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknt-interface.l")
      (hrp2jsknt-init)
      (setq *hrp2* *hrp2jsknt*)))
    )
  (objects (list *hrp2*))
  )

(defun hand-hook-pose (&optional arm)
  (if *use-hrpsys-ros-bridge*
      (progn
	(unless arm (setq arm :arms))
	(send *hrp2* :hand arm :hook-pose)
	(send *ri* :hand-angle-vector (send *hrp2* :hand-angle-vector))
	)
    (send *ci* :hand-hook-pose arm)
    ))
(defun hand-start-grasp (arm)
  (if *use-hrpsys-ros-bridge*
      (progn
	(send *hrp2* :hand arm :grasp-pose)
	(send *ri* :hand-angle-vector (send *hrp2* :hand-angle-vector))
	)
    (send *ci* :hand-start-grasp arm)
    ))

(defun hrp2-camera-coords ()
  (if *use-hrpsys-ros-bridge*
      (send (send *hrp2* :carmine) :copy-worldcoords)
    (send *hrp2* :head :camera :copy-worldcoords))
  )

(defun setup-real ()
  (if *use-hrpsys-ros-bridge*
      (progn
        (cond 
         ((string= "HRP2JSKNTS" (unix:getenv "ROBOT"))
          (load "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknts-interface.l")
          (hrp2jsknts-init)
          (setq *hrp2* *hrp2jsknts*))
         ((string= "HRP2JSKNT" (unix:getenv "ROBOT"))
          (load "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknt-interface.l")
          (hrp2jsknt-init)
          (setq *hrp2* *hrp2jsknt*)))
	(setq *ci* *ri*)
	(objects (list *hrp2*))
	(setq *pickview* *irtviewer*)
	)
    (progn
      (unless (boundp '*hrp2*) (setup-model))
      (load (format nil "~A/prog/hrp2/corba/orbixif.l" (unix:getenv "HOME")))
      (load (format nil "~A/prog/hrp2/plugins/plugins.l" (unix:getenv "HOME")))
      (eusorbix-init))
    ))

(defun set-ref-angle ()
  (send *hrp2* :angle-vector (send *ci* :state :reference-vector)))

(defun set-pot-angle ()
  (send *hrp2* :angle-vector (send *ci* :state :potentio-vector)))

(defun proc-wait (tm &key (timer)
                     (func #'proc-3dpos))
  (let ((st (if timer (send timer :start)
              (send (setq timer (instance mtimer :init)) :start)))
        )
    (while (< (* 1000 (send timer :stop)) tm)
      (funcall func))))

(defun move-hrp2 (&key (real t) (tm 4000) (wait)
                       ;;(seq-lst (list "seq"))
                       (seq-lst (list "or"))
                       (func))
  (if *use-hrpsys-ros-bridge*
      (send *ci* :angle-vector (send *hrp2* :angle-vector) tm)
    (dolist (seq seq-lst)
      (send *ci* :angle-vector (send *hrp2* :angle-vector) tm seq)))
  (send *pickview* :draw-objects)
  (when wait
    (if func (proc-wait (* 2 tm) :func func))
    (if *use-hrpsys-ros-bridge*
	(send *ci* :wait-interpolation)
      (dolist (seq seq-lst)
	(send *ci* :wait-interpolation seq))))
  )

(defun setup-calib-motion ()
  (load "make-calibration-pose.l")
  (make-pose-list))

;;
;; motion definition
;;
(defun calibrate-motion-hrp2 (&key (arm :larm) (proc-func))
  (let ((lst (case arm (:larm *larm-pose-list*) (:rarm *rarm-pose-list*))))
    (dolist (r lst)
      (set-hrp2-pose r)
      (when proc-func
        (unless (funcall proc-func)
          (return-from calibrate-motion-hrp2))))
    t))

;;
;; chess board proc
;;
(setq *detect-pose-times* 4)
(setq *wait-ms* 0)
(setq *lst* nil)
(defun wait-for-new-detection-msg ()
  (setq *detection-msg* nil)
  (let ((now (ros::time-now)))
    (while (and (< (send (ros::time- (ros::time-now) now) :to-sec) 1)
                (not
                 (and *detection-msg*
                      (> (send (ros::time- (send *detection-msg* :header :stamp) now) :to-sec) 0))))
      (ros::spin-once)
      (ros::sleep))))

(defun detect-pose ()
  ;; (let (res ret)
  ;;   (setq res (ros::service-call "get_checkerboard_pose" *req*))
  ;;   (let ((bpose (send res :board_pose)))
  ;;     (if bpose
  ;;         (ros::ros-info "detected checkerboard")
  ;;       (ros::ros-info "didn't detect checkerboard"))
  ;;     (setq ret (ros::tf-pose->coords (send bpose :pose))))
  ;;   ret)
  ;; run spin-once 3 sec
  (wait-for-new-detection-msg)
  (when *detection-msg*
    ;; one more new message
    (wait-for-new-detection-msg)
    (if (and *detection-msg* (> (length (send *detection-msg* :objects)) 0))
        (progn
          (format t "found checker board~%")
          (ros::tf-pose->coords (send (elt (send *detection-msg* :objects) 0) :pose))))
    ))

(defun view-detect-chessboard ()
  (let ((cds (detect-pose))
        )
    (unless (boundp '*arrow*)
      (setq *arrow* (make-arrow 150))
      (objects (list *hrp2* *arrow*) *pickview* :fix t)
      )
    (when cds
      (let ((cam (send *hrp2* :head :camera :copy-worldcoords)))
        (send cam :transform cds)
        (send *arrow* :newcoords cam)
        (send *arrow* :worldcoords)))
    (set-pot-angle)
    (send *pickview* :draw-objects)
    ))

(defun object-detection-cb (msg)
  (setq *detection-msg* msg)
  )

(defun view-update2 (&key (real t) (confirm) (tm 1600) (wait t)
                          (vision-wait *wait-ms*) (times *detect-pose-times*))
  (if (boundp '*pickview*) (send *pickview* :draw-objects))
  (if (if confirm (y-or-n-p) t)
      (move-hrp2 :real real :tm tm :wait wait)
    ;;(move-hrp2 :seq-lst (list "or") :tm tm :wait wait)
    (return-from view-update2 nil))
  (if vision-wait (unix::usleep (* vision-wait 1000)))
  (let (av-lst cds-lst av cds (diff 0.0))
    (dotimes (i times)
      (let ((r (detect-pose)))
	(when r
	  (push (send *ci* :state :potentio-vector) av-lst)
	  (push r cds-lst)))
      (unix::usleep (* 1000 150)) ;; sleep 150ms (7.5fps)
      )
    ;; average
    (when (= (length av-lst) times)
    (setq cds
          (make-coords :pos (vector-mean (mapcar #'(lambda (x) (send x :pos)) cds-lst))
                       :rot (matrix-exponent
                             (vector-mean (mapcar #'(lambda (x) (matrix-log (send x :rot))) cds-lst)))))
    (setq av (vector-mean av-lst))
    ;; validity check
    (dolist (c cds-lst)
      (incf diff
            (+ (norm (send cds :difference-position c))
               (* 150 (norm (send cds :difference-rotation c))))))
    (if (< (/ diff times)
           16.0)
           ;; 2.0)
        (push (list cds av) *lst*))
    ))
  t)

;;
;; calibration main
;;
(defun calibrate-exec (&key (arm) (fname "hrp2-calibration-result.l"))
  (setq *lst* nil)
  (calibrate-motion-hrp2
   :proc-func #'view-update2 :arm arm)
  (dump-structure fname *lst*))

(defun calibration-exec-all (&optional (prefix ""))
  (unless (y-or-n-p "Do you want to start calibration?")
    (return-from calibration-exec-all))
  (cond
   ((or (string= (unix::getenv "ROBOT") "HRP2JSKNT")
	(string= (unix::getenv "ROBOT") "HRP2JSKNTS"))
    (hand-hook-pose :larm))
   (t
    (send *hrp2* :larm :thumb-r :joint-angle -45)
    (move-hrp2 :wait t)))

  (unless (y-or-n-p "Please set the chessboard on the left hand, then input y")
    (return-from calibration-exec-all))
  (cond
   ((or (string= (unix::getenv "ROBOT") "HRP2JSKNT")
	(string= (unix::getenv "ROBOT") "HRP2JSKNTS"))
    (hand-start-grasp :larm))
   (t
    (send *ci* :start-grasp :larm)))

  (unless (y-or-n-p "Do you want to start calibration of larm")
    (return-from calibration-exec-all))
  (calibrate-exec :arm :larm :fname (format nil "~Ahrp2-calibration-result-l.l" prefix))

  (send *hrp2* :reset-pose)
  (move-hrp2 :wait t)

  (unless (y-or-n-p "HRP2 will release the chessboard from the left hand. Are you ready?")
    (return-from calibration-exec-all))
  (cond
   ((or (string= (unix::getenv "ROBOT") "HRP2JSKNT")
	(string= (unix::getenv "ROBOT") "HRP2JSKNTS"))
    (hand-hook-pose))
   (t
    (send *hrp2* :larm :thumb-r :joint-angle -45)
    (send *hrp2* :rarm :thumb-r :joint-angle -45)
    (move-hrp2 :wait t)
    (send *ci* :stop-grasp)))
  #|
  (unless (y-or-n-p "Do you want to start calibration?")
  (return-from calibration-exec-all))
  (cond
  ((string= (unix::getenv "ROBOT") "HRP2JSKNTS")
  (send *ci* :hand-hook-pose :rarm))
  (t
  (send *hrp2* :rarm :thumb-r :joint-angle -45)
  (move-hrp2 :wait t)))
  |#
  (unless (y-or-n-p "Please set chess board on the right hand, then input y")
    (return-from calibration-exec-all))
  (cond
   ((or (string= (unix::getenv "ROBOT") "HRP2JSKNT")
	(string= (unix::getenv "ROBOT") "HRP2JSKNTS"))
    (hand-start-grasp :rarm))
   (t
    (send *ci* :start-grasp :rarm)))

  (unless (y-or-n-p "Do you want to start calibration of rarm")
    (return-from calibration-exec-all))
  (calibrate-exec :arm :rarm :fname (format nil "~Ahrp2-calibration-result-r.l" prefix))

  (send *hrp2* :reset-pose)
  (move-hrp2 :wait t)

  (unless (y-or-n-p "HRP2 will release the chessboard from the right hand. Are you ready?")
    (return-from calibration-exec-all))
  (cond
   ((or (string= (unix::getenv "ROBOT") "HRP2JSKNT")
	(string= (unix::getenv "ROBOT") "HRP2JSKNTS"))
    (hand-hook-pose))
   (t
    (send *hrp2* :larm :thumb-r :joint-angle -45)
    (send *hrp2* :rarm :thumb-r :joint-angle -45)
    (move-hrp2 :wait t)
    (send *ci* :stop-grasp)))
  )
(warn "
;;
;; before exec this file
;; $ roslaunch checkerboard_pose test.launch
;;; or
;; $ rosrun checkerboard_pose cb_detector.py image_stream:=/HRP2016V/wide_stereo/left/image_rect camera_info:=/HRP2016V/wide_stereo/left/camera_info
;;
;;(setup-calib-motion)
;;(calibration-exec-all)
;;
")
