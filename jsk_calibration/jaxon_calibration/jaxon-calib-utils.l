(ros::roseus "makecalibdata")
(setq *tfl* (instance ros::transform-listener :init))

(unix::sleep 1)
;; make camera.wrl
(format t "~%~%;; description for camera.wrl~%")
(progn
  (setq cds (send *tfl* :lookup-transform "HEAD_LINK1" "left_camera_optical_frame"  (ros::time 0)))
  (send cds :rotate pi :x)
  (setq rangle (rotation-angle (send cds :rot)))
  (format t "translation ~9,9F ~9,9F ~9,9F~%" (/ (elt (send cds :worldpos) 0) 1000) (/ (elt (send cds :worldpos) 1) 1000) (/ (elt (send cds :worldpos) 2) 1000))
  (format t "rotation ~9,9F ~9,9F ~9,9F ~9,9F~%" (elt (cadr rangle) 0) (elt (cadr rangle) 1) (elt (cadr rangle) 2) (car rangle))
  )

;; make HEAD_LINK1 -> head_root
(format t "~%;; description for urdf.xacro / HEAD_LINK1 -> head_root~%")
(progn
  (setq cds (send *tfl* :lookup-transform "HEAD_LINK1" "left_camera_optical_frame"  (ros::time 0)))
  ;;(setq calib-cds (send *tfl* :lookup-transform "HEAD_LINK1" "head_root" (ros::time 0)))
  ;;(setq cam-root-offset (send *tfl* :lookup-transform "head_root" "left_camera_optical_frame"  (ros::time 0)))
  (setq cam-root-offset (make-coords :pos #f(0 35 -2) :rpy (list -pi/2 0 -pi/2)))
  (send cds :transform (send cam-root-offset :inverse-transformation))
  (setq rpy-ang (car (send cds :rpy-angle)))
  (format t "xyz=\"~9,9F ~9,9F ~9,9F\"~%" (/ (elt (send cds :worldpos) 0) 1000) (/ (elt (send cds :worldpos) 1) 1000) (/ (elt (send cds :worldpos) 2) 1000))
  (format t "rpy=\"~9,9F ~9,9F ~9,9F\"~%" (elt rpy-ang 2) (elt rpy-ang 1) (elt rpy-ang 0))
  )

;; see hands
(setq *jaxon-see-hands-pose* #f(0.0 0.0 -20.0 40.0 -20.0 0.0 0.0 0.0 -20.0 40.0 -20.0 0.0 0.0 0.0 0.0 0.0 30.0 1.95577 -34.5634 -26.2543 26.4703 -76.5816 3.26925 -1.83683 -21.6092 -1.90393 -34.5097 26.0499 -26.3081 -76.519 5.68488 -5.32062 -21.3715))

;; see hands and feet
(setq *jaxon-see-hands-feet-pose* #f(0.0 0.0 -60.0 15.0 -20.0 0.0 0.0 0.0 -60.0 15.0 -20.0 0.0 0.0 15.0 0.0 0.0 30.0 1.95577 -34.5634 -26.2543 26.4703 -76.5816 3.26925 -1.83683 -21.6092 -1.90393 -34.5097 26.0499 -26.3081 -76.519 5.68488 -5.32062 -21.3715))

;;(load "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon-interface.l")
;;(load "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon_red-interface.l")
