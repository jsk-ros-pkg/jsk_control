#!/usr/bin/env roseus

(load "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknts-interface.l")
(ros::roseus "head_controller")

(hrp2jsknts-init)
(setq *robot* *hrp2jsknts*)

;; 首振り動作はrtabmapとの相性が良くない
;; (setq *head-av-list* (list (float-vector 0 0)
;;                            (float-vector 40 20)
;;                            (float-vector 0 40)
;;                            (float-vector -40 20)))
(setq *head-av-list* (list (float-vector 0 40)))

(setq *av-id* 0)

(ros::rate 5)
(while (ros::ok)
  (send *ri* :spin-once)
  (unless (send *ri* :interpolatingp :head-controller)
    (incf *av-id*)
    (when (>= *av-id* (length *head-av-list*))
      (setq *av-id* 0))
    (send *robot* :head :angle-vector (elt *head-av-list* *av-id*))
    (send *ri* :angle-vector (send *robot* :angle-vector) 5000 :head-controller)
    )
  (ros::sleep)
  )
