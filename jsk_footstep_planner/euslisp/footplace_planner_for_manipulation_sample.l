(load "package://jsk_footstep_planner/euslisp/footplace_planner_for_manipulation.l")
(load "package://hrpsys_ros_bridge_tutorials/euslisp/samplerobot-interface.l")
(require "models/arrow-object.l")

(defun setup
  ()
  (samplerobot-init)
  (setq *robot* *sr*)
  (setq *coords-list* (list (make-coords :pos (float-vector 1000 -200 1000) :rpy (list 0 0 0)) (make-coords :pos (float-vector 1000 200 1000) :rpy (list 0 0 0)) (make-coords :pos (float-vector 1000 200 1200) :rpy (list 0 0 0)) (make-coords :pos (float-vector 1000 -200 1200) :rpy (list 0 0 0))))
  (setq *arrow-list* nil)
  (dotimes (i (length *coords-list*))
    (let ((single-arrow (arrow)))
      (send single-arrow :newcoords (elt *coords-list* i))
      (setq *arrow-list* (append *arrow-list* (list single-arrow)))
      )
    )
  (objects (append *arrow-list* (list *robot*)))
  (re-setup)
  )

(defun re-setup
  ()
  (send *robot* :reset-manip-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  )

(defun calc
  ()
  (let
      (mt)
    (setq mt (make-cascoords :coords (send (send (send *robot* :rarm :end-coords) :copy-worldcoords) :translate (float-vector 100 0 0)) :parent (send (send *robot* :rarm :end-coords) :parent)))
    (setq *answer* (fullbody-inverse-kinematics-with-standcoords *robot* *coords-list* :rotation-axis :z :debug t :move-centroid t :move-target (list mt)))
    ;(send *robot* :fix-leg-to-coords (car *answer*))
    )
  )

(defun send-to-robot
  ()
  (let ((dest-coords (car *answer-list*))
        (angle-seq (cadr *answer-list*))
        )
    (send *ri* :go-pos (*0.001 (elt (send dest-coords :pos) 0)) (*0.001 (elt (send dest-coords :pos) 1)) (rad2deg (elt (car (rpy-angle (send dest-coords :rot))) 0))) 
    (send *ri* :angle-vector-sequence angle-seq)
    )
  )

(setup)
(calc)
; (send-to-robot)
