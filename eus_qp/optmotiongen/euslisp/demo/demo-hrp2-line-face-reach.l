(load "package://eus_qp/optmotiongen/euslisp/inverse-kinematics-wrapper.l")
(load "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknts-interface.l")

(defun setup-hrp2-line-environment
  ()
  (setq *robot* (instance hrp2jsknts-robot :init))
  (setq *pipe* (make-cylinder 3 1000))
  (send *pipe* :newcoords
        (make-coords :pos (float-vector 600 -300 800)
                     :rpy (list (deg2rad -30) (deg2rad 30) (deg2rad 30))))
  (send *pipe* :set-color (float-vector 1.0 0.6 0.6))
  (objects (list *robot* *pipe*))
  )
(warn "(setup-hrp2-line-environment)~%")

(defun sample-hrp2-reach-ik-line
  (&key
   (pre-process-func)
   (post-process-func)
   &allow-other-keys
   )
  (setup-hrp2-line-environment)
  (setq *edge* (elt (send *pipe* :edges) 15))
  (send *edge* :put :margin 20) ;; [mm]
  (send *edge* :put :direction-move-target-axis :z)
  (send *robot* :inverse-kinematics-optmotiongen
        *edge*
        :move-target (send *robot* :rarm :end-coords)
        :debug-view t
        :pre-process-func pre-process-func
        :post-process-func post-process-func
        )
  )
(warn "(sample-hrp2-reach-ik-line)~%")

(defun setup-hrp2-face-environment
  ()
  (setq *robot* (instance hrp2jsknts-robot :init))
  (setq *board* (make-cube 300 500 5))
  (send *board* :newcoords
        (make-coords :pos (float-vector 400 -200 1000)
                     :rpy (list (deg2rad 30) (deg2rad -30) (deg2rad -30))))
  (send *board* :set-color (float-vector 1.0 0.6 0.6))
  (objects (list *robot* *board*))
  )
(warn "(setup-hrp2-face-environment)~%")

(defun sample-hrp2-reach-ik-face
  (&key
   (pre-process-func)
   (post-process-func)
   &allow-other-keys
   )
  (setup-hrp2-face-environment)
  (setq *face* (elt (send *board* :faces) 1))
  (send *face* :put :margin 20) ;; [mm]
  (send *face* :put :normal-move-target-axis :-x)
  (send *face* :put :normal-tolerance 10.0) ;; [deg]
  (send *robot* :inverse-kinematics-optmotiongen
        *face*
        :move-target (send *robot* :rarm :end-coords)
        :debug-view t
        :pre-process-func pre-process-func
        :post-process-func post-process-func
        )
  )
(warn "(sample-hrp2-reach-ik-face)~%")
