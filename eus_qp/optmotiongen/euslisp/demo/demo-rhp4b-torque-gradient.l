(load "package://eus_qp/optmotiongen/euslisp/sample/sample-torque-gradient.l")
(require :rhp4b "package://khi_ros_bridge_tutorials/euslisp/rhp4b-interface.l")



(defun demo-rhp4b-torque-gradient
    (&key
     (root-virtual-mode nil)
     (drive-root-virtual? nil)
     )
  (setq *robot* (instance rhp4b-robot :init))
  (when root-virtual-mode
    (setq *robot-env* (instance robot-environment :init :robot *robot* :root-virtual-mode root-virtual-mode)))
  (send *robot* :reset-pose)

  (sample-torque-gradient-common
   :robot
   (if root-virtual-mode (send *robot-env* :robot-with-root-virtual) *robot*)
   :joint-list
   (if root-virtual-mode
       (send *robot-env* :robot-with-root-virtual :joint-list)
     (send *robot* :joint-list))
   :drive-joint-list
   (if drive-root-virtual?
       (send *robot-env* :robot-with-root-virtual :joint-list)
     (send *robot* :joint-list))
   :wrench-list (list (float-vector 10 -15 20 1 2 -3))
   :contact-target-coords-list (list (send *robot* :rarm :end-coords))
   :contact-attention-coords-list (list (send *robot* :larm :end-coords))
   )
  )

(defun demo-rhp4b-torque-gradient-all
    ()
  (demo-rhp4b-torque-gradient)
  (demo-rhp4b-torque-gradient :root-virtual-mode :6dof)
  (demo-rhp4b-torque-gradient :root-virtual-mode :6dof :drive-root-virtual? t)
  )
(warn "(demo-rhp4b-torque-gradient-all)~%")
