;; -*- mode: lisp;-*-

(require :pr2-interface "package://pr2eus/pr2-interface.l")
(require :robot-teleop-interface "package://eus_teleop/euslisp/lib/robot-teleop-interface.l")
(require :robot-move-base-teleop-interface "package://eus_teleop/euslisp/lib/robot-move-base-teleop-interface.l")
(require :pr2-teleop-interface "package://eus_teleop/euslisp/lib/pr2-teleop-interface.l")
(require :robot-vive-interface "package://eus_teleop/euslisp/lib/robot-vive-interface.l")
(require :robot-move-base-vive-interface "package://eus_teleop/euslisp/lib/robot-move-base-vive-interface.l")


(defclass pr2-vive-interface
  :super robot-vive-interface
  :slots ())


(defmethod pr2-vive-interface
  (:init (&rest args)
    (prog1
      (send-super* :init args)
      ;; for vive
      (send self :set-val 'scale 1.5)
      (send self :set-val 'trigger-wait-count 5)
      (send self :set-val 'menu-wait-count 5)
      (send self :set-val 'trackpad-wait-count 5)
      (send self :set-val 'grip-wait-count 5))))


(defun pr2-vive-init (&key (loop-enable t))
  (pr2-init)
  (make-pr2-irtviewer :no-window t)
  (make-pr2-camera-model :no-window t)
  (setq *ti* (instance pr2-vive-interface :init :loop-enable loop-enable))
  (send *ti* :ros-init))
