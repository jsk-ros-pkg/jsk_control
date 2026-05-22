#!/usr/bin/env roseus
(ros::roseus "jsk_generic_g1_spacenav_teleop")
(require "package://g1eus/g1-interface.l")
(require "package://jsk_generic_teleop/euslisp/jsk-generic-teleop.l")

(g1-init)

(defvar *g1-spacenav-grasp-state-alist*
  '((:larm . nil) (:rarm . nil)))

(defun g1-spacenav-grasp-closed-p (arm)
  (jgt-alist-get arm *g1-spacenav-grasp-state-alist* nil))

(defun g1-spacenav-set-grasp-closed (arm closed)
  (setq *g1-spacenav-grasp-state-alist*
        (jgt-alist-set arm closed *g1-spacenav-grasp-state-alist*)))

(defun g1-spacenav-toggle-grasp (arm)
  (if (g1-spacenav-grasp-closed-p arm)
      (progn
        (send *ri* :prepare-grasp arm)
        (g1-spacenav-set-grasp-closed arm nil)
        (ros::ros-info "prepare-grasp: ~A" arm))
    (progn
      (send *ri* :start-grasp arm)
      (g1-spacenav-set-grasp-closed arm t)
      (ros::ros-info "start-grasp: ~A" arm))))

(defun g1-spacenav-handle-arm-button (button-index arm)
  (when (and arm
             (jgt-button-on-p *jgt-relative-joy* button-index)
             (not (jgt-button-on-p *jgt-relative-prev-joy* button-index)))
    (if (eq *jgt-active-arm* arm)
        (g1-spacenav-toggle-grasp arm)
      (progn
        (setq *jgt-active-arm* arm)
        (ros::ros-info "active teleop arm: ~A" *jgt-active-arm*)))))

;; Override the generic spacenav button behavior for G1:
;; opposite-side button switches the teleop arm, current-side button opens/closes
;; the active hand.  With :arms '(:larm :rarm), button 0 is :larm and button 1 is
;; :rarm.
(defun jgt-select-relative-arm ()
  (when (and *jgt-relative-joy* *jgt-relative-prev-joy*)
    (g1-spacenav-handle-arm-button 0 (car *jgt-arms*))
    (g1-spacenav-handle-arm-button 1 (cadr *jgt-arms*))))

(jsk-generic-teleop
 :robot *g1*
 :arms '(:larm :rarm)
 :device-type :spacenav
 :init-pose nil
 :origin-offsets nil
 :active-arm :rarm
 :ratio 0.5
 :relative-translation-scale 30.0
 :relative-rotation-scale (/ pi 24.0)
 :relative-deadzone 0.2
 :spacenav-topic "/spacenav/joy"
 :send-time 200
 :rate 30
 :debug-view t)
