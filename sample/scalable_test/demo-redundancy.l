#!/usr/bin/env roseus

(setq *load-as-lib* t)
(require "demo-table-standup.l")

(require "learn.l")
(require "package://eus_robot/euslisp/eus_robot_ik.l")
(require "package://eus_robot/euslisp/eus_qp_collide.l")

(send *robot* :reset-pose)
(send *robot* :fix-leg-to-coords (make-coords) :rleg)
(send-all (send *robot* :links) :worldcoords)

;; setup ik solver
(send *robot* :legs :toe-p :min-angle 0)
(send *robot* :legs :toe-p :max-angle 0)
;;
(send-all (send user::*robot* :links) :worldcoords)
(eus_robot::clear-links)
(eus_robot::copy-robot :set-parameter-for-eus-robot-args (list :root-limb :rleg))
(eus_robot::forward-kinematics 0)
(eus_robot::print-link-tree)
(eus_robot_ik::clear-all)
(eus_robot_ik::add-target :limb :larm)
(eus_robot_ik::add-target :limb :rarm)
(eus_robot_ik::add-target :limb :lleg)
(eus_robot::configuration-copy-to :robot user::*robot*)
;;
(send *robot* :legs :toe-p :min-angle -60)
(send *robot* :legs :toe-p :max-angle 16)
(setq *links* (mapcar #'(lambda (j) (find-if #'(lambda (l) (eq (send l :get :joint) j)) (send *robot* :get :links))) *joint-list1*))

;; setup collision detection
(eus_qp_collide::test-gen-collidable-pairs-old)
(setq *collidable-pairs* eus_qp_collide::*collidable-pairs*)
(eus_qp_collide::add-all-link-convex *robot*)
(eus_qp_collide::gen-all-convex-matrices)
(eus_robot::register-collide-pairs *collidable-pairs*)

;; dnn setup
(smart-initialize-net)
(setq *input* (get-vector-from-ef-coords :ef-coords *ef-coords* :root-link *root-link*))
(setq *redundancy* (instantiate float-vector (- (caffe::get-blob-count "input")
						(length *input*))))
(setq *input* (concatenate float-vector *input* *redundancy*))

;; setup viewer
(send *irtviewer* :change-background (float-vector 1 1 1))
(objects (cons *robot* *arrow*))

(defun random-demo-ik
  (&rest
   args
   &key
   (joint-list *joint-list1*)
   (random-angle
    (let* ((ret
	    (mapcar '(lambda (j) (send j :joint-angle
				       (-
					(+ (send j :min-angle)
					   (* (random 1.0)
					      (- (send j :max-angle)
						 (send j :min-angle))))
					(send j :joint-angle))
				       :relative t))
		    joint-list)))
      (send-all (send-all joint-list :child-link) :worldcoords)
      (send *robot* :fix-leg-to-coords (make-coords) :rleg)
      ret))
   (update-arrow-coords
    (mapcar '(lambda (ar c) (send ar :newcoords (send c :copy-worldcoords)))
	    *arrow* *ef-coords*))
   &allow-other-keys)
  (send *viewer* :draw-objects)
  (setq *support-face* nil)
  (cpplog::change-output-stream "/dev/null")
  (apply 'demo-ik :check-balance? nil :support-face nil :ik-args (list :max 30) args)
  (cpplog::change-output-stream "")
  )
