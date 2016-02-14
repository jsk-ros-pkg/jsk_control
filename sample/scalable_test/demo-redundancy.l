#!/usr/bin/env roseus

(setq *load-as-lib* t)
(require "demo-table-standup.l")

(defun demo-setup
  nil
  (require "learn.l")
  (require "package://eus_robot/euslisp/eus_robot_ik.l")
  (require "package://eus_robot/euslisp/eus_qp_collide.l")
  ;;
  (send-all (send *robot* :joint-list) :set-val 'joint-min-max-table nil)
  (send-all (send *robot* :joint-list) :set-val 'joint-min-max-target nil)
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords) :rleg)
  (send-all (send *robot* :links) :worldcoords)
  ;;
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
  ;;
  ;; setup collision detection
  (eus_qp_collide::test-gen-collidable-pairs-old)
  (setq *collidable-pairs* eus_qp_collide::*collidable-pairs*)
  (eus_qp_collide::add-all-link-convex *robot*)
  (eus_qp_collide::gen-all-convex-matrices)
  (eus_robot::register-collide-pairs *collidable-pairs*)
  ;;
  ;; dnn setup
  (smart-initialize-net)
  (setq *input* (get-vector-from-ef-coords :ef-coords *ef-coords* :root-link *root-link*))
  (setq *redundancy* (instantiate float-vector (- (caffe::get-blob-count "input")
						  (length *input*))))
  (setq *input* (concatenate float-vector *input* *redundancy*))
  ;;
  ;; setup viewer
  (send *irtviewer* :change-background (float-vector 1 1 1))
  (objects (cons *robot* *arrow*))
  )

(defun demo-reset-pose-redundancy
  (&rest
   args
   &key
   (step-max 100)
   (check-balance? nil)
   (ik-args
    (list
     :mode :all
     :step-max step-max
     :traj? nil
     :support-limbs '(:rleg :lleg)
     :check-balance? check-balance?
     :ik-args (list :max 15 :pthre (* 1e-3 10) :rthre (deg2rad 5))
     )
    )
   (reset-pose (copy-seq (send *robot* :reset-pose)))
   (cnt-buf (instantiate integer-vector 6))
   (tm-buf (instantiate float-vector 6))
   tm
   )
  (fill cnt-buf 0)
  (fill tm-buf 0)
  (apply 'demo-random-redundancy-check
	 :event-cnt cnt-buf
	 :event-tm tm-buf
	 :draw? nil
	 :random-angle
	 (progn (send *robot* :angle-vector (copy-seq reset-pose))
		(send *robot* :fix-leg-to-coords (make-coords) :rleg)
		(send-all (send *robot* :links) :worldcoords)
		(send *irtviewer* :objects
		      (append ;; (list (copy-object *robot*))
			      *arrow*
			      (remove-if '(lambda (a) (subclassp (class a) robot-model))
					 (send *irtviewer* :objects))))
		(copy-seq (send *robot* :angle-vector)))
	 (append args ik-args))
  (apply 'demo-random-redundancy-check
	 :draw? t
	 :random-angle
	 (progn (send *robot* :angle-vector (copy-seq reset-pose))
		(send *robot* :fix-leg-to-coords (make-coords) :rleg)
		(send-all (send *robot* :links) :worldcoords)
		(send *irtviewer* :objects
		      (append (list (copy-object *robot*))
			      *arrow*
			      (remove-if '(lambda (a) (subclassp (class a) robot-model))
					 (send *irtviewer* :objects))))
		(copy-seq (send *robot* :angle-vector)))
	 :user-valid-check
	 #'(lambda (&rest args)
	     (eus_robot::configuration-copy-from)
	     (let* ((rl (remove-if-not '(lambda (r) (subclassp (class r) robot-model))
				       (send *irtviewer* :objects)))
		    (ds (mapcar '(lambda (r) (norm2 (v- (send r :angle-vector)
							(send *robot* :angle-vector))))
				rl))
		    (min-ds (apply 'min ds)) min-r)
	       (cond
		((> min-ds 10000)
		 ;; (setq min-r (cdr (find-if #'(lambda (dr) (eq (car dr) min-ds))
		 ;; (mapcar 'cons ds rl))))
		 (send *irtviewer* :objects
		       (append
			(list (copy-object *robot*))
			(send *irtviewer* :objects)))
		 (send *viewer* :draw-objects)
		 )))
	     t)
	 (append args ik-args))
  (warning-message 6 "DNN/IK/COG/COL/TREE/TOTAL = ~A(~A)/~A(~A)/~A(~A)/~A(~A)/~A(~A)/~A~%"
		   (aref tm-buf 0) (aref cnt-buf 0)
		   (aref tm-buf 1) (aref cnt-buf 1)
		   (aref tm-buf 2) (aref cnt-buf 2)
		   (aref tm-buf 3) (aref cnt-buf 3)
		   (aref tm-buf 4) (aref cnt-buf 4)
		   (aref tm-buf 5))
  )

(defun demo-random-redundancy-check
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
  (let* ((ret (apply 'demo-ik args)))
    (cpplog::change-output-stream "")
    ret)
  )

#|

(demo-setup)

(demo-reset-pose-redundancy :check-balance? nil)
(send *viewer* :viewsurface :write-to-image-file "4limb_ik_redundancy_reset_pose.jpg")
;; 30/100 samples are valid, traj=nil
;; DNN/IK/COG/COL/TREE/TOTAL = 0.020883(100)/0.040057(100)/0.0(0)/0.01819(70)/0.0(0)/0.090259

(demo-reset-pose-redundancy :check-balance? t)
(send *viewer* :draw-objects)
(send *support-face* :draw-on :flush t :width 5 :color (float-vector 0.3 1 0.3)))
(send *viewer* :viewsurface :write-to-image-file "4limb_ik_redundancy_reset_pose_with_centroid.jpg")
;; 9/100 samples are valid, traj=nil
;; DNN/IK/COG/COL/TREE/TOTAL = 0.019434(100)/0.039366(100)/0.001511(70)/0.006923(24)/0.0(0)/0.0737

(require "demo-egress-table.l")
(eus_qp_collide::clear-convex)
(demo-setup)
(send *irtviewer* :objects (append *arrow* (send *irtviewer* :objects)))
;;
(demo-reset-pose-redundancy :check-balance? nil :reset-pose (copy-seq (send *robot* :angle-vector)))
;; 15/100 samples are valid, traj=nil
;; DNN/IK/COG/COL/TREE/TOTAL = 0.021968(100)/0.040384(100)/0.0(0)/0.042718(74)/0.0(0)/0.114899
;;
(send *viewer* :viewing :look #f(3832.96 1118.63 1649.89) #f(-173.901 119.606 620.28))
(send *viewer* :draw-objects)
(send *viewer* :viewsurface :write-to-image-file "4limb_ik_redundancy_egress_front.jpg")
;;
;; (print (list 'send '*viewer* :viewing :look (send *irtviewer* :viewpoint) (send *irtviewer* :viewtarget)))
(send *viewer* :viewing :look #f(-127.655 4513.81 607.946) #f(-201.92 259.166 682.223))
(send *viewer* :draw-objects)
(send *viewer* :viewsurface :write-to-image-file "4limb_ik_redundancy_egress_side.jpg")

(send *robot* :angle-vector *init-pose*)
(send *robot* :fix-leg-to-coords (make-coords) :rleg)
(demo-reset-pose-redundancy :check-balance? t :reset-pose (copy-seq (send *robot* :angle-vector)))
;; 5/100 samples are valid, traj=nil
;; DNN/IK/COG/COL/TREE/TOTAL = 0.019568(100)/0.038555(100)/0.001516(74)/0.0094(16)/0.0(0)/0.075337
;;
(send *viewer* :viewing :look #f(3832.96 1118.63 1649.89) #f(-173.901 119.606 620.28))
(send *viewer* :draw-objects)
(send *viewer* :viewsurface :write-to-image-file "4limb_ik_redundancy_egress_front_with_centroid.jpg")
;;
;; (print (list 'send '*viewer* :viewing :look (send *irtviewer* :viewpoint) (send *irtviewer* :viewtarget)))
(send *viewer* :viewing :look #f(-127.655 4513.81 607.946) #f(-201.92 259.166 682.223))
(send *viewer* :draw-objects)
(send *viewer* :viewsurface :write-to-image-file "4limb_ik_redundancy_egress_side_with_centroid.jpg")
