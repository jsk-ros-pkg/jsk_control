#!/usr/bin/env roseus

(if (not (find-package "EUS_ROBOT")) (make-package "EUS_ROBOT"))
(if (not (find-package "EUS_ROBOT_IK")) (make-package "EUS_ROBOT_IK"))
(if (not (find-package "EUS_QP_COLLIDE")) (make-package "EUS_QP_COLLIDE"))
(if (not (find-package "CPPLOG")) (make-package "CPPLOG"))
(if (not (find-package "CAFFE")) (make-package "CAFFE"))

(defun demo-setup
  nil
  ;; load files
  (require "learn.l")
  (require "package://contact_behavior_generation/euslisp/model/four-leg-seat.lisp")
  (require "package://eus_robot/euslisp/eus_robot_ik.l")
  (require "package://eus_robot/euslisp/eus_qp_collide.l")
  ;;
  ;; create environment objects
  (setq *seat* (instance four-leg-seat :init :name :seat :depth 1000 :width 500 :height 370))
  (send *seat* :newcoords (make-coords))
  (send *seat* :rotate (deg2rad 90) :z)
  (send *seat* :translate (float-vector -350 100 0) :world)
  (send *seat* :set-val 'height 400)
  ;;
  (setq *table* (instance four-leg-seat :init :name :table :depth 1000 :width 400 :height (+ 200 (send *seat* :get-val 'height)) :thick 50))
  (send *table* :newcoords (make-coords))
  (send *table* :rotate (deg2rad 90) :z)
  (send *table* :translate (float-vector 150 100 0) :world)
  ;;
  ;; setup initial postures
  (setup-hip-end-coords)
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords) :rleg)
  (let* ((mv (append (list (send *robot* :get :hip-end-coords))
		     (send *robot* :arms :end-coords)
		     (send *robot* :legs :end-coords)))
	 (ll (mapcar '(lambda (m) (send *robot* :link-list (send m :parent))) mv)))
    (send *robot* :fullbody-inverse-kinematics
	  (append
	   (list (make-coords :pos (float-vector -300 0 (send *seat* :get-val 'height)))
		 (make-coords :pos (float-vector 0 0 (send *table* :get-val 'height))
			      :rpy (list 0 (deg2rad 90) 0))
		 (make-coords :pos (float-vector 0 0 (send *table* :get-val 'height))
			      :rpy (list 0 (deg2rad 90) 0)))
	   (send *robot* :legs :end-coords :copy-worldcoords))
	  :translation-axis '(:y :y :y t t)
	  :move-target mv
	  :link-list ll
	  :debug-view :no-message
	  :target-centroid-pos nil
	  ))
  (send-all (send *robot* :links) :worldcoords)
  (setq *input* (get-vector-from-ef-coords :ef-coords *ef-coords* :root-link *root-link*))
  (setq *support-face*
	(instance face :init
		  :vertices
		  (quickhull
		   (mapcar
		    '(lambda (v) (float-vector (aref v 0) (aref v 1) 0))
		    (append (send *robot* :arms :end-coords :worldpos)
			    (send (send *robot* :support-polygon :rleg) :vertices)
			    (send (send *robot* :support-polygon :lleg) :vertices))))))
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
  (eus_robot_ik::add-target :limb :rarm)
  (eus_robot_ik::add-target :limb :larm)
  (eus_robot_ik::add-target :limb :lleg)
  (eus_robot::configuration-copy-to :robot user::*robot*)
  ;;
  ;;
  (send *robot* :legs :toe-p :min-angle -60)
  (send *robot* :legs :toe-p :max-angle 16)
  (setq *links* (mapcar #'(lambda (j) (find-if #'(lambda (l) (eq (send l :get :joint) j)) (send *robot* :get :links))) *joint-list1*))
  ;;
  (objects (flatten (list *seat* *table*
			  (car (send *robot* :links))
			  (send-all *joint-list3* :child-link)
			  (send *robot* :legs :toe-p :child-link)
			  *arrow*)))
  ;;
  ;; iniitliaze dnn
  (smart-initialize-net)
  ;;
  ;; setup collision detection
  (eus_qp_collide::test-gen-collidable-pairs-old)
  (setq *collidable-pairs*
	(append
	 eus_qp_collide::*collidable-pairs*
	 (mapcar #'(lambda (l) (cons *table* l)) *links*)
	 (mapcar #'(lambda (l) (cons *seat* l)) *links*)))
  (eus_qp_collide::add-all-link-convex *robot*)
  ;;
  (send *table* :put :id
	(+ (apply 'max (send-all (send *robot* :links) :get :id)) 1))
  (send *seat* :put :id
	(+ (apply 'max (send-all (send *robot* :links) :get :id)) 2))
  (dolist (obj (list *table* *seat*))
    (eus_robot::add-link
     :name (format nil "~A" (send obj :name))
     :id (send obj :get :id)
     :pid -1)
    (eus_robot::set-link-coords
     (send obj :get :id) (send (send obj :worldcoords) :copy-worldcoords))
    (dotimes (i 3)
      (if (zerop (eus_robot::register-collide-model
		  obj :id (send obj :get :id)
		  :vertices-list (flatten (send-all (send (send obj :get-val 'seat-plane) :faces) :vertices)))) (return-from nil nil))
      (warning-message 6 "register collide model retry ~A~%" (send obj :name))
      ))
  ;;
  (eus_qp_collide::gen-all-convex-matrices)
  (eus_robot::register-collide-pairs *collidable-pairs*)
  )

(defun check-all-collision nil
  ;; (return-from check-all-collision 0)
  (let* ((ret 0))
    (dolist (ll *collidable-pairs*)
      (let* ((m1 (get (car ll) :pqpmodel))
	     (m2 (get (cdr ll) :pqpmodel))
	     (c1 (if (send (car ll) :get :id) (eus_robot::get-link-coords (send (car ll) :get :id)) (car ll)))
	     (c2 (if (send (cdr ll) :get :id) (eus_robot::get-link-coords (send (cdr ll) :get :id)) (cdr ll)))
	     (r1 (send c1 :worldrot))
	     (t1 (send c1 :worldpos))
	     (r2 (send c2 :worldrot))
	     (t2 (send c2 :worldpos))
	     (flag geometry::PQP_FIRST_CONTACT))
	(if (null m1) (setq m1 (send (car ll) :make-pqpmodel :fat 0)))
	(if (null m2) (setq m2 (send (cdr ll) :make-pqpmodel :fat 0)))
	(setq ret (+ ret (geometry::pqpcollide r1 t1 m1 r2 t2 m2 flag)))
	(if (plusp ret) (return-from check-all-collision ret))))
    ret))

(defun sep-vector
  (sep-cnt depth vec)
  (if (>= depth (length vec))
      (list (copy-seq vec))
    (let* (buf (step (/ 1.0 (- sep-cnt 1))))
      (dotimes (i sep-cnt)
	(setf (aref vec depth) (* i step))
	(push (sep-vector sep-cnt (+ depth 1) vec) buf))
      (apply 'append buf))))

(defun demo-main
  (&key
   (ef-coords *ef-coords*)
   (root-link *root-link*)
   (joint-list *joint-list1*)
   (input *input*)
   (redundancy
    (instantiate float-vector (- (caffe::get-blob-count "input") (length input))))
   (step-max 500)
   (av (instantiate float-vector (length joint-list)))
   ret (suc-cnt 0) (cnt 0) tm
   (draw? t) (mode :first)
   ;;
   (sep-vector (sep-vector (expt step-max (/ 1.0 (length redundancy))) 0 redundancy))
   (cog (float-vector 0 0 0))
   )
  ;;
  (cpplog::change-output-stream "/dev/null")
  (setq input (concatenate float-vector input redundancy))
  (bench
   (dolist (redundancy sep-vector)
   ;; (dotimes (i step-max)
     (dotimes (j (length redundancy))
       (setf (aref input (- (- (length input) 1) j)) (aref redundancy j)))
     ;;(random 1.0)))
     (setq tm (bench2 (caffe::calc-forward-double :isize (length input) :input input
						  :osize (length av) :output av)))
     (if draw? (warning-message 6 "DNN calc forward: ~A~%" tm))
     ;; (map cons #'(lambda (j a) (send j :joint-angle
     ;; 				       (- (rad2deg a) (send j :joint-angle))
     ;; 				       :relative t))
     ;; 	    joint-list av)
     ;; (send *robot* :fix-leg-to-coords (make-coords) :rleg)
     (setq
      tm
      (bench2
       (progn
	 ;; (eus_robot::configuration-copy-to)
	 (map cons
	      '(lambda (l pos)
		 (eus_robot::set-configuration (send l :get :id) pos 1))
	      *links* av)
	 (eus_robot::forward-kinematics 0)
	 (setq ret (eus_robot_ik::solve-ik :max 15))
	 ;; check centroid
	 (eus_robot::forward-kinematics 0)
	 (eus_robot::calc-links-centroid :ret cog)
	 (setf (aref cog 0) (* 1e+3 (aref cog 0)))
	 (setf (aref cog 1) (* 1e+3 (aref cog 1)))
	 (setf (aref cog 2) 0)
	 (if (not (eq (send *support-face* :insidep cog) :inside)) (setq ret 0))
	 ))
      )
     (if draw? (warning-message 6 "solve IK: ~A~%" tm))
     (setq
      tm
      (bench2
       (progn
	 (cond
	  (ret
	   ;; (eus_robot::configuration-copy-from)
	   (if (plusp ;;(check-all-collision))
		(eus_robot::check-all-collide))
	       (setq ret 0)))))))
     (if draw? (warning-message 6 "collision: ~A~%" tm))
     (incf cnt)
     ;;
     (cond
      (draw?
       (eus_robot::configuration-copy-from)
       (send *viewer* :draw-objects :flush nil)
       (send *support-face* :draw-on :flush nil :width 30)
       (send cog :draw-on :flush nil :color (float-vector 1 0 0) :size 100 :width 30)
       (send *viewer* :viewsurface :flush)
       ))
     (cond
      ((plusp ret)
       (incf suc-cnt)
       (if (eq draw? :step) (read-line))
       (if (eq mode :first) (return-from nil nil)))
      )))
  (cpplog::change-output-stream "")
  (warning-message 6 "~A outof ~A samples are valid~%" suc-cnt cnt)
  ;; (eus_robot::configuration-copy-from)
  ;; (send *viewer* :draw-objects)
  suc-cnt)

;; (demo-setup) (demo-main)
