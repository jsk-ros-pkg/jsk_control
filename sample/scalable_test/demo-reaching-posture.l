#!/usr/bin/env roseus

(setq *load-as-lib* t)
(require "demo-table-standup.l")

(defun demo-setup
  nil
  ;; load files
  (require "learn.l")
  (require "package://contact_behavior_generation/euslisp/model/four-leg-seat.lisp")
  (require "package://eus_robot/euslisp/eus_robot_ik.l")
  (require "package://eus_robot/euslisp/eus_qp_collide.l")
  ;;
  ;; create environment objects
  (setq *table* (instance four-leg-seat :init :name :table :depth 1000 :width 400 :height 700 :thick 50))
  (send *table* :newcoords (make-coords :pos (float-vector 200 0 0)))
  (send *table* :rotate (deg2rad 90) :z)
  (send *table* :translate (float-vector 220 100 0) :world)
  ;;
  ;; setup initial postures
  ;; (setup-hip-end-coords)
  (send-all (send *robot* :joint-list) :set-val 'joint-min-max-table nil)
  (send-all (send *robot* :joint-list) :set-val 'joint-min-max-target nil)
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords) :rleg)
  (let* ((mv (append (list (send *robot* :larm :end-coords))
		     (send *robot* :legs :end-coords)))
	 (ll (mapcar '(lambda (m) (send *robot* :link-list (send m :parent))) mv)))
    (send *robot* :fullbody-inverse-kinematics
	  (append
	   (list
	    (make-coords :pos (float-vector 250 300 (+ 30 (send *table* :get-val 'height)))
			 :rpy (list 0 0 (deg2rad 90))))
	   (send *robot* :legs :end-coords :copy-worldcoords))
	  :translation-axis '(t t t)
	  :rotation-axis '(t t t)
	  :move-target mv
	  :link-list ll
	  :debug-view :no-message
	  ;; :target-centroid-pos nil
	  ))
  (send-all (send *robot* :links) :worldcoords)
  (setq *input* (get-vector-from-ef-coords :ef-coords *ef-coords* :root-link *root-link*))
  (setq *support-face*
	(instance face :init
		  :vertices
		  (quickhull
		   (mapcar
		    '(lambda (v) (float-vector (aref v 0) (aref v 1) 0))
		    (append (list (send *robot* :larm :end-coords :worldpos))
			    (send (send *robot* :support-polygon :rleg) :vertices)
			    (send (send *robot* :support-polygon :lleg) :vertices))))))
  (setq *init-pose* (copy-seq (send *robot* :angle-vector)))
  ;;
  (setq *rarm-target-sphere* (make-sphere 100))
  (send *rarm-target-sphere* :set-color (float-vector 1 0 0))
  (gl::transparent *rarm-target-sphere* 0.8)
  (setq *rarm-coords*
	(instance vector-coords :init
		  '(lambda (&rest args)
		     (make-coords :pos (send (send *rarm-target-sphere* :worldcoords) :worldpos)
				  :rpy (random-vector 3.14)))))
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
  (eus_robot_ik::add-target :limb :rarm :rotation-axis-rot (make-matrix 3 3))
  (eus_robot_ik::add-target :limb :larm)
  (eus_robot_ik::add-target :limb :lleg)
  (eus_robot::configuration-copy-to :robot user::*robot*)
  ;;
  (send *robot* :legs :toe-p :min-angle -60)
  (send *robot* :legs :toe-p :max-angle 16)
  (setq *links* (mapcar #'(lambda (j) (find-if #'(lambda (l) (eq (send l :get :joint) j)) (send *robot* :get :links))) *joint-list1*))
  ;;
  (objects (flatten (list *table* *robot* *rarm-target-sphere*)))
  ;;
  ;; iniitliaze dnn
  (smart-initialize-net)
  ;;
  ;; setup collision detection
  (eus_qp_collide::test-gen-collidable-pairs-old)
  (setq *collidable-pairs*
	(append
	 eus_qp_collide::*collidable-pairs*
	 (mapcar #'(lambda (l) (cons *table* l)) *links*)))
  (eus_qp_collide::add-all-link-convex *robot*)
  ;;
  (send *table* :put :id
	(+ (apply 'max (send-all (send *robot* :links) :get :id)) 1))
  (dolist (obj (list *table*))
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

(defun demo-main
  (&key
   (ef-coords *ef-coords*)
   (root-link *root-link*)
   (joint-list *joint-list1*)
   (input *input*)
   (redundancy
    (instantiate float-vector (- (caffe::get-blob-count "input") (length input))))
   (step-max 700)
   (av (instantiate float-vector (length joint-list)))
   ret (suc-cnt 0) (cnt 0) tm
   (draw? t) (mode :first) (step? nil)
   ;;
   (sep-vector (sep-vector (expt step-max (/ 1.0 (length redundancy))) 0 redundancy))
   (cog (float-vector 0 0 0))
   (traj-set (instance traj-set :init))
   (user-valid-check #'(lambda (cog) t))
   rarm-coords
   )
  ;;
  (cpplog::change-output-stream "/dev/null")
  (setq input (concatenate float-vector input redundancy))
  (send *robot* :angle-vector (copy-seq *init-pose*))
  (send *robot* :fix-leg-to-coords (make-coords) :rleg)
  (if draw? (send *viewer* :draw-objects))
  (eus_robot::configuration-copy-to)
  (send traj-set :put :ok nil)
  (bench
   (send traj-set :root (instance traj-elem :init (copy-seq (eus_robot::get-configurations))))
   ;; (dolist (redundancy sep-vector)
   (dotimes (i step-max)
     (dotimes (j (length redundancy))
       ;; (setf (aref input (- (- (length input) 1) j)) (aref redundancy j)))
       (setf (aref input (- (- (length input) 1) j)) (random 1.0)))
     (setq rarm-coords (send *rarm-coords* :vector :org-coords *root-link*))
     (dotimes (i (length rarm-coords)) (setf (aref input (+ 6 i)) (aref rarm-coords i)))
     ;;
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
	 ;; aho
	 ;; (eus_robot_ik::clear-all)
	 ;; (eus_robot_ik::add-target :limb :rarm :target-coords (send *rarm-coords* :coords-from-vector rarm-coords :org-coords *root-link*))
	 (eus_robot_ik::change-target-coords 0 (send *rarm-coords* :coords-from-vector rarm-coords :org-coords *root-link*))
	 ;; (eus_robot_ik::add-target :limb :larm)
	 ;; (eus_robot_ik::add-target :limb :lleg)
	 ;;
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
     (if (eq t draw?) (draw-wrapper :cog cog))
     (cond
      ((plusp ret)
       (incf suc-cnt)
       ;;
       ;; (let* ((elem (instance traj-elem :init (copy-seq (eus_robot::get-configurations)))))
       ;; (send traj-set :add-elem elem)
       ;; (if (funcall user-valid-check cog)
       ;; (send traj-set :add-valids elem)))
       ;;
       (if (eq :only-valid draw?) (draw-wrapper :cog cog))
       (if step? (read-line))
       (cond
	((and (eq mode :first)
	      ;; (let* ((elem
	      ;; 	      (find-if #'(lambda (valid) (send (send traj-set :root) :find valid))
	      ;; 		       (send traj-set :valids))))
	      ;; 	(if elem (send traj-set :put :valid elem))
	      ;; 	elem)
	      )
	 (send traj-set :put :ok t)
	 (return-from nil nil))))
      )))
  (cpplog::change-output-stream "")
  (warning-message 6 "~A/~A samples are valid~%" suc-cnt cnt)
  ;;
  (if (send traj-set :get :ok)
      (send *rarm-target-sphere* :set-color (float-vector 0 1 0))
    (send *rarm-target-sphere* :set-color (float-vector 1 0 0)))
  (draw-wrapper)
  ;; (eus_robot::configuration-copy-from)
  ;; (send *viewer* :draw-objects)
  traj-set)

(defun demo-main2
  nil
  (dotimes (x 5)
    (dolist (z (if (eq x 0) (list 0 1 2 3 4 5 6 7) (list 0 1 2 5 6 7)))
      (send *rarm-target-sphere* :newcoords
	    (make-coords :pos (float-vector (+ (* -100 x) 700) 0 (+ (* z 100) 300))))
      (demo-main :draw? nil)
      ;; (draw-wrapper)
      (read-line))))

(cond
 ((find-if #'(lambda (str) (string-equal "--run-test" str)) lisp::*eustop-argument*)
  (warning-message 6  "find option --run-test~%")
  (warning-message 6  "demo setup ...~%")
  (demo-setup)
  (send *rarm-target-sphere* :translate (float-vector 400 0 300) :world)
  (warning-message 6  "run demo ...~%")
  (setq *traj-set* (demo-main :draw? nil))
  (warning-message 6  "show results ...~%")
  ;; (if (send *traj-set* :get :valid)
  ;; (dolist (elm (send (send *traj-set* :root) :find (send *traj-set* :get :valid))) (draw-wrapper :conf (send elm :value)) (unix::sleep 1))
  ;; (warning-message 1 "no answer~%"))
  (warning-message 6 "done~%")
  (warning-message 6 "output results are set to *traj-set*~%")
  (print *traj-set*)
  ))
