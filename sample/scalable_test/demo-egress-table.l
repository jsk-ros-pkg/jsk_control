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
  (setq *seat* (instance four-leg-seat :init :name :seat :depth 1000 :width 500 :height 370 :thick 300))
  (send *seat* :newcoords (make-coords))
  (send *seat* :rotate (deg2rad 90) :z)
  (send *seat* :translate (float-vector -380 100 0) :world)
  (send *seat* :set-val 'height 390)
  ;;
  (setq *table* (instance four-leg-seat :init :name :table :depth 1000 :width 400 :height (+ 230 (send *seat* :get-val 'height)) :thick 50))
  (send *table* :newcoords (make-coords))
  (send *table* :rotate (deg2rad 90) :z)
  (send *table* :translate (float-vector 150 100 0) :world)
  ;;
  ;; setup initial postures
  (send-all (send *robot* :joint-list) :set-val 'joint-min-max-table nil)
  (send-all (send *robot* :joint-list) :set-val 'joint-min-max-target nil)
  (setup-hip-end-coords)
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords) :rleg)
  (let* ((mv (append (list (send *robot* :get :hip-end-coords))
		     (send *robot* :arms :end-coords)
		     (send *robot* :legs :end-coords)))
	 (ll (mapcar '(lambda (m) (send *robot* :link-list (send m :parent))) mv)))
    (send *robot* :fullbody-inverse-kinematics
	  (append
	   (list (make-coords :pos (float-vector -270 0 (send *seat* :get-val 'height)))
		 (make-coords :pos (float-vector 100 0 (+ 50 (send *table* :get-val 'height)))
			      :rpy (list 0 (deg2rad 0) 0))
		 (make-coords :pos (float-vector 100 0 (+ 50 (send *table* :get-val 'height)))
			      :rpy (list 0 (deg2rad 0) 0)))
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
  (setq *init-pose* (copy-seq (send *robot* :angle-vector)))
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
  ;;
  (send *robot* :legs :toe-p :min-angle -60)
  (send *robot* :legs :toe-p :max-angle 16)
  (setq *links* (mapcar #'(lambda (j) (find-if #'(lambda (l) (eq (send l :get :joint) j)) (send *robot* :get :links))) *joint-list1*))
  ;;
  (objects (flatten (list *seat* (cddr (send *table* :bodies)) *robot*)))
  ;;
  ;; iniitliaze dnn
  (smart-initialize-net)
  (setq *redundancy* (instantiate float-vector (- (caffe::get-blob-count "input")
						  (length *input*))))
  (setq *input* (concatenate float-vector *input* *redundancy*))
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
  (if (plusp (eus_robot::check-all-collide))
      (warning-message 1 "invalid initial states with collision~%"))
  ;;
  (send *robot* :angle-vector (copy-seq *init-pose*))
  (send *robot* :fix-leg-to-coords (make-coords) :rleg)
  (setq *ef-coords-list*
	(let* ((larm0
		(instance vector-coords :init (send (nth 0 *ef-coords*) :copy-worldcoords)))
	       (rarm0
		(instance vector-coords :init (send (nth 1 *ef-coords*) :copy-worldcoords)))
	       (lleg0
		(instance vector-coords :init (send (nth 2 *ef-coords*) :copy-worldcoords)))
	       (caffe-rleg0 (send (send *root-link* :worldcoords) :copy-worldcoords))
	       (ik-rleg0 (send (send (send *robot* :get :root-link) :worldcoords)
			       :copy-worldcoords)))
	  (list
	   (list :ef-coords (list larm0 rarm0 lleg0)
		 :support-limbs '(:larm :rarm :lleg :rleg)
		 :caffe-root-link caffe-rleg0
		 :ik-root-link ik-rleg0)
	   ;;
	   (list :ef-coords (list larm0 rarm0 lleg0)
		 :support-limbs '(:larm :rarm :rleg)
		 :caffe-root-link caffe-rleg0
		 :ik-root-link ik-rleg0)
	   (list :ef-coords
		 (list larm0 rarm0
		       (setq lleg0 (send lleg0 :transform-copy
					 (make-coords :pos (float-vector 0 0 50)))))
		 :support-limbs '(:larm :rarm :rleg)
		 :caffe-root-link caffe-rleg0
		 :ik-root-link ik-rleg0)
	   (list :ef-coords
		 (list larm0 rarm0
		       (setq lleg0 (send lleg0 :transform-copy
					 (make-coords :pos (float-vector 0 80 0)))))
		 :support-limbs '(:larm :rarm :rleg)
		 :caffe-root-link caffe-rleg0
		 :ik-root-link ik-rleg0)
	   (list :ef-coords
		 (list larm0 rarm0
		       (setq lleg0 (send lleg0 :transform-copy
					 (make-coords :pos (float-vector 0 0 -50)))))
		 :support-limbs '(:larm :rarm :rleg)
		 :caffe-root-link caffe-rleg0
		 :ik-root-link ik-rleg0)
	   (list :ef-coords
		 (list larm0 rarm0 lleg0)
		 :support-limbs '(:larm :rarm :lleg :rleg)
		 :caffe-root-link caffe-rleg0
		 :ik-root-link ik-rleg0)
	   ;;
	   (list :ef-coords (list larm0 rarm0 lleg0)
	   	 :support-limbs '(:larm :rarm :lleg)
		 :caffe-root-link
		 (setq caffe-rleg0 (send (send caffe-rleg0 :copy-worldcoords)
					 :translate (float-vector 0 0 50) :world))
		 :ik-root-link
		 (setq ik-rleg0 (send (send ik-rleg0 :copy-worldcoords)
				      :translate (float-vector 0 0 50) :world)))
	   (list :ef-coords (list larm0 rarm0 lleg0)
	   	 :support-limbs '(:larm :rarm :lleg)
		 :caffe-root-link
		 (setq caffe-rleg0 (send (send caffe-rleg0 :copy-worldcoords)
					 :translate (float-vector 0 80 0) :world))
		 :ik-root-link
		 (setq ik-rleg0 (send (send ik-rleg0 :copy-worldcoords)
				      :translate (float-vector 0 80 0) :world)))
	   (list :ef-coords (list larm0 rarm0 lleg0)
	   	 :support-limbs '(:larm :rarm :lleg)
		 :caffe-root-link
		 (setq caffe-rleg0 (send (send caffe-rleg0 :copy-worldcoords)
					 :translate (float-vector 0 0 -50) :world))
		 :ik-root-link
		 (setq ik-rleg0 (send (send ik-rleg0 :copy-worldcoords)
				      :translate (float-vector 0 0 -50) :world)))
	   (list :ef-coords (list larm0 rarm0 lleg0)
	   	 :support-limbs '(:larm :rarm :lleg :rleg)
		 :caffe-root-link caffe-rleg0
		 :ik-root-link ik-rleg0
		 )
	   ;;
	   (list :ef-coords
		 (list (setq larm0
			     (send larm0 :transform-copy (make-coords :pos (float-vector 0 0 50))))
		       rarm0 lleg0)
	   	 :support-limbs '(:rarm :lleg :rleg)
		 :caffe-root-link caffe-rleg0
		 :ik-root-link ik-rleg0)
	   (list :ef-coords
		 (list (setq larm0
			     (send larm0 :transform-copy (make-coords :pos (float-vector 0 80 0))))
		       rarm0 lleg0)
	   	 :support-limbs '(:rarm :lleg :rleg)
		 :caffe-root-link caffe-rleg0
		 :ik-root-link ik-rleg0)
	   (list :ef-coords
		 (list (setq larm0
			     (send larm0 :transform-copy (make-coords :pos (float-vector 0 0 -50))))
		       rarm0 lleg0)
	   	 :support-limbs '(:rarm :lleg :rleg)
		 :caffe-root-link caffe-rleg0
		 :ik-root-link ik-rleg0)
	   (list :ef-coords (list larm0 rarm0 lleg0)
	   	 :support-limbs '(:larm :rarm :lleg :rleg)
		 :caffe-root-link caffe-rleg0
		 :ik-root-link ik-rleg0)
	   ;;
	   (list :ef-coords
		 (list larm0
		       (setq rarm0
			     (send rarm0 :transform-copy (make-coords :pos (float-vector 0 0 50))))
		       lleg0)
	   	 :support-limbs '(:rarm :lleg :rleg)
		 :caffe-root-link caffe-rleg0
		 :ik-root-link ik-rleg0)
	   (list :ef-coords
		 (list larm0
		       (setq rarm0
			     (send rarm0 :transform-copy (make-coords :pos (float-vector 0 80 0))))
		       lleg0)
	   	 :support-limbs '(:rarm :lleg :rleg)
		 :caffe-root-link caffe-rleg0
		 :ik-root-link ik-rleg0)
	   (list :ef-coords
		 (list larm0
		       (setq rarm0
			     (send rarm0 :transform-copy (make-coords :pos (float-vector 0 0 -50))))
		       lleg0)
	   	 :support-limbs '(:rarm :lleg :rleg)
		 :caffe-root-link caffe-rleg0
		 :ik-root-link ik-rleg0)
	   (list :ef-coords (list larm0 rarm0 lleg0)
	   	 :support-limbs '(:larm :rarm :lleg :rleg)
		 :caffe-root-link caffe-rleg0
		 :ik-root-link ik-rleg0)
	   ))))

(defun demo-main
  (&rest
   args
   &key
   (step-max 1000)
   (draw? nil)
   (traj? t)
   (step? nil)
   (animate? t)
   (user-valid-check #'(lambda (cog) (< (abs (aref cog 0)) 70)))
   ret
   &allow-other-keys
   )
  (cpplog::change-output-stream "/dev/null")
  ;; set initial stata
  (send *robot* :angle-vector (copy-seq *init-pose*))
  (send *robot* :fix-leg-to-coords (make-coords) :rleg)
  (if draw? (send *viewer* :draw-objects))
  (eus_robot::configuration-copy-to)
  (eus_robot::forward-kinematics 0)
  ;;
  (bench
   (dolist (efl *ef-coords-list*)
     (push (apply 'demo-ik (append efl (list :step-max step-max :draw? draw? :traj? traj? :step? nil :user-valid-check user-valid-check) args)) ret)
     (if (not (send (car ret) :get :valid)) (return-from nil (setq ret nil))
       (let* ((elems (send (car ret) :get :valid))
	      (now (car (last elems))))
	 (if draw? (warning-message 6 "success~%"))
	 ;; (bench
	 (eus_robot::configuration-copy-from
	  :conf (send now :value)
	  :root-link-coords (send now :get :root-link-coords))
	 (eus_robot::configuration-copy-to)
	 (eus_robot::forward-kinematics 0)
	 (if step? (progn (warning-message 6 "animate?~%")
			  (draw-traj-wrapper (car ret)) (read-line)))
	 ))))
  (cpplog::change-output-stream "")
  (cond
   (ret
    (setq ret (reverse ret))
    (if animate? (do-until-key (draw-traj-wrapper ret)))
    ret)))

(cond
 ((find-if #'(lambda (str) (string-equal "--run-test" str)) lisp::*eustop-argument*)
  (warning-message 6  "find option --run-test~%")
  (warning-message 6  "demo setup ...~%")
  (demo-setup)
  (warning-message 6  "run demo ...~%")
  (setq *traj-set* (demo-main :draw? nil :animate? nil))
  (warning-message 6  "show results ...~%")
  (if *traj-set* (do-until-key (draw-traj-wrapper *traj-set*))
    (warning-message 1 "no answer~%"))
  (warning-message 6 "done~%")
  (warning-message 6 "output results are set to *traj-set*~%")
  (print *traj-set*)
  ))

#|

(defun demo-main
  (&rest
   args
   &key
   (step-max 1000)
   (draw? t)
   (traj? t)
   (user-valid-check #'(lambda (cog) (< (print (abs (aref cog 0))) 80)))
   ret
   &allow-other-keys
   )
  (cpplog::change-output-stream "/dev/null")
  ;; set initial stata
  (send *robot* :angle-vector (copy-seq *init-pose*))
  (send *robot* :fix-leg-to-coords (make-coords) :rleg)
  (if draw? (send *viewer* :draw-objects))
  (eus_robot::configuration-copy-to)
  (eus_robot::forward-kinematics 0)
  (bench (setq ret (apply 'demo-ik :step-max step-max :draw? draw? :traj? traj? :user-valid-check user-valid-check args)))
  (cpplog::change-output-stream "")
  ret)
