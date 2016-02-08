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
  (setq *seat* (instance four-leg-seat :init :name :seat :depth 800 :width 500 :height 370 :thick 300))
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
	 (bc (aref (send *robot* :worldpos) 1))
	 (ll (mapcar '(lambda (m) (send *robot* :link-list (send m :parent))) mv)))
    (send *robot* :fullbody-inverse-kinematics
	  (append
	   (list (make-coords :pos (float-vector -270 0 (send *seat* :get-val 'height)))
		 (make-coords :pos (float-vector 150 (+ bc 250)
						 (+ 50 (send *table* :get-val 'height)))
			      :rpy (list (deg2rad -20) (deg2rad 0) 0))
		 (make-coords :pos (float-vector 150 (- bc 250)
						 (+ 50 (send *table* :get-val 'height)))
			      :rpy (list (deg2rad +20) (deg2rad 0) 0))
		 )
	   (send *robot* :legs :end-coords :copy-worldcoords))
	  :translation-axis '(:y t t t t)
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
  (if (probe-file "egress-ef-coords-list.l") (require "egress-ef-coords-list.l")
    (if (probe-file "../egress-ef-coords-list.l") (require "../egress-ef-coords-list.l")))
  )

(defun gen-move-limb-ef-coords-list
  (limb move
	&key
	(last-ef-coords
	 (if *ef-coords-list*
	     (append (cadr (member :ef-coords (car (last *ef-coords-list*))))
		     (list (cadr (member :caffe-root-link (car (last *ef-coords-list*))))
			   (cadr (member :ik-root-link (car (last *ef-coords-list*))))))))
	(support-limbs (remove limb '(:larm :rarm :lleg :rleg)))
	(larm0 (or (send (nth 0 last-ef-coords) :copy-worldcoords)
		   (send (nth 0 *ef-coords*) :copy-worldcoords)))
	(rarm0 (or (send (nth 1 last-ef-coords) :copy-worldcoords)
		   (send (nth 1 *ef-coords*) :copy-worldcoords)))
	(lleg0 (or (send (nth 2 last-ef-coords) :copy-worldcoords)
		   (send (nth 2 *ef-coords*) :copy-worldcoords)))
	(caffe-rleg0 (or (send (nth 3 last-ef-coords) :copy-worldcoords)
			 (send (send *root-link* :worldcoords) :copy-worldcoords)))
	(ik-rleg0 (or (send (nth 4 last-ef-coords) :copy-worldcoords)
		      (send (send (send *robot* :get :root-link) :worldcoords)
			    :copy-worldcoords)))
	(caffe-ik-rleg0 (send caffe-rleg0 :transformation ik-rleg0 :local))
	;;
	(coords (let* ((c (cond
			   ((eq limb :larm) larm0)
			   ((eq limb :rarm) rarm0)
			   ((eq limb :lleg) lleg0)
			   (t ik-rleg0))))
		  (cond
		   ((numberp move) (send c :rotate (deg2rad move) :z))
		   ((vectorp move) (send c :translate move :world))
		   ((subclassp (class move) coordinates)
		    (send c :transform move :world))
		   ((functionp move) (funcall c))))))
  (setq caffe-rleg0 (send caffe-rleg0 :transform ik-rleg0 :local))
  (list :ef-coords (mapcar '(lambda (c) (instance vector-coords :init c))
			   (list larm0 rarm0 lleg0))
	:support-limbs support-limbs
	:caffe-root-link caffe-rleg0
	:ik-root-link ik-rleg0))

(defvar *last-detach-results*)
(defvar *detach-results*)
(defun interactive-demo-proc-detach
  (limb vec &optional (rot))
  (setq *last-detach-results* nil)
  (dolist (move (append (list (map float-vector '* (float-vector 0 0 0) vec)
			      (map float-vector '* (float-vector 0 0 1) vec)
			      (map float-vector '* (float-vector 1 1 0) vec))
			(if rot (list rot))
			(list
			 (map float-vector '* (float-vector 0 0 -1) vec))))
    (push (interactive-demo-proc
	   :ef-coords-list (list (gen-move-limb-ef-coords-list limb move)))
	  *last-detach-results*)
    (if (not (car *last-detach-results*))
	(progn (push *last-detach-results* *detach-results*)
	       (revert-interactive-demo-proc-detach)
	       (return-from nil (setq *last-detach-results* nil)))))
  (if *last-detach-results* (push *last-detach-results* *detach-results*))
  )

(defun revert-interactive-demo-proc-detach
  nil
  (setq *last-detach-results* (car *detach-results*))
  (setq *detach-results* (cdr *detach-results*))
  (dolist (l *last-detach-results*) (if l (revert-ret))))

(defun interactive-demo-proc
  (&rest args
	 &key
	 (ef-coords-list)
	 (angle-vector-org (copy-seq (send *robot* :angle-vector)))
	 (rleg-coords-org (send *robot* :rleg :end-coords :copy-worldcoords))
	 (ret-org (if (not (boundp '*ret*)) (setq *ret* nil) *ret*))
	 (animate? nil)
	 suc?
	 &allow-other-keys)
  ;;
  (cpplog::change-output-stream "/dev/null")
  ;;
  (send *robot* :angle-vector (copy-seq angle-vector-org))
  (send *robot* :fix-leg-to-coords (send rleg-coords-org :copy-worldcoords) :rleg)
  (send *viewer* :draw-objects)
  (eus_robot::configuration-copy-to)
  (eus_robot::forward-kinematics 0)
  ;;
  (bench (setq suc? (apply 'demo-proc :ef-coords-list ef-coords-list
			   :ret *ret* :animate? animate? args)))
  ;;
  (cond
   (suc?
    (setq *ef-coords-list* (append *ef-coords-list* ef-coords-list))
    ;; (send *viewer* :draw-objects)
    (draw-wrapper)
    (warning-message 6 "demo-proc succeeded~%")
    )
   (t
    ;;
    ;; revert
    (send *robot* :angle-vector (copy-seq angle-vector-org))
    (send *robot* :fix-leg-to-coords (send rleg-coords-org :copy-worldcoords) :rleg)
    (send *viewer* :draw-objects)
    (eus_robot::configuration-copy-to)
    (eus_robot::forward-kinematics 0)
    ;;
    (setq *ret* ret-org)
    ;;
    (warning-message 1 "demo-proc has no answer, reverted~%")
    )
   )
  (cpplog::change-output-stream "")
  suc?
  )

(defun revert-ret
  nil
  (cond
   ((not (and (boundp '*ret*) *ret*))
    (warning-message 1 "empty ret detected~%"))
   (t
    (setq *ret-org* (car *ret*))
    (setq *ret* (cdr *ret*))
    (setq *ret-now* (car *ret*))
    (setq *last-efl* (car (last *ef-coords-list*)))
    (setq *ef-coords-list* (butlast *ef-coords-list*))
    (if (send *ret-now* :get :valid)
	(let* ((elems (send *ret-now* :get :valid))
	       (now (car (last elems))))
	  (eus_robot::configuration-copy-from
	   :conf (send now :value)
	   :root-link-coords (send now :get :root-link-coords))
	  (eus_robot::configuration-copy-to)
	  (eus_robot::forward-kinematics 0)
	  ;; (send *viewer* :draw-objects)
	  (draw-wrapper)
	  )
      (warning-message 1 "latest ret has no posture~%")))))

(defun demo-proc
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
   (ef-coords-list *ef-coords-list*)
   &allow-other-keys
   )
  (dolist (efl ef-coords-list)
    (push (apply 'demo-ik (append efl (list :step-max step-max :draw? draw? :traj? traj? :step? nil :user-valid-check user-valid-check) args)) ret)
    (setq *ret* ret)
    (setq *last-efl* efl)
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
	)))
  (cond
   (ret
    (setq ret (reverse ret))
    (if animate? (do-until-key (draw-traj-wrapper ret)))
    ret)))

(defun demo-main
  (&rest
   args
   &key
   (draw? nil)
   ret-buf
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
  (bench (setq ret-buf (apply 'demo-proc args)))
  (cpplog::change-output-stream "")
  ret-buf)

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

;; (interactive-demo-proc-detach :larm (float-vector 0 80 30))
(interactive-demo-proc-detach :rarm (float-vector 0 80 30) -20)
(interactive-demo-proc-detach :lleg (float-vector 0 80 30))
(interactive-demo-proc-detach :rleg (float-vector 0 80 30))
