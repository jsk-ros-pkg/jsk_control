#!/usr/bin/env roseus

(setq *load-as-lib* t)
(require "demo-table-standup.l")

(defclass traj-elem
  :super propertied-object
  :slots (val neighbors path-map))
(defmethod traj-elem
  (:init (v &key ((:neighbors nn) nil)) (setq val v) (setq neighbors nn) (setq path-map (instance hash-table :init)))
  (:neighbors nil neighbors)
  (:value nil val)
  (:distance (v) (/ (norm2 (v- val v)) (length v)))
  (:neighbor? (elem) (find elem neighbors))
  (:add-neighbor
   (elem &key (add-thre -1))
   (cond
    ((< (send self :distance (send elem :value)) add-thre)
     (send elem :_add-neighbor self)
     (send self :_add-neighbor elem)
     self)
    (t nil)))
  (:_add-neighbor
   (elem)
   (push elem neighbors)
   (send self :add-path-neighbors elem (list self elem))
   )
  (:add-path-neighbors
   (elem path &optional (depth 0))
   (cond
    ((not (send path-map :get elem))
     (send path-map :enter elem path)
     (send (send elem :get-val 'path-map) :enter self (reverse path))
     ;;(dolist (neighbor neighbors)
     ;;  (send neighbor :add-path-neighbors elem (cons neighbor path) (+ depth 1)))
     )
    ;; (t (print depth))
    ))
  (:find
   (elem
    &optional (black-list (instance hash-table :init)))
   (or (send path-map :get elem)
       (let* ((path))
	 (send black-list :enter self t)
	 (dolist (neighbor neighbors)
	   (cond
	    ((not (send black-list :get neighbor))
	     (setq path (send neighbor :find elem black-list))
	     (cond
	      (path
	       (setq path (cons self path))
	       (send self :add-path-neighbors elem path)
	       (return-from nil nil))))))
	 path)))
  )

(defclass traj-set
  :super propertied-object
  :slots (name members root valids range))
(defmethod traj-set
  (:init
   (&key ((:name nm) :traj-set)
	 ((:range rg) (expt (deg2rad 17) 2))
	 ((:members mem) nil))
   (setq name nm)
   (setq members mem)
   (setq range rg)
   )
  (:members nil members)
  (:clear-all-path-map
   nil (send-all (send-all members :get-val 'path-map) :clear))
  (:root (&optional rt)
	 (cond
	  (rt (setq root rt) (push root members)))
	 root)
  (:valids nil valids)
  (:add-valids (elem) (push elem valids))
  (:add-elem
   (elem)
   (push elem members)
   (remove-if #'(lambda (mem)
		  (send mem :add-neighbor elem :add-thre range))
	      (cdr members)))
  )

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
  (setq *rarm-reachable-cube* (make-cube 700 700 1000))
  (send *rarm-reachable-cube* :set-color (float-vector 0 1 0))
  (send *rarm-reachable-cube* :translate (float-vector 200 -100 500))
  (gl::transparent *rarm-reachable-cube* 0.3)
  ;;
  (setq *random-rarm-coords*
	(instance vector-coords :init
		  '(lambda (&rest args)
		     (let* ((range (cdr (assoc :cube (send *rarm-reachable-cube* :csg))))
			    (center (send *rarm-reachable-cube* :worldcoords)))
		       (make-coords :pos
				    (v+
				     (send center :worldpos)
				     (float-vector
				      (* (nth 0 range) (- (random 1.0) 0.5))
				      (* (nth 1 range) (- (random 1.0) 0.5))
				      (* (nth 2 range) (- (random 1.0) 0.5))))
				    :rpy (random-vector 3.14))))))
  ;;
  ;; setup initial postures
  ;; (setup-hip-end-coords)
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords) :rleg)
  (let* ((mv (append (send *robot* :arms :end-coords)
		     (send *robot* :legs :end-coords)))
	 (ll (mapcar '(lambda (m) (send *robot* :link-list (send m :parent))) mv)))
    (send *robot* :fullbody-inverse-kinematics
	  (append
	   (list
	    (make-coords :pos (float-vector 250 300 (+ 0 (send *table* :get-val 'height)))
			 :rpy (list 0 0 (deg2rad 90)))
	    (make-coords :pos (float-vector 400 0 (+ -400 (send *table* :get-val 'height)))))
	   (send *robot* :legs :end-coords :copy-worldcoords))
	  :translation-axis '(t nil t t)
	  :rotation-axis '(t nil t t)
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
  ;; (eus_robot_ik::add-target :limb :rarm)
  (eus_robot_ik::add-target :limb :larm)
  (eus_robot_ik::add-target :limb :lleg)
  (eus_robot::configuration-copy-to :robot user::*robot*)
  ;;
  (send *robot* :legs :toe-p :min-angle -60)
  (send *robot* :legs :toe-p :max-angle 16)
  (setq *links* (mapcar #'(lambda (j) (find-if #'(lambda (l) (eq (send l :get :joint) j)) (send *robot* :get :links))) *joint-list1*))
  ;;
  (objects (flatten (list *table*
			  (car (send *robot* :links))
			  (send *robot* :hand :arms)
			  (send-all *joint-list3* :child-link)
			  (send *robot* :legs :toe-p :child-link)
			  *arrow*
			  *rarm-reachable-cube*)))
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
   (step-max 5000)
   (av (instantiate float-vector (length joint-list)))
   ret (suc-cnt 0) (cnt 0) tm
   (draw? t) (mode :first) (step? nil)
   ;;
   ;; (sep-vector (sep-vector (expt step-max (/ 1.0 (length redundancy))) 0 redundancy))
   (cog (float-vector 0 0 0))
   (traj-set (instance traj-set :init))
   (user-valid-check
    #'(lambda (cog)
	(let* ((c (eus_robot::get-link-coords  (send (send *robot* :link "RARM_LINK6") :get :id))))
	  (and (> (aref (send c :worldpos) 0) 200)
	       (> (aref (send c :worldpos) 1) -200)
	       (< (aref (send c :worldpos) 2) (send *table* :get-val 'height))))))
   )
  ;;
  (cpplog::change-output-stream "/dev/null")
  (setq input (concatenate float-vector input redundancy))
  (send *robot* :angle-vector (copy-seq *init-pose*))
  (send *robot* :fix-leg-to-coords (make-coords) :rleg)
  (if draw? (send *viewer* :draw-objects))
  (eus_robot::configuration-copy-to)
  (bench
   (send traj-set :root (instance traj-elem :init (copy-seq (eus_robot::get-configurations))))
   ;; (dolist (redundancy sep-vector)
   (dotimes (i step-max)
     (dotimes (j (length redundancy))
       (setf (aref input (- (- (length input) 1) j)) (random 1.0)))
     ;; (dotimes (j (length redundancy))
     ;; (setf (aref input (- (- (length input) 1) j)) (aref redundancy j)))
     ;;
     (let* ((v (send *random-rarm-coords* :vector :org-coords *root-link*)))
       (dotimes (i (length v)) (setf (aref input (+ 6 i)) (aref v i))))
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
     (if (eq t draw?) (draw-wrapper :cog cog))
     (cond
      ((plusp ret)
       (incf suc-cnt)
       ;;
       (let* ((elem (instance traj-elem :init (copy-seq (eus_robot::get-configurations)))))
	 (send traj-set :add-elem elem)
	 (if (funcall user-valid-check cog)
	     (send traj-set :add-valids elem)))
       ;;
       (if (eq :only-valid draw?) (draw-wrapper :cog cog))
       (if step? (read-line))
       (cond
	((and (eq mode :first)
	      (let* ((clear (send traj-set :clear-all-path-map))
		     (elem
	      	      (find-if #'(lambda (valid) (send (send traj-set :root) :find valid))
	      		       (send traj-set :valids))))
	      	(if elem (send traj-set :put :valid elem))
	      	elem)
	      )
	 (return-from nil nil))))
      )))
  (cpplog::change-output-stream "")
  (warning-message 6 "~A/~A samples are valid~%" suc-cnt cnt)
  (draw-wrapper)
  ;; (eus_robot::configuration-copy-from)
  ;; (send *viewer* :draw-objects)
  traj-set)

(cond
 ((find-if #'(lambda (str) (string-equal "--run-test" str)) lisp::*eustop-argument*)
  (warning-message 6  "find option --run-test~%")
  (warning-message 6  "demo setup ...~%")
  (demo-setup)
  (warning-message 6  "run demo ...~%")
  (setq *traj-set* (demo-main :draw? t))
  (warning-message 6  "show results ...~%")
  (if *traj-set*
      (dolist (elm (send (send *traj-set* :root) :find (send *traj-set* :get :valid))) (draw-wrapper :conf (send elm :value)) (unix::sleep 1)))
  (warning-message 6 "done~%")
  (warning-message 6 "output results are set to *traj-set*~%")
  (print *traj-set*)
  ))
