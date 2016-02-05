#!/usr/bin/env roseus

(if (not (find-package "EUS_ROBOT")) (make-package "EUS_ROBOT"))
(if (not (find-package "EUS_ROBOT_IK")) (make-package "EUS_ROBOT_IK"))
(if (not (find-package "EUS_QP_COLLIDE")) (make-package "EUS_QP_COLLIDE"))
(if (not (find-package "CPPLOG")) (make-package "CPPLOG"))
(if (not (find-package "CAFFE")) (make-package "CAFFE"))

(defclass traj-elem
  :super propertied-object
  :slots (val neighbors path-map))
(defmethod traj-elem
  (:init (v &key ((:neighbors nn) nil)) (setq val v) (setq neighbors nn) (setq path-map (instance hash-table :init)) (send path-map :enter self (list self)) self)
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
   (elem path &optional (depth 0) (depth-max 1000))
   (cond
    ((not (send path-map :get elem))
     (send path-map :enter elem path)
     (send (send elem :get-val 'path-map) :enter self (reverse path))
     (if (> depth-max depth)
	 (dolist (neighbor neighbors)
	   (send neighbor :add-path-neighbors elem (cons neighbor path)
		 (+ depth 1) depth-max))
       ))
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
	       (send self :add-path-neighbors elem path 0 0)
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
  (:root (&optional rt)
	 (cond
	  (rt (setq root rt) (push root members)))
	 root)
  (:valids nil valids)
  (:add-valids (elem) (push elem valids))
  (:add-elem
   (elem &optional (merge? t))
   (push elem members)
   (if merge?
       (mapcar #'(lambda (mem)
		   (send mem :add-neighbor elem :add-thre range))
	       (cdr members))))
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
  (setq *seat* (instance four-leg-seat :init :name :seat :depth 1000 :width 500 :height 370))
  (send *seat* :newcoords (make-coords))
  (send *seat* :rotate (deg2rad 90) :z)
  (send *seat* :translate (float-vector -350 100 0) :world)
  (send *seat* :set-val 'height 400)
  ;;
  (setq *table* (instance four-leg-seat :init :name :table :depth 1000 :width 400 :height (+ 230 (send *seat* :get-val 'height)) :thick 50))
  (send *table* :newcoords (make-coords))
  (send *table* :rotate (deg2rad 90) :z)
  (send *table* :translate (float-vector 120 100 0) :world)
  ;;
  ;; setup initial postures
  (setup-hip-end-coords)
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords :pos (float-vector -30 0 0)) :rleg)
  (let* ((mv (append (list (send *robot* :get :hip-end-coords))
		     (send *robot* :arms :end-coords)
		     (send *robot* :legs :end-coords)))
	 (ll (mapcar '(lambda (m) (send *robot* :link-list (send m :parent))) mv)))
    (send *robot* :fullbody-inverse-kinematics
	  (append
	   (list (make-coords :pos (float-vector -300 0 (send *seat* :get-val 'height)))
		 (make-coords :pos (float-vector 50 0 (+ 50 (send *table* :get-val 'height)))
			      :rpy (list 0 (deg2rad 0) 0))
		 (make-coords :pos (float-vector 50 0 (+ 50 (send *table* :get-val 'height)))
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
  (objects (flatten (list *seat* *table*
			  (car (send *robot* :links))
			  (send *robot* :hand :arms)
			  (send-all *joint-list3* :child-link)
			  (send *robot* :legs :toe-p :child-link)
			  *arrow*)))
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

(defun draw-wrapper
  (&key (cog nil) (conf (eus_robot::get-configurations)))
  (send *robot* :reset-pose)
  ;;
  (eus_robot::configuration-copy-from :conf conf)
  (send *robot* :fix-leg-to-coords (make-coords :pos (float-vector -30 0 0)) :rleg)
  (send *viewer* :draw-objects :flush nil)
  (send *support-face* :draw-on :flush nil :width 5)
  (send (or cog (map float-vector '* (float-vector 1 1 0) (send *robot* :centroid)))
	:draw-on :flush nil :color (float-vector 1 0 0) :size 100 :width 5)
  (send *viewer* :viewsurface :flush)
  )

(defun demo-ik
  (&key
   ;;
   ;; problem settings
   (ef-coords *ef-coords*)
   (caffe-root-link *root-link*)
   (ik-root-link (send *robot* :get :root-link))
   (joint-list *joint-list1*)
   (redundancy *redundancy*)
   (support-limbs '(:rarm :larm :rleg :lleg))
   (support-face
    (setq
     *support-face*
     (instance face :init
	       :vertices
	       (quickhull
		(mapcar
		 '(lambda (v) (float-vector (aref v 0) (aref v 1) 0))
		 (flatten
		  (mapcar
		   '(lambda (k)
		      (if (find k '(:rarm :larm))
			  (send *robot* k :end-coords :worldpos)
			(send (send *robot* :support-polygon k) :vertices)))
		   support-limbs)))))))
   (user-valid-check #'(lambda (cog) t))
   (step-max 300)
   ;;
   ;; buf
   (input (concatenate float-vector (get-vector-from-ef-coords :ef-coords ef-coords :root-link caffe-root-link) redundancy))
   (av (instantiate float-vector (length joint-list)))
   (ok? nil) (suc-cnt 0) (cnt 0) (traj? t) tm
   (ref-redundancy (sep-vector (expt step-max (/ 1.0 (length redundancy))) 0 redundancy))
   (cog (float-vector 0 0 0))
   (traj-set (instance traj-set :init))
   ;;
   ;; debug option
   (draw? t) (mode :first) (step? nil)
   ;;
   &allow-other-keys
   )
  ;;
  (bench
   (cpplog::change-output-stream "/dev/null")
   (send traj-set :put :ok nil)
   (send traj-set :root (instance traj-elem :init (copy-seq (eus_robot::get-configurations))))
   ;; update ik settings
   (eus_robot::set-link-coords 0 (send ik-root-link :copy-worldcoords))
   (dotimes (i (length ef-coords))
     (eus_robot_ik::change-target-coords i (send (nth i ef-coords) :worldcoords)))
   ;;
   (dotimes (i step-max)
     ;;
     ;; start loop
     (incf cnt)
     ;;
     ;; dnn kinematics calcuration
     (if ref-redundancy
	 (let* ((rd (car ref-redundancy)))
	   (dotimes (j (length redundancy))
	     (setf (aref input (- (- (length input) 1) j)) (aref rd j)))
	   (setq ref-redundancy (cdr ref-redundancy)))
       (dotimes (j (length redundancy)) ;; random redundant parameters
	 (setf (aref input (- (- (length input) 1) j)) (random 1.0))))
     (setq tm (bench2 (caffe::calc-forward-double :isize (length input) :input input
						  :osize (length av) :output av)))
     (if draw? (warning-message 6 "DNN calc forward: ~A~%" tm))
     ;;
     ;; solve inverse kinematics
     (setq
      tm
      (bench2
       (progn
	 (map cons
	      '(lambda (l pos)
		 (eus_robot::set-configuration (send l :get :id) pos 1))
	      *links* av)
	 (eus_robot::forward-kinematics 0)
	 (setq ok? (plusp (eus_robot_ik::solve-ik :max 15)))
	 ;; check centroid
	 (eus_robot::forward-kinematics 0)
	 (eus_robot::calc-links-centroid :ret cog)
	 (setf (aref cog 0) (* 1e+3 (aref cog 0)))
	 (setf (aref cog 1) (* 1e+3 (aref cog 1)))
	 (setf (aref cog 2) 0)
	 (if (not (eq (send support-face :insidep cog) :inside)) (setq ok? nil))
	 ))
      )
     (if draw? (warning-message 6 "solve IK: ~A~%" tm))
     ;;
     ;; check collision
     (setq
      tm
      (bench2
       (if (and ok? (plusp (eus_robot::check-all-collide))) (setq ok? nil))))
     (if draw? (warning-message 6 "collision: ~A~%" tm))
     ;;
     ;;
     (if (eq t draw?) (draw-wrapper :cog cog))
     (cond
      (ok?
       (incf suc-cnt)
       (let* ((elem (instance traj-elem :init (copy-seq (eus_robot::get-configurations)))))
	 (send traj-set :add-elem elem traj?)
	 (if (funcall user-valid-check cog)
	     (send traj-set :add-valids elem)))
       (if (eq :only-valid draw?) (draw-wrapper :cog cog))
       (if step? (read-line))
       (cond
	((and (eq mode :first)
	      (or
	       (not traj?)
	       (let* ((elem
		       (find-if #'(lambda (valid) (send (send traj-set :root) :find valid))
				(send traj-set :valids))))
		 (if elem (send traj-set :put :valid elem))
		 elem)))
	 (send traj-set :put :ok t)
	 (return-from nil nil))))
      )))
  (cpplog::change-output-stream "")
  (if draw? (warning-message 6 "~A/~A samples are valid~%" suc-cnt cnt))
  traj-set)

(defun demo-main
  (&rest
   args
   &key
   (step-max 1000)
   (draw? t)
   (traj? t)
   (user-valid-check #'(lambda (cog) (< (print (abs (aref cog 0))) 80)))
   &allow-other-keys
   )
  ;; set initial stata
  (send *robot* :angle-vector (copy-seq *init-pose*))
  (send *robot* :fix-leg-to-coords (make-coords :pos (float-vector -30 0 0)) :rleg)
  (if draw? (send *viewer* :draw-objects))
  (eus_robot::configuration-copy-to)
  (apply 'demo-ik :step-max step-max :draw? draw? :traj? traj? :user-valid-check user-valid-check args))

(cond
 ((and
   (not (boundp '*load-as-lib*))
   (find-if #'(lambda (str) (string-equal "--run-test" str)) lisp::*eustop-argument*))
  (warning-message 6  "find option --run-test~%")
  (warning-message 6  "demo setup ...~%")
  (demo-setup)
  (warning-message 6  "run demo ...~%")
  (setq *traj-set* (demo-main :draw? nil))
  (warning-message 6  "show results ...~%")
  (if (send *traj-set* :get :valid)
      (dolist (elm (send (send *traj-set* :root) :find (send *traj-set* :get :valid))) (draw-wrapper :conf (send elm :value)) (unix::usleep (* 100 1000)))
    (warning-message 1 "no answer~%"))
  (warning-message 6 "done~%")
  (warning-message 6 "output results are set to *traj-set*~%")
  (print *traj-set*)
  ))


#|

(defun demo-main
  (&key
   (ef-coords *ef-coords*)
   (root-link *root-link*)
   (joint-list *joint-list1*)
   (input *input*)
   (redundancy *redundancy*)
   (step-max 700)
   (av (instantiate float-vector (length joint-list)))
   ret (suc-cnt 0) (cnt 0) tm
   (draw? t) (mode :first) (step? nil)
   ;;
   (sep-vector (sep-vector (expt step-max (/ 1.0 (length redundancy))) 0 redundancy))
   (cog (float-vector 0 0 0))
   (traj-set (instance traj-set :init))
   (user-valid-check #'(lambda (cog) (< (print (abs (aref cog 0))) 80)))
   )
  ;;
  (cpplog::change-output-stream "/dev/null")
  ;; (setq input (concatenate float-vector input redundancy))
  (send *robot* :angle-vector (copy-seq *init-pose*))
  (send *robot* :fix-leg-to-coords (make-coords :pos (float-vector -30 0 0)) :rleg)
  (if draw? (send *viewer* :draw-objects))
  (eus_robot::configuration-copy-to)
  (bench
   (send traj-set :root (instance traj-elem :init (copy-seq (eus_robot::get-configurations))))
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
	      (let* ((elem
		      (find-if #'(lambda (valid) (send (send traj-set :root) :find valid))
			       (send traj-set :valids))))
		(if elem (send traj-set :put :valid elem))
		elem))
	 (return-from nil nil))))
      )))
  (cpplog::change-output-stream "")
  (warning-message 6 "~A/~A samples are valid~%" suc-cnt cnt)
  ;; (eus_robot::configuration-copy-from)
  ;; (send *viewer* :draw-objects)
  traj-set)
