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
   (elem path &optional (depth 0) (depth-max 0))
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
  (:wide-first-find
   (elem
    &optional
    (selfs (progn (send self :put :path (list self)) (list self)))
    (black-list (instance hash-table :init)))
   (let* ((path)
	  (neighbors
	   (apply
	    'append
	    (mapcar
	     #'(lambda (slf)
		 (remove-if
		  #'(lambda (elm)
		      (if (send black-list :get elm) t
			(progn
			  (send black-list :enter elm t)
			  (send elm :put :parent slf)
			  (send elm :put :path (cons elm (send slf :get :path)))
			  nil)))
		  (send slf :neighbors)))
	     selfs))))
     (cond
      (neighbors
       (dolist (neighbor neighbors)
	 (cond
	  ((find neighbor elem)
	   (setq path (send neighbor :get :path))
	   ;; (send self :add-path-neighbors elem path 0 0)
	   (return-from nil nil))))
       (if path (reverse path)
	 (send self :wide-first-find elem neighbors black-list)))))))

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
  (send *seat* :translate (float-vector -320 100 0) :world)
  (send *seat* :set-val 'height 400)
  ;;
  (setq *table* (instance four-leg-seat :init :name :table :depth 1000 :width 400 :height (+ 230 (send *seat* :get-val 'height)) :thick 50))
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
	   (list (make-coords :pos (float-vector -270 0 (send *seat* :get-val 'height)))
		 (make-coords :pos (float-vector 80 0 (+ 50 (send *table* :get-val 'height)))
			      :rpy (list 0 (deg2rad 0) 0))
		 (make-coords :pos (float-vector 80 0 (+ 50 (send *table* :get-val 'height)))
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
	(let* ((col (geometry::pqpcollide r1 t1 m1 r2 t2 m2 flag)))
	  (if (plusp col) (warning-message 1 "~A vs ~A has collide~%"
					   (send (car ll) :name) (send (cdr ll) :name)))
	  (setq ret (+ ret col)))
	(if (plusp ret) (return-from check-all-collision ret))))
    ret))

(defun _sep-vector
  (sep-cnt depth vec)
  (if (>= depth (length vec))
      (list (copy-seq vec))
    (let* (buf (step (/ 1.0 (- sep-cnt 1))))
      (dotimes (i sep-cnt)
	(setf (aref vec depth) (* i step))
	(push (_sep-vector sep-cnt (+ depth 1) vec) buf))
      (apply 'append buf))))

(defvar *sep-vector*)
(defun sep-vector
  (sep-cnt depth vec)
  (if (not (and (eq sep-cnt (car *sep-vector*))
		(eq depth (cadr *sep-vector*))
		(eq (length vec) (length (caddr *sep-vector*)))))
      (setq *sep-vector*
	    (list sep-cnt depth vec (_sep-vector sep-cnt depth vec))))
  (cadddr *sep-vector*))

(defun draw-wrapper
  (&key (cog nil) (conf (eus_robot::get-configurations))
	root-link-coords (support-face *support-face*))
  ;; (send *robot* :reset-pose)
  (eus_robot::configuration-copy-from :conf conf :root-link-coords root-link-coords)
  ;; (if rleg-coords (send *robot* :fix-leg-to-coords rleg-coords :rleg))
  (send *viewer* :draw-objects :flush nil)
  (if support-face
      (send support-face :draw-on :flush nil :width 5 :color (float-vector 0.3 1 0.3)))
  (send (or cog (map float-vector '* (float-vector 1 1 0) (send *robot* :centroid)))
	:draw-on :flush nil :color (float-vector 1 0.3 0.3) :size 100 :width 5)
  (send *viewer* :viewsurface :flush)
  (x::window-main-one)
  )

(defun draw-traj-wrapper
  (traj &key (output-dir nil) (traj-id 0) (elem-id 0))
  (if (atom traj) (setq traj (list traj)))
  (dolist (trj traj)
    (incf traj-id)
    (setq elem-id 0)
    (dolist (elm (send trj :get :valid))
      (incf elem-id)
      (draw-wrapper :conf (send elm :value)
		    :support-face (or (send elm :get :support-face) *support-face*)
		    :root-link-coords (send elm :get :root-link-coords))
      (if output-dir
	  (let* ((path (format nil "~A/~A_~A.jpg" output-dir traj-id elem-id)))
	    (if (not (probe-file output-dir))
		(unix::system (format nil "mkdir -p ~A" output-dir)))
	    (if (not (probe-file path))
		(send *viewer* :viewsurface :write-to-image-file path))))
      (unix::usleep (* 100 1000)))))

(defun demo-ik
  (&key
   ;;
   ;; problem settings
   (ef-coords *ef-coords*)
   (caffe-root-link *root-link*)
   (ik-root-link (send *robot* :get :root-link))
   (offset-coords (make-list (length ef-coords)))
   (joint-list *joint-list1*)
   (redundancy *redundancy*)
   (support-limbs '(:larm :rarm :lleg :rleg))
   (support-coords
    (mapcar '(lambda (k)
	       (if (find k '(:rarm :larm)) (send *robot* k :end-coords :copy-worldcoords)
		 (send (send *robot* k :end-coords :parent) :copy-worldcoords)))
	    support-limbs))
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
		   #'(lambda (k c)
		       (cond
			((not (find k '(:rleg :lleg)))
			 (send c :worldpos))
			(t
			 (if (not (and (boundp '*leg-support-polygon*) *leg-support-polygon*))
			     (setq *leg-support-polygon*
				   (mapcar
				    #'(lambda (k)
					(cons k (mapcar #'(lambda (v) (send (send (send *robot* k :end-coords :parent) :worldcoords) :inverse-transform-vector v)) (send (send *robot* :support-polygon k) :vertices))))
				    '(:lleg :rleg))))
			 (mapcar
			  #'(lambda (v) (send c :transform-vector v))
			  (cdr (assoc k *leg-support-polygon*))))))
		   support-limbs support-coords)))))))
   (target-centroid-pos nil)
   (user-valid-check #'(lambda (cog) t))
   (step-max 300)
   (max-time 30) ;; seconds
   (_max-time (+ max-time (car (unix::gettimeofday))))
   ;;
   ;; buf
   (input (concatenate float-vector (get-vector-from-ef-coords :ef-coords ef-coords :root-link caffe-root-link :offset-coords offset-coords) redundancy))
   (av (instantiate float-vector (length joint-list)))
   (event-cnt (instantiate integer-vector 6))
   (event-tm (instantiate float-vector 6))
   (ok? nil) (suc-cnt 0) (cnt 0) (traj? t) tm
   (ref-redundancy (sep-vector (expt (min 8000 step-max) (/ 1.0 (length redundancy)))
			       0 redundancy))
   (cog (float-vector 0 0 0))
   (traj-set (instance traj-set :init))
   ;;
   ;; debug option
   (draw? t) (mode :first) (step? nil) (solve-ik? t) (check-balance? t) (ik-args nil)
   ;;
   &allow-other-keys
   )
  (setf (aref event-tm 5) (let* ((tm (unix::gettimeofday))) (+ (car tm) (* 1e-6 (cadr tm)))))
  ;;
  (send traj-set :put :ok nil)
  (send traj-set :root
	(instance traj-elem :init (copy-seq (eus_robot::get-configurations))))
  (send (send traj-set :root) :put :root-link-coords
	(eus_robot::get-link-coords (send (car (send *robot* :links)) :get :id)))
  (send (send traj-set :root) :put :support-face support-face)
  ;;
  ;; update ik settings
  (eus_robot::set-link-coords 0 (send ik-root-link :copy-worldcoords))
  (eus_robot::forward-kinematics 0)
  ;;
  (dotimes (i (length ef-coords))
    (eus_robot_ik::change-target-coords
     i
     (let* ((_c (send (nth i ef-coords) :worldcoords))
	    (off (nth i offset-coords))
	    (c (if off (send (send _c :copy-worldcoords) :transform off :world) _c)))
       c)))
  ;;
  (cond
   (target-centroid-pos
    (eus_robot_ik::set-cog-borders support-face)
    (if (vectorp target-centroid-pos)
	(eus_robot_ik::change-cog-target-coords (scale 1e-3 target-centroid-pos)))))
  ;;
  (dotimes (i step-max)
    ;;
    ;; start loop
    (incf cnt)
    (if (> (car (unix::gettimeofday)) _max-time) (return-from nil nil))
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
    (setf (aref event-cnt 0) (+ (aref event-cnt 0) 1))
    (setf (aref event-tm 0) (+ (aref event-tm 0) tm))
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
	(setq ok? (or (not solve-ik?) (plusp (apply 'eus_robot_ik::solve-ik (append ik-args (list :max 15))))))
	)))
    (setf (aref event-cnt 1) (+ (aref event-cnt 1) 1))
    (setf (aref event-tm 1) (+ (aref event-tm 1) tm))
    (if draw? (warning-message 6 "solve IK: ~A~%" tm))
    ;;
    ;; check centroid
    (cond
     (ok?
      (setq
       tm
       (bench2
	(progn
	  (eus_robot::forward-kinematics 0)
	  (eus_robot::calc-links-centroid :ret cog)
	  (setf (aref cog 0) (* 1e+3 (aref cog 0)))
	  (setf (aref cog 1) (* 1e+3 (aref cog 1)))
	  (setf (aref cog 2) 0)
	  (if (and check-balance?
		   (not (eq (send support-face :insidep cog) :inside)))
	      (setq ok? nil)))))
      (if check-balance?
	  (progn (setf (aref event-cnt 2) (+ (aref event-cnt 2) 1))
		 (setf (aref event-tm 2) (+ (aref event-tm 2) tm))))
      (if draw? (warning-message 6 "check balance: ~A~%" tm))
      (cond
       (ok?
	;;
	;; check collision
	(setq
	 tm
	 (bench2
	  (if (and ok? (plusp (eus_robot::check-all-collide))) (setq ok? nil))))
	(setf (aref event-cnt 3) (+ (aref event-cnt 3) 1))
	(setf (aref event-tm 3) (+ (aref event-tm 3) tm))
	(if draw? (warning-message 6 "collision: ~A~%" tm))
	;;
	(cond
	 (ok?
	  (incf suc-cnt)
	  (let* ((elem (instance traj-elem :init (copy-seq (eus_robot::get-configurations)))))
	    (send elem :put :root-link-coords
		  (eus_robot::get-link-coords (send (car (send *robot* :links)) :get :id)))
	    (send elem :put :support-face support-face)
	    (send traj-set :add-elem elem t)
	    (if (funcall user-valid-check cog)
		(send traj-set :add-valids elem)))
	  (if (eq :only-valid draw?) (draw-wrapper :cog cog))
	  (if step? (read-line))
	  (cond
	   ((and (eq mode :first)
		 (or
		  (not traj?)
		  (let* ((elem))
		    (setq tm (bench2 (setq elem (send (send traj-set :root)
						      :wide-first-find (send traj-set :valids)))))
		    ;; (find-if #'(lambda (valid) (send (send traj-set :root)
		    ;; :wide-first-find valid))
		    ;; (send traj-set :valids))))
		    (setf (aref event-cnt 4) (+ (aref event-cnt 4) 1))
		    (setf (aref event-tm 4) (+ (aref event-tm 4) tm))
		    (if draw? (warning-message 6 "wide-first-search: ~A~%" tm))
		    (if elem (send traj-set :put :valid elem))
		    elem)))
	    (send traj-set :put :ok t)
	    (return-from nil nil)))))))))
    ;;
    (if (eq t draw?) (draw-wrapper :cog cog))
    )
  (if draw? (warning-message 6 "~A/~A samples are valid, traj=~A~%" suc-cnt cnt (send traj-set :get :valid)))
  (setf (aref event-tm 5)
	(let* ((tm (unix::gettimeofday)))
	  (- (+ (car tm) (* 1e-6 (cadr tm)))
	     (aref event-tm 5))))
  traj-set)

(defun demo-main
  (&rest
   args
   &key
   (step-max 1000)
   (draw? t)
   (traj? t)
   (user-valid-check #'(lambda (cog) (< (print (abs (aref cog 0))) 60)))
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
      (do-until-key (draw-traj-wrapper *traj-set*))
    (warning-message 1 "no answer~%"))
  (warning-message 6 "done~%")
  (warning-message 6 "output results are set to *traj-set*~%")
  (print *traj-set*)
  ))

