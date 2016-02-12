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
	    (make-coords :pos (float-vector 250 400 (+ 0 (send *table* :get-val 'height)))
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
  (gl::transparent *rarm-target-sphere* 0.4)
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
  (eus_robot_ik::add-target :limb :larm)
  (eus_robot_ik::add-target :limb :rarm :rotation-axis-rot (make-matrix 3 3))
  (eus_robot_ik::add-target :limb :lleg)
  (eus_robot::configuration-copy-to :robot user::*robot*)
  ;;
  (send *robot* :legs :toe-p :min-angle -60)
  (send *robot* :legs :toe-p :max-angle 16)
  (setq *links* (mapcar #'(lambda (j) (find-if #'(lambda (l) (eq (send l :get :joint) j)) (send *robot* :get :links))) *joint-list1*))
  ;;
  (objects (flatten (list *table* *robot* *rarm-target-sphere*)))
  (send *irtviewer* :change-background (float-vector 1 1 1))
  (send *viewer* :draw-objects)
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
  (&rest
   args
   &key
   ;;
   (draw? t)
   (init
    (progn
      (send *robot* :angle-vector (copy-seq *init-pose*))
      (send *robot* :fix-leg-to-coords (make-coords) :rleg)
      (if draw? (send *viewer* :draw-objects))
      (eus_robot::configuration-copy-to)))
   ;;
   (support-limbs '(:larm :rleg :lleg))
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
   (_ef-coords (append (list (nth 0 *ef-coords*) *rarm-coords*) (subseq *ef-coords* 2)))
   (ef-coords (send-all _ef-coords :copy-worldcoords))
   (caffe-root-link *root-link*)
   (ik-root-link (send *robot* :get :root-link))
   (redundancy *redundancy*)
   (input (concatenate float-vector (get-vector-from-ef-coords :ef-coords _ef-coords :root-link caffe-root-link) redundancy))
   (traj? nil)
   (step-max 10)
   (rarm-random-step-max 100)
   rarm-coords
   ret (tm-buf (float-vector 0))
   )
  (cpplog::change-output-stream "/dev/null")
  ;;
  (setf
   (aref tm-buf 0)
   (+
    (aref tm-buf 0)
    (bench2
     (dotimes (i rarm-random-step-max)
       (setq rarm-coords (send *rarm-coords* :vector :org-coords caffe-root-link))
       (dotimes (j (length rarm-coords)) (setf (aref input (+ 6 j)) (aref rarm-coords j)))
       (setq ret (apply 'demo-ik :input input :redundancy redundancy :ik-root-link ik-root-link :caffe-root-link caffe-root-link :ref-redundancy nil :support-limbs support-limbs :support-face support-face :ef-coords ef-coords :draw? draw? :step-max step-max :traj? traj? args))
       (if (send ret :get :ok) (return-from nil nil))))))
  (if (send ret :get :ok)
      (send *rarm-target-sphere* :set-color (float-vector 0 1 0))
    (send *rarm-target-sphere* :set-color (float-vector 1 0 0)))
  (gl::transparent *rarm-target-sphere* 0.4)
  (cpplog::change-output-stream "")
  (format t "time: ~A~%" (aref tm-buf 0))
  ret)

(defun demo-main2
  (&key (img-dir))
  (dotimes (x 5)
    (dolist (z (if (eq x 0) (list 0 1 2 3 4 5) (list 0 1 2 5)))
      (send *rarm-target-sphere* :newcoords
	    (make-coords :pos (float-vector (+ (* -100 x) 700) 0 (+ (* z 100) 300))))
      (if (send (demo-main :draw? nil) :get :ok)
	  (draw-wrapper) (send *viewer* :draw-objects))
      (let* ((cmd (read-line)))
	(if (and (> (length cmd) 0) (eq (aref cmd 0) #\q))
	    (return-from demo-main2 nil)))
      (if img-dir
	  (let* ((path (format nil "~A/x~Az~A.png" img-dir x z)))
	    (if (not (probe-file img-dir))
		(unix::system (format nil "mkdir -p ~A" img-dir)))
	    (if (not (probe-file path))
		(send *viewer* :viewsurface :write-to-image-file path))))
      )))

(defun draw-line-man
  (&key
   (robot *robot*)
   (flush t)
   (color (float-vector 0 0 0))
   )
  (send *viewer* :viewsurface :color color)
  (send (send *viewer* :viewsurface) :line-width 4.0)
  (mapcar
   #'(lambda (l)
       (if (send l :parent-link)
	   (send *irtviewer*
		 :viewer :viewsurface
		 :3d-line
		 (send l :worldpos)
		 (send (send l :parent-link) :worldpos))))
   (send robot :links))
  (if flush (send *irtviewer* :viewer :viewsurface :flush)))

;; time: 5.81668/21
(defun demo-main3
  (&key (tm-buf (float-vector 0)) (cnt 0) (robot-buf (list *robot*))
	(sphere-buf (list *rarm-target-sphere*)) (line-robot? nil)
	(robot-max 50) (robot-step 3))
  (objects (append (list *table*) robot-buf sphere-buf))
  (dotimes (x 4)
    (dolist (z (if (eq x 0) (list 0 1 2 3 4 5) (list 0 1 2 4 5)))
      (incf cnt)
      (send *rarm-target-sphere* :newcoords
	    (make-coords :pos (float-vector (+ (* -150 x) 700) 0 (+ (* z 150) 200))))
      (setq *rarm-target-sphere-shadow* (make-sphere 100))
      (send *rarm-target-sphere-shadow*
	    :newcoords (send *rarm-target-sphere* :copy-worldcoords))
      (gl::transparent *rarm-target-sphere-shadow* 0.4)
      (cond
       ((send (demo-main :draw? nil :tm-buf tm-buf) :get :ok)
	(send *rarm-target-sphere-shadow* :set-color (float-vector 0 1 0))
	(draw-wrapper))
       (t
	(send *rarm-target-sphere-shadow* :set-color (float-vector 1 0 0))
	(send *viewer* :draw-objects)))
      (push *rarm-target-sphere-shadow* sphere-buf)
      (if (and (plusp (decf robot-max))
	       (zerop (mod robot-max robot-step)))
	  (push (copy-object *robot*) robot-buf))
      (send *irtviewer* :objects
	    (append (if (not line-robot?) robot-buf)
		    (list *table*)
		    sphere-buf))
      (if line-robot?
	  (mapcar '(lambda (r) (draw-line-man :robot r)) robot-buf))
      ;; (let* ((cmd (read-line)))
      ;; (if (and (> (length cmd) 0) (eq (aref cmd 0) #\q))
      ;; (return-from demo-main3 nil)))
      ))
  (format t "time: ~A/~A~%" (aref tm-buf 0) cnt)
  )

(cond
 ((find-if #'(lambda (str) (string-equal "--run-test" str)) lisp::*eustop-argument*)
  (warning-message 6  "find option --run-test~%")
  (warning-message 6  "demo setup ...~%")
  (demo-setup)
  (send *rarm-target-sphere* :translate (float-vector 400 0 300) :world)
  (warning-message 6  "run demo ...~%")
  (setq *traj-set* (demo-main :draw? nil))
  (draw-wrapper)
  (warning-message 6  "show results ...~%")
  ;; (if (send *traj-set* :get :valid)
  ;; (dolist (elm (send (send *traj-set* :root) :find (send *traj-set* :get :valid))) (draw-wrapper :conf (send elm :value)) (unix::sleep 1))
  ;; (warning-message 1 "no answer~%"))
  (warning-message 6 "done~%")
  (warning-message 6 "output results are set to *traj-set*~%")
  (print *traj-set*)
  ))


;; (defun demo-main
;;   (&key
;;    (ef-coords *ef-coords*)
;;    (root-link *root-link*)
;;    (joint-list *joint-list1*)
;;    (input *input*)
;;    (redundancy *redundancy*)
;;    (step-max 300)
;;    (av (instantiate float-vector (length joint-list)))
;;    ret (suc-cnt 0) (cnt 0) tm
;;    (draw? t) (mode :first) (step? nil)
;;    ;;
;;    (sep-vector (sep-vector (expt step-max (/ 1.0 (length redundancy))) 0 redundancy))
;;    (cog (float-vector 0 0 0))
;;    (traj-set (instance traj-set :init))
;;    (user-valid-check #'(lambda (cog) t))
;;    rarm-coords
;;    )
;;   ;;
;;   (cpplog::change-output-stream "/dev/null")
;;   ;; (setq input (concatenate float-vector input redundancy))
;;   (send *robot* :angle-vector (copy-seq *init-pose*))
;;   (send *robot* :fix-leg-to-coords (make-coords) :rleg)
;;   (if draw? (send *viewer* :draw-objects))
;;   (eus_robot::configuration-copy-to)
;;   (send traj-set :put :ok nil)
;;   (bench
;;    (send traj-set :root (instance traj-elem :init (copy-seq (eus_robot::get-configurations))))
;;    ;; (dolist (redundancy sep-vector)
;;    (dotimes (i step-max)
;;      (dotimes (j (length redundancy))
;;        ;; (setf (aref input (- (- (length input) 1) j)) (aref redundancy j)))
;;        (setf (aref input (- (- (length input) 1) j)) (random 1.0)))
;;      (setq rarm-coords (send *rarm-coords* :vector :org-coords *root-link*))
;;      (dotimes (i (length rarm-coords)) (setf (aref input (+ 6 i)) (aref rarm-coords i)))
;;      ;;
;;      (setq tm (bench2 (caffe::calc-forward-double :isize (length input) :input input
;; 						  :osize (length av) :output av)))
;;      (if draw? (warning-message 6 "DNN calc forward: ~A~%" tm))
;;      ;; (map cons #'(lambda (j a) (send j :joint-angle
;;      ;; 				       (- (rad2deg a) (send j :joint-angle))
;;      ;; 				       :relative t))
;;      ;; 	    joint-list av)
;;      ;; (send *robot* :fix-leg-to-coords (make-coords) :rleg)
;;      (setq
;;       tm
;;       (bench2
;;        (progn
;; 	 ;; aho
;; 	 ;; (eus_robot_ik::clear-all)
;; 	 ;; (eus_robot_ik::add-target :limb :rarm :target-coords (send *rarm-coords* :coords-from-vector rarm-coords :org-coords *root-link*))
;; 	 (eus_robot_ik::change-target-coords 1 (send *rarm-coords* :coords-from-vector rarm-coords :org-coords *root-link*))
;; 	 ;; (eus_robot_ik::add-target :limb :larm)
;; 	 ;; (eus_robot_ik::add-target :limb :lleg)
;; 	 ;;
;; 	 (map cons
;; 	      '(lambda (l pos)
;; 		 (eus_robot::set-configuration (send l :get :id) pos 1))
;; 	      *links* av)
;; 	 (eus_robot::forward-kinematics 0)
;; 	 (setq ret (eus_robot_ik::solve-ik :max 15))
;; 	 ;; check centroid
;; 	 (eus_robot::forward-kinematics 0)
;; 	 (eus_robot::calc-links-centroid :ret cog)
;; 	 (setf (aref cog 0) (* 1e+3 (aref cog 0)))
;; 	 (setf (aref cog 1) (* 1e+3 (aref cog 1)))
;; 	 (setf (aref cog 2) 0)
;; 	 (if (not (eq (send *support-face* :insidep cog) :inside)) (setq ret 0))
;; 	 ))
;;       )
;;      (if draw? (warning-message 6 "solve IK: ~A~%" tm))
;;      (setq
;;       tm
;;       (bench2
;;        (progn
;; 	 (cond
;; 	  (ret
;; 	   ;; (eus_robot::configuration-copy-from)
;; 	   (if (plusp ;;(check-all-collision))
;; 		(eus_robot::check-all-collide))
;; 	       (setq ret 0)))))))
;;      (if draw? (warning-message 6 "collision: ~A~%" tm))
;;      (incf cnt)
;;      ;;
;;      (if (eq t draw?) (draw-wrapper :cog cog))
;;      (cond
;;       ((plusp ret)
;;        (incf suc-cnt)
;;        ;;
;;        ;; (let* ((elem (instance traj-elem :init (copy-seq (eus_robot::get-configurations)))))
;;        ;; (send traj-set :add-elem elem)
;;        ;; (if (funcall user-valid-check cog)
;;        ;; (send traj-set :add-valids elem)))
;;        ;;
;;        (if (eq :only-valid draw?) (draw-wrapper :cog cog))
;;        (if step? (read-line))
;;        (cond
;; 	((and (eq mode :first)
;; 	      ;; (let* ((elem
;; 	      ;; 	      (find-if #'(lambda (valid) (send (send traj-set :root) :find valid))
;; 	      ;; 		       (send traj-set :valids))))
;; 	      ;; 	(if elem (send traj-set :put :valid elem))
;; 	      ;; 	elem)
;; 	      )
;; 	 (send traj-set :put :ok t)
;; 	 (return-from nil nil))))
;;       )))
;;   (cpplog::change-output-stream "")
;;   (warning-message 6 "~A/~A samples are valid~%" suc-cnt cnt)
;;   ;;
;;   (if (send traj-set :get :ok)
;;       (send *rarm-target-sphere* :set-color (float-vector 0 1 0))
;;     (send *rarm-target-sphere* :set-color (float-vector 1 0 0)))
;;   (gl::transparent *rarm-target-sphere* 0.4)
;;   ;; (draw-wrapper)
;;   ;; (eus_robot::configuration-copy-from)
;;   ;; (send *viewer* :draw-objects)
;;   traj-set)

;; (send *rarm-target-sphere* :newcoords (make-coords :pos (float-vector 700.0 0.0 600.0)))
;; (send *rarm-target-sphere* :set-color (float-vector 1 0 0))
;; (let* ((key '(:rarm :lleg :rleg))
;;        (mt (mapcar '(lambda (k) (send *robot* k :end-coords)) key))
;;        (ll (mapcar '(lambda (m) (send *robot* :link-list (send m :parent))) mt))
;;        (tc (send-all mt :copy-worldcoords)))
;;   (
