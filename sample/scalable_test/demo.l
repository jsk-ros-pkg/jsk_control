
(require "learn.l")
(require "package://contact_behavior_generation/euslisp/model/four-leg-seat.lisp")
(require "package://eus_robot/euslisp/eus_robot_ik.l")

(setq *seat* (instance four-leg-seat :init :name :seat :depth 1000 :width 500))
(send *seat* :newcoords (make-coords))
(send *seat* :rotate (deg2rad 90) :z)
(send *seat* :translate (float-vector -400 100 0) :world)

(setq *table* (instance four-leg-seat :init :name :table :depth 1000 :width 400 :height (+ 200 (send *seat* :get-val 'height)) :thick 50))
(send *table* :newcoords (make-coords))
(send *table* :rotate (deg2rad 90) :z)
(send *table* :translate (float-vector 200 100 0) :world)

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
	       (make-coords :pos (float-vector 100 0 (send *table* :get-val 'height))
			    :rpy (list 0 (deg2rad 90) 0))
	       (make-coords :pos (float-vector 100 0 (send *table* :get-val 'height))
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

(send *robot* :legs :toe-p :min-angle 0)
(send *robot* :legs :toe-p :max-angle 0)
(progn
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
  )
(send *robot* :legs :toe-p :min-angle -60)
(send *robot* :legs :toe-p :max-angle 16)

(objects (append (list *seat* *table*) (send *irtviewer* :objects)))
(smart-initialize-net)

(cpplog::change-output-stream "/dev/null")

#|

(let* ((ef-coords *ef-coords*)
       (root-link *root-link*)
       (joint-list *joint-list1*)
       (input *input*)
       (redundancy
	(instantiate float-vector (- (caffe::get-blob-count "input") (length input))))
       (step-max 300)
       (av (instantiate float-vector (length joint-list))) ret)
  (setq input (concatenate float-vector input redundancy))
  (dotimes (i step-max)
    (dotimes (j (length redundancy))
      (setf (aref input (- (- (length input) 1) j)) (random 1.0)))
    (warning-message 6 "DNN calc forward: ")
    (bench (caffe::calc-forward-double :isize (length input) :input input
				       :osize (length av) :output av))
    (warning-message 6 "solve IK: ")
    (map cons #'(lambda (j a) (send j :joint-angle
				    (- (rad2deg a) (send j :joint-angle))
				    :relative t))
	 joint-list av)
    (send *robot* :fix-leg-to-coords (make-coords) :rleg)
    (bench
     (progn
       (eus_robot::configuration-copy-to)
       (eus_robot::forward-kinematics 0)
       ;; (map cons
       ;; 	    '(lambda (j pos)
       ;; 	       (eus_robot::set-configuration (send (send j :child-link) :get :id) pos 1))
       ;; 	    joint-list av)
       (setq ret (eus_robot_ik::solve-ik :max 30))))
    (cond
     ((plusp ret)
      (eus_robot::configuration-copy-from)
      (send *viewer* :draw-objects)
      ;; (read-line)
      ))))


