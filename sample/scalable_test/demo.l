
(require "learn.l")
(require "package://contact_behavior_generation/euslisp/model/four-leg-seat.lisp")
(require "package://eus_robot/euslisp/eus_robot_ik.l")

(eus_robot::clear-links)
(eus_robot::copy-robot :set-parameter-for-eus-robot-args (list :root-limb :rleg))
(eus_robot::forward-kinematics 0)
(eus_robot::print-link-tree)
(send *robot* :reset-pose)
(send *robot* :fix-leg-to-coords (make-coords) :rleg)
(send-all (send user::*robot* :links) :worldcoords)
(eus_robot_ik::add-target :limb :rarm)
(eus_robot_ik::add-target :limb :larm)
(eus_robot_ik::add-target :limb :lleg)
(eus_robot::configuration-copy-to :robot user::*robot*)

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

(objects (append (list *seat* *table*) (send *irtviewer* :objects)))
(smart-initialize-net)

(let* ((ef-coords *ef-coords*)
       (root-link *root-link*)
       (joint-list *joint-list1*)
       (input (get-vector-from-ef-coords :ef-coords ef-coords :root-link root-link))
       (redundancy
	(instantiate float-vector (- (caffe::get-blob-count "input") (length input))))
       (step-max 300)
       (av (instantiate float-vector (length joint-list))))
  (setq input (concatenate float-vector input redundancy))
  (dotimes (i step-max)
    (dotimes (j (length redundancy))
      (setf (aref input (- (- (length input) 1) j)) (random 1.0)))
    (warning-message 7 "DNN calc forward: ")
    (bench (caffe::calc-forward-double :isize (length input) :input input
				       :osize (length av) :output av))
    (warning-message 7 "solve IK: ")
    (bench
     (progn
       (map cons
	    '(lambda (j pos)
	       (eus_robot::set-configuration (send (send j :child-link) :get :id) pos 1))
	    joint-list av)
       (eus_robot_ik::solve-ik)))
    (eus_robot::configuration-copy-from)
    (send *viewer* :draw-objects)))

#|
(ik-random-test-min
 :random-angle (send-all *joint-list1* :joint-angle) :initialize? nil)

