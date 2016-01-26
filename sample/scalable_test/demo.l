
(require "learn.l")
(require "package://contact_behavior_generation/euslisp/model/four-leg-seat.lisp")
(require "package://eus_robot/euslisp/eus_robot_ik.l")

(eus_robot::clear-links)
(eus_robot::copy-robot)
(eus_robot::forward-kinematics 0)
(eus_robot::print-link-tree)

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

#|
(ik-random-test-min
 :random-angle (send-all *joint-list1* :joint-angle) :initialize? nil)

