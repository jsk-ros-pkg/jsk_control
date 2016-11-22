#!/usr/bin/env roseus

(require :unittest "lib/llib/unittest.l")
(init-unit-test)

(deftest test-sample-init
  (assert
   (progn
     ;;(require "package://jsk_footstep_planner/euslisp/footstep-planner-client-sample.l")
     (ros::load-ros-manifest "jsk_footstep_planner")
     (load "package://jsk_footstep_controller/euslisp/util.l")
     (initialize-eus-footstep-planning-client)
     (ros::set-dynamic-reconfigure-param "footstep_planner" "planning_timeout" :double 30.0)
     *footstep-planning-client*)))

(deftest test-sample-without-obstacle
  (assert
   (progn
     ;; Set empty point cloud == no obstacle
     (publish-footstep-planning-obstacle-model-from-eus-pointcloud (instance pointcloud :init))
     (let* ((result (plan-footstep-from-goal-coords (make-coords :pos (float-vector 3000 0 0)))))
       (and result (footstep-array->coords result))
       ))))

(deftest test-sample-with-obstacle-1
  (assert
   (progn
     (let ((obstacle-point-cloud
            (get-pointcloud-within-bodies-2D (list (let ((bb (make-cube 400 400 50))) (send bb :translate (float-vector 400 0 0)) (send bb :worldcoords) bb)))))
       ;; Set obstacle by cube
       (publish-footstep-planning-obstacle-model-from-eus-pointcloud obstacle-point-cloud)
       (let* ((result (plan-footstep-from-goal-coords (make-coords :pos (float-vector 3000 0 0)))))
         (and result (footstep-array->coords result))
         )))))

(deftest test-sample-with-obstacle-2
  (assert
   (progn
     (load "models/room73b2-scene")
     (Objects (list (room73b2)))
     (let ((obstacle-point-cloud
            (get-pointcloud-within-bodies-2D
             (list
              (send (make-bounding-box (flatten (send-all (send (send *room73b2* :object "room73b2-gifuplastic-900-cart") :bodies) :vertices))) :body)
              ))))
       (publish-footstep-planning-obstacle-model-from-eus-pointcloud obstacle-point-cloud)
       (let ((result (plan-footstep-from-goal-coords
                      (send (send (send (send *room73b2* :object "room73b2-gifuplastic-900-cart") :copy-worldcoords) :translate (float-vector -600 -500 0) :world) :rotate pi/2 :z)
                      :start-coords (send (send (send *room73b2* :object "room73b2-gifuplastic-900-cart") :copy-worldcoords) :translate (float-vector 0 300 0) :world))))
         (and result (footstep-array->coords result))
         )))))

;; Test for problem https://github.com/jsk-ros-pkg/jsk_control/issues/650
(deftest test-sample-footstep-successors
  (assert
   (progn
     ;; Generate robot model
     (let ((robot-name (ros::get-param "robot/type")))
       (warn ";; ROBOT=~A~%" robot-name)
       (cond
        ((string= robot-name "HRP2JSKNTS")
         (load "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknts-interface.l")
         (hrp2jsknts)
         (setq *robot* *hrp2jsknts*))
        ((string= robot-name "sample_robot")
         (load "irteus/demo/sample-robot-model.l")
         (setq *robot* (instance sample-robot :init))
         (objects (list *robot*)))
        (t)))
     (if (not (boundp '*robot*))
         ;; Currently, if no robot is specified, omit this sample
         t
       ;; Get successors (consider default_lfoot_to_rfoot_offset from :default-half-offset)
       (let* ((offset (* 2 (elt (cadr (memq :default-half-offset (send *robot* :footstep-parameter))) 1)))
              (successors (mapcar #'(lambda (ss)
                                      (float-vector (* 1e3 (cdr (assoc "x" ss :test #'string=))) (+ (- offset) (* 1e3 (cdr (assoc "y" ss :test #'string=))))))
                                  (ros::get-param "/footstep_planner/successors"))))
         ;; Footstep plan
         (publish-footstep-planning-obstacle-model-from-eus-pointcloud (instance pointcloud :init))
         (let* ((result (plan-footstep-from-goal-coords (make-coords :pos (float-vector 600 -200 0) :rpy (list (deg2rad 45) 0 0)) :robot *robot*))
                (result-footstep (footstep-array->coords result))
                (footstep-dif-list
                 (mapcar #'(lambda (fs0 fs1)
                             (let ((tmp (subseq (send (send fs0 :transformation fs1) :worldpos) 0 2))) ;; (float-vector x y)
                               (if (eq (send fs0 :name) :lleg)
                                   tmp
                                 (float-vector (elt tmp 0) (- (elt tmp 1)))))) ;; If rleg -> lleg, revert y component
                         (butlast result-footstep) (cdr result-footstep)))
                (ret
                 ;; Check difference between footstep results and successors
                 (mapcar #'(lambda (dif) (null (null (member dif successors :test #'(lambda (x y) (< (distance x y) 1e-1))))))
                         (butlast footstep-dif-list 2)))) ;; Last 2 elements are finalize footsteps, which does not equal to successors
           (warn ";; ret = ~A (~A)~%" (every #'identity ret) ret)
           (every #'identity ret)
           ))))))

(defun set-parameters-for-planning-on-poincloud-model
  ()
  (ros::set-dynamic-reconfigure-param "footstep_planner" "project_start_state" :bool t)
  (ros::set-dynamic-reconfigure-param "footstep_planner" "project_goal_state" :bool t)
  (ros::set-dynamic-reconfigure-param "footstep_planner" "use_pointcloud_model" :bool t)
  (ros::set-dynamic-reconfigure-param "footstep_planner" "use_obstacle_model" :bool nil)
  (ros::set-dynamic-reconfigure-param "footstep_planner" "transition_limit_x" :double
                                      (cdr (assoc "x" (find-extream (ros::get-param "/footstep_planner/successors") #'(lambda (x) (cdr (assoc "x" x :test #'string=))) #'>) :test #'string=)))
  )

(deftest test-sample-stair-walking-0
  (assert
   (progn
     ;; Use pointcloud model
     (set-parameters-for-planning-on-poincloud-model)
     ;; Setup models
     (setq *b1* (make-cube 600 1000 10))
     (setq *b2* (make-cube 600 1000 10))
     (setq *b3* (make-cube 600 1000 10))
     (send *b2* :translate (float-vector 620 0 100))
     (send *b3* :translate (float-vector 1240 0 0))
     (send-all (list *b1* *b2* *b3*) :worldcoords)
     (objects (list *b1* *b2* *b3*))
     ;; Publis pointcloud model
     (publish-footstep-planning-pointcloud-model-from-eus-pointcloud
      (get-pointcloud-within-faces (mapcar #'(lambda (b) (find-if #'(lambda (x) (memq :top (send x :id))) (send b :faces))) (list *b1* *b2* *b3*)) ;; top faces
                                   :resolution 20))
     ;; Plan
     (let* ((result (plan-footstep-from-goal-coords
                     (car (project-coords-on-to-plane2 (make-coords :pos #f(1300 0 0)) (list (cadr (send *b3* :faces))) #f(0 0 1)))
                     :start-coords (car (project-coords-on-to-plane2 (make-coords :pos #f(0 0 0)) (list (cadr (send *b1* :faces))) #f(0 0 1)))
                     )))
       (and result (footstep-array->coords result))))))

(deftest test-sample-stair-walking-1
  (assert
   (progn
     ;; Use pointcloud model
     (set-parameters-for-planning-on-poincloud-model)
     ;; Setup models
     (setq *blist* (mapcar #'(lambda (x) (make-cube 400 600 10)) (make-list 6)))
     (let ((tmp (float-vector 0 0 0)))
       (dolist (b (cdr *blist*))
         (send b :translate tmp)
         (send b :worldcoords)
         (setq tmp (v+ tmp (float-vector 400 0 100))))
       (objects *blist*))
     ;; Publis pointcloud model
     (publish-footstep-planning-pointcloud-model-from-eus-pointcloud
      (get-pointcloud-within-faces (mapcar #'(lambda (b) (find-if #'(lambda (x) (memq :top (send x :id))) (send b :faces))) *blist*) ;; top faces
                                   :resolution 20))
     ;; Plan
     (let* ((result (plan-footstep-from-goal-coords
                     (car (project-coords-on-to-plane2 (make-coords :pos (send (car (last *blist*)):worldpos)) (list (cadr (send (car (last *blist*)) :faces))) #f(0 0 1)))
                     :start-coords (car (project-coords-on-to-plane2 (make-coords :pos #f(0 0 0)) (list (cadr (send (car *blist*) :faces))) #f(0 0 1)))
                     )))
       (and result (footstep-array->coords result))))))

(deftest test-sample-slope-walking
  (assert
   (progn
     ;; Use pointcloud model
     (set-parameters-for-planning-on-poincloud-model)
     ;; Setup models
     (let ((slope-ang1 (deg2rad 7.5))
           (slope-ang2 (deg2rad -20))
           (slope-ang3 (deg2rad -5)))
       (setq *b1* (make-cube 1000 1000 10))
       (setq *b2* (make-cube 1000 1000 10))
       (setq *b3* (make-cube 1000 1000 10))
       (send *b1* :rotate slope-ang1 :y)
       (send *b2* :rotate slope-ang2 :y)
       (send *b3* :rotate slope-ang3 :y)
       (send *b2* :translate (float-vector 1000 0 (+ (* -500 (sin slope-ang1)) (* -500 (sin slope-ang2)))) :world)
       (send *b3* :translate (float-vector 2000 0 (+ (* -500 (sin slope-ang1)) (* -1000 (sin slope-ang2)) (* -500 (sin slope-ang3)))) :world)
       (send-all (list *b1* *b2* *b3*) :worldcoords)
       (objects (list *b1* *b2* *b3*))
       ;; Publis pointcloud model
       (publish-footstep-planning-pointcloud-model-from-eus-pointcloud
        (get-pointcloud-within-faces (mapcar #'(lambda (b) (find-if #'(lambda (x) (memq :top (send x :id))) (send b :faces))) (list *b1* *b2* *b3*)) ;; top faces
                                     :resolution 20))
       ;; Plan
       (let* ((result (plan-footstep-from-goal-coords
                       (car (project-coords-on-to-plane2 (make-coords :pos #f(2000 0 0)) (list (cadr (send *b3* :faces))) #f(0 0 1)))
                       :start-coords (car (project-coords-on-to-plane2 (make-coords :pos #f(0 0 0)) (list (cadr (send *b1* :faces))) #f(0 0 1)))
                       )))
         (and result (footstep-array->coords result)))))))

(run-all-tests)
(exit 0)
