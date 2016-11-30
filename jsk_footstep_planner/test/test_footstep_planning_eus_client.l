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

(run-all-tests)
(exit 0)
