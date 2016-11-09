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

(run-all-tests)
(exit 0)
