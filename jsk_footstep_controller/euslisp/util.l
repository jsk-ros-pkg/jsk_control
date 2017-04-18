(ros::load-ros-manifest "jsk_footstep_planner")
(ros::load-ros-manifest "dynamic_reconfigure")

(defun robot-interface-file (name)
  "You can get robot-interface file according to `name' argument.
You can create `*ri*' like 
(progn (load (robot-file (ros::get-param \"/robot/type\")))
       (init-robot-from-name (ros::get-param \"/robot/type\")))
"
  (let ((lower-name (string-downcase name)))
    (cond
     ((string= lower-name "pr2")
      "package://pr2eus/pr2-interface.l")
     ((string= lower-name "hrp2jsk")
      "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsk-interface.l")
     ((string= lower-name "hrp2jsknt")
      "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknt-interface.l")
     ((string= lower-name "hrp2jsknts")
      "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknts-interface.l")
     ((string= lower-name "staro")
      "package://hrpsys_ros_bridge_tutorials/euslisp/staro-interface.l")
     ((string= lower-name "jaxon")
      "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon-interface.l")
     ((string= lower-name "jaxon_red")
      "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon_red-interface.l")
     ((string= lower-name "urataleg")
      "package://hrpsys_ros_bridge_tutorials/euslisp/urataleg-interface.l")
     ((string= lower-name "samplerobot")
      "package://hrpsys_ros_bridge_tutorials/euslisp/samplerobot-interface.l")
     ((string= lower-name "hrp2w")
      "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2w-interface.l")
     ((string= lower-name "pepper")
      "package://peppereus/pepper-interface.l")
     ((string= lower-name "nao")
      "package://naoeus/euslisp/nao-interface.l")
     (t (error "uknown robot ~A" name))
     )))

(defun footstep->coords (f)
  (let ((c (ros::tf-pose->coords (send f :pose))))
    (cond ((= (send f :leg) jsk_footstep_msgs::Footstep::*LEFT*)
           (send c :name :lleg))
          ((= (send f :leg) jsk_footstep_msgs::Footstep::*RIGHT*)
           (send c :name :rleg)))
    c))
(defun footstep-array->coords (arr)
  (mapcar #'(lambda (f)
              (footstep->coords f))
          (send arr :footsteps)))

(defun print-readable-coords (coords)
  (dolist (c coords)
    (format t "(make-coords :name ~A :pos ~A :rpy '~A)~%"
            (send c :name)
            (send c :worldpos)
            (car (send c :rpy-angle)))))

(defun append-steps (now-steps next-steps)
  (cond
   ((eq (send (car (last now-steps)) :name) (send (car next-steps) :name))
    (append now-steps (cdr next-steps))
    )
   (t ;; different last/first foot
    ;;(append now-steps2 next-steps2)
    (append (butlast now-steps) (cdr next-steps))
    )))

(defun coords->ros-footstep (coords &key (leg))
  (let ((footstep (instance jsk_footstep_msgs::Footstep :init)))
    (send footstep :pose (ros::coords->tf-pose coords))
    (unless leg
      (setq leg (send coords :name))
      (unless leg
        (warn ";; :name or keyword leg should be set for footstep~%");
        (return-from coords->ros-footstep)))
    (cond
     ((or (eq leg :lleg)
          (eq leg :left)
          (string= leg "lleg")
          (string= leg "left"))
      (setq leg jsk_footstep_msgs::Footstep::*LEFT*)
      )
     ((or (eq leg :rleg)
          (eq leg :right)
          (string= leg "rleg")
          (string= leg "right"))
      (setq leg jsk_footstep_msgs::Footstep::*RIGHT*)
      )
     (t
      (warn "expected leg is :lleg, :rleg, :left and :right~%")
      ))
    (send footstep :leg leg)
    footstep))

(defun make-footsteps (steps &key (frame-id "map") (stamp (ros::time 0)))
  (let ((footstep-list (instance jsk_footstep_msgs::FootstepArray :init)))
    (send footstep-list :header :stamp stamp)
    (send footstep-list :header :frame_id frame-id)
    (send footstep-list :footsteps
          (mapcar #'coords->ros-footstep steps))
    footstep-list
    ))

(defun make-footstep-planning-msgs (start-coords target-coords
                                    &key (frame-id "map") (stamp (ros::time 0))
                                    (start-leg :lleg) (end-leg :lleg)
                                    (robot) (timeout)
                                    (default-half-offset-length (if robot (elt (cadr (memq :default-half-offset (send robot :footstep-parameter))) 1) 100))
                                    (lleg-offset (make-coords :pos (float-vector 0 default-half-offset-length 0)))
                                    (rleg-offset (make-coords :pos (float-vector 0 (- default-half-offset-length) 0)))
                                    (relative-target nil)
                                    &allow-other-keys)
  (let (lstart rstart
        lgoal rgoal)
    (cond
     ((listp start-coords)
      (setq lstart (car  start-coords)
            rstart (cadr start-coords)))
     (t
      (setq lstart (send (send start-coords :copy-worldcoords)
                         :transform lleg-offset))
      (setq rstart (send (send start-coords :copy-worldcoords)
                         :transform rleg-offset))
      ))
    (send lstart :name :lleg)
    (send rstart :name :rleg)

    (when (and relative-target (derivedp target-coords coordinates))
      (let* ((midc (midcoords 0.5 rstart lstart)))
        (send midc :transform target-coords)
        (setq target-coords midc)
        ))

    (cond
     ((listp target-coords)
      (setq lgoal (car  target-coords)
            rgoal (cadr target-coords)))
     (t
      (setq lgoal (send (send target-coords :copy-worldcoords)
                        :transform lleg-offset))
      (setq rgoal (send (send target-coords :copy-worldcoords)
                        :transform rleg-offset))
      ))
    (send lgoal :name :lleg)
    (send rgoal :name :rleg)

    (let ((initial-steps (if (eq start-leg :lleg) (list lstart rstart) (list rstart lstart)))
          (goal-steps    (if (eq end-leg :lleg)   (list lgoal rgoal) (list rgoal lgoal)))
          (goal (instance jsk_footstep_msgs::PlanFootstepsActionGoal :init)))
      (send goal :goal :initial_footstep
            (make-footsteps initial-steps :frame-id frame-id :stamp stamp))
      (send goal :goal :goal_footstep
            (make-footsteps goal-steps :frame-id frame-id :stamp stamp))
      (when timeout
        (let ((tm (instance ros::Time :init)))
          (send tm :from-sec timeout)
          (send goal :goal :timeout tm)))
      goal)
    ))

(defun make-coords-footstep-planning-msgs (target-coords
                                           &rest args
                                           &key (frame-id "map") (stamp (ros::time 0))
                                           (lleg-tf-name "lleg_end_coords")
                                           (rleg-tf-name "rleg_end_coords")
                                           (robot)
                                           &allow-other-keys)
  (let ((lstart (send *tfl* :lookup-transform frame-id lleg-tf-name stamp))
        (rstart (send *tfl* :lookup-transform frame-id rleg-tf-name stamp)))
    (apply #'make-footstep-planning-msgs
           (list lstart rstart) target-coords :frame-id frame-id :stamp stamp :robot robot args)
    ))
(defun make-go-pos-planning-msgs (x y th &rest args)
  (let* ((trans (make-coords :pos (float-vector (* x 1000) (* y 1000) 0))))
    (send trans :rotate (deg2rad th) :z)
    (apply #'make-coords-footstep-planning-msgs trans :relative-target t args)
    ))

(defun calc-step-error-on-map (steps map->destination
                               &key (min-remaining-steps 2) (debug t))
  (when (< (length steps) (max 2 min-remaining-steps))
    #|
    (let (support-leg map->ft end-coords)
      (if (eq (send map->destination :name) :lleg)
          (setq support-leg :lleg end-coords "lleg_end_coords")
        (setq support-leg :rleg end-coords "rleg_end_coords"))
      (setq map->ft (send *tfl* :lookup-transform "map" end-coords (ros::time 0)))
      (setq coords-difference
            (send map->ft :transformation map->destination))
      )
    |#
    (return-from calc-step-error-on-map)
    )
  (let (foot->laststep
        map->odom)
    (setq map->odom (send *tfl* :lookup-transform "map" "odom" (ros::time 0)))
    (setq map->laststep (send (send map->odom :copy-worldcoords) :transform
                              (elt steps (- (length steps) 2))))
    (setq coords-difference (send map->laststep :transformation map->destination))
    (when debug
      (pprint (list 'map->laststep map->laststep))
      (pprint (list 'map->destination map->destination))
      (pprint (list 'diff coords-difference)))
    coords-difference
    )
  )

(defun check-footstep-collision (current next &key (debug nil))
  ;; fix next footstep to avoid collision
  ;; return original next footstep if current and next would not cause collision
  (let ((current-foot (make-cube 235 145 20))
        (next-foot (make-cube 235 145 20)))

    (send current-foot :move-to (send current :coords) :world)
    (send next-foot :move-to (send next :coords) :world)
    (let ((dy 0))
      (while (pqp-collision-check-objects (list current-foot) (list next-foot))
        (if debug (ros::ros-info "Collision detected between footsteps. Modify next step."))
        ;;(pprint next-foot)
        (cond
         ((equal (send next :name) :lleg)
          (setq dy (+ 1 dy))
          (send next-foot :translate #f( 0 1 0) :local))
         ((equal (send next :name) :rleg)
          (setq dy (- dy 1))
          (send next-foot :translate #f( 0 -1 0) :local))
         )
        )
      (send next :translate (float-vector 0 dy 0 ) :local)
      (if debug (pprint next))
      )
    )
  next
  )

(defun refine-steps (remaining-steps
                     &key (offset 2) (expand-step) (exec nil)
                     (collision-avoid nil) (debug t))
  (let ((steps (butlast (car remaining-steps)))
        (idx (cadr remaining-steps))
        (large-error-p nil)
        istep newsteps trans tr-list)
    (setq istep (elt steps offset))
    (setq steps (subseq steps (+ offset 1)))
    ;;
    (when expand-step
      (let* ((last-cds (car (last steps))) ;; coordinates of last step
             (trans (send last-cds :transformation
                          (send (send last-cds :copy-worldcoords) :transform expand-step)
                          :world))
             (len (float (- (length steps) 2)))
             (cntr 0) (diff-norm-limit 10))
        (when debug
          (pprint (list 'trans trans expand-step))
          (pprint (list 'org-steps steps)))
        (mapcar #'(lambda (s)
                    (let ((tr (midcoords (/ cntr len) (make-coords) trans))
                          nm)
                      ;; limit trans
                      #|
                      (setq nm (norm (send tr :worldpos)))
                      (when (> nm 15)
                        (setq tr (midcoords (/ 15 nm) (make-coords) tr)))
                      |#
                      (setq nm (norm (send tr :worldpos)))
                      (ros::ros-warn "diff norm ~A" nm)
                      (when (> nm (* diff-norm-limit cntr))
                        (setq large-error-p t)
                        )
                      (push tr tr-list)
                      (send s :transform tr :world)
                      (incf cntr)
                      s)) (butlast steps))
        (send last-cds :transform trans :world)
        (if debug (pprint (list 'trans-steps steps)))
        ))
    ;;
    (cond
     (collision-avoid
      (let ((start-time (ros::time-now))
            fslist modifiedfs) ;; instant collision check
        (push istep fslist)
        (dotimes (cnt (length steps))
          (if (eq cnt 0)
              (setq modifiedfs (check-footstep-collision istep (elt steps cnt)))
            (setq modifiedfs (check-footstep-collision (elt steps (- cnt 1)) (elt steps cnt))))
          (push modifiedfs fslist)
          )
        (setq newsteps (reverse fslist))
        (when debug
          (let ((now (ros::time-now)))
            (ros::ros-info "took ~A sec for collision avoid"
                           (send (ros::time- now start-time) :to-sec))))
        ))
     (t
      (setq newsteps (append (list istep) steps))))

    (if debug (pprint (list 'new newsteps)))
    (when exec
      (if large-error-p
          (progn
            (ros::ros-warn "Too large error in step refine, cancel refine steps")
            ;; (send *ri* :go-stop)
            (if debug (pprint (list 'trans-list tr-list)))
            )
        (send *ri* :set-foot-steps-no-wait newsteps
              :overwrite-footstep-index (+ idx offset 1))
        )
      )
    ))

;; copied from obsoluted euslisp footstep planner: footstep_planner_util.l
(defun project-coords-on-to-plane2 (coords planes z-axis)
  (let ((point (send coords :worldpos)))
    ;; first, creating line from the point and z-axis
    ;; and, compute the points projected on the planes
    ;; x = P + aZ: line
    ;; nx + D = 0: plane
    ;; n(P + aZ) + D = 0
    ;; nP + anZ + D = 0
    ;; anZ = -(D + nP)
    ;; a = -(D + nP) / nZ
    (let ((candidates (mapcar #'(lambda (pln)
                                  (let ((projected-point
                                         (let ((plane-normal (send pln :normal))
                                               (plane-D (- (send pln :plane-distance (float-vector 0 0 0)))))
                                           (let ((alpha (/ (- plane-D (v. plane-normal point))
                                                           (v. plane-normal z-axis))))
                                             (v+ point (scale alpha z-axis))))))
                                    (if (not (eq (send pln :insidep projected-point) :outside))
                                        (let* ((n (send pln :normal))
                                               (nf (matrix-column
                                                    (send (send coords :copy-worldcoords) :worldrot) 2)))
                                          (if (< (v. nf n) 0)
                                              (setq n (scale -1.0 n)))
                                          (let ((ret (send coords :copy-worldcoords)))
                                            (send ret :locate projected-point :world)
                                            (if (eps= (norm (v* n nf)) 0)
                                                ret
                                              (let* ((b (v* nf n))
                                                     (b* (normalize-vector b))
                                                     (theta (asin (norm b))))
                                                (send ret :rotate theta b* :world)
                                                ret))
                                            ret)))))
                              planes)))
      (let ((non-null-candidates (remove-if #'null candidates)))
        (ros::ros-info "project coordinates to ~A planes" (length non-null-candidates))
        (ros::ros-info "  ~A planes" (length planes))
        (ros::ros-info "  ~A failed to project" (- (length candidates) (length non-null-candidates)))
        (if non-null-candidates non-null-candidates ;car is not good
          nil)))))

(defun project-footsteps (planes footstep-coords)
  (let (ret-coords projected-coords)    
    (if planes
        (progn 
          (setq projected-coords (mapcar #'(lambda (coords)
                                             (let* ((candidates (project-coords-on-to-plane2 coords planes (float-vector 0 0 1)))
                                                    (non-null-candidates (remove-if #'null candidates)) (ret nil))
                                               ;; now only check distance for validity but rotation should be considered
                                               (when (> (length non-null-candidates) 0)
                                                 (dolist (c non-null-candidates)
                                                   (if (> (norm (send (send coords :transformation c) :pos)) 100)
                                                       (ros::ros-warn "projected footstep is too distant (~A [mm])" (norm (send (send coords :transformation c) :pos)))
                                                     (if ret
                                                         (when (> (norm (send (send coords :transformation ret) :pos))
                                                                  (norm (send (send coords :transformation c) :pos))) ;; use nearest footcoords from original
                                                           (setq ret c)
                                                           (send ret :name (send coords :name))
                                                           )
                                                       (progn
                                                         (setq ret c)
                                                         (send ret :name (send coords :name)))
                                                       )
                                                     )
                                                   )
                                                 )
                                               ret
                                               )
                                             )
                                         footstep-coords))
          (if (member nil projected-coords)
              (progn 
                (ros::ros-warn "footstep projection is failed")
                (setq ret-coords footstep-coords))
            (progn
              (pprint "before projection:")
              (print-readable-coords footstep-coords)
              (pprint "after projection:")
              (print-readable-coords projected-coords)
              (setq ret-coords projected-coords))
            )
          )
      (progn
        (ros::ros-warn "plane is not subscribed")
        (setq ret-coords footstep-coords))
      )
    ret-coords
    )
  )

;; Functions to use footstep planning from euslisp
(defun initialize-eus-footstep-planning-client
  (&key (server "footstep_planner")
        (node-name "footstep_planner_client")
        (marker-topic "/footstep_from_marker")
        (pointcloud-topic "/accumulated_heightmap_pointcloud_static/output")
        (obstacle-topic "/footstep_planner/obstacle_model"))
  "Initialize footstep planning client from euslisp."
  (when node-name
    (ros::roseus node-name))
  ;; For obstacles
  (when obstacle-topic
    (ros::advertise obstacle-topic sensor_msgs::PointCloud2 1))
  (when pointcloud-topic
    (ros::advertise pointcloud-topic sensor_msgs::PointCloud2 1))
  (ros::load-ros-manifest "sensor_msgs")
  ;; For visualization
  (when marker-topic
    (ros::advertise marker-topic jsk_footstep_msgs::FootstepArray 1))
  ;; For footstep planning action server
  (setq *footstep-planning-client*
        (instance ros::simple-action-client :init
                  server jsk_footstep_msgs::PlanFootstepsAction))
  (unix:sleep 3)
  (ros::ros-info "waiting actionlib server")
  (send *footstep-planning-client* :wait-for-server)
  (ros::ros-info "waited actionlib server")
  *footstep-planning-client*
  )

(defun plan-footstep-from-goal-coords
  (goal-coords &key (publish-result t) (marker-topic "/footstep_from_marker")
               (client *footstep-planning-client*)
               (timeout 0.0) ;; if timeout == 0.0 then use default
               (start-coords (make-coords)) (frame-id "odom") (robot))
  "Plan footstep from goal-coords."
  (let ((goal (make-footstep-planning-msgs
               start-coords goal-coords :frame-id frame-id :robot robot :timeout timeout)))
    (ros::ros-info "sending goal")
    (send client :send-goal goal)
    (ros::ros-info "waiting for result")
    (send client :wait-for-result)
    ;; (ros::ros-info "result: ~A" (send *footstep-planning-client* :get-result))
    (when publish-result
      (ros::publish marker-topic
                    (send (send client :get-result) :result)))
    (send (send client :get-result) :result)
    ))

(defun set-heuristic (&key (heuristic "step_cost")
                           (server "footstep_planner")
                           (service-name (format nil "~A/set_parameters" server)))
  "change heuristic function of footstep_planner"
  (let ((req (instance dynamic_reconfigure::ReconfigureRequest :init)))
    (send req :config
          :strs (list (instance dynamic_reconfigure::StrParameter :init :name "heuristic" :value heuristic))
          )
    (ros::service-call service-name req)
    ))

(defun set-timeout (&key (timeout 10.0)
                         (server "footstep_planner")
                         (service-name (format nil "~A/set_parameters" server)))
  "change default timeout of footstep_planner"
  (let ((req (instance dynamic_reconfigure::ReconfigureRequest :init)))
    (send req :config
          :doubles (list (instance dynamic_reconfigure::DoubleParameter :init :name "planning_timeout" :value timeout))
          )
    (ros::service-call service-name req)
    ))

(defun publish-footstep-planning-obstacle-model-from-eus-pointcloud
  (a-pointcloud &key (frame-id "odom") (obstacle-topic "/footstep_planner/obstacle_model"))
  "Publish obsacle_model used in footstep planning.
   A eus-style pointcloud is required as an argument."
  (when obstacle-topic
    (ros::publish obstacle-topic
                  (make-ros-msg-from-eus-pointcloud a-pointcloud :frame frame-id)))
  )

(defun publish-footstep-planning-pointcloud-model-from-eus-pointcloud
  (a-pointcloud &key (frame-id "odom") (pointcloud-topic-name "/accumulated_heightmap_pointcloud_static/output"))
  "Publish obsacle_model used in footstep planning.
   A eus-style pointcloud is required as an argument."
  (when pointcloud-topic-name
    (ros::publish pointcloud-topic-name
                  (make-ros-msg-from-eus-pointcloud a-pointcloud :frame frame-id)))
  )

(defun get-pointcloud-within-bodies-2D
  (blist &key (return-point-cloud t) ((:resolution dif) 50.0))
  "Get pointcloud within bodies.
   blist is list of bodies.
   All bodies are assumed to be vertical prisms, so inclusion check is 2D."
  (labels
      ((get-points-within-object-2D-tmp
        (bb)
        (let* ((ret1) (vv (send bb :vertices))
               (xmin (elt (find-extream vv #'(lambda (x) (elt x 0)) #'<) 0))
               (xmax (elt (find-extream vv #'(lambda (x) (elt x 0)) #'>) 0))
               (ymin (elt (find-extream vv #'(lambda (x) (elt x 1)) #'<) 1))
               (ymax (elt (find-extream vv #'(lambda (x) (elt x 1)) #'>) 1))
               (zmin (elt (find-extream vv #'(lambda (x) (elt x 0)) #'<) 2))
               (zmax (elt (find-extream vv #'(lambda (x) (elt x 0)) #'>) 2)))
          (do ((x (- xmin dif) (+ dif x))) ((< (+ dif xmax) x))
              (do ((y (- ymin dif) (+ dif y))) ((< (+ ymax dif) y))
                  (let ((z (* 0.5 (+ zmin zmax))))
                    (when (not (eq :outside (send bb :insidep (float-vector x y z))))
                      (push (float-vector x y z) ret1))
                    )))
          ret1)))
    (let ((ret1 (apply #'append (mapcar #'get-points-within-object-2D-tmp blist))))
      (if return-point-cloud
          (instance pointcloud :init :points ret1)
        ret1))))

(defun get-pointcloud-within-faces
  (face-list &key (return-point-cloud t) ((:resolution dif) 50.0))
  "Get pointcloud within faces.
   face-list is list of face."
  (labels ((get-points-within-face
            (ff)
            (let* ((ret1) (global-vv (send ff :vertices))
                   (origin-coords
                    (make-coords :rot (make-matrix 3 3 (list (send (elt (send ff :edges) 0) :direction)
                                                             (v* (send ff :normal) (send (elt (send ff :edges) 0) :direction))
                                                             (send ff :normal)))
                                 :pos (cadr (send ff :centroid))))
                   (vv (mapcar #'(lambda (x) (send origin-coords :inverse-transform-vector x)) global-vv))
                   (xmin (elt (find-extream vv #'(lambda (x) (elt x 0)) #'<) 0))
                   (xmax (elt (find-extream vv #'(lambda (x) (elt x 0)) #'>) 0))
                   (ymin (elt (find-extream vv #'(lambda (x) (elt x 1)) #'<) 1))
                   (ymax (elt (find-extream vv #'(lambda (x) (elt x 1)) #'>) 1))
                   (tmp))
              (do ((x (- xmin dif) (+ dif x))) ((< (+ dif xmax) x))
                  (do ((y (- ymin dif) (+ dif y))) ((< (+ ymax dif) y))
                      (let ((tmp (send origin-coords :transform-vector (float-vector x y 0))))
                        (when (not (eq :outside (send ff :insidep tmp)))
                          (push tmp ret1))
                        )))
              ret1)))
    (let ((ret1 (apply #'append (mapcar #'get-points-within-face face-list))))
      (if return-point-cloud
          (instance pointcloud :init :points ret1)
        ret1)
      )))
