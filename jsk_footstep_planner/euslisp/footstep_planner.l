;; (load "models/arrow-object.l")
;; (load "models/single-arrow-object.l")
;;(load "~/ros/hydro_parent/src/jskeus/irteus/irtgraph.l")
;; memo
;; Footstep is represented in coordinates.
;; Several additional information are stored in property-list.
;;   * :projecting-grid
;;      Occupancy-Grid instance which is used to project the coordinates
;;      on planar region
;; these information too. It means that following should work.
;;      `(eq (get (get c :previous-coordinates) :projecting-grid)
;;           (get c :projecting-grid))`


(defclass footstep-parameter
  :super propertied-object
  :slots (legs-num))
;; footsteps = (footstep footstep ...)
;; footstep = (footstep-name . transformation)

(defmethod footstep-parameter
  (:init (&rest args
          &key
          ((:legs-num ln) 2)
          &allow-other-keys)
    (setf legs-num ln)
    (send* self :gen-footstep args)
    self)
  (:foot-vertices (leg coords)
    (error
     "You have to define :gen-footstep in subclass of footstep-parameter"))
  ;; common interface
  (:footstep-num ()
    0)
  (:gen-footstep ()
    (error
     "You have to define :gen-footstep in subclass of footstep-parameter"))
  (:move-robot-model (robot next-leg-coords support-leg-coords
                      next-leg robot-coords)
    (error
     "You have to define :move-robot-model in subclass of footstep-parameter"))
  (:tramsformation-by-leg-and-step-name (leg-name step-name)
    (error
     "You have to define :transformation-by-name in subclass of ~
footstep-parameter"))
  (:next-leg (prev-leg)
    (error
     "You have to define :next-leg-name in subclass of footstep-parameter"))
  (:next-step-coords (support-leg-coords l/r)
    (error
     "You have to define :next-step-coords in subclass of footstep-parameter"))
  ;; common function
  (:move-robot-model-by-step-name (robot leg-name step-name)
    (let ((trans (send self :transformation-by-leg-and-step-name
                       leg-name step-name)))
      (send robot :transform trans)))
  (:move-robot-model-by-step-names (robot start-leg step-names)
    (dolist (step step-names)
      (send self :move-robot-model-by-step-name robot start-leg step)
      ;; dirty...
      (setq start-leg (send self :next-leg start-leg))))
  )

(defun leg-index->leg-symbol (index)
  (if (eq index jsk_footstep_msgs::Footstep::*LEFT*)
      :lleg
    :rleg))

(defclass footstep-planner-graph
  :super graph
  :slots (thr rthr target-scene
          grow obstacles robot
          leg-object step-max
          rotate-max
          wd wp
          z-axis
          footstep-param planes grids
          goal-footstep
          goal-coords
          goal-leg-index
          near-side-goal
          roll-paranoid
          g-sensor-coords
          collision-robot-model timeout
          successablep-timer
          transition-limit
          successors-timer append-footstep-candidates-by-grids-timer
          is-face-possible-to-put-on-grid-timer))

(defmethod footstep-planner-graph
  (:init (&rest args
          &key ((:scene sc) nil)
               ((:scene-grow sg) 0.0)
               ((:roll-paranoid rp) nil)
               ((:grow g) 0.2)
               ((:timeout tm) nil)
               ((:wd d) 1.0)
               ((:planes ps) nil)
               ((:grids gs) nil)
               ((:z-axis z) nil)
               ((:g-sensor-coords gsc) nil)
               ((:wp p) 0.0)
               ((:robot r) nil)
               ((:leg-object lo) nil)
               ((:footstep-parameter param))
               ((:transition-limit tlimit) nil)
               ((:step-max sm) 100.0)
               ((:rotate-max rm) (deg2ead 45.0))
               ((:thr th) 100.0)
               ((:obstacles ob) nil)
               ((:rthr rth) (deg2rad 10))
               ((:collision-robot-model crm) nil)
               &allow-other-keys)
    (setq g-sensor-coords gsc)
    (setq roll-paranoid rp)
    (setq transition-limit tlimit)
    (setq successablep-timer (instance counter-timer :init ":successablep"))
    (setq successors-timer (instance counter-timer :init ":successors"))
    (setq append-footstep-candidates-by-grids-timer (instance counter-timer :init "append-footstep-candidates-by-grids"))
    (setq is-face-possible-to-put-on-grid-timer (instance counter-timer :init "is-face-possible-to-put-on-grid"))
    (setq footstep-param param)
    (setq target-scene sc)
    (setq timeout tm)
    (setq planes ps)
    (setq grids gs)
    (setq z-axis z)
    (if target-scene
        (setq obstacles
              (remove-if #'null
                         (append
                          ob
                          (mapcar
                           #'(lambda (x) (send x :model-obb-box :grow sg))
                          (remove-if
                           #'(lambda (x) (derivedp x ground))
                            (send target-scene :all-objects))))))
      (setq obstacles ob))
    (setq collision-robot-model crm)
    (setq step-max sm)
    (setq rotate-max rm)
    (setq grow g)
    (setq robot r)
    (setq thr th)
    (setq rthr rth)
    (setq leg-object lo)
    (setq wd d)
    (setq wp p)
    ;; (unless robot
    ;;   (warn "[WARN] footstep planning works in no-robot model mode~%")
    ;;   (if collision-robot-model
    ;;       (warn "[INFO] use collision robot model for collision check~%")
    ;;     (warn "[INFO] use simple leg model for collision check~%")))
    self)
  (:timeout () timeout)
  (:grids () grids)
  (:report ()
    (send successablep-timer :report)
    (send successors-timer :report)
    (send append-footstep-candidates-by-grids-timer :report)
    (send is-face-possible-to-put-on-grid-timer :report)
    )
  (:goal-footstep (&optional arg)
    (if arg (setq goal-footstep arg))
    goal-footstep)
  (:goal-leg-index ()
    goal-leg-index)
  (:goal-coords ()
    goal-coords)
  (:setup-goal (initial-coords)
    ;; initial-coords := [(leg-index . coordinate) ...]
    (let ((distances (mapcar #'(lambda (g)
                                 (let ((leg-index (car g))
                                       (leg-coords (cdr g)))
                                   ;; find
                                   (let ((same-leg (find-if #'(lambda (i) (eq leg-index (car i)))
                                                            initial-coords)))
                                     (cons leg-index
                                           (distance (send leg-coords :worldpos)
                                                     (send (cdr same-leg) :worldpos))))))
                             goal-footstep)))
      ;; distances := [(leg-index . distance) ...]
      (let ((min-distance (cdr (car distances)))
            (min-index (car (car distances))))
        (dolist (d (cdr distances))
          (when (> min-distance (cdr d))
            (setq min-distance (cdr d))
            (setq min-index (car d)))
          )
        (ros::ros-info "minimum index is ~A, distance is ~A~%" min-index min-distance)
        (setq goal-leg-index (leg-index->leg-symbol min-index))
        (setq goal-corods (find-if #'(lambda (g) (eq (car g) goal-leg-index)) goal-footstep))
        )))
  (:goal-footstep-coords (l/r)
    (cdr (assoc l/r goal-footstep)))
  (:goal-test (state &key (debug-view nil))
    (let ((current-coords (send state :worldcoords)))
      (let ((ndiff-pos (distance (send current-coords :worldpos)
                                 (send (send self :goal-footstep-coords (send state :l/r-index)) :worldpos))))
        (if (or (not thr) (< ndiff-pos thr))
            (let* ((diff-rot (send current-coords
                                   :difference-rotation (send self :goal-footstep-coords (send state :l/r-index))))
                   (ndiff-rot (norm diff-rot)))
              ;; when thr or rthr is nil, test returns t
              (or (not rthr) (< ndiff-rot rthr)))))))
  (:goal-test2 (state &key (debug-view nil))
    (let ((current-coords (send state :robot-coords)))
      (let ((ndiff-pos (distance (send current-coords :worldpos)
                                 (send goal-state :worldpos))))
        (if (or (not thr) (< ndiff-pos thr))
            (let* ((diff-rot (send current-coords
                                   :difference-rotation goal-state))
                   (ndiff-rot (norm diff-rot)))
              ;; when thr or rthr is nil, test returns t
              (or (not rthr) (< ndiff-rot rthr)))))))
  (:path-cost (from-node action to-state)
    ;; [J.Kuffner et. al.,
    ;;  Footstep Planning Among Obstacles for Biped Robots,
    ;;  ICRA 2001]
    ;; L(Q) = w_d D(N_Q) + w_p p(N_Q) + w_g X(Q, Q_g)
    ;;        ^^^^^^^^^^^^^^^^^^^^^^^
    (+ (* wd (send self :depth-cost from-node action to-state))
       (* wp (send self :penalty-cost from-node action to-state))))
  (:depth-cost (from-node action to-state)
    ;; [J.Kuffner et. al.,
    ;;  Footstep Planning Among Obstacles for Biped Robots,
    ;;  ICRA 2001]
    ;; L(Q) = w_d D(N_Q) + w_p p(N_Q) + w_g X(Q, Q_g)
    ;;            ^^^^^^
    (1+ (length (send from-node :path))))
  (:penalty-cost (from-node action to-state)
    ;; not implemented
    ;; [J.Kuffner et. al.,
    ;;  Footstep Planning Among Obstacles for Biped Robots,
    ;;  ICRA 2001]
    ;; L(Q) = w_d D(N_Q) + w_p p(N_Q) + w_g X(Q, Q_g)
    ;;                         ^^^^^
    0.0)
  (:move-robot-model (next-leg-coords support-leg-coords next-leg robot-coords)
    (when robot                         ;only use with robot model
      (send footstep-param :move-robot-model
            robot
            next-leg-coords
            support-leg-coords
            next-leg
            robot-coords)))
  (:successablep (&rest args
                        &key
                        (debug nil)
                        (next-leg-coords nil)
                        (robot-coords nil)
                        (next-leg-face nil)
                        &allow-other-keys)
    "this method is called after robot model is moved."
    (bench-timer
     successablep-timer
     (incf *evaled-node-num*)
     (and
      ;; (cond (collision-robot-model
      ;;        (not (send self :collision-with-obstacles-collision-robot-model-p
      ;;                   (send (send robot-coords :copy-worldcoords)
      ;;                         :transform (send footstep-param :foot->robot))
      ;;                   :debug debug)))
      ;;       (robot
      ;;        (not (send self :collision-with-obstacles-p robot :debug debug)))
      ;;       (t
      ;;        (not (send self :collision-with-obstacles-no-robot-mode-p
      ;;                   next-leg-coords :debug debug))))
      t
      ;; plane check
      (cond ((and next-leg-face planes)
             (face-on-faces next-leg-face planes))
            ((and next-leg-face grids)
             (bench-timer2
              is-face-possible-to-put-on-grid-timer
              (face-placable-p next-leg-face grids)))
            (t
             t)))))
  (:successors-without-grids (state &key (verbose nil))
    (let* ((current-leg (send state :l/r))
           (current-step-coords (send state :worldcoords))
           (next-leg (send footstep-param :next-leg current-leg)))
      (let ((next-coords-candidate (send footstep-param :next-step-coords
                                         (send state :worldcoords)
                                         next-leg
                                         (send state :step-name))))
        ;;(ros::ros-info "~A candidates" (length next-coords-candidate))
        ;; remove steps collide with obstacles
        next-coords-candidate)))
  (:successors-with-grids (state &key (verbose nil))
    (let* ((current-leg (send state :l/r))
           (current-step-coords (send state :worldcoords))
           (current-grid (get current-step-coords :projecting-grid))
           (next-leg (send footstep-param :next-leg current-leg)))
      ;; check footstep level transition
      ;;     prev       current     next
      ;; 1. level0  ->   level1  -> level1
      ;; 2. level0  ->   level0  -> level1
      ;; 3. level0  ->   level0  -> level0
      ;; 4. level0  ->   level1  -> level2 (not supported)
      ;; First is this the case of 1?
      (if (not (eq (get (send state :prev-coords) :projecting-grid)
                   (get current-step-coords :projecting-grid)))
          ;; case 1
          (let ((next-coord-candidate
                 (send footstep-param :transition-second-step-coords
                       current-step-coords
                       next-leg
                       (send state :step-name))))
            (project-and-remove-on-grids
             next-coord-candidate (list current-grid) t))
        ;; case 2-4
        (let ((same-level-candidate       ;case  3
                (send footstep-param :same-grid-next-steps
                      current-step-coords
                      next-leg
                      (send state :step-name)))
              (different-level-candidate ;case 2
               (send footstep-param :transition-first-step-coords
                       current-step-coords
                       next-leg
                       (send state :step-name))))
          (append
           (remove-not-on-grids same-level-candidate current-grid)
           (remove-if-not
            #'(lambda (x)
                (check-transition-limit current-step-coords (cdr x) transition-limit))
            (project-and-remove-on-grids
             different-level-candidate grids (list current-grid) t
             ;; For roll-paranoid
             roll-paranoid g-sensor-coords current-grid
             next-leg current-step-coords)))))))
  (:successors (state &key (verbose nil))
    (bench-timer
     successors-timer
     (let* ((next-coords-candidate
             (cond
              (grids (send self :successors-with-grids state :verbose verbose))
              (t (send self :successors-without-grids state :verbose verbose))))
            (current-step-coords (send state :worldcoords))
            (current-leg (send state :l/r))
            (next-leg (send footstep-param :next-leg current-leg)))
       (let ((ret nil))
           (dolist (c next-coords-candidate)
             (let ((cdr-c (cdr c)))
               (let ((robot-coords
                      (send footstep-param :calc-robot-coords
                            cdr-c current-step-coords)))
                 (when robot (send robot :worldpos))
                 (setf (cdr c)
                       (instance step-node :init
                                 :projecting-grid (get cdr-c :projecting-grid)
                                 :worldcoords cdr-c
                                 :leg-face (instance face :init
                                                     :vertices
                                                     (send footstep-param :foot-vertices
                                                           next-leg cdr-c))
                                 :robot-coords robot-coords
                                 :prev-coords current-step-coords
                                 :angle-vector nil
                                 ;;(send robot :angle-vector)
                                 :step-name (car c)
                                 :l/r next-leg))
                 (push c ret))))
           ret))))
  (:collision-with-obstacles-collision-robot-model-p (coords &key (debug nil))
    (send collision-robot-model :newcoords coords)
    (send collision-robot-model :worldpos)
    (when debug
      (send-all (send self :obstacles) :draw-on :flush nil)
      (send collision-robot-model :draw-on :flush t))
      (dolist (x (send self :obstacles))
        (if (= (pqp-collision-check collision-robot-model x) 1)
            (return-from :collision-with-obstacles-collision-robot-model-p
              t)))
    nil)
  (:collision-with-obstacles-no-robot-mode-p (coords &key (debug nil))
    "returns t when colliding. when no robot model mode, we utilize collision
check between leg-object and obstacles."
    ;; move leg-object to coords
    ;; NB: it makes a copy, its slow...
    (send leg-object :newcoords coords)
    (send leg-object :worldpos)
    (let ((coords-worldpos (send coords :worldpos)))
      (dolist (x (send self :obstacles))
        (if (< (distance (send x :worldpos) (send coords :worldpos))
               1500.0)
            (if (= (pqp-collision-check leg-object x) 1)
                (return-from :collision-with-obstacles-no-robot-mode-p t)))))
    ;; all passed
    nil)
  (:collision-with-obstacles-p (leg-box &key (debug nil))
    "returns t when colliding."
    (dolist (x (send self :obstacles))
      (if (< (distance (send x :worldpos) (send robot :worldpos))
             1500.0)
          (if (pqp-collision-check-objects (send robot :links)
                                           (list x)
                                           :fat 1.4)
              (return-from :collision-with-obstacles-p t))))
    ;; all passed
    nil)
  ;; accessors
  (:get-leg-object ()
    leg-object)
  (:scene ()
    scene)
  (:thr ()
    thr)
  (:rthr ()
    rthr)
  (:grow ()
    grow)
  (:robot ()
    robot)
  (:step-max ()
    step-max)
  (:rotate-max ()
    rotate-max)
  (:obstacles ()
    obstacles)
  )

(defclass step-node
  :super node
  :slots (worldcoords l/r robot-coords prev-coords angle-vector step-name
                      root leg-face projecting-grid))

(defmethod step-node
  (:init (&key ((:worldcoords wc) nil)
               ((:robot-coords rc) nil)
               ((:prev-coords pc) nil)
               ((:angle-vector av) nil)
               ((:root rt) nil)
               ((:projecting-grid pg) nil)
               ((:l/r lr) nil)
               ((:leg-face lg) nil)
               ((:step-name sn) nil))
    (setq projecting-grid pg)
    (setq root rt)
    (setq worldcoords wc)
    (setq step-name sn)
    (setq leg-face lg)
    (setq prev-coords pc)
    (setq robot-coords rc)
    (setq angle-vector av)
    (setq l/r lr)
    self)
  ;; accessor
  (:root () root)
  (:projecting-grid () projecting-grid)
  (:robot-coords () robot-coords)
  (:leg-face () leg-face)
  ;;(:robot-coords () worldcoords)
  (:prev-coords () prev-coords)
  (:worldcoords () worldcoords)
  (:angle-vector (&optional (arg nil))
    (if arg (setq angle-vector arg))
    angle-vector)
  (:l/r () l/r)
  (:l/r-index ()
    (case l/r
          (:lleg jsk_footstep_msgs::Footstep::*LEFT*)
          (:rleg jsk_footstep_msgs::Footstep::*RIGHT*)))
  (:step-name () step-name)
  )

(defun step-node= (a b)
  (if (or (send a :root) (send b :root))
      nil
    (and (eq (send a :l/r) (send b :l/r))
         (< (distance (send (send a :worldcoords) :worldpos)
                      (send (send b :worldcoords) :worldpos))
            10.0)
         (< (norm (send (send a :worldcoords) :difference-rotation
                        (send b :worldcoords)))
            #.(deg2rad 10.0)))))

(defun distance2d (a b)
  (let ((diff (v- a b)))
    (sqrt (+ (* (elt diff 0) (elt diff 0))
             (* (elt diff 1) (elt diff 1))))))
  
;; it has A* in the name of class, but
;; this algorithm is not "A*" in exact meaning.
;; Because estimation of the number of rotation and translation steps in
;; heuristic may be bigger than cost function.
;; However, in the most case, the algorithm behaves as A* algorithm.
(defclass a*-footstep-graph-search-solver
  :super a*-graph-search-solver
  ;;:super depth-first-graph-search-solver
  :slots (count target-problem heuristic-method wg arrived-at-position-p
          render-leg-object debug-counter))
                

(defmethod a*-footstep-graph-search-solver
  (:init (w m plbm &rest args
            &key ((:render-leg-object rlo) nil) &allow-other-keys)
    (setq wg w)
    (setq heuristic-method m)
    (setq debug-counter 0)
    (setq render-leg-object rlo)
    (send-super :init plbm)
    ;;(send-super :init)
    self)
  ;; override :solve-init and :pop-from-open-list for debug view.
  (:solve-init (prblm)
    (setq target-problem prblm)         ;for debug view
    (send-super :solve-init prblm))
  (:find-node-in-close-list (n)
    "I want to replace 'find' in this method by dynamic kd-tree
building and searching."
    (find (send n :state) close-list :test #'step-node=))
  ;; (:pop-from-open-list (&key (debug nil))
  ;;   (send-super :pop-from-open-list :debug t))
  (:null-open-list? ()
    (when (ros::time< (send target-problem :timeout) (ros::time-now))
      (ros::ros-warn "Planning take ~A secs, Timeout"
        (- (send (send target-problem :timeout) :to-sec) (send (ros::time-now) :to-sec)))
      (return-from :null-open-list? t))
    (let ((foundp nil)
          (rejected-counter 0))
      (while (not foundp)
        (if (send-super :null-open-list?)
            (return-from :null-open-list? t)
          (let ((candidate-node (send self :pop-from-open-list)))
            (cond ((null candidate-node)
                   (return-from :null-open-list? nil))
                  ((null (send (send candidate-node :state) :leg-face)) ;start state
                   (send self :add-to-open-list candidate-node)
                   (setq foundp t))
                  ((face-placable-p
                    (send (send candidate-node :state) :leg-face)
                    (send (send candidate-node :state) :projecting-grid))
                   (send self :add-to-open-list candidate-node)
                   (when *gui-debug*
                     (send (send (send candidate-node :state) :leg-face)
                           :draw-on :flush nil))
                   (setq foundp t))
                  (t
                   (when *gui-debug*
                     (send (send (send candidate-node :state) :leg-face)
                           :draw-on :flush nil :color (float-vector 0 1 0)))
                   (incf rejected-counter)
                   nil))
            )))
      (incf debug-counter)
      (when *gui-debug*
        (send *irtviewer* :flush)
        (x::window-main-one))
      (not foundp)))
  
  (:hn (n p)
    ;; [J.Kuffner et. al.,
    ;;  Footstep Planning Among Obstacles for Biped Robots,
    ;;  ICRA 2001]
    ;; L(Q) = w_d D(N_Q) + w_p p(N_Q) + w_g X(Q, Q_g)
    ;;                                 ^^^^^^^^^^^^^^
    (* wg (send self heuristic-method n p)))
  (:straight-heuristic (n p)
    ;; n => next node
    ;; p => problem
    ;; In the kuffner thesis, only use estimation of translation step,
    ;; but in this implementation, I use estimation of rotation step too.
    (let ((goal-coords (send p :goal-state))
          (step (send p :step-max))
          (rotate (send p :rotate-max))
          (current-coords
           (send (send n :state) :robot-coords)))
      (+
       ;; translation...
       (/ (distance (send goal-coords :worldpos)
                    (send current-coords :worldpos))
          step)
       ;; rotation...
       (/ (norm (send goal-coords :difference-rotation
                      current-coords))
          rotate))))
  (:manhattan-heuristic (n p)
    ;; manhattan distance
    (let ((goal-coords (send p :goal-state))
          (step (send p :step-max))
          (rotate (send p :rotate-max))
          (current-coords
           (send (send n :state) :robot-coords)))
      (+
       (/ (manhattan-distance goal-coords
                              current-coords)
          step)
       (/ (norm (send goal-coords :difference-rotation
                      current-coords))
          rotate))))
  (:stepcost-heuristic (n p)
    ;; n => next node
    ;; p => problem
    ;; In the kuffner thesis, only use estimation of translation step,
    ;; but in this implementation, I use estimation of rotation step too.
    ;; this method is implemented by K.Okada.
    (let* ((goal-coords (send p :goal-state))
           (step (send p :step-max))
           (rotate (send p :rotate-max))
           (current-coords
            (send (send n :state) :robot-coords))
           (v1 (normalize-vector (v- (send goal-coords :worldpos)
                                     (send current-coords :worldpos))))
           (v2 (matrix-column (send current-coords :worldrot) 0)))
      (+
       ;; translation...
       (/ (distance (send goal-coords :worldpos)
                    (send current-coords :worldpos))
          step)
       ;; rotation...
       (* 2 (floor (/ (acos (v. v1 v2)) rotate))))))
  (:stepcost-half-heuristic (n p)
    ;; n => next node
    ;; p => problem
    ;; In the kuffner thesis, only use estimation of translation step,
    ;; but in this implementation, I use estimation of rotation step too.
    ;; this method is implemented by K.Okada.
    (let* ((goal-coords (send p :goal-state))
           (step (send p :step-max))
           (rotate (send p :rotate-max))
           (current-coords
            (send (send n :state) :robot-coords))
           (v1 (normalize-vector (v- (send goal-coords :worldpos)
                                     (send current-coords :worldpos))))
           (v2 (matrix-column (send current-coords :worldrot) 0)))
      (+
       ;; translation...
       (/ (distance (send goal-coords :worldpos)
                    (send current-coords :worldpos))
          step)
       ;; rotation...
       (* 1 (floor (/ (acos (v. v1 v2)) rotate))))))
  (:stepcost-heuristic* (n p)
    ;; this method is implemented by R.Ueda based on :stepcost-heuristic
    ;; written by K.Okada.
    ;; this heuristic can solve footstep planning with rotation constraint!
    (let* ((goal-coords (send p :goal-footstep-coords (send (send n :state) :l/r-index)))
           (step (send p :step-max))
           (rotate (send p :rotate-max))
           (current-coords
            (send (send n :state) :worldcoords))
           (v1 (normalize-vector (v- (send goal-coords :worldpos)
                                     (send current-coords :worldpos))))
           (v2 (matrix-column (send current-coords :worldrot) 0))
           (v3 (matrix-column (send goal-coords :worldrot) 0)))
      (let ((trans-cost (* (/ (distance (send goal-coords :worldpos)
                                        (send current-coords :worldpos))
                              step) 2)))
        (+ trans-cost
               ;; rotation of direction
               (* 2  (/ (acos (v. v1 v2)) rotate))
               ;; rotation of goal
               (* 2 (/ (acos (v. v1 v3)) rotate))))))
        ;; (if (> trans-cost 1)
        ;;     (+ trans-cost
        ;;        ;; rotation of direction
        ;;        (* 2  (/ (acos (v. v1 v2)) rotate))
        ;;        ;; rotation of goal
        ;;        (* 2 (/ (acos (v. v1 v3)) rotate)))
        ;;   ;; rotation of goal
        ;;   (* 2 (/ (acos (v. v1 v3)) rotate))))))
  (:stepcost-heuristic** (n p)
    (let* ((start-state (send p :start-state))
           (initial-expected-cost (send self :stepcost-heuristic* (instance solver-node :init start-state) p))
           (cost (send self :stepcost-heuristic* n p)))
      (let ((rate (/ cost initial-expected-cost)))
        (* rate rate cost))))
  (:stepcost-z-heuristic* (n p)
    (let* ((start-state (send p :start-state))
           (initial-expected-cost (send self :stepcost-z-heuristic (instance solver-node :init start-state) p))
           (cost (send self :stepcost-z-heuristic n p)))
      (let ((rate (/ cost initial-expected-cost)))
        (* rate rate cost))))
  (:stepcost-z-heuristic (n p)
    (let* ((goal-coords (send p :goal-footstep-coords (send (send n :state) :l/r-index)))
           (z-factor 10.0)
           (step (send p :step-max))
           (rotate (send p :rotate-max))
           (leg-coords
            (send (send n :state) :worldcoords))
           (v1 (normalize-vector (v- (send goal-coords :worldpos)
                                     (send leg-coords :worldpos))))
           (v2 (matrix-column (send leg-coords :worldrot) 0))
           (v3 (matrix-column (send goal-coords :worldrot) 0)))
      (let* ((2d-trans-cost (* (/ (distance2d (send goal-coords :worldpos)
                                              (send leg-coords :worldpos))
                                  step) 2))
             (z-trans-cost (* (* (/ (abs (elt (v- (send goal-coords :worldpos)
                                                  (send leg-coords :worldpos)) 2))
                                    100)
                                 2) z-factor))
             (trans-cost (+ 2d-trans-cost z-trans-cost)))
        ;;(format t "z-trans-cost ~A~%" z-trans-cost)
        (cond
         ;; ((< trans-cost 1)
         ;;  (+ trans-cost (* 2 (/ (acos (v. v1 v3)) rotate))))
         (t
          (+ trans-cost
             ;; rotation of direction
             (* 2  (/ (acos (v. v1 v2)) rotate))
             ;; rotation of goal
             (* 2 (/ (acos (v. v1 v3)) rotate))))))))
  
  ;; for visualize
  (:vertices ()
    (when target-problem
      (list (send (send (send target-problem :start-state) :worldcoords)
                  :worldpos)
            (send (send target-problem :goal-state) :worldpos))
      (list (float-vector 0 0 0))))     ;NB: we cannot (objects (list solver))
  (:collision-check-objects ()
    nil)
  (:draw (vwer)
    (sys::mutex
      gl::*opengl-lock*
      (gl::glPushAttrib gl::GL_ALL_ATTRIB_BITS)
      (let ((offset (float-vector 0 0 50))
            (buf (float-vector 0 0 0)))
        (unwind-protect
            ;; draw nodes in close list
            (dolist (n open-list)
              (when (send n :parent)
                ;; draw line
                (unwind-protect
                    (progn
                      (gl::glDisable gl::GL_LIGHTING)
                      (gl::glBegin gl::GL_LINES)
                      (gl::glColor3fv #.(float-vector 1 0 0))
                      (gl::glVertex3fv
                        (v+ 
                         (send (send (send n :state) :worldcoords) :worldpos)
                         offset buf))
                      (gl::glVertex3fv 
                        (v+
                         (send (send (send (send n :parent) :state)
                                     :worldcoords) :worldpos)
                         offset buf))
                      (gl::glEnd))
                  (gl::glEnable gl::GL_LIGHTING))))
          (gl::glPopAttrib)))))
  )

(defun automatically-select-start-leg (from to)
  (let ((diff-y (elt_y (send from :difference-position to))))
    (if (> diff-y 0) :lleg :rleg)))


(defun make-footstep-visualizer (o r)
  (instance footstep-visualizer :init o r))

;;(provide :footstep-planner)

(defclass ros-footstep-parameter
  :super footstep-parameter
  :slots (left-footsteps
          right-footsteps
          left-first-transition-footsteps
          right-first-transition-footsteps
          left-second-transition-footsteps
          right-second-transition-footsteps
          size-x size-y size-z
          ))
(defmethod ros-footstep-parameter
  (:init (x y)
    (setq size-x x)
    (setq size-y y)
    (send-super :init :legs-num 2))
  (:left-footsteps ()
    left-footsteps)
  (:right-footsteps ()
    right-footsteps)
  (:foot-vertices-old (leg coords)
    ;;     x     x
    ;;
    ;;        c
    ;;
    ;;     x     x
    (let* ((size-x/2 (/ size-x 2))
           (size-y/2 (/ size-y 2))
           (local-vertices (list
                            (float-vector size-x/2 size-y/2 0)
                            (float-vector (- size-x/2) size-y/2 0)
                            (float-vector (- size-x/2) (- size-y/2) 0)
                            (float-vector size-x/2 (- size-y/2) 0))))
      ;;(send coords :worldpos)
      (mapcar #'(lambda (v)
                  (let ((vv (send coords :transform-vector v)))
                    ;;(format t "~A -> ~A~%" v vv)
                    vv))
              local-vertices)))
  (:foot-vertices (leg coords)
    ;;     x     x
    ;;
    ;;        c
    ;;
    ;;     x     x
    (let* ((size-x/2 (/ size-x 2))
           (size-y/2 (/ size-y 2))
           (local-vertices (list
                            (float-vector size-x/2 size-y/2 0)
                            (float-vector (- size-x/2) size-y/2 0)
                            (float-vector (- size-x/2) (- size-y/2) 0)
                            (float-vector size-x/2 (- size-y/2) 0))))
      (dolist (v local-vertices)
        (v+ v *leg-offset* v))
      ;;(send coords :worldpos)
      (mapcar #'(lambda (v)
                  (let ((vv (send coords :transform-vector v)))
                    vv))
              local-vertices)))
  (:gen-footstep (&rest args)
    (send self :gen-left-footstep)
    (send self :gen-right-footstep)
    (send self :gen-left-first-transition-footstep)
    (send self :gen-right-first-transition-footstep)
    (send self :gen-left-second-transition-footstep)
    (send self :gen-right-second-transition-footstep)
    )
  (:_gen-footstep (successors l/r &optional reversed)
    (let ((footsteps (mapcar
                      #'(lambda (x)
                          (if (not reversed)
                              (send x :copy-worldcoords)
                            (let ((pos (send x :worldpos))
                                  (theta (car (car (send x :rpy-angle)))))
                              (make-coords :pos (float-vector (elt pos 0)
                                                              (- (elt pos 1))
                                                              0)
                                           :rpy (float-vector (- theta) 0 0)))))
                      successors)))
      (dotimes (i (length footsteps))
        (let ((f (elt footsteps i)))
          (send self :gen-step l/r i f)))
      ))
  (:gen-left-footstep ()
    (send self :_gen-footstep *footstep-successors* :left))
  (:gen-right-footstep ()
    (send self :_gen-footstep *footstep-successors* :right t))
  (:gen-left-first-transition-footstep ()
    (send self :_gen-footstep *footstep-transifion-first-successors* :left-first-transition))
  (:gen-right-first-transition-footstep ()
    (send self :_gen-footstep *footstep-transifion-first-successors* :right-first-transition t))
  (:gen-left-second-transition-footstep ()
    (send self :_gen-footstep *footstep-transifion-second-successors* :left-second-transition))
  (:gen-right-second-transition-footstep ()
    (send self :_gen-footstep *footstep-transifion-second-successors* :right-second-transition t))
  (:gen-step (l/r name coords)
    (let ((s (cons name coords)))
      (case l/r
        ((:left :lleg) (push s left-footsteps))
        ((:right :rleg) (push s right-footsteps))
        ((:left-first-transition :lleg-first-transition)
         (push s left-first-transition-footsteps))
        ((:right-first-transition :rleg-first-transition)
         (push s right-first-transition-footsteps))
        ((:left-second-transition :lleg-second-transition)
         (push s left-second-transition-footsteps))
        ((:right-second-transition :rleg-second-transition)
         (push s right-second-transition-footsteps))
        (t (error "unknown leg ~A" l/r)))
      ))
  (:transformation-by-leg-and-step-name (leg-name step-name)
    (case leg-name
      ((:left :lleg) (cdr (assoc step-name left-footsteps)))
      ((:right :rleg) (cdr (assoc step-name right-footsteps)))
      (t (error "unknown leg ~A" leg-name))))
  (:next-leg (l/r)
    (if (eq l/r :lleg) :rleg :lleg))
  (:next-step-coords (support-leg-coords l/r &optional (name nil))
    (let ((transformations
           (case l/r
             ((:left :lleg) left-footsteps)
             ((:right :rleg) right-footsteps)
             ((:left-first-transition :lleg-first-transition)
              left-first-transition-footsteps)
             ((:right-first-transition :rleg-first-transition)
              right-first-transition-footsteps)
             ((:left-second-transition :lleg-second-transition)
              left-second-transition-footsteps)
             ((:right-second-transition :rleg-second-transition)
              right-second-transition-footsteps)
             (t (error "unknown leg ~A" l/r)))))
      (mapcar #'(lambda (trans)
                  (let ((next-pose
                         (send (send support-leg-coords :copy-worldcoords)
                               :transform (cdr trans))))
                    ;; Keep plist-values
                    (setf (get next-pose :projecting-grid)
                          (get support-leg-coords :projecting-grid))
                    ;; (unless (get support-leg-coords :projecting-grid)
                    ;;   (ros::ros-error "projecting-grid is nil!")
                    ;;   (error "projecting-grid is nil!"))
                    (cons (car trans) next-pose)))
              transformations)))
  (:transition-second-step-coords (support-leg-coords l/r &optional (name nil))
    (send self :next-step-coords support-leg-coords
          (case l/r
           ((:left :lleg) :left-second-transition)
           ((:right :rleg) :right-second-transition))
          name))
  (:transition-first-step-coords (support-leg-coords l/r &optional (name nil))
    (send self :next-step-coords support-leg-coords
          (case l/r
           ((:left :lleg) :left-first-transition)
           ((:right :rleg) :right-first-transition))
          name))
  (:same-grid-next-steps (support-leg-coords l/r &optional (name nil))
    ;; Do not need to project that
    (send self :next-step-coords support-leg-coords l/r name))
  (:move-robot-model (&rest args) nil)
  (:calc-robot-coords (a-leg b-leg &rest args) ;dummy argumentsn
    (midcoords 0.5 a-leg b-leg))
  )

(defun footstep->consed-coords (leg-step goal)
  (let ((pose-stamped
         (instance geometry_msgs::PoseStamped :init)))
    (send pose-stamped :header (send goal :goal :initial_footstep :header))
    (send pose-stamped :pose (send leg-step :pose))
    (cons (send leg-step :leg)
          (send *tf* :transform-pose *global-frame-id* pose-stamped)))
  )

(defun manhattan-distance (a b &key (debug-view nil))
  (let ((direction (make-line (send a :worldpos)
                              (v+ (send a :worldpos)
                                  (send (send a :worldcoords) :x-axis)))))
    (let ((foot-point (send direction :point
                            (send direction :foot (send b :worldpos)))))
      (when debug-view
        (let ((prev-color (send *pickview* :viewer :viewsurface :color)))
          (send *pickview* :viewer :viewsurface :color (float-vector 0 1 0))
          (send *pickview* :viewer :viewsurface :3d-line
                (send a :worldpos)
                foot-point)
          (send *pickview* :viewer :viewsurface :3d-line
                foot-point
                (send b :worldpos))
          (send *pickview* :viewer :viewsurface :color prev-color))
        (send *pickview* :viewer :flush)
        )
      (let ((distance-a (distance (send a :worldpos) foot-point))
            (distance-b (distance (send b :worldpos) foot-point)))
        (+ distance-a distance-b)))))

(defun check-transition-limit (from-coords to-coords limits)
  (let ((diff-coords (send from-coords :transformation to-coords)))
    (if limits
        (let ((x-limit (cdr (assoc :x limits)))
              (z-limit (cdr (assoc :z limits)))
              (roll-limit (cdr (assoc :roll limits)))
              (pitch-limit (cdr (assoc :pitch limits))))
          (let ((rpy (car (send diff-coords :rpy-angle)))
                (pos (send diff-coords :worldpos)))
            (let ((result
                   (and (< (abs (elt pos 0)) x-limit)
                        (< (abs (elt pos 2)) z-limit)
                        (< (abs (elt rpy 1)) pitch-limit)
                        (< (abs (elt rpy 2)) roll-limit))))
              result)))
      t)))
