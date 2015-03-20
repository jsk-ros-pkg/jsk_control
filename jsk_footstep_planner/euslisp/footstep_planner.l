(load "models/arrow-object.l")
(load "models/single-arrow-object.l")
(comp:compile-file-if-src-newer
  (format nil "~A/euslisp/footstep_planner_util.l"
          (ros::rospack-find "jsk_footstep_planner")))
(load "footstep_planner_util.l")

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
          collision-robot-model timeout
          successablep-timer
          successors-timer append-footstep-candidates-by-grids-timer
          is-face-possible-to-put-on-grid-timer))

(defmethod footstep-planner-graph
  (:init (&rest args
          &key ((:scene sc) nil)
               ((:scene-grow sg) 0.0)
               ((:grow g) 0.2)
               ((:timeout tm) nil)
               ((:wd d) 1.0)
               ((:planes ps) nil)
               ((:grids gs) nil)
               ((:z-axis z) nil)
               ((:wp p) 0.0)
               ((:robot r) nil)
               ((:leg-object lo) nil)
               ((:footstep-parameter param))
               ((:step-max sm) 100.0)
               ((:rotate-max rm) (deg2ead 45.0))
               ((:thr th) 100.0)
               ((:obstacles ob) nil)
               ((:rthr rth) (deg2rad 10))
               ((:collision-robot-model crm) nil)
               &allow-other-keys)
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
  (:goal-test (state &key (debug-view nil))
    ;;(if (eq (send state :l/r) goal-leg-index)
        (let ((current-coords (send state :robot-coords)))
          (let ((ndiff-pos (distance (send current-coords :worldpos)
                                     (send goal-state :worldpos))))
            (if (or (not thr) (< ndiff-pos thr))
                (let* ((diff-rot (send current-coords
                                       :difference-rotation goal-state))
                       (ndiff-rot (norm diff-rot)))
                  ;; when thr or rthr is nil, test returns t
                  (or (not rthr) (< ndiff-rot rthr)))))))
;;    )
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
             (bench-timer
              is-face-possible-to-put-on-grid-timer
              (is-face-possible-to-put-on-grid next-leg-face grids)))
            (t
             t)))))
  (:successors-old (state &key (verbose nil))
    (when timeout                            ;check the timeout
      (when (< (- (send timeout :to-sec) (send (ros::time-now) :to-sec)) 0)
        (return-from :successors-old nil)))
    (let* ((current-leg (send state :l/r))
           (current-step-coords (send state :worldcoords))
           (next-leg (send footstep-param :next-leg current-leg)))
      (let ((next-coords-candidate (send footstep-param :next-step-coords
                                         (send state :worldcoords)
                                         next-leg
                                         (send state :step-name))))
        ;; mapping to the planes if needed
        (when (and planes z-axis)
          (let ((candidates-set
                 (mapcar #'(lambda (c)
                             (let ((candidate-set2
                                    (project-coords-on-to-plane
                                     (cdr c) planes z-axis)))
                               (if candidate-set2
                                   (mapcar #'(lambda (s)
                                               (cons (car c) s))
                                           candidate-set2)
                                 nil)))
                         next-coords-candidate)))
            (let ((new-candidate nil))
              (dolist (candidates candidates-set)
                (dolist (c candidates)
                  (if (and c (cdr c))
                      (if new-candidate
                          (setq new-candidate (append new-candidate (list c)))
                        (setq new-candidate (list c))
                        ))))
              (setq next-coords-candidate new-candidate))))
        ;; mapping to the grids if needed
        ;;(when (and grids z-axis)
        (ros::ros-info "~A candidates" (length next-coords-candidate))
        (when (and grids)
          (setq next-coords-candidate
                (append-footstep-candidates-by-grids
                 next-coords-candidate grids)))
        ;; check x transition
        (when nil
        (dolist (c next-coords-candidate)
          (let ((trans (send current-step-coords :transformation (cdr c))))
            ;; TODO: fix it
            (when (> (elt (send trans :worldpos) 0) 250)
              ;; force to use 250
              (send trans :locate (float-vector
                                   250
                                   (elt (send trans :worldpos) 1)
                                   (elt (send trans :worldpos) 2))
                    :world)
              (send (cdr c) :newcoords (send (send current-step-coords :copy-worldcoords) :transform trans))
              (send (cdr c) :worldpos)
              ))))
        ;; check z transition
        (when (send state :prev-coords)
          (let* ((prev-step-coords (send (send state :prev-coords) :copy-worldcoords))
                 (trans (send prev-step-coords :transformation (send current-step-coords :copy-worldcoords))))
            (when (> (abs (elt (send trans :worldpos) 2)) 50) ;over 5cm change
              ;; reduce next-coords-candidate
              (setq next-coords-candidate 
                    (remove-if #'(lambda (c)
                                   (let* ((cc (cdr c)) ;cc is coordinate
                                          (next-trans (send current-step-coords :transformation
                                                            cc)))
                                     (if (> (abs (elt (send next-trans :worldpos) 0)) 10) ;over 1cm 
                                         (progn
                                           (ros::ros-info "reduce candidate because ~A x transition"
                                                          (elt (send next-trans :worldpos) 0))
                                           nil)
                                       c)))
                               next-coords-candidate)))))
        ;; remove steps collide with obstacles
        (let ((ret nil))
          (dolist (c next-coords-candidate)
            (let ((cdr-c (cdr c)))
              (let ((robot-coords
                     (send footstep-param :calc-robot-coords
                           cdr-c current-step-coords)))
                (when robot
                  (send robot :worldpos))
                (if (send self :successablep :debug verbose
                          :next-leg next-leg ;:lleg, :rleg and so on
                          :next-leg-coords cdr-c
                          :robot-coords robot-coords
                          :next-leg-face (instance face :init
                                                   :vertices
                                                   (send footstep-param :foot-vertices
                                                         next-leg cdr-c)))
                    (progn
                      (setf (cdr c)
                            (instance step-node :init
                                      :worldcoords cdr-c
                                      :leg-face leg-face
                                      :robot-coords robot-coords
                                      :prev-coords current-step-coords
                                      :angle-vector nil
                                      ;;(send robot :angle-vector)
                                      :step-name (car c)
                                      :l/r next-leg))
                      (push c ret))
                  (progn
                    (ros::ros-info "Not successable")
                    )
                  ))))
          ret))))
  (:successors (state &key (verbose nil))
    (bench-timer
     successors-timer
     (when timeout                            ;check the timeout
       (when (< (- (send timeout :to-sec) (send (ros::time-now) :to-sec)) 0)
         (ros::ros-warn "Planning take ~A secs, Timeout"
           (- (send timeout :to-sec) (send (ros::time-now) :to-sec)))
         (return-from :successors nil)))
     (let* ((current-leg (send state :l/r))
            (current-step-coords (send state :worldcoords))
            (next-leg (send footstep-param :next-leg current-leg)))
       (let ((next-coords-candidate (send footstep-param :next-step-coords
                                          (send state :worldcoords)
                                          next-leg
                                          (send state :step-name))))
         ;; mapping to the grids if needed
         (when (and grids)
           (setq next-coords-candidate
                 (bench-timer
                  append-footstep-candidates-by-grids-timer
                  (append-footstep-candidates-by-grids next-coords-candidate grids))))
         (ros::ros-info "~A candidates" (length next-coords-candidate))
         ;; remove steps collide with obstacles
         (let ((ret nil))
           (dolist (c next-coords-candidate)
             (let ((cdr-c (cdr c)))
               (let ((robot-coords
                      (send footstep-param :calc-robot-coords
                            cdr-c current-step-coords)
                      ))
                 (when robot
                   (send robot :worldpos))
                 (if (or t (send self :successablep :debug verbose
                                 :next-leg next-leg ;:lleg, :rleg and so on
                                 :next-leg-coords cdr-c
                                 :robot-coords robot-coords
                                 :next-leg-face (instance face :init
                                                          :vertices
                                                          (send footstep-param :foot-vertices
                                                                next-leg cdr-c))))
                     (progn
                       (setf (cdr c)
                             (instance step-node :init
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
                       (push c ret))
                   (progn
                     ;;(ros::ros-info "Not successable")
                     )
                   ))))
           ret)))))
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
  :slots (worldcoords l/r robot-coords prev-coords angle-vector step-name leg-face))

(defmethod step-node
  (:init (&key ((:worldcoords wc) nil)
               ((:robot-coords rc) nil)
               ((:prev-coords pc) nil)
               ((:angle-vector av) nil)
               ((:l/r lr) nil)
               ((:leg-face lg) nil)
               ((:step-name sn) nil))
    (setq worldcoords wc)
    (setq step-name sn)
    (setq leg-face lg)
    (setq prev-coords pc)
    (setq robot-coords rc)
    (setq angle-vector av)
    (setq l/r lr)
    self)
  ;; accessor
  (:robot-coords () robot-coords)
  (:leg-face () leg-face)
  ;;(:robot-coords () worldcoords)
  (:prev-coords () prev-coords)
  (:worldcoords () worldcoords)
  (:angle-vector (&optional (arg nil))
    (if arg (setq angle-vector arg))
    angle-vector)
  (:l/r () l/r)
  (:step-name () step-name)
  )

(defun step-node= (a b)
  (and (eq (send a :l/r) (send b :l/r))
       (< (distance (send (send a :worldcoords) :worldpos)
                    (send (send b :worldcoords) :worldpos))
          50.0)
       (< (norm (send (send a :worldcoords) :difference-rotation
                      (send b :worldcoords)))
          #.(deg2rad 5.0))))

;; it has A* in the name of class, but
;; this algorithm is not "A*" in exact meaning.
;; Because estimation of the number of rotation and translation steps in
;; heuristic may be bigger than cost function.
;; However, in the most case, the algorithm behaves as A* algorithm.
(defclass a*-footstep-graph-search-solver
  :super a*-graph-search-solver
  :slots (count target-problem heuristic-method wg arrived-at-position-p
          render-leg-object))
                

(defmethod a*-footstep-graph-search-solver
  (:init (w m plbm &rest args
            &key ((:render-leg-object rlo) nil) &allow-other-keys)
    (setq wg w)
    (setq heuristic-method m)
    (setq render-leg-object rlo)
    (send-super :init plbm)
    self)
  ;; override :solve-init and :pop-from-open-list for debug view.
  (:solve-init (prblm)
    (setq target-problem prblm)         ;for debug view
    (send-super :solve-init prblm))
  (:find-node-in-close-list (n)
    "I want to replace 'find' in this method by dynamic kd-tree
building and searching."
    (find (send n :state) close-list :test #'step-node=))
  (:null-open-list? ()
    (if (ros::time< (send target-problem :timeout) (ros::time-now))
        (return-from :null-open-list? t))
    (let ((foundp nil))
      (while (not foundp)
        (if (send-super :null-open-list?)
            (return-from :null-open-list? t)
          (let ((candidate-node (send-super :pop-from-open-list :debug nil)))
            (cond ((null candidate-node)
                   (return-from :null-open-list? nil))
                  ((null (send (send candidate-node :state) :leg-face)) ;start state
                   (send self :add-to-open-list candidate-node)
                   (setq foundp t))
                  ((is-face-possible-to-put-on-grid
                    (send (send candidate-node :state) :leg-face)
                    (send target-problem :grids))
                   (send self :add-to-open-list candidate-node)
                   (setq foundp t))
                  (t
                   nil))
            )))
      (not foundp)))
  ;; (:pop-from-open-list (&key (debug nil))
  ;;   (let ((next (send-super :pop-from-open-list :debug nil)))
  ;;     (if (send (send next :state) :leg-face)
  ;;         (send (send (send next :state) :leg-face) :draw-on :flush t))
  ;;     next))
  ;; (:pop-from-open-list (&key (debug nil))
  ;;   (let ((target-node nil))
  ;;     (while (null target-node)
  ;;       (if (send self :null-open-list?)
  ;;           (return-from :pop-from-open-list nil)
  ;;         (let ((candidate-node (send-super :pop-from-open-list :debug nil)))
  ;;           (cond ((null candidate-node)
  ;;                  (return-from :pop-from-open-list nil))
  ;;                 ((null (send (send candidate-node :state) :leg-face)) ;start state
  ;;                  (setq target-node candidate-node))
  ;;                 ((is-face-possible-to-put-on-grid
  ;;                   (send (send candidate-node :state) :leg-face)
  ;;                   (send target-problem :grids))
  ;;                  (setq target-node candidate-node))
  ;;                 (t
  ;;                  nil))
  ;;           )))
  ;;     target-node))
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
    (let* ((goal-coords (send p :goal-state))
           (step (send p :step-max))
           (rotate (send p :rotate-max))
           (current-coords
            (send (send n :state) :robot-coords))
           (v1 (normalize-vector (v- (send goal-coords :worldpos)
                                     (send current-coords :worldpos))))
           (v2 (matrix-column (send current-coords :worldrot) 0))
           (v3 (matrix-column (send goal-coords :worldrot) 0)))
      (let ((trans-cost (* (/ (distance (send goal-coords :worldpos)
                                        (send current-coords :worldpos))
                              step) 2)))
        (if (> trans-cost 1)
            (+ trans-cost
               ;; rotation of direction
               (* 2  (/ (acos (v. v1 v2)) rotate))
               ;; rotation of goal
               (* 2 (/ (acos (v. v1 v3)) rotate)))
          ;; rotation of goal
          (* 2 (/ (acos (v. v1 v3)) rotate))))))
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
  :slots (left-footsteps right-footsteps size-x size-y size-z))
(defmethod ros-footstep-parameter
  (:init (x y)
    (setq size-x x)
    (setq size-y y)
    (send-super :init :legs-num 2))
  (:left-footsteps ()
    left-footsteps)
  (:right-footsteps ()
    right-footsteps)
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
      (mapcar #'(lambda (v)
                  (let ((vv (send coords :transform-vector v)))
                    ;;(format t "~A -> ~A~%" v vv)
                    vv))
              local-vertices)))
  (:gen-footstep (&rest args)
    (send self :gen-left-footstep)
    (send self :gen-right-footstep)
    )
  (:gen-left-footstep ()
    (let ((footsteps
           (mapcar #'(lambda (x y theta)
                       (let ((c (make-coords :pos (float-vector (* 1000.0 x)
                                                                (* 1000.0 y) 0))))
                         (send c :rotate theta :z)
                         c))
                   *footstep-xs* *footstep-ys* *footstep-thetas*)))
      (dotimes (i (length footsteps))
        (let ((f (elt footsteps i)))
          (send self :gen-step :left i f)))
      ))
  (:gen-right-footstep ()
    (let ((footsteps
           (mapcar #'(lambda (x y theta)
                       (let ((c (make-coords :pos (float-vector (* 1000.0 x)
                                                                (* -1000.0 y)
                                                                0))))
                         (send c :rotate (- theta) :z)
                         c))
                   *footstep-xs* *footstep-ys* *footstep-thetas*)))
      (dotimes (i (length footsteps))
        (let ((f (elt footsteps i)))
          (send self :gen-step :right i f)))
      ))
  (:gen-step (l/r name coords)
    (let ((s (cons name coords)))
      (case l/r
        ((:left :lleg) (push s left-footsteps))
        ((:right :rleg) (push s right-footsteps))
        (t (error "unkown leg ~A" l/r)))
      ))
  (:transformation-by-leg-and-step-name (leg-name step-name)
    (case leg-name
      ((:left :lleg) (cdr (assoc step-name left-footsteps)))
      ((:right :rleg) (cdr (assoc step-name right-footsteps)))
      (t (error "unkown leg ~A" leg-name))))
  (:next-leg (l/r)
    (if (eq l/r :lleg) :rleg :lleg))
  (:next-step-coords (support-leg-coords l/r &optional (name nil))
    (let ((transformations
           (case l/r
             ((:left :lleg) left-footsteps)
             ((:right :rleg) right-footsteps)
             (t (error "unkown leg ~A" l/r)))))
      (mapcar #'(lambda (trans)
                  (cons (car trans)     ;transform
                        (send (send support-leg-coords :copy-worldcoords) ;new pose
                              :transform (cdr trans))))
              transformations)))
  (:move-robot-model (&rest args) nil)
  (:calc-robot-coords (a-leg b-leg &rest args) ;dummy argumentsn
    (midcoords 0.5 a-leg b-leg))
  )

(defun footstep->consed-coords (leg-step)
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
