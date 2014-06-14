(load "models/arrow-object.l")
(load "models/single-arrow-object.l")

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

(defun face-on-a-face (f support-face)
  (dolist (v (send f :vertices))
    (if (eq (send support-face :insidep v) :outside)
        (return-from face-on-a-face nil)
      ))
  ;;(if *debug* (send support-face :draw-on :flush t))
  t)

(defun face-on-faces (f support-faces)
  (if *use-env-server*
      (progn
        (let ((req (instance jsk_pcl_ros::PolygonOnEnvironmentRequest :init)))
          (send req :environment_id *env-id*)
          ;; build polygon samped
          (send req :polygon :header *goal-header*)
          (send req :polygon :polygon :points
                (mapcar #'(lambda (v)
                            (instance geometry_msgs::Point32 :init
                                      :x (* 0.001 (elt v 0))
                                      :y (* 0.001 (elt v 1))
                                      :z (* 0.001 (elt v 2))))
                        (send f :vertices)))
          (let ((res (ros::service-call "polygon_on_env" req t)))
            (send res :result))))
    (progn
      (dolist (v (send f :vertices))
        (let ((on-face-p nil))
          (dolist (support-face support-faces)
            (when (and (eps= (send support-face :distance v) 0.0)
                       (not (eq (send support-face :insidep v) :outside)))
              ;; the v is ok
              (setq on-face-p t)
              (return-from nil nil)
              )
            )
          (unless on-face-p
            (return-from face-on-faces nil))
          )
        )
      t)))

;; for debug
(defun make-arrow-for-direction (direction)
  (let ((arrow (instance arrow-object :init))
        (direction* (normalize-vector direction)))
    (let ((x-axis (send arrow :x-axis)))
      (let ((a (v* x-axis direction*)))
        (if (eps= (norm a) 0.0)
            arrow
          (let* ((a* (normalize-vector a))
                 (theta (asin (norm a))))
            (send arrow :rotate theta a* :world)
            arrow))))))
    
#|
(setq a (float-vector 1 0 0))
(setq b (float-vector 0 0 1))
(setq axis (v* a b))
(setq axis* (normalize-vector axis))
(setq coords (make-coords))
(send coords :rotate (asin (norm axis)) axis* :world)
(send coords :x-axis)
|#

(defun project-coords-on-to-plane (coords planes z-axis)
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
        (if non-null-candidates non-null-candidates ;car is not good
          nil)))))

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
          footstep-param planes
          goal-footstep
          goal-coords
          goal-leg-index
          near-side-goal
          collision-robot-model timeout))

(defmethod footstep-planner-graph
  (:init (&rest args
          &key ((:scene sc) nil)
               ((:scene-grow sg) 0.0)
               ((:grow g) 0.2)
               ((:timeout tm) nil)
               ((:wd d) 1.0)
               ((:planes ps) nil)
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
    (setq footstep-param param)
    (setq target-scene sc)
    (setq timeout tm)
    (setq planes ps)
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
    (and
     (cond (collision-robot-model
            (not (send self :collision-with-obstacles-collision-robot-model-p
                       (send (send robot-coords :copy-worldcoords)
                             :transform (send footstep-param :foot->robot))
                       :debug debug)))
           (robot
            (not (send self :collision-with-obstacles-p robot :debug debug)))
           (t
            (not (send self :collision-with-obstacles-no-robot-mode-p
                       next-leg-coords :debug debug))))
     ;; plane check
     (if (and next-leg-face planes)
         (face-on-faces next-leg-face planes)
       t)))
  (:successors (state &key (verbose nil))
    (when timeout                            ;check the timeout
      (when (< (- (send timeout :to-sec) (send (ros::time-now) :to-sec)) 0)
        (return-from :successors nil)))
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
        ;; remove steps collide with obstacles
        (let ((ret nil))
          (dolist (c next-coords-candidate)
            (let ((cdr-c (cdr c)))
              (let ((robot-coords
                     (send footstep-param :calc-robot-coords
                           cdr-c current-step-coords)))
                ;; (send self :move-robot-model
                ;;       cdr-c
                ;;       current-step-coords next-leg robot-coords)
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
                                      :robot-coords robot-coords
                                      :prev-coords current-step-coords
                                      :angle-vector nil
                                      ;;(send robot :angle-vector)
                                      :step-name (car c)
                                      :l/r next-leg))
                      (push c ret))))))
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
  :slots (worldcoords l/r robot-coords prev-coords angle-vector step-name))

(defmethod step-node
  (:init (&key ((:worldcoords wc) nil)
               ((:robot-coords rc) nil)
               ((:prev-coords pc) nil)
               ((:angle-vector av) nil)
               ((:l/r lr) nil)
               ((:step-name sn) nil))
    (setq worldcoords wc)
    (setq step-name sn)
    (setq prev-coords pc)
    (setq robot-coords rc)
    (setq angle-vector av)
    (setq l/r lr)
    self)
  ;; accessor
  (:robot-coords () robot-coords)
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
          #.(deg2rad 10.0))))

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
  (:pop-from-open-list (&key (debug nil))
    (let ((target-node (send-super :pop-from-open-list :debug nil)))
      (when debug                                  ;debug view
        (send *pickview* :draw-objects :flush nil) ;clear
        ;; draw step
        (send render-leg-object 
              :newcoords (send (send target-node :state) :worldcoords))
        (send render-leg-object  :worldpos)
        (send render-leg-object  :draw-on :flush nil)
        (send *pickview* :viewer :viewsurface :flush)
        (x::window-main-one))
      target-node))
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

