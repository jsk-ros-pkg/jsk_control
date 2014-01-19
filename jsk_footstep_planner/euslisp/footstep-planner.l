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

(defclass footstep-planner-graph
  :super graph
  :slots (thr rthr target-scene
          grow obstacles robot
          leg-object step-max
          rotate-max
          wd wp
          footstep-param
          collision-robot-model))

(defmethod footstep-planner-graph
  (:init (&rest args
          &key ((:scene sc) nil)
               ((:scene-grow sg) 0.0)
               ((:grow g) 0.2)
               ((:wd d) 1.0)
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
    (unless robot
      (warn "[WARN] footstep planning works in no-robot model mode~%")
      (if collision-robot-model
          (warn "[INFO] use collision robot model for collision check~%")
        (warn "[INFO] use simple leg model for collision check~%")))
    self)
  (:goal-test (state &key (debug-view nil))
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
                  &allow-other-keys)
    "this method is called after robot model is moved."
    (cond (collision-robot-model
            (not (send self :collision-with-obstacles-collision-robot-model-p
                       (send (send robot-coords :copy-worldcoords)
                             :transform (send footstep-param :foot->robot))
                       :debug debug)))
          (robot
            (not (send self :collision-with-obstacles-p robot :debug debug)))
          (t
           (not (send self :collision-with-obstacles-no-robot-mode-p
                      next-leg-coords :debug debug)))))
  (:successors (state &key (verbose nil))
    (let* ((current-leg (send state :l/r))
           (current-step-coords (send state :worldcoords))
           (next-leg (send footstep-param :next-leg current-leg)))
      (let ((next-coords-candidate (send footstep-param :next-step-coords
                                         (send state :worldcoords)
                                         next-leg
                                         (send state :step-name))))
        ;; remove steps collide with obstacles
        (let ((ret nil))
          (dolist (c next-coords-candidate)
            (let ((cdr-c (cdr c)))
              (let ((robot-coords
                     (send footstep-param :calc-robot-coords
                           cdr-c current-step-coords)))
                (send self :move-robot-model
                      cdr-c
                      current-step-coords next-leg robot-coords)
                (when robot
                  (send robot :worldpos))
                (if (send self :successablep :debug verbose
                          :next-leg next-leg ;:lleg, :rleg and so on
                          :next-leg-coords cdr-c
                          :robot-coords robot-coords)
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
;;    (let ((coords-worldpos (send coords :worldpos)))
      (dolist (x (send self :obstacles))
;;        (if (< (distance (send x :worldpos) coords-worldpos)
;;               2000.0)
        (if (= (pqp-collision-check collision-robot-model x) 1)
            (return-from :collision-with-obstacles-collision-robot-model-p
              t)))
      ;;)
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

(defun footstep-planning (from-coords
                          goal-coords
                          leg-object
                          start-leg
                          parameter
                          &rest
                          args
                          &key
                          (render-leg-object leg-object)
                          (scene-grow 0.0)
                          ((:scene sc) nil)
                          ((:robot r) nil)
                          ((:reference-robot rf) nil)
                          ((:thr th) 10.0)
                          ((:rthr rth) nil)
                          (prefix-footsteps nil) ;by name
                          (wg 1.0)
                          (wd 1.0)
                          (wp 1.0)
                          (obstacles nil)
                          (step-max 200.0)
                          (rotate-max (deg2rad 45))
                          (solver-class
                           a*-footstep-graph-search-solver)
                          (problem-class
                           footstep-planner-graph)
                          (heuristic-method :stepcost-heuristic*)
                          (finalize-footstep t)
                          (debug-view nil)
                          (return-problem nil)
                          (append-initial-step t)
                          (grow-model 0.0)
                          (collision-model-bodies-func #'(lambda (rf) (send rf :bodies)))
                          ;; for subclasses of footstep-planner-graph
                          (problem-class-rest-arguments nil)
                          &allow-other-keys)
  "wrapper function to use footstep-planner-graph and
a*-footstep-graph-search-solver."
  ;; initialize robot model
  ;; error check
  (if (eq r rf)
      (error "You have to specify not the same model ~
in :robot and :reference-robot"))
  (unless (evenp (length prefix-footsteps))
    (error "prefix-footstep must be even"))
  (with-temp-robot-model
   (rf)
   ;; check start-legs
   (if (eq start-leg :auto)
       (setq start-leg                  ;dirty...
             (automatically-select-start-leg from-coords goal-coords)))
   ;; move robot mode according to prefix-footsteps
   (when prefix-footsteps
     ;; NB: should use with-temp-robot-model
     (warn "[WARN] ~A is moved acoording to prefix-footsteps~%" rf)
     (send parameter :move-robot-model-by-step-names rf
           start-leg prefix-footsteps))
   (when r
     ;; if planning robot model is give, we need to sync it to reference-robot
     (send r :newcoords (send rf :copy-worldcoords))
     (send r :angle-vector (send rf :angle-vector)))
   (send rf :worldpos)
   (let ((prblm (instance* problem-class :init
                           :scene sc :scene-grow scene-grow
                           :footstep-parameter parameter
                           :obstacles obstacles :robot r
                           :step-max step-max :rotate-max rotate-max
                           :leg-object leg-object
                           :thr th :rthr rth :wd wd :wp wp
                           :collision-robot-model
                           (with-temp-robot-model
                            (rf)
                            (send rf :fix-leg-to-coords (make-coords))
                            (send rf :worldpos)
                            (send-all (funcall collision-model-bodies-func rf) :worldpos)
                            (geo::make-model-obb-box
                              (send rf :copy-worldcoords)
                              (funcall collision-model-bodies-func rf) :grow grow-model))
                           problem-class-rest-arguments))
         (support-leg (send parameter :next-leg start-leg)))
     ;; the start state must be support leg.
     ;; the support leg is the reverse leg of start-leg.
     (send prblm :start-state
           (instance step-node :init    ;need to set robot-coords
                     :worldcoords
                     (send rf support-leg :end-coords :copy-worldcoords)
                     :prev-coords
                     (send rf start-leg :end-coords :copy-worldcoords)
                     :l/r support-leg
                     :robot-coords
                     (send parameter :calc-robot-coords
                           (send rf support-leg :end-coords)
                           (send rf start-leg :end-coords))
                     :angle-vector (send rf :angle-vector)))
     ;; goal
     (send prblm :goal-state goal-coords)
     (send parameter :init-foot->robot rf)
     (let ((asolver (instance solver-class :init wg heuristic-method prblm
                              :render-leg-object render-leg-object)))
       (if debug-view (objects asolver *pickview* :fix t))
       (let ((solver-nodes (send asolver :solve prblm :verbose debug-view))) ;solve!
         (unless solver-nodes (return-from footstep-planning nil))
         (if append-initial-step
             (setq solver-nodes
              (append-initial-dummy-footstep
               :solver-nodes solver-nodes
               :leg start-leg
               :leg-coords (send rf start-leg :end-coords :copy-worldcoords)
               :cross-leg-coords
               (send rf support-leg :end-coords :copy-worldcoords)
               :angle-vector (send rf :angle-vector)
               :robot-coords (send rf :foot-midcoords))))
         ;; append solver-nodes with default footstep 
         ;; calling with :finalize-footstep t
         (if finalize-footstep
             (setq solver-nodes
                   (send parameter :append-finalize-footstep
                         solver-nodes finalize-footstep goal-coords)))
         (when debug-view
           (draw-footstep-path solver-nodes render-leg-object parameter
                               :robot r))
         ;; ugly hack for benchmark
         (if return-problem (list prblm solver-nodes) solver-nodes))))))

(defun append-initial-dummy-footstep (&key
                                      (solver-nodes nil)
                                      (angle-vector nil)
                                      (leg nil)
                                      (leg-coords nil)
                                      (cross-leg-coords nil)
                                      (robot-coords nil)
                                      (prefix-footsteps nil))
  ;; if :prefix-footsteps is specified,
  ;; initial-dummy -> prefix-footsteps -> solver-nodes
  (cons (instance solver-node :init
                  (instance step-node :init
                            :worldcoords leg-coords
                            :l/r leg
                            :robot-coords robot-coords
                            :prev-coords cross-leg-coords
                            :angle-vector angle-vector))
        solver-nodes))

(defun draw-footstep-path (solver-nodes leg-object parameter
                           &key
                           (one-by-one nil)
                           (color (float-vector 1 0 0))
                           ((:viewer vw) *pickview*)
                           (robot nil)
                           (collision-model nil))
  (send vw :draw-objects)
  (dolist (n solver-nodes)
    (let ((solver-state (send n :state)))
      (when robot
        (print (send n :action))
        (send parameter :move-robot-model
              robot
              (send solver-state :worldcoords)
              (send solver-state :prev-coords)
              (send solver-state :l/r)
              (send solver-state :robot-coords))
        (let ((av (send solver-state :angle-vector)))
          (cond ((derivedp av cons)
                 (send robot :angle-vector av))
;;                ((and (boundp 'angle-vector-sequence)
;;                      (derivedp av angle-vector-sequence))
;;                 (send robot :angle-vector (Send av :last-angle-vector)))
                (t
                 (warn "solver-state has no angle-vector!~%")))))
      (let ((tmp (copy-object leg-object)))
        (if one-by-one (send vw :draw-objects))
        (if robot (send vw :draw-objects)) ;hmmmm
        (send tmp :newcoords (send solver-state :worldcoords))
        (send tmp :worldpos)
        (send tmp :draw-on :flush t :color color)
        (when collision-model
          (send collision-model :newcoords
                (send
                 (send (send solver-state :robot-coords) :copy-worldcoords)
                 :transform (send parameter :foot->robot)))
          (send collision-model :worldpos)
          (send collision-model :draw-on :flush t))
        (unix:usleep (* 500 1000))))))


;; (manhattan-distance (make-coords)
;;                     (make-coords :pos (float-vector 100 0 0))
;;                     :debug-view t)
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

(defun collision-check-paranoia (obj1 obj2)
  (let ((fast-collide-p (pqp-collision-check obj1 obj2)))
    (if (= fast-collide-p 1)
        (progn
          (send obj2 :draw-on :flush t)
          t)                            ;collide!
      (progn
        (dolist (v (send obj2 :vertices))
          (unless (eq (send obj1 :insidep-org v) :outside)
            (warn "~A is inside~%" v)
            (send obj2 :draw-on :flush t)
            (return-from collision-check-paranoia t))) ;collide!
        nil))))

;; (defun pqp-collision-check-object* (obj1 obj2)
;;   "returns t when obj1 and obj2 are colliding"
;;   (let ((fast-collide-p (pqp-collision-check obj1 obj2)))
;;     (if (= fast-collide-p 1)
;;         (progn
;;           (send obj2 :draw-on :flush t)
;;           t)
;;       ;; we need to check obj1 and obj2 is in `inside' relation
;;       (multiple-value-bind (distance point1 point2)
;;           (pqp-collision-distance obj1 obj2)
;;         (let ((face-normal (normal-on-point obj1 point1))
;;               (v (v- point2 point1)))
;;           (if (< (v. v face-normal) 0)
;;               (progn
;;                 (send obj2 :draw-on :flush t)
;;                 t)
;;             nil))))))

(defclass footstep-visualizer
  :super cascaded-coords                ;its not good...
  :slots (result-path                   ;result path is a list of solver-node
          leg-object))

(defmethod footstep-visualizer
  (:init (leg-obj r)
    (setq result-path (if (derivedp (car r) solver-node) (send-all r :state) r))
    (setq leg-object leg-obj)
    ;; set the coordinates position to the middle of result-path
    (let ((p (vector-mean 
              (send-all (send-all result-path :worldcoords)
                        :worldpos))))
      (send-super :init :pos p))
    self)
  (:vertices ()
    (send-all (send-all result-path :worldcoords) :worldpos))
  (:collision-check-objects ())
  (:draw (vwer)
    (sys::mutex
      gl::*opengl-lock*
      (gl::glPushAttrib gl::GL_ALL_ATTRIB_BITS)
      (if vwer (send vwer :viewsurface :makecurrent))
      ;; draw tree
      (gl::glDisable gl::GL_LIGHTING)
      (gl::glBegin gl::GL_LINES)
      (gl::glColor3fv (float-vector 1 0 0))
      (reduce #'(lambda (from to)
                  (gl::glVertex3fv from)
                  (gl::glVertex3fv to)
                  to)
              (send-all (send-all result-path :worldcoords) :worldpos))
      (gl::glEnd)
      (gl::glEnable gl::GL_LIGHTING)
      ;; draw footstep
      (dolist (c (send-all result-path :worldcoords))
        (send leg-object :newcoords (send c :copy-worldcoords))
        (send leg-object :worldpos)
        (send vwer :viewsurface :gldraw leg-object))
      (gl::glPopAttrib)
      ))
  )

(defun make-footstep-visualizer (o r)
  (instance footstep-visualizer :init o r))

;;(provide :footstep-planner)

