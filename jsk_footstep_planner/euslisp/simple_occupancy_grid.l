;; Utility to deal with jsk_recognition_msgs::SimpleOccupancyGrid(Array)

(ros::roseus-add-msgs "jsk_recognition_msgs")
(setq *debug-verbose* nil)
;; (comp:compile-file-if-src-newer
;;   (format nil "~A/euslisp/footstep_planner_util.l"
;;           (ros::rospack-find "jsk_footstep_planner")))
;; (let ((old-module (find "footstep_planner_util" *loaded-modules*
;;                         :key #'lisp::load-module-file-name :test #'equal)))
;;   (if old-module (sys::unbinload old-module)))
;; (load "footstep_planner_util.so")
;;(load "package://jsk_footstep_planner/euslisp/footstep_planner_util.l")
(defun quaternion-from-two-vectors (a b)
  (let* ((v0 (normalize-vector a))
         (v1 (normalize-vector b))
         (c (v. v1 v0)))
    (if (eps= c -1.0)
        (float-vector -1 0 0 0)
      (let* ((axis (v* v0 v1))
             (s (sqrt (* 2 (+ 1 c))))
             (invs (/ 1.0 s)))
        (let ((vec (scale invs axis))
              (w (* 0.5 s)))
          (normalize-vector (float-vector w (elt vec 0) (elt vec 1) (elt vec 2))))))))

(defun coefficients->plane-coords (coefficients)
  (let ((z-axis (normalize-vector (float-vector (elt coefficients 0)
                                                (elt coefficients 1)
                                                (elt coefficients 2))))
        (unitz (float-vector 0 0 1)))
    (let ((c (make-coords :rot (quaternion2matrix (quaternion-from-two-vectors unitz z-axis)))))
      (send c :translate (float-vector 0 0 (- (* 1000.0 (elt coefficients 3))))))))

(defclass occupancy-grid
  :super propertied-object
  :slots (resolution
          coefficients plane-coords frame-id
          plane-obj
          cell-num cell-bits
          min-cell-x min-cell-y max-cell-x max-cell-y
          cell-width cell-height
          local-point-cache point-tmp point-tmp2 rot-tmp
          occupiedp-timer on-plane-p-timer index-pair-timer
          placablep-timer inverse-transform-vector-timer
          init-timer draw-vertices
          ))

(defmethod occupancy-grid
  (:fill-float-vector (start-index fill-vector filled-vector)
    (setf (elt filled-vector (+ 0 start-index)) (elt fill-vector 0))
    (setf (elt filled-vector (+ 1 start-index)) (elt fill-vector 1))
    (setf (elt filled-vector (+ 2 start-index)) (elt fill-vector 2))
    (+ start-index 3))
  (:update-draw-vertices ()
    (setq draw-vertices (instantiate float-vector (* 8 cell-num 3)))
    (let* ((counter 0)
           (w (/ resolution 2.0))
           (-w (* -1 w))
           (v1 (float-vector 0 0 0))
           (v2 (float-vector 0 0 0))
           (v3 (float-vector 0 0 0))
           (v4 (float-vector 0 0 0))
           (gv1 (float-vector 0 0 0))
           (gv2 (float-vector 0 0 0))
           (gv3 (float-vector 0 0 0))
           (gv4 (float-vector 0 0 0))
           (w1 (float-vector w w 0))
           (w2 (float-vector -w w 0))
           (w3 (float-vector -w -w 0))
           (w4 (float-vector w -w 0))
           (center (float-vector 0 0 0)))
      (dotimes (i cell-width)
        (dotimes (j cell-height)
          (when (send self :occupiedp* i j)
            (send self :cell-index-to-local-point i j center)
            (v+ center w1 v1)
            (v+ center w2 v2)
            (v+ center w3 v3)
            (v+ center w4 v4)
            (send self :local-point-to-global-point v1 gv1)
            (send self :local-point-to-global-point v2 gv2)
            (send self :local-point-to-global-point v3 gv3)
            (send self :local-point-to-global-point v4 gv4)
            (setq counter (send self :fill-float-vector counter gv1 draw-vertices))
            (setq counter (send self :fill-float-vector counter gv2 draw-vertices))
            (setq counter (send self :fill-float-vector counter gv2 draw-vertices))
            (setq counter (send self :fill-float-vector counter gv3 draw-vertices))
            (setq counter (send self :fill-float-vector counter gv3 draw-vertices))
            (setq counter (send self :fill-float-vector counter gv4 draw-vertices))
            (setq counter (send self :fill-float-vector counter gv4 draw-vertices))
            (setq counter (send self :fill-float-vector counter gv1 draw-vertices))
            )))
      ))
  (:info ()
    (ros::ros-info "grid info:")
    (ros::ros-info "  minimum: (~A, ~A)" min-cell-x min-cell-y)
    (ros::ros-info "  maximum: (~A, ~A)" max-cell-x max-cell-y)
    (ros::ros-info "  size: ~Ax~A=~A (~A, ~A% filled)"
      cell-width cell-height (* cell-width cell-height) cell-num
      (round (* 100 (/ cell-num (float (* cell-width cell-height))))))
    )
  (:init (msg global-trans)
    "Instantiating from ros message"
    (send self :init-common)
    (bench-timer
     init-timer
     (setq frame-id (send msg :header :frame_id))
     (setq resolution (* 1000 (send msg :resolution)))
     (setq coefficients (send msg :coefficients))
     (setq plane-coords
           (send (send global-trans :copy-worldcoords)
                 :transform (coefficients->plane-coords coefficients)))
     (setq plane-obj (instance plane :init
                               (send plane-coords :rotate-vector (float-vector 0 0 1))
                               (send plane-coords :worldpos)))
     (setq cell-num (length (send msg :cells)))
     (setq min-cell-x 10000000
           min-cell-y 10000000
           max-cell-x -10000000
           max-cell-y -10000000)
     (dolist (cell-point (send msg :cells))
       (let ((cell-x (* 1000 (send cell-point :x)))
             (cell-y (* 1000 (send cell-point :y))))
         (let ((i (round (/ cell-x resolution)))
               (j (round (/ cell-y resolution))))
           (setq min-cell-x (min min-cell-x i))
           (setq min-cell-y (min min-cell-y j))
           (setq max-cell-x (max max-cell-x i))
           (setq max-cell-y (max max-cell-y j)))))
     (setq cell-width (1+ (- max-cell-x min-cell-x)))
     (setq cell-height (1+ (- max-cell-y min-cell-y)))
     
     (setq cell-bits (instantiate integer-vector (* cell-width cell-height)))
     (dolist (cell-point (send msg :cells))
       (let* ((cell-x (* 1000 (send cell-point :x)))
              (cell-y (* 1000 (send cell-point :y)))
              (i (- (round (/ cell-x resolution)) min-cell-x))
              (j (- (round (/ cell-y resolution)) min-cell-y)))
         (send self :register-cell* i j)))
     (send self :update-draw-vertices)
     )
    (send init-timer :report)
    self)
  (:init-common ()
    (setq local-point-cache (float-vector 0 0 0))
    (setq point-tmp (float-vector 0 0 0))
    (setq point-tmp2 (float-vector 0 0 0))
    (setq rot-tmp (make-matrix 3 3))
    (setq occupiedp-timer (instance counter-timer :init ":occupiedp"))
    (setq on-plane-p-timer (instance counter-timer :init ":on-plane-p"))
    (setq index-pair-timer (instance counter-timer :init ":index-pair"))
    (setq placablep-timer (instance counter-timer :init ":placablep"))
    (setq inverse-transform-vector-timer
          (instance counter-timer :init ":inverse-transform-vector"))
    (setq init-timer
          (instance counter-timer :init ":init"))
    )
  (:init-from-face (f &optional (aresolution 5))
    (send self :init-common)
    (setq resolution aresolution)
    (let ((normal (send f :normal))
          (c (cadr (send f :centroid))))
      (let ((d (- (v. normal c))))
        (setq coefficients (float-vector (elt normal 0)
                                         (elt normal 1)
                                         (elt normal 2)
                                         (* 0.001 d)))))
    (setq plane-coords (coefficients->plane-coords coefficients))
    (setq plane-obj (instance plane :init
                              (send plane-coords :rotate-vector (float-vector 0 0 1))
                              (send plane-coords :worldpos)))
    ;; fill cells
    (setq cell-num 0)
    (let ((local-vertices
           (mapcar #'(lambda (v)
                       (send plane-coords :inverse-transform-vector v))
                   (cdr (send f :vertices)))))
      (let* ((bbox (make-bounding-box local-vertices))
             (min-point (send bbox :minpoint))
             (max-point (send bbox :maxpoint)))
        (let ((cell-x (elt min-point 0)))
          (while (<= cell-x (elt max-point 0))
            (let ((cell-y (elt min-point 1)))
              (while (<= cell-y (elt max-point 1))
                (let ((p (send plane-coords :transform-vector
                               (float-vector cell-x cell-y 0))))
                  (when (not (eq (send f :insidep p) :outside))
                    (send self :register-cell
                          (send self :index-pair (float-vector cell-x cell-y 0)
                                :local t))
                    (incf cell-num))
                (setq cell-y (+ cell-y resolution))))
              (setq cell-x (+ cell-x resolution))
              )))))
    (send self :update-draw-vertices)
    )
  ;; not correct
  (:grid-world-roll-angle (g-sensor-coords x-coords)
    (let* ((q (quaternion-from-two-vectors
               (send x-coords :rotate-vector (float-vector 0 0 1))
               (send plane-coords :rotate-vector (float-vector 0 0 1))))
           (trans (make-coords :rot (quaternion2matrix q))))
      (let ((good-plane-coords (send (send x-coords :copy-worldcoords) :transform trans)))
        ;;(let ((good-plane-world-coords (send (send g-sensor-coords :inverse-transformation)
        (let ((good-plane-world-coords (send (send g-sensor-coords :copy-worldcoords)
                                             :transform good-plane-coords)))
          ;; (ros::ros-info "g-sensor-corods: ~A" g-sensor-coords)
          ;; (ros::ros-info "good-plane-coords: ~A" good-plane-coords)
          (let* ((n (send good-plane-world-coords :rotate-vector (float-vector 0 0 1)))
                 (yz-n* (normalize-vector (float-vector 0 (elt n 1) (elt n 2)))))
            (let ((ang (atan2 (elt yz-n* 1) (elt yz-n* 2))))
              ;;(ros::ros-info "ang: ~A" (rad2deg ang))
              ang))))))
  (:grid-world-roll-angle-old (g-sensor-coords x-coords)
    (let* ((q (quaternion-from-two-vectors
               (send x-coords :rotate-vector (float-vector 0 0 1))
               (send g-sensor-coords :rotate-vector (float-vector 0 0 1))))
           (r (quaternion2matrix q))
           (trans (make-coords :rot r)))
      (let* ((origin (send (send x-coords :copy-worldcoords) :transform trans))
             (n*
              (send origin :inverse-rotate-vector (send plane-obj :normal)))
             (yz-n* (normalize-vector (float-vector 0 (elt n* 1) (elt n* 2)))))
        (atan2 (elt yz-n* 1) (elt yz-n* 2)))))
  (:rolledp (g-sensor-coords x-coords &optional (threshold (deg2rad 10)))
    (let* ((theta (send self :grid-world-roll-angle g-sensor-coords x-coords)))
      (when *debug-verbose*
        ;;(ros::ros-info "theta: ~A" (rad2deg theta)))
        )
      (> (abs theta) threshold)))
  (:roll-direction (g-sensor-coords x-coords &optional (threshold (deg2rad 10)))
    (let* ((theta (send self :grid-world-roll-angle g-sensor-coords x-coords)))
      (cond ((> theta threshold)
             :left)
            ((< theta (- threshold))
             :right)
            (t
             nil))))
  (:local-point (index-pair &optional buf)
    (if buf
        (progn
          (setf (elt buf 0) (* resolution (car index-pair)))
          (setf (elt buf 1) (* resolution (cdr index-pair)))
          (setf (elt buf 2) 0)
          buf)
      (float-vector (* resolution (car index-pair))
                    (* resolution (cdr index-pair))
                    0)))
  (:global-point (index-pair)
    (send plane-coords :transform-vector (send self :local-point index-pair)))
  (:prepare-for-planning ()
    "This method is called bedfore each planning"
    (send occupiedp-timer :reset-timer)
    (send on-plane-p-timer :reset-timer)
    (send index-pair-timer :reset-timer)
    (send placablep-timer :reset-timer)
    (send inverse-transform-vector-timer :reset-timer))
  (:report ()
    (send occupiedp-timer :report)
    (send on-plane-p-timer :report)
    (send index-pair-timer :report)
    (send placablep-timer :report)
    (send inverse-transform-vector-timer :report)
    )
  (:plane () plane-obj)
  (:on-plane-p (p n)
    "Check whether p (in global coordinates) is on the plane or not"
    (bench-timer
     on-plane-p-timer
     (and
      (> (v. (send plane-obj :normal) n) 0.95)
      (send plane-obj :on-plane-p p 10.0))))  ;1cm error is allowed
  (:cell-num () cell-num)
  (:worldcoords ()
    plane-coords)
  (:vertices ()
    (let ((vs nil))
      (dotimes (i cell-width)
        (dotimes (j cell-height)
          (when (send self :occupiedp* i j)
            (setq vs (cons (send self :cell-index-to-global-point i j) vs))
          )))
      vs))
  (:collision-check-objects ()
    nil)
  (:draw (vwer)
    (gl::glPushAttrib gl::GL_ALL_ATTRIB_BITS)
    (gl::glDisable gl::GL_LIGHTING)
    (gl::glColor3fv (gl::find-color (get self :face-color)))
    (gl::glEnableClientState gl::GL_VERTEX_ARRAY)
    (gl::glVertexPointer 3 gl::GL_DOUBLE 0 draw-vertices)
    (gl::glDrawArrays gl::GL_LINES 0 (/ (length draw-vertices) 3))
    (gl::glDisableClientState gl::GL_VERTEX_ARRAY)
    (gl::glEnable gl::GL_LIGHTING)
    (gl::glPopAttrib)
    )
  (:project-coords (coords)
    "Project coordinates on to the plane and it does not take into account
which is inside of occupied cell"
    (let ((point (send plane-obj :project (send coords :worldpos))))
      ;; rotation
      (let ((rot (quaternion2matrix
                  (quaternion-from-two-vectors
                   (send coords :rotate-vector (float-vector 0 0 1))
                   (send plane-coords :rotate-vector (float-vector 0 0 1))))))
        (let ((ret (make-coords :pos point :rot (m* rot (send coords :worldrot)))))
          (setf (get ret :projecting-grid) self)
          ret))))
  (:project-point (p)
    "Project point on to the plane"
    (send plane-obj :project p))
  (:placablep (point n)
    (bench-timer2
     placablep-timer
     ;;(ros::ros-info "plane-obj: ~A n: ~A~%" (send plane-obj :normal) n)
     (let ((dot (abs (v. (send plane-obj :normal) n))))
       (if (or (> dot 1.0) (< (rad2deg (acos dot)) 5))
           (progn
             (setq local-point-cache
                   (bench-timer2
                    inverse-transform-vector-timer
                    (send plane-coords :inverse-transform-vector point)))
             (if (< (abs (elt local-point-cache 2)) 10.0)
                 (progn
                   ;; (ros::ros-info "plane check is OK")
                   ;;(send point :draw-on :flush nil :size 0.5)
                   (send self :occupiedp
                         (send self :index-pair local-point-cache :local t)))
               (progn
                 ;;(ros::ros-info "z is ~A" (elt local-point-cache 2))
                 nil)
               ))
         (progn
           ;;(ros::ros-info "theta is ~A" (rad2deg (acos dot)))
           nil)
         ))))
  (:index-pair (pos &key (local t))
    (bench-timer2
     index-pair-timer
     (unless local
       ;; convert to local coordinates
       (setq pos (send plane-coords :inverse-transform-vector pos)))
     (cons (round (/ (elt pos 0) resolution))
           (round (/ (elt pos 1) resolution)))))
  (:cell-index (index-pair)
    "Convert index-pair into cell-local index"
    ;; Convert to local index coordinates
    (send self :cell-index*
          (- (car index-pair) min-cell-x)
          (- (cdr index-pair) min-cell-y)))
  (:cell-index-to-local-point (i j &optional buf)
    (send self :local-point (cons (+ i min-cell-x)
                                  (+ j min-cell-y))
          buf))
  (:local-point-to-global-point (lp &optional buf)
    (if buf
        (v+ (transform (send plane-coords :rot) lp buf) (send plane-coords :worldpos) buf)
      (v+ (transform (send plane-coords :rot) lp) (send plane-coords :worldpos))))
  (:cell-index-to-global-point (i j)
    (send self :local-point-to-global-point
          (send self :cell-index-to-local-point i j)))
  (:cell-index* (i j)
    (+ i (* cell-width j)))
  (:check-range* (i j)
    (and (>= i 0) (>= j 0)
         (< i cell-width) (< j cell-height)))
  (:check-range (pair)
    (send self :check-range*
          (- (car pair) min-cell-x)
          (- (cdr pair) min-cell-y)))
  (:occupiedp* (i j)
    ;; chancke range
    (and (send self :check-range* i j)
         (= (elt cell-bits (send self :cell-index* i j)) 1)))
  (:occupiedp (index-pair)
    (and (send self :check-range index-pair)
         (= (elt cell-bits (send self :cell-index index-pair)) 1)))
  (:register-cell (index-pair)
    (setf (elt cell-bits (send self :cell-index index-pair)) 1))
  (:register-cell* (i j)
    (setf (elt cell-bits (send self :cell-index* i j)) 1))
  (:remove-cell (index-pair)
    (setf (elt cell-bits (send self :cell-index index-pair)) 0))
  (:remove-cell* (i j)
    (setf (elt cell-bits (send self :cell-index* i j)) 0))
  (:map-cells (f)
    "f should be lambda form like #'(lambda (x y) ...)"
    (dotimes (i cell-width)
      (dotimes (j cell-height)
        (when (send self :occupiedp* i j)
          (funcall f i j)
          ))))
  )

;; Utilify functions for grid-maps (array of occupancy-grid)

(defun lookup-highest-corresponding-grid-map (coords grid-maps)
  (let ((max-height -10000)
        (max-grid nil))
    (dolist (grid grid-maps)
      (let ((projected-coords (send grid :project-coords coords)))
        (if (send grid :occupiedp
                  (send grid :index-pair (send projected-coords :worldpos)
                        :local nil))
          (let ((height (elt (send projected-coords :worldpos) 2)))
            (when (> height max-height)
              (setq max-height height)
              (setq max-grid grid))))))
    max-grid))


(defun range (num)
  (let ((ret nil))
    (dotimes (i num)
      (push i ret))
    (nreverse ret)))

(setq *face-sampling-num* 2)
(setq *sampling-points-cache* (mapcar #'(lambda (i)
                                          (float-vector 0 0 0))
                                      (range (* (1+ *face-sampling-num*)
                                                (1+ *face-sampling-num*)))))

(defun sampling-foot-points (leg-cube &optional (num *face-sampling-num*))
  (send leg-cube :worldpos)             ;need?
  (let ((bottom-face (find-if #'(lambda (f)
                                  (eq (cadr (send f :face-id)) :bottom))
                              (send leg-cube :faces))))
    (sampling-face-points bottom-face num)))


(defun sampling-face-points (bottom-face
                             &optional
                             (num *face-sampling-num*)
                             (ret (mapcar #'(lambda (i)
                                              (float-vector 0 0 0))
                                          (range (* (1+ num) (1+ num))))))
  (let ((vertices (cdr (send bottom-face :vertices))))
    (let* ((ex (v- (elt vertices 1) (elt vertices 0)))
           (ey (v- (elt vertices 3) (elt vertices 0)))
           (x-length (norm ex))
           (y-length (norm ey)))
      (let ((ux (normalize-vector ex))
            (uy (normalize-vector ey)))
        (dotimes (ix (1+ num))
          (dotimes (iy (1+ num))
            (let ((c (v+ (scale (* (/ x-length num) ix) ux)
                         (v+ (scale (* (/ y-length num) iy) uy)
                             (elt vertices 0)))))
              (setf (elt ret (+ iy (* ix (1+ num)))) c))))
        ret))))

(defun cube-placable-p (leg-cube grids)
  (let ((bottom-face (find-if #'(lambda (f)
                                  (eq (cadr (send f :face-id)) :bottom))
                              (send leg-cube :faces))))
    (face-placable-p bottom-face grids)))

(defun face-placable-p (leg-face grids-or-grid)
  (if (atom grids-or-grid)
      (_face-placable-p leg-face grids-or-grid)
    (_face-placable-p/list leg-face grids-or-grid)))

(defun _face-placable-p/list (leg-face grids)
  (if (null grids)
      t
    (dolist (g grids)                 ;Should cache previous result
      (if (_face-placable-p leg-face g)
          (return-from _face-placable-p/list t)))))

(defun _face-placable-p (leg-face grid)
  (if (null grid)
      t
    (let ((points (sampling-face-points
                   leg-face
                   *face-sampling-num*)))
      ;;(send-all points :draw-on :flush t :color (float-vector 0 0 1))
      ;;(send-all points :draw-on :flush t)
      (dotimes (i (length points))
        (unless (send grid :placablep (elt points i) (send leg-face :normal))
          ;;(ros::ros-info "~A is not placable" (elt points i))
          (return-from _face-placable-p nil)))
      t)))

(defun append-footstep-candidates-by-grids (next-coords-candidate grids)
  ;; next-coords-candidate =  ((transformation . next-leg-pose) ...)
  (if (null grids)
      next-coords-candidate
  (let ((new-candidates nil))
    (dolist (grid grids)
      (dolist (candidate next-coords-candidate)
        (let ((projected-coords (send grid :project-coords (cdr candidate))))
          ;; (ros::ros-info "diff: ~A" (rad2deg (acos (v. (send projected-coords :rotate-vector (float-vector 0 0 1))
          ;;                                              (send (send grid :plane) :normal)))))
          ;; Check center only first
          (if (send grid :occupiedp
                    (send grid
                          :index-pair (send projected-coords :worldpos)
                          :local nil))
              (progn
                ;; memoize projecting grid
                (setf (get projected-coords :projecting-grid) grid)
                (setq new-candidates (cons
                                      (cons (car candidate)
                                            projected-coords)
                                      new-candidates)))))))
    new-candidates)))

(defun project-coordinates-on-nearest-grid (coords grids)
  (let ((min-distance 1000000)
        (min-grid nil))
    (dolist (grid grids)
      (let ((projected-coords (send grid :project-coords coords)))
        (if (send grid :occupiedp
                  (send grid :index-pair (send projected-coords :worldpos)
                        :local nil))
            (let ((distance (norm (send coords :difference-position projected-coords))))
              (when (< distance min-distance)
                (setq min-distance distance)
                (setq min-grid grid))))))
    (if min-grid (send min-grid :project-coords coords))))

(defun project-and-remove-on-grids (candidate-coordinates
                                    grids-candidate
                                    &optional
                                    (grids-blacklist)
                                    (enable-local-search)
                                    (roll-paranoid nil)
                                    (g-sensor-coords nil)
                                    (current-grid)
                                    (next-leg nil)
                                    (parent-coords nil))
  "
   1. Project coordinates on grid
   2. Check center of coordinate can be supported by the grid
   "
  (when roll-paranoid
    ;; If roll-paranoid mode,
    ;; we need to check legs for different levels
    ;;(ros::ros-info "is current-grid rolling: ~A" (send current-grid :rolledp g-sensor-coords parent-coords))
    (when (send current-grid :rolledp g-sensor-coords parent-coords)
      (when *debug-verbose*
        (ros::ros-info "current grid is ~A"
                       (send current-grid :roll-direction g-sensor-coords parent-coords)))
      (cond
       ((eq (send current-grid :roll-direction g-sensor-coords parent-coords)
            :left)                  ;should use right leg
        (if (not (eq next-leg :rleg))
            (return-from project-and-remove-on-grids nil)))
       ((eq (send current-grid :roll-direction g-sensor-coords parent-coords)
            :right)                  ;should use right leg
        (if (not (eq next-leg :lleg))
            (return-from project-and-remove-on-grids nil)))
       )))
  (let ((new-candidates nil))
    (dolist (grid grids-candidate)
      (let ((roll-check t))
        (when roll-paranoid
          (when *debug-verbose*
            (ros::ros-info "rolledp2: ~A" (send grid :rolledp g-sensor-coords parent-coords)))
          ;;(ros::ros-info "is next-grid rolling: ~A" (send grid :rolledp g-sensor-coords parent-coords))
          (when (send grid :rolledp g-sensor-coords parent-coords)
            (when *debug-verbose*
              (ros::ros-info "candidate grid is ~A" (send grid :roll-direction g-sensor-coords parent-coords)))
            (cond
             ((eq (send grid :roll-direction g-sensor-coords parent-coords)
                  :left)                  ;should use right leg
              (if (not (eq next-leg :lleg))
                  (setq roll-check nil)))
             ((eq (send grid :roll-direction g-sensor-coords parent-coords)
                  :right)                  ;should use right leg
              (if (not (eq next-leg :rleg))
                  (setq roll-check nil))
              ))))
        (when (and (not (member grid grids-blacklist))
                   roll-check)
          (dolist (candidate candidate-coordinates)
            (let ((projected-coords (send grid :project-coords (cdr candidate))))
              (when (send grid :occupiedp
                          (send grid
                                :index-pair (send projected-coords :worldpos)
                                :local nil))
                (setf (get projected-coords :projecting-grid) grid) ;memoize :projecting-grid
                (setf new-candidates (cons (cons (car candidate)
                                                 projected-coords)
                                           new-candidates))
                (when (and enable-local-search *use-local-search*)
                  (let ((local-x-max 50)
                        (local-x-min 0)
                        (local-y-max 50)
                        (local-y-min -50)
                        (local-theta-min (- (deg2rad 5)))
                        (local-theta-max (deg2rad 5))
                        (sampling-num 3))
                    (dotimes (xi sampling-num)
                      (dotimes (yi sampling-num)
                        (dotimes (thetai sampling-num)
                          (let ((x (+ (* (/ (- local-x-max local-x-min) sampling-num) xi) local-x-min))
                                (y (+ (* (/ (- local-y-max local-y-min) sampling-num) yi) local-y-min))
                                (theta (+ (* (/ (- local-theta-max local-theta-min) sampling-num) thetai) local-theta-min)))
                            (let ((trans (make-coords :pos (float-vector x y 0)
                                                      :rpy (float-vector theta 0 0))))
                              (let ((transed-coords (send (send projected-coords :copy-worldcoords) :transform trans)))
                                (setf (get transed-coords :projecting-grid) grid)
                                (setf new-candidates (cons (cons (car candidate)
                                                                 transed-coords)
                                                           new-candidates))))))))))))))))
    ;;(ros::ros-info "~A/~A projecting candidate" (length new-candidates) (length candidate-coordinates))
    new-candidates))


(defun remove-not-on-grids (candidate-coordinates
                            grid)
  (remove-if-not #'(lambda (c)
                     (let ((coords (cdr c)))
                       (send grid :occupiedp
                             (send grid :index-pair (send coords :worldpos) :local nil))))
                 candidate-coordinates))

(defun remove-shadow-cell-with-index (target-grid
                                      remover-grid i j
                                      x-padding y-padding)
  (let ((p (send remover-grid :local-point-to-global-point
                 (send remover-grid :cell-index-to-local-point i j)))
        (counter 0))
    (let ((projected-p (send target-grid :project-point p)))
      (when (< (elt (v- projected-p p) 2) 0)
        ;; remover-grid is upper of target-grid
        (let* ((index-pair (send target-grid :index-pair p :local t))
               (center-x (car index-pair))
               (center-y (cdr index-pair)))
         (dotimes (ix (1+ (* x-padding)))
           (dotimes (iy (1+ (* y-padding)))
             (let ((x (+ center-x ix (- x-padding)))
                   (y (+ center-y iy (- y-padding))))
               (when (send target-grid :occupiedp (cons x y))
                 (send target-grid :remove-cell (cons x y))
                 (incf counter))))))))
    counter))

(defun remove-shadow-cells (grids)
  (let ((counter 0))
    (dotimes (i (length grids))
      (let ((remover-grid (elt grids i)))
        (send remover-grid :map-cells
              #'(lambda (jx jy)
                  (dotimes (j (length grids))
                    (when (not (= i j))
                      (let ((target-grid (elt grids j)))
                        (incf counter 
                              (remove-shadow-cell-with-index
                               target-grid remover-grid
                               jx jy 1 1))
                        )))))))
    (send-all grids :update-draw-vertices)
    (ros::ros-info "Removed ~A cells as shadow" counter)))

(defun footstep-set-placable-p (coords grid footstep-param lleg-trans rleg-trans
                                       &optional debug)
  "Check both of left and right footsteps can be supported by the grid"
  (let ((lleg-coords
         (send (send coords :copy-worldcoords) :transform lleg-trans))
        (rleg-coords
         (send (send coords :copy-worldcoords) :transform rleg-trans)))
    (let ((lleg-face
           (instance face :init :vertices
                     (send footstep-param :foot-vertices :lleg lleg-coords)))
          (rleg-face
           (instance face :init :vertices
                     (send footstep-param :foot-vertices :rleg rleg-coords))))
      (if debug
          (send-all (list lleg-face rleg-face)
                    :draw-on :flush t))
      (and (face-placable-p lleg-face grid)
           (face-placable-p rleg-face grid)))))

(defun local-search-footstep-on-grid
  (projected-coords grid footstep-param lleg-trans rleg-trans)
  "Locally search coordinates where can put left and right foot"
  (if (footstep-set-placable-p
       projected-coords grid footstep-param lleg-trans rleg-trans)
      projected-coords
    (let ((xrange 200.0)
          (yrange 200.0)
          (trange (deg2rad 30.0))
          (sampling-num 10)
          (candidates nil))
      (ros::ros-warn "Projecting with local search")
      (dotimes (ix sampling-num)
        (dotimes (iy sampling-num)
          (dotimes (it sampling-num)
            (let ((x (- (/ (* xrange ix) sampling-num)
                        (/ xrange 2.0)))
                  (y (- (/ (* yrange iy) sampling-num)
                        (/ yrange 2.0)))
                  (theta (- (/ (* trange it) sampling-num)
                            (/ trange 2.0))))
              (let ((c (send (send projected-coords :copy-worldcoords)
                             :transform
                             (make-coords :pos (float-vector x y 0)
                                          :rpy (float-vector theta 0 0)))))
                (when (footstep-set-placable-p
                       c grid footstep-param lleg-trans rleg-trans)
                  (setf (get c :ix) ix)
                  (setf (get c :iy) iy)
                  (setf (get c :it) it)
                  (push c candidates)))))))
      (car (sort candidates #'< #'(lambda (c)
                                    (+ (expt (- (get c :ix) (/ sampling-num 2.0)) 2)
                                       (expt (- (get c :iy) (/ sampling-num 2.0)) 2)
                                       (expt (- (get c :it) (/ sampling-num 2.0)) 2))))))))

#|
(load "package://jsk_footstep_planner/euslisp/simple_occupancy_grid.l")
(ros::roseus "hoge")
(setq *tf* (instance ros::transform-listener :init))
(defun callback (msg)
  (setq *grids* (remove-if #'null (mapcar #'(lambda (g)
                            (let ((trans (lookup-transform-with-duration
                                          *tf*
                                          "map"
                                          (send g :header :frame_id)
                                          (send g :header :stamp)
                                          1.0)))
                              (if trans
                                  (instance occupancy-grid :init g trans))))
                        (send msg :grids))))
  (dotimes (i (length *grids*))
    (setf (get (elt *grids* i) :face-color) (gl::color-category20 i)))
  (objects *grids*))
(ros::subscribe "/locomotion/env_server/output" jsk_recognition_msgs::SimpleOccupancyGridArray #'callback)
(ros::spin-once)
|#
