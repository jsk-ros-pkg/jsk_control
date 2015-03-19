;; Utility to deal with jsk_recognition_msgs::SimpleOccupancyGrid(Array)
(load "package://jsk_footstep_planner/euslisp/footstep_planner_util.l")
(ros::roseus-add-msgs "jsk_recognition_msgs")

(defun quaternion-from-two-vectors (a b)
  (let* ((v0 (normalize-vector a))
         (v1 (normalize-vector b))
         (c (v. v1 v0)))
    (let* ((axis (v* v0 v1))
           (s (sqrt (* 2 (+ 1 c))))
           (invs (/ 1.0 s)))
      (let ((vec (scale invs axis))
            (w (* 0.5 s)))
        (normalize-vector (float-vector w (elt vec 0) (elt vec 1) (elt vec 2)))))))

(defun coefficients->plane-coords (coefficients)
  (let ((z-axis (normalize-vector (float-vector (elt coefficients 0)
                                                (elt coefficients 1)
                                                (elt coefficients 2))))
        (unitz (float-vector 0 0 1)))
    (let ((c (make-coords :rot (quaternion2matrix (quaternion-from-two-vectors unitz z-axis)))))
      (send c :translate (float-vector 0 0 (- (* 1000.0 (elt coefficients 3))))))))

(defclass occupancy-grid
  :super propertied-object
  :slots (resolution coefficients plane-coords frame-id cell-hash plane-obj))

(defmethod occupancy-grid
  (:init (msg global-trans)
    "Instantiating from ros message"
    ;; lookup transformation
    (setq frame-id (send msg :header :frame_id))
    (setq resolution (* 1000 (send msg :resolution)))
    (setq coefficients (send msg :coefficients))
    (setq plane-coords
          (send (send global-trans :copy-worldcoords)
                :transform (coefficients->plane-coords coefficients)))
    (setq plane-obj (instance plane :init
                              (send plane-coords :rotate-vector (float-vector 0 0 1))
                              (send plane-coords :worldpos)))
    ;; hash table
    (setq cell-hash (make-hash-table :test #'equal :size 10000))
    (ros::ros-info "~A points" (length (send msg :cells)))
    (dolist (cell-point (send msg :cells))
      ;; cell-point is geometry_msgs::Point
      (let ((cell-x (* 1000 (send cell-point :x)))
            (cell-y (* 1000 (send cell-point :y))))
        (setf (gethash (send self :index-pair (float-vector cell-x cell-y 0))
                       cell-hash)
              (float-vector cell-x cell-y))))
    
    self)
  (:worldcoords ()
    plane-coords)
  (:vertices ()
    (let ((ret nil))
      (maphash #'(lambda (key val)
                   (setf ret (cons (send plane-coords :transform-vector (float-vector (elt val 0) (elt val 1) 0)) ret)))
               cell-hash)
      ret))
  (:collision-check-objects ()
    nil)
  (:draw (vwer)
    (gl::glPushAttrib gl::GL_ALL_ATTRIB_BITS)
    (gl::glDisable gl::GL_LIGHTING)
    (let ((col (gl::find-color (get self :face-color)))
          (w (/ resolution 2.0)))
      (maphash #'(lambda (key val)
                   (let* ((center (float-vector (elt val 0) (elt val 1) 0))
                          (v1 (v+ center
                                  (float-vector w w 0)))
                          (v2 (v+ center
                                  (float-vector (- w) w 0)))
                          (v3 (v+ center
                                  (float-vector (- w) (- w) 0)))
                          (v4 (v+ center
                                  (float-vector w (- w) 0))))
                     (let ((gv1 (send plane-coords :transform-vector v1))
                           (gv2 (send plane-coords :transform-vector v2))
                           (gv3 (send plane-coords :transform-vector v3))
                           (gv4 (send plane-coords :transform-vector v4)))
                       (gl::glBegin gl::GL_LINE_STRIP)
                       (gl::glColor3fv col)
                       (gl::glVertex3fv gv1)
                       (gl::glColor3fv col)
                       (gl::glVertex3fv gv2)
                       (gl::glColor3fv col)
                       (gl::glVertex3fv gv3)
                       (gl::glColor3fv col)
                       (gl::glVertex3fv gv4)
                       (gl::glColor3fv col)
                       (gl::glVertex3fv gv1)
                       (gl::glEnd)
                       )))
               cell-hash))
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
        (make-coords :pos point :rot rot))))
  (:index-pair (pos &key (local t))
    (unless local
      ;; convert to local coordinates
      (setq pos (send plane-coords :inverse-transform-vector pos)))
    (list (round (/ (elt pos 0) resolution))
          (round (/ (elt pos 1) resolution))))
  (:occupiedp (index-pair)
    (gethash index-pair cell-hash))
  )

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
