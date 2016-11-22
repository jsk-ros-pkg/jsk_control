;; utility function for footstep planning
(ros::roseus-add-msgs "jsk_footstep_msgs")
(ros::roseus-add-msgs "jsk_recognition_msgs")

(defun project-coords-on-to-plane (coords planes z-axis)
  "z-axis is dummy parameter.

   sample:
   (send-all (project-coords-on-to-plane
           (make-coords) (send (make-cube 100 100 100) :faces)
           (float-vector 0 0 1))
          :draw-on :flush t)"
  (let ((candidates (mapcar #'(lambda (p)
                                (let ((rot (quaternion2matrix
                                            (geo::quaternion-from-two-vectors
                                              (float-vector 0 0 1)
                                              (send p :normal)))))
                                  (make-coords
                                   :rot (m* rot (send coords :rot))
                                   :pos (send p :project (send coords :worldpos)))))
                            planes)))
    (mapcar #'car (remove-if #'(lambda (arg)
                                 (let ((c (car arg))
                                       (p (cdr arg)))
                                   (eq (send p :insidep
                                             (send p :project
                                                   (send c :worldpos)))
                                       :outside)))
                             (mapcar #'cons candidates planes)))))
                       

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
        (let ((req (instance jsk_recognition_msgs::PolygonOnEnvironmentRequest :init)))
          ;;(send req :environment_id *env-id*)
          (send req :environment_id 0)
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

(defun lookup-transform-with-duration (tf-listener from to stamp duration)
  (if (send tf-listener :wait-for-transform from to stamp duration)
      (send tf-listener :lookup-transform from to stamp)
    (progn (ros::ros-error "Failed to lookup transform ~A to ~A" from to)
           nil)))

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

;; TimerClass
(defclass counter-timer
  :super propertied-object
  :slots (name count timer duration))

(defmethod counter-timer
  (:init (aname)
    (setq name aname)
    (send self :reset-timer)
    self)
  (:count () count)
  (:proc-start ()
    (send timer :start))
  (:proc-end ()
    (incf count)
    (setq duration (+ duration (send timer :stop))))
  (:reset-timer ()
    (setq count 0)
    (setq duration 0)
    (setq timer (instance mtimer :init))
    )
  (:report ()
    (if (not (= count 0))
        (ros::ros-info "~A takes ~A secs (~A sec per trial, ~A trials)"
          name duration (/ duration count) count)))
  )

(defmacro bench-timer (timer &rest prog)
  `(progn
     (send ,timer :proc-start)
     (prog1 (progn ,@prog)
       (send ,timer :proc-end)
       )))

(defmacro bench-timer2 (timer &rest prog)
  `(progn
     ,@prog))


#|
(setq tmp-timer (instance counter-timer :init "tmp"))
(dotimes (i 10000)
  (bench-timer
   tmp-timer
   (length (make-list 10000))))
|#