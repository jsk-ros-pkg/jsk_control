(load "../smooth-kinematics.l")


;; copied from
;; https://github.com/jsk-ros-pkg/euslib/blob/master/jsk/sample_poisson.l
(defun sample-stochastic-over-mesh-uniform
    (obj
     &key
     (N 100)
     (triangulate-before-sample nil)
     )
  (flet ((binsearch-face
          (tab val)
          (labels ((binsearch-aux
                    (i j)
                    (let ((m (/ (+ i j) 2)))
                      (cond
                       ((= m i) (aref tab m))
                       ((< val (car (aref tab m))) (binsearch-aux i m))
                       (t (binsearch-aux m j)))))
                   )
            (binsearch-aux 0 (length tab))
            )))
    (let* ((obj2
            (if triangulate-before-sample
                (body-to-triangles obj) obj))
           (tab (make-array (length (send obj2 :faces))))
           (i 0)
           (cumulative-area 0.0)
           points)
      (dolist (fc (send obj2 :faces))
        (setq cumulative-area (+ cumulative-area (send fc :area)))
        (aset tab i (cons cumulative-area fc))
        (incf i))
      (dotimes (i N)
        (let* ((r (random cumulative-area))
               (fc (cdr (binsearch-face tab r))))
          (multiple-value-bind
           (A B C _) (send fc :vertices)
           (let ((P (sample-stochastic-from-triangle-uniform A B C)))
             (push P points)
             ))))
      points)
    ))

(defun sample-stochastic-from-triangle-uniform
    (A B C)
  (let ((sqrr1 (sqrt (random 1.0)))
        (r2 (random 1.0)))
    (v+
     (v+ (scale (- 1 sqrr1) A) (scale (* sqrr1 (- 1 r2)) B))
     (scale (* sqrr1 r2) C))
    ))

(defun sample-smooth-normal
    (&key
     (smooth-dist 50.0)
     (body-type :cube)
     )
  (cond ((equal body-type :cube)
         (setq *body* (make-cube 200 400 100))
         )
        ((equal body-type :cone)
         (setq *body* (make-cone #f(0 100 200) (list #f(200 -100 0) #f(-200 -100 0) #f(0 200 0))))
         )
        ((equal body-type :prism)
         (setq *body* (make-prism (list #f(100 -100 0) #f(-100 -100 0) #f(0 100 0)) 200))
         )
        ((equal body-type :ellipsoid)
         (setq *body* (make-ellipsoid))
         )
        ((equal body-type :link)
         (require :rhp3 "package://khi_ros_bridge_tutorials/euslisp/rhp3-interface.l")
         (setq *robot* (instance rhp3-robot :init))
         (setq *body* (elt (send (elt (send *robot* :links) 10) :bodies) 0))
         (setq *body* (convex-hull-3d (send (elt (send (elt (send *robot* :links) 10) :bodies) 0) :vertices)))
         (send *body* :translate (scale -1 (vector-mean (send *body* :vertices))))
         ))
  (unless (boundp '*irtviewer*)
    (make-irtviewer)
    (send *irtviewer* :change-background (float-vector 1 1 1))
    (send *irtviewer* :draw-floor 500)
    (send *irtviewer* :floor-color (float-vector 0 0 0))
    )
  (send *irtviewer* :objects (list *body*))
  (send *irtviewer* :draw-objects)

  (setq *sampled-points*
        (sample-stochastic-over-mesh-uniform *body* :N 200 :triangulate-before-sample t))
  (with-set-color-line-width
   *viewer* #f(1 0 0) 4
   (dolist (sp *sampled-points*)
     (send *irtviewer* :viewer :draw-arrow
           sp
           (v+ sp (scale 50 (calc-smooth-normal *body* sp :smooth-dist smooth-dist)))
           nil)
   ))
  (send *viewer* :flush)
  )
(warn "(sample-smooth-normal)~%")
