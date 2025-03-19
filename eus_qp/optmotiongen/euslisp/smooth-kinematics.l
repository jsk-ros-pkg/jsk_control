(load "./util.l")


(defmethod line
  (:nearest-point-distance
   (p)
   "get the nearest point and distance to the nearest point"
   (let* ((par (send self :foot p))
          )
     (cond
      ((> par 1) (list nvert (distance nvert p)))
      ((< par 0) (list pvert (distance pvert p)))
      (t (list (send self :point par) (distance (send self :point par) p))))
     ))
  )

(defmethod face
  (:nearest-point-distance
   (point)
   "get the nearest point and distance to the nearest point"
   (let (foot)
     (setq foot (send self :foot point))
     (cond ((memq (send self :insidep foot) '(:inside :parallel))
            (list foot (distance foot point))
            )
           (t
            (find-extream (send-all (send self :all-edges) :nearest-point-distance point)
                          #'(lambda (x) (elt x 1)) #'<)
            ))
     ))
  (:neighbor-face-point-distance
   (pos &key (exclude-faces))
   "get the nearest point and distance for all neighbor faces"
   (remove
    nil
    (mapcar
     #'(lambda (e &aux (f (car (remove self (send e :faces)))))
         (if (not (member f exclude-faces :test #'equal))
             (append (list f) (send e :nearest-point-distance pos)))
         )
     (send self :edges)
     ))
   )
  (:serach-neighbor-face-within-distance
   (pos
    &key
    (dist 0.0)
    (dist-max 500.0)
    (depth 1)
    (depth-max 3)
    (exclude-faces)
    &aux
    (neighbor-face-point-distance
     (send self :neighbor-face-point-distance
           pos
           :exclude-faces exclude-faces
           ))
    )
   "search faces within distance"
   (setq neighbor-face-point-distance
         (remove-if-not
          #'(lambda (ret &aux (new-dist (+ dist (elt ret 2))))
              (and (<= new-dist dist-max) (<= depth depth-max)))
          neighbor-face-point-distance))
   (append
    (mapcan
     #'(lambda (ret &aux (new-dist (+ dist (elt ret 2))))
         (list (list (elt ret 0) new-dist)))
     neighbor-face-point-distance
     )
    (mapcan
     #'(lambda (ret)
         (send (elt ret 0) :serach-neighbor-face-within-distance
               (elt ret 1)
               :dist (+ dist (elt ret 2))
               :dist-max dist-max
               :depth (+ depth (1+ depth))
               :depth-max depth-max
               :exclude-faces (list self)
               )
         )
     neighbor-face-point-distance
     ))
   )
  (:neighbor-face-distance
   (pos
    &key
    (dist-max 500.0)
    (depth-max 3)
    &aux
    (face-dist-list
     (send self :serach-neighbor-face-within-distance
           pos :dist-max dist-max :depth-max depth-max))
    (faces
     (remove
      self
      (remove-duplicates
       (mapcar #'(lambda (x) (elt x 0)) face-dist-list)
       :test #'equal)
      :test #'equal)
     )
    )
   "get face and minimum distance within distance"
   (mapcar
    #'(lambda (f)
        (find-extream
         (remove-if-not #'(lambda (ret) (equal (elt ret 0) f)) face-dist-list)
         #'(lambda (ret) (elt ret 1)) #'<=))
    faces
    ))
  )

(defmethod faceset
  (:set-projection-mode
   (mode)
   "set projection mode"
   (send self :put :projection-mode mode)
   )
  (:project
   (pos)
   "project point to body surface. projected point is the nearest point on the body."
   (cond ((equal (send self :get :projection-mode) :bt-collision)
          (send self :project-with-bt-collision pos)
          )
         (t ;; (equal (send self :get :projection-mode) :distance)
          (send self :project-with-distance pos)
          ))
   )
  (:project-with-bt-collision
   (pos &aux (sphere (send self :get :sphere-for-projection)))
   "project point to body surface by using bullet collision function."
   (unless sphere
     (send self :put :sphere-for-projection (make-sphere 1))
     (setq sphere (send self :get :sphere-for-projection))
     (send sphere :make-btmodel)
     (geo::_bt-set-margin (send sphere :get :btmodel) -0.04)
     )
   (send sphere :newcoords (make-coords :pos pos))
   (elt (bt-collision-distance sphere self) 2)
   )
  (:project-with-distance
   (pos)
   "project point to body surface by computing the distance."
   (elt
    (find-extream (send-all (send self :faces) :nearest-point-distance pos)
                  #'(lambda (x) (elt x 1)) #'<)
    0)
   )
  (:calc-normal-tangent
   (pos
    &key
    (flip-normal? nil)
    &aux
    (smooth-dist 50.0)
    (normal
     (scale (if flip-normal? -1 1)
            (calc-smooth-normal self pos :smooth-dist smooth-dist)))
    (tangent1
     (cond ((eps= (norm (v* normal #f(1 0 0))) 0.0)
            (normalize-vector (v* normal #f(0 1 0)))
            )
           (t
            (normalize-vector (v* normal #f(1 0 0)))
            )))
    (tangent2 (v* normal tangent1))
    )
   "calculate one smooth normal vector and two tanget vectors on the surface point."
   (list normal (list tangent1 tangent2))
   )
  )

(defun calc-smooth-normal
    (body0 pos &key (smooth-dist 20.0))
  "
Calculate smooth normal by weighted average of neighbor normals.
Average is calculaged by using slearp.
  "
  (let* ((belong-face
          (find-extream (send body0 :faces) #'(lambda (f) (send f :distance pos)) #'>))
         (neighbor-normal-weight-list
          (mapcar
           #'(lambda (x)
               (list (send (elt x 0) :normal) (interpolate-one-to-zero (elt x 1) smooth-dist))
               )
           (send belong-face :neighbor-face-distance pos :dist-max smooth-dist :depth-max 3))
          ;; original implementation
          ;; (remove
          ;;  nil
          ;;  (mapcar
          ;;   #'(lambda (e &aux (d (send e :distance-point pos)) (neighbor-face))
          ;;       (cond ((< d smooth-dist)
          ;;              (setq neighbor-face
          ;;                    (car (remove belong-face (send e :faces))))
          ;;              (list (send neighbor-face :normal) (interpolate-one-to-zero d smooth-dist))
          ;;              )))
          ;;   (send belong-face :edges)
          ;;   ))
          )
         (normal-weight-list
          (append
           (list (list (send belong-face :normal) 1.0))
           neighbor-normal-weight-list
           ))
         (normal-list
          (mapcar #'(lambda (x) (elt x 0)) normal-weight-list))
         (weight-list
          (mapcar #'(lambda (x) (elt x 1)) normal-weight-list))
         (smooth-normal
          (calc-weighted-average normal-list weight-list)
          ;; (calc-weighted-average-with-interpolation normal-list weight-list :interpolate-func #'slearp)
          )
         )
    smooth-normal
    ))

(defun slearp
    (ratio
     v0
     v1
     &key
     (dot-thre 0.9995)
     &aux
     (dot (v. v0 v1))
     )
  "
Spherical linear interpolation.
Implemented based on Python sample in \url{https://en.wikipedia.org/wiki/Slerp}
  "
  ;; (when (< dot 0.0)
  ;;   (setq v1 (scale -1 v1))
  ;;   (setq dot (- dot))
  ;;   )

  ;; two vectors are near enough to use ordinal linear interpolation
  (when (> dot dot-thre)
    (return-from slearp (normalize-vector (v+ v0 (scale ratio (v- v1 v0)))))
    )

  (let* ((theta0 (acos dot))
         (sin-theta0 (sin theta0))
         (theta (* ratio theta0))
         (sin-theta (sin theta))
         (s0 (- (cos theta) (/ (* dot sin-theta) sin-theta0)))
         (s1 (/ sin-theta sin-theta0))
         )
    (v+ (scale s0 v0) (scale s1 v1))
    ))

(defun slearp2
    (ratio
     v0
     v1
     &aux
     (n (v* v0 v1))
     (theta0 (acos (v. v0 v1)))
     (theta (* ratio theta0))
     )
  "
Spherical linear interpolation.
Implemented based on 3D rotation matrix.
  "
  (transform (rotation-matrix theta n) v0)
  )

(defun interpolate-one-to-zero
    (x
     &optional
     (a 1.0)
     &aux
     (x/a (/ x a))
     (x/a-1 (- x/a 1.0))
     )
  "
Interpolate from 1 to 0 with piecewise quadratic function.
\begin{eqnarray}
f(x) = \left\{ \begin{array}{ll}
-2 \left( \frac{x}{a} \right)^2 + 1 & (x \leq 0.5) \\
2 \left( \frac{x}{a} - 1 \right)^2 & (x \geq 0.5)
\end{array} \right.
\end{eqnarray}
  "
  ;; gnuplot
  ;; a=1.0
  ;; set xr [0:a]
  ;; plot x<0.5*a?-2.0*(x/a)**2+1:2.0*(x/a-1)**2
  ;; plot x<0.5*a?-4.0*(x/a)/a:4.0*(x/a-1)/a # derivative
  (cond ((< x 0) 1.0)
        ((< x (* 0.5 a)) (+ (* -2.0 x/a x/a) 1))
        ((< x a) (* 2.0 x/a-1 x/a-1))
        (t 0.0))
  )

(defun calc-weighted-average
    (value-list
     weight-list
     )
  (normalize-vector
   (scale
    (/ 1.0 (apply #'+ weight-list))
    (reduce
     #'v+
     (mapcar
      #'(lambda (v w) (scale w v))
      value-list
      weight-list
      ))))
  )

(defun calc-weighted-average-with-interpolation
    (value-list
     weight-list
     &key
     (interpolate-func #'midpoint)
     &aux
     (ave-value (car value-list))
     (accumulated-weight (float (car weight-list)))
     (weight)
     (value)
     )
  "
Calculate weighted average from interpolation function.
  "
  (dotimes (i (1- (length value-list)))
    (setq value (elt value-list (1+ i)))
    (setq weight (elt weight-list (1+ i)))
    (setq accumulated-weight (+ accumulated-weight weight))
    (setq ave-value (funcall interpolate-func (/ weight accumulated-weight) ave-value value))
    )
  ave-value
  )

(defun make-ellipsoid
    (&optional
     (a 100.0)
     (b 100.0)
     (c 200.0)
     &key
     (theta-divide-num 20)
     (phi-divide-num (* 2 theta-divide-num))
     (half? nil)
     )
  "
Make ellipsoid body.
  "
  (let* ((theta 0.0)
         (phi 0.0)
         (delta-theta (/ pi theta-divide-num))
         (delta-phi (/ 2pi phi-divide-num))
         (vertices)
         (ellips)
         )
    (while (<= theta (if half? pi/2 pi))
      (setq phi 0.0)
      (while (<= phi 2pi)
        (push
         (float-vector
          (* a (sin theta) (cos phi))
          (* b (sin theta) (sin phi))
          (* c (cos theta)))
         vertices)
        (setq phi (+ phi delta-phi))
        )
      (setq theta (+ theta delta-theta))
      )
    (setq ellips
          (convex-hull-3d
           (remove-duplicates
            vertices
            :test #'(lambda (x y) (< (norm (v- x y)) 1e-3)))))
    (send ellips :csg (list (list :ellipsoid a b c)))
    ellips
    ))

(defun generate-surface-configuration-task
    (&key
     ((:link link0))
     ((:body body0)
      (convex-hull-3d
       (remove-duplicates
        (flatten (send-all (send link0 :bodies) :vertices))
        :test #'(lambda (x y) (<= (norm (v- x y)) 1e-3))))
      )
     (kin-target-coords)
     (initial-surface-pos
      (cond ((functionp kin-target-coords)
             (send (funcall kin-target-coords) :worldpos)
             )
            (t
             (send kin-target-coords :worldpos)
             )))
     (delta-u 0.05)
     (flip-normal? nil)
     (normal-task-scale 2.0)
     (norm-regular-scale-max 1e10)
     (norm-regular-scale-offset 1e0)
     (config-task)
     )
  "
generate surface-configuration-task instance from link.
  "
  (when link0
    (send link0 :assoc body0))
  (setq config-task
        (instance surface-configuration-task :init
                  :body body0
                  :initial-surface-pos initial-surface-pos
                  :kin-target-coords kin-target-coords
                  :delta-u delta-u
                  :flip-normal? flip-normal?
                  :normal-task-scale normal-task-scale
                  :norm-regular-scale-max norm-regular-scale-max
                  :norm-regular-scale-offset norm-regular-scale-offset
                  ))
  (send config-task :put :initial-surface-pos initial-surface-pos)
  config-task
  )
