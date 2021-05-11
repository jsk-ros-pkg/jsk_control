;;;;;;;;;;;;;;;;;;;;;;;;;
;; joint (add or overwrite methods)
;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod joint
  (:child-link
   (&rest args)
   "
Returns child link of this joint. If any arguments is set, it is passed to the child-link.\\
Override to support the case that child-link is cascaded-link instantiate. Return the root link of child cascaded-link instantiate in that case.
   "
   (if (derivedp child-link cascaded-link)
       (user::forward-message-to (car (send child-link :links)) args)
     (user::forward-message-to child-link args))
   )
  (:axis-vector
   ()
   "
Return joint axis vector. Represented in world coordinates.

return $\bm{a}_i \in \mathbb{R}^3$
  "
   (let* ((paxis
           (case axis
                 (:x #f(1 0 0)) (:y #f(0 1 0)) (:z #f(0 0 1))
                 (:xx #f(1 0 0)) (:yy #f(0 1 0)) (:zz #f(0 0 1))
                 (:-x #f(-1 0 0)) (:-y #f(0 -1 0)) (:-z #f(0 0 -1))
                 (t axis))
           )
          (world-default-coords
           (send (send parent-link :copy-worldcoords) :transform default-coords))
          )
     (normalize-vector (send world-default-coords :rotate-vector paxis))
     ))
  (:pos
   ()
   "
Return joint position. Represented in world coordinates.

return $\bm{p}_i \in \mathbb{R}^3$
  "
   (send child-link :worldpos)
   )
  )


;;;;;;;;;;;;;;;;;;;;;;;;;
;; bodyset-link (add methods)
;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod bodyset-link
  (:centroid-with-fixed-child-links
   ()
   "
return $\bm{p}_{\mathit{cog,k}} \in \mathbb{R}^3$ [mm]
   "
   (or (send (send self :worldcoords) :transform-vector (send self :get :local-centroid-with-fixed-child-links))
       (send self :centroid))
   )
  (:weight-with-fixed-child-links
   ()
   "
return $m \in \mathbb{R}$ [g]
   "
   (or (send self :get :weight-with-fixed-child-links)
       (send self :weight))
   )
  (:mg
   ()
   "
return $mg = \|m \bm{g}\| \in \mathbb{R}$ [N]
   "
   (or (send self :get :mg)
       (send self :put :mg
             (* 1e-3 (send self :weight-with-fixed-child-links)
                1e-3 (elt *g-vec* 2)))) ;; [N]
   )
  (:mg-vec
   ()
   "
return $m \bm{g} \in \mathbb{R}^3$ [N]
   "
   (or (send self :get :mg-vec)
       (send self :put :mg-vec
             (scale (* 1e-3 (send self :weight-with-fixed-child-links))
                    (scale 1e-3 *g-vec*)))) ;; [N]
   )
  )

(defun find-fixed-child-links
    (l &key joint-list)
  (append
   (list l)
   (mapcan
    #'(lambda (cl)
        (if (not (member (send cl :joint) joint-list))
            (find-fixed-child-links cl :joint-list joint-list))
        )
    (send l :child-links)))
  )

(defun set-mass-property-with-fixed-child-links
    (robot)
  (let* (child-links)
    (dolist (l (send robot :links))
      (unless (send l :get :weight-with-fixed-child-links)
        (setq child-links (find-fixed-child-links l :joint-list (send robot :joint-list)))
        (send l :put :weight-with-fixed-child-links
              (reduce #'+ (send-all child-links :weight)))
        (cond ((eps= (send l :get :weight-with-fixed-child-links) 0.0 1e-10)
               (send l :put :local-centroid-with-fixed-child-links (float-vector 0 0 0))
               )
              (t
               (send l :put :local-centroid-with-fixed-child-links
                     (send (send l :worldcoords)
                           :inverse-transform-vector
                           (apply
                            #'v++
                            (mapcar
                             #'(lambda (cl)
                                 (scale (/ (send cl :weight) (send l :get :weight-with-fixed-child-links))
                                        (send cl :centroid)))
                             child-links
                             )))
                     )
               ))
        )))
  )


;;;;;;;;;;;;;;;;;;;;;;;;;
;; cascaded-link (add or overwrite methods)
;;;;;;;;;;;;;;;;;;;;;;;;;

(setq *tmp-v0* (instantiate float-vector 0))
(setq *tmp-v1* (instantiate float-vector 1))
(setq *tmp-v2* (instantiate float-vector 2))
(setq *tmp-v3* (instantiate float-vector 3))
(setq *tmp-v3a* (instantiate float-vector 3))
(setq *tmp-v3b* (instantiate float-vector 3))
(setq *tmp-m33* (make-matrix 3 3))

(setq *tmp-va* (instantiate float-vector 3))
(setq *tmp-vb* (instantiate float-vector 3))
(setq *tmp-vc* (instantiate float-vector 3))
(setq *tmp-vd* (instantiate float-vector 3))
(setq *tmp-ma* (make-matrix 3 3))

(defmethod cascaded-link
  (:calc-jacobian-from-joint-list
   (&key
    (union-joint-list) ;; required
    (move-target) ;; required
    (joint-list
     (mapcar
      #'(lambda (mt) (send-all (send self :link-list (send mt :parent)) :joint))
      move-target))
    (transform-coords
     (mapcar #'(lambda (mt) (make-coords)) move-target))
    (translation-axis
     (mapcar #'(lambda (mt) t) move-target))
    (rotation-axis
     (mapcar #'(lambda (mt) t) move-target))
    )
   "
\begin{description}
  \item[\it{union-joint-list}] list of all joints considered in jacobian. column num of jacobian is same with length of union-joint-list.
  \item[\it{move-target}] list of move-target.
  \item[\it{joint-list}] list of joint-list which is contained in each chain of move-target.
  \item[\it{transform-coords}] list of transform-coords of each move-target.
  \item[\it{translation-axis}] list of translation-axis of each move-target.
  \item[\it{rotation-axis}] list of rotation-axis of each move-target.
\end{description}

Get jacobian matrix from following two information: (1) union-joint-list and (2) list of move-target.
One recession compared with :calc-jacobian-from-link-list is that child-reverse is not supported.
(Only not implemented yet because I do not need such feature in current application.)
   "
   (let* ((joint-dim
           (calc-target-joint-dimension union-joint-list))
          (task-dim
           (send self :calc-target-axis-dimension rotation-axis translation-axis))
          (jacobi
           (make-matrix task-dim joint-dim))
          (col-idx 0)
          (row-idx 0)
          )
     (dotimes (joint-idx joint-dim) ;; for each joint in union-joint-list
       (setq col-idx joint-idx) ;; update idx
       (setq row-idx 0) ;; update idx
       (dotimes (mt-idx (length move-target)) ;; for each move-target
         (let* ((jnt (elt union-joint-list joint-idx))
                (mt (elt move-target mt-idx))
                (jnt-list (elt joint-list mt-idx))
                (tc (elt transform-coords mt-idx))
                (ta (elt translation-axis mt-idx))
                (ra (elt rotation-axis mt-idx))
                )
           ;; check whether joint is contained in joint-list of move-target.
           (when (and (derivedp jnt joint) (member jnt jnt-list :test #'equal))
             ;; set jacobian matrix calcualted from jnt, mt, ra, ta to (row-idx, col-idx) block.
             (let* ((paxis (case (jnt . axis)
                                 (:x #f(1 0 0)) (:y #f(0 1 0)) (:z #f(0 0 1))
                                 (:xx #f(1 0 0)) (:yy #f(0 1 0)) (:zz #f(0 0 1))
                                 (:-x #f(-1 0 0)) (:-y #f(0 -1 0)) (:-z #f(0 0 -1))
                                 (t (jnt . axis))))
                    (child-link (send jnt :child-link))
                    (parent-link (send jnt :parent-link))
                    (default-coords (jnt . default-coords))
                    (world-default-coords (send (send parent-link :copy-worldcoords)
                                                :transform default-coords))
                    )
               (send jnt :calc-jacobian
                     jacobi row-idx col-idx jnt paxis child-link world-default-coords
                     nil ;; child-reverse is not supported
                     mt tc ra ta
                     *tmp-v0* *tmp-v1* *tmp-v2* *tmp-v3* *tmp-v3a* *tmp-v3b* *tmp-m33*
                     )
               )
             )
           (incf row-idx (send self :calc-target-axis-dimension ra ta)) ;; update idx
           )))
     jacobi
     ))
  (:calc-cog-jacobian-from-joint-list
   (&key
    (union-joint-list) ;; required
    (update-mass-properties t)
    (translation-axis :z)
    )
   "
\begin{description}
  \item[\it{union-joint-list}] list of all joints considered in jacobian. column num of jacobian is same with length of union-joint-list.
\end{description}

Get CoG jacobian matrix from union-joint-list.
   "
   (let* ((joint-dim
           (calc-target-joint-dimension union-joint-list))
          (inertia-matrix
           (make-matrix (if (equal translation-axis t) 3 2) joint-dim))
          (all-mass
           (* 1e-3 (send self :weight update-mass-properties)))
          )
     (send self :update-mass-properties)
     ;; calc inertia matrix
     (dotimes (joint-idx joint-dim)
       (let* ((jnt (elt union-joint-list joint-idx))
              (child-link (send jnt :child-link))
              )
         (when (find jnt joint-list)
           (send child-link :calc-inertia-matrix-column joint-idx
                 :inertia-matrix inertia-matrix
                 :axis-for-angular (send self :centroid nil)
                 :translation-axis translation-axis
                 :rotation-axis nil
                 :tmp-v0 *tmp-v0*
                 :tmp-v1 *tmp-v1*
                 :tmp-v2 *tmp-v2*
                 :tmp-va *tmp-va*
                 :tmp-vb *tmp-vb*
                 :tmp-vc *tmp-vc*
                 :tmp-vd *tmp-vd*
                 :tmp-ma *tmp-ma*
                 ))
         ))
     ;; calc cog jacobian from inertia matrix
     (dotimes (i (elt (array-dimensions inertia-matrix) 0))
       (dotimes (j (elt (array-dimensions inertia-matrix) 1))
         (setf (aref inertia-matrix i j) (/ (aref inertia-matrix i j) all-mass))))
     inertia-matrix
     ))
  (:find-link-route
   (to &optional from)
   "
Override to support the case that joint does not exist between links.
Change from (send to :parent-link) to (send to :parent).
   "
   (let ((pl (and (derivedp to bodyset-link) (send to :parent-link))))
     (cond
      ;; if to is not included in (send self :links), just trace parent-link
      ((and pl (not (find to (send self :links))))
       (send self :find-link-route pl from))
      ;; if (send self :links), append "to" link
      ((and pl (not (eq to from)))
       (append (send self :find-link-route pl from) (list to)))
      ;; if link-route, just return "from" link
      ((and pl (eq to from))
       (list from))
      )))
  )
