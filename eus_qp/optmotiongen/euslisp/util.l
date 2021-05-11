(load "package://eus_qp/euslisp/contact-optimization.l")


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; math
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun concatenate-matrix-column
    (&rest args)
  "
Concatenate matrix in column direction.
Override to support zero row matrix.
  "
  (let (m ml (size-of-column (array-dimension (car args) 1)))
    (dolist (mat args)
      (unless (= (array-dimension mat 1) size-of-column)
        (error ";; concatenate-matrix-column matrix size error (size=~A)~%" (mapcar #'(lambda (x) (array-dimension x 1)) args)))
      (setq m (if mat (array-dimension mat 0) 0))
      (dotimes (i m)
        (push (matrix-row mat i) ml)))
    (cond (ml (apply #'matrix (reverse ml)))
          (t (make-matrix 0 size-of-column)))
    ))

(defun concatenate-matrix-row
    (&rest args)
  "
Concatenate matrix in row direction.
Override to support column row matrix.
  "
  (let (m ml (size-of-column (array-dimension (car args) 0)))
    (dolist (mat args)
      (unless (= (array-dimension mat 0) size-of-column)
        (error ";; concatenate-matrix-row matrix size error (size=~A)~%" (mapcar #'(lambda (x) (array-dimension x 0)) args)))
      (setq m (if mat (array-dimension mat 1) 0))
      (dotimes (i m)
        (push (matrix-column mat i) ml)))
    (cond (ml (transpose (apply #'matrix (reverse ml))))
          (t (make-matrix size-of-column 0)))
    ))

(defun concatenate-matrix-diagonal
  (&rest args)
  "
Concatenate matrix in diagonal.
Override to support zero row matrix.
  "
  (when (every #'(lambda (mm) (= (array-dimension mm 0) 0)) args)
    (return-from concatenate-matrix-diagonal
                 (make-matrix 0 (apply #'+ (mapcar #'(lambda (mm) (array-dimension mm 1)) args)))))
  (let (mat m ll ml vl)
    (dolist (mm args)
      (push (if mm (array-dimension mm 1) 0) ll))
    (setq ll (reverse ll))
    (dotimes (i (length args))
      (setq mat (nth i args))
      (setq m (if mat (array-dimension mat 0) 0))
      (dotimes (j m)
        (setq vl nil)
        (dotimes (k (length ll))
          (if (= i k) (push (matrix-row mat j) vl)
            (push (make-array (nth k ll)
                              :element-type float-vector
                              :initial-element 0) vl)))
        (push (apply #'concatenate
                     (cons float-vector (reverse vl))) ml)))
    (when ml (apply #'matrix (reverse ml)))
    ))

(defun block-matrix
  (m &optional (i 0) (j 0) (ii nil) (jj nil))
  ;; i, j : start row and column idx
  ;; ii, jj : row and column length of block matrix
  (unless ii (setq ii (- (array-dimension m 0) i)))
  (unless jj (setq jj (- (array-dimension m 1) j)))
  (let ((ret (make-matrix ii jj)))
    (dotimes (_i ii)
      (dotimes (_j jj)
        (setf (aref ret _i _j) (aref m (+ _i i) (+ _j j)))))
    ret))

(defun block-vec
  ;; i : start idx
  ;; ii : length of block vector
  (m &optional (i 0) (ii nil))
  (unless ii (setq ii (- (length m) i)))
  (let ((ret (instantiate float-vector ii)))
    (dotimes (_i ii)
      (setf (aref ret _i) (aref m (+ _i i))))
    ret))

(defun copy-block-matrix
  (m bm &optional (i 0) (j 0))
  (dotimes (_i (length (matrix-column bm 0)))
    (dotimes (_j (length (matrix-row bm 0)))
      (setf (aref m (+ _i i) (+ _j j)) (aref bm _i _j))))
  m)

(defun copy-block-vec
  (m bm &optional (i 0))
  (dotimes (_i (length bm))
    (setf (aref m (+ _i i)) (aref bm _i)))
  m)

(defun range (n)
  (cond ((<= n 0)
         (return-from range nil))
        ((= n 1)
         (list 0))
        (t
         (append (range (- n 1)) (list (- n 1))))))

(defun divide-vector
    (vec
     &key
     (segment-length 3)
     (interval-length 0)
     )
  "
  "
  (let* ((ret)
         (i 0)
         )
    (while (< i (length vec))
      (push (subseq vec i (+ i segment-length)) ret)
      (incf i (+ segment-length interval-length))
      )
    (reverse ret)
    ))

(defmacro push-back (el lst)
  `(if (null ,lst)
       (setf ,lst (list ,el))
     (nconc ,lst (list ,el))))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; geometry
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun create-support-polygon
    (robot
     link
     &key
     (normal (float-vector 0 0 -1))
     (thre 5.0)
     )
  (send link :worldcoords)
  (let* ((sole-body-nm
          (read-from-string
           (format nil ":~A-sole-body" (string-left-trim ":" (format nil "~a" (send link :name))))))
         (support-polygon-nm
          (read-from-string
           (format nil ":~A-support-polygon" (string-left-trim ":" (format nil "~a" (send link :name))))))
         (vs (remove-duplicates
              (flatten (send-all (send link :bodies) :vertices))
              :test #'(lambda (x y) (eps-v= x y *epsilon*))))
         (max-vs (find-extream vs #'(lambda (x) (v. x normal)) #'>))
         (support-vs
          (remove-if
           #'(lambda (p) (< thre (- (v. max-vs normal) (v. p normal)))) vs))
         (projected-vs
          (mapcar #'(lambda (p) (v+ p (scale (v. (v- max-vs p) normal) normal))) support-vs))
         (hull-vs
          (quickhull projected-vs normal))
         (b (make-prism hull-vs (scale -1 normal)))
         )
    (send link :assoc b)
    (send robot :put sole-body-nm b)
    (send link :put :sole-body b)
    (send b :worldcoords)
    (let ((f (find-if #'(lambda (x) (memq :bottom (send x :id))) (send b :faces))))
      (send f :name (send link :name))
      (send robot :put support-polygon-nm f)
      (send link :put :support-polygon f)
      f)
    ))

(defun generate-coords-from-face
    (f
     &key
     (flip? nil)
     )
  (let* ((normal (scale (if flip? -1 1) (send f :normal)))
         (x-vec (send (find-extream (send f :edges) #'(lambda (e) (send e :length)) #'>) :direction))
         )
    (make-coords
     :pos
     (elt (send f :centroid) 1)
     :rot
     (transpose (matrix x-vec (v* normal x-vec) normal))
     )
    ))

(defun generate-coords-from-rectangle-face
    (f
     &key
     (flip? nil)
     )
  (make-coords
   :pos
   (elt (send f :centroid) 1)
   :rot
   (transpose
    (matrix
     (send (elt (send f :edges) 0) :direction)
     (send (elt (send f :edges) 1) :direction)
     (scale (if flip? -1 1) (send f :normal))))
   )
  )


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; visualize
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun visualize-trajectory-result
    (ret
     &key
     (robot *robot*)
     (visualize-callback-func)
     )
  (dotimes (i (1- (length ret)))
    (let* ((av-current
            (cadr (assoc :angle-vector (elt ret i))))
           (av-next
            (cadr (assoc :angle-vector (elt ret (1+ i)))))
           (rc-current
            (cadr (assoc :root-coords (elt ret i))))
           (rc-next
            (cadr (assoc :root-coords (elt ret (1+ i)))))
           )
      (dotimes (j 10)
        (send robot :angle-vector (midpoint (/ (float (1+ j)) 10) av-current av-next))
        (send robot :newcoords (midcoords (/ (float (1+ j)) 10) rc-current rc-next))
        (x::window-main-one)
        (send *irtviewer* :draw-objects :flush nil)
        (send *irtviewer* :viewer :flush)
        (unix::usleep (* 40 1000))
        (when visualize-callback-func
          (funcall visualize-callback-func))
        )
      ))
  )

(defmacro with-set-color-line-width
    (vw color line-width &rest bodies)
  (with-gensyms
   (prev-color prev-width)
   `(let* ((,prev-color (send ,vw :viewsurface :color))
           (,prev-width (send ,vw :viewsurface :line-width)))
      (unwind-protect
          (progn
            (send ,vw :viewsurface :color ,color)
            (send ,vw :viewsurface :line-width ,line-width)
            ,@bodies)
        (send ,vw :viewsurface :color ,prev-color)
        (send ,vw :viewsurface :line-width ,prev-width)
        ))))

(defun draw-force-value
    (&key
     (force-list)
     (moment-list)
     (coords-list)
     (robot *robot*)
     (draw-cop? t)
     (force-color (float-vector 1 0.3 1))
     (force-width 5)
     )
  ;; draw contact wrench
  (when draw-cop?
    (mapcar
     #'(lambda (f m cc)
         ;; draw cop
         (when cc
           (let* ((cop (send robot :calc-cop-from-force-moment ;; dummy using of robot
                             f m cc cc)))
             (when cop
               (send cop :draw-on :flush nil :color (float-vector 1 0 0) :width 4 :size 30)
               ))))
     force-list
     moment-list
     coords-list
     ))
  (mapcar
   #'(lambda (f m cc)
       ;; draw force
       (when cc
         (with-set-color-line-width
          *viewer* force-color force-width
          (send *irtviewer* :viewer :draw-arrow
                (send cc :worldpos)
                (v+ (send cc :worldpos) (scale 1 f))
                nil))))
   force-list
   moment-list
   coords-list
   )
  )

(defun draw-torque
    (&key
     (torque-vector)
     (torque-max-vector)
     (joint-list)
     (flush)
     (width 2)
     (size 100)
     (color (float-vector 1 0.3 0))
     (warning-color (float-vector 1 0 0))
     (torque-threshold 1.0)
     )
  (mapcar
   #'(lambda (j tq tq-max)
       (let* ((rtorque
               (/ tq tq-max))
              (tmp-width width)
              (tmp-color color)
              )
         (when (and torque-threshold (> (abs rtorque) torque-threshold))
           (setq tmp-width (* 2 width) tmp-color warning-color))
         (with-set-color-line-width
          *viewer* tmp-color tmp-width
          (send *viewer* :draw-circle
                (orient-coords-to-axis
                 (make-coords :pos (send j :pos))
                 (scale (if (> rtorque 0.0) 1.0 -1.0) (send j :axis-vector)))
                :radius (* size (abs rtorque)) :arrow t :arc (deg2rad 330)))
         ))
   joint-list
   (coerce torque-vector cons)
   (coerce torque-max-vector cons)
   )
  (if flush (send *viewer* :viewsurface :flush))
  )

(defun play-robot-state-list
    (&key
     (rs-list)
     (robot-env)
     (joint-name-list
      (send-all (send robot-env :robot :joint-list) :name))
     (root-link-name
      (send (car (send robot-env :robot :links)) :name))
     (limb-list
      (list :rleg :lleg :rarm :larm))
     )
  (dolist (rs rs-list)
    (send (send robot-env :robot) :move-coords
          (send (cadr (memq :root-coords rs)) :copy-worldcoords)
          (send (send robot-env :robot :link root-link-name) :copy-worldcoords)
          )
    (map nil
         #'(lambda (joint-name angle)
             (send (send robot-env :robot :joint joint-name) :joint-angle angle))
         joint-name-list
         (cadr (memq :angle-vector rs))
         )
    (send *irtviewer* :draw-objects :flush nil)
    (map nil
         #'(lambda (limb state)
             (when (equal state :support)
               (send (send robot-env :robot limb :end-coords)
                     :draw-on :color (float-vector 1 0 0) :size 100 :width 5)
               ))
         limb-list
         (cadr (memq :contact-statet rs))
         )
    (draw-force-value
     :force-list
     (elt (cadr (memq :wrench-list rs)) 0)
     :moment-list
     (elt (cadr (memq :wrench-list rs)) 1)
     :coords-list
     (mapcar #'(lambda (limb) (send robot-env :robot limb :end-coords)) limb-list)
     :robot (send robot-env :robot)
     :draw-cop? nil
     )
    (send (cadr (memq :zmp rs)) :draw-on :flush nil :color (float-vector 1 0 0) :width 5 :size 100)
    (send *irtviewer* :viewer :flush)
    (x::window-main-one)
    )
  )


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; QP
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun solve-qp-with-transformation-until-no-error
  (&rest args)
  "
   try to solve QP until qp_oases internal error is avoided
  "
  (labels ((make-transformation-matrix
            (&key (initial-state) &allow-other-keys)
            (let* ((state-dim (length initial-state))
                   (trans-mat-unit
                    (rotation-matrix (random 2pi) (random-vector)))
                   (trans-mat
                    (apply #'concatenate-matrix-diagonal
                           (append (make-list (/ state-dim 3) :initial-element trans-mat-unit)
                                   (list (unit-matrix (mod state-dim 3))))))
                   )
              trans-mat
              ))
           )
    (let* ((tmp-status)
           (ret)
           (first-trial? t)
           )
      (setq tmp-status (float-vector 0))
      (dotimes (i 10)
        (setq ret
              (apply #'solve-qp-with-transformation
                     :transformation-matrix (if first-trial? nil (apply #'make-transformation-matrix args))
                     (append
                      args
                      (if (find :status args) nil (list :status tmp-status)))
                     ))
        ;; return if status is not internal error, i.e. #f(-1)
        (unless (eps-v= #f(-1) (if (find :status args) (cadr (memq :status args)) tmp-status))
          (return-from nil nil))
        (unless first-trial? ;; do not print message in first trial because frequency of failure is high in first trial
          (warning-message 3 "qp_oases internal error is detected. i = ~a~%" i))
        (when first-trial?
          (setq first-trial? nil))
        )
      ret
      )))

(defun solve-qp-with-transformation
  (&key
   (qp-solver #'solve-qpoases-qp)
   ((:transformation-matrix trans-mat))
   (status (float-vector 0))
   (initial-state)
   (eval-weight-matrix)
   (eval-coeff-vector)
   (equality-matrix)
   (equality-vector)
   (inequality-matrix)
   (inequality-min-vector)
   (solve-qp-mode)
   (debug) ;; not debug? for compatibility
   )
  "
   transform optimization variable with full rank transformation matrix
   this might enable to avoid the internal error of qp_oases
   transed-opt-state = trans-mat * opt-state
  "
  (let* ((transed-eval-weight-matrix
          (if trans-mat (m* (m* (transpose trans-mat) eval-weight-matrix) trans-mat) eval-weight-matrix))
         (transed-eval-coeff-vector
          (if trans-mat (transform eval-coeff-vector trans-mat) eval-coeff-vector))
         (transed-equality-matrix
          (if (and trans-mat equality-matrix)
              (m* equality-matrix trans-mat) equality-matrix))
         (transed-inequality-matrix
          (if (and trans-mat inequality-matrix)
              (m* inequality-matrix trans-mat) inequality-matrix))
         (opt-state)
         (transed-opt-state)
         )
    ;; solve qp
    (setq transed-opt-state
          (funcall qp-solver
                   :status status
                   ;; initial-state is not transformed because calculating inverse of trans-mat has large computation cost
                   :initial-state initial-state
                   :eval-weight-matrix transed-eval-weight-matrix
                   :eval-coeff-vector transed-eval-coeff-vector
                   :equality-matrix transed-equality-matrix
                   :equality-vector equality-vector
                   :inequality-matrix transed-inequality-matrix
                   :inequality-min-vector inequality-min-vector
                   :solve-qp-mode solve-qp-mode
                   :debug debug
                   ))
    ;; inverse transform
    (when transed-opt-state
      (cond (trans-mat
             (setq opt-state (transform trans-mat transed-opt-state))
             )
            (t
             (setq opt-state transed-opt-state)
             )))
    opt-state
    ))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; SQP
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun sqp-convergence-check
    (sqp)
  (let* ((config-task
          (send sqp :config-task))
         (kin-task-value
          (send config-task :kinematics-task-value :update? nil))
         (pos-diff-list
          (divide-vector kin-task-value :segment-length 3 :interval-length 3))
         (rot-diff-list
          (divide-vector (subseq kin-task-value 3) :segment-length 3 :interval-length 3))
         (thre
          (send config-task :get :thre))
         (rthre
          (send config-task :get :rthre))
         (eom-trans-diff
          (send config-task :eom-trans-task-value :update? nil))
         (eom-rot-diff
          (send config-task :eom-rot-task-value :update? nil))
         (torque-diff
          (send config-task :torque-task-value :update? nil))
         (eom-trans-thre
          (send config-task :get :eom-trans-thre))
         (eom-rot-thre
          (send config-task :get :eom-rot-thre))
         (torque-thre
          (send config-task :get :torque-thre))
         )
    (and (every #'(lambda (pos-diff pos-th) (< (* 1e3 (norm pos-diff)) pos-th)) pos-diff-list thre) ;; [mm]
         (every #'(lambda (rot-diff rot-th) (< (rad2deg (norm rot-diff)) rot-th)) rot-diff-list rthre) ;; [deg]
         (< (norm eom-trans-diff) eom-trans-thre) ;; [N]
         (< (norm eom-rot-diff) eom-rot-thre) ;; [Nm]
         (< (norm torque-diff) torque-thre) ;; [Nm] [N]
         )
    ))

(defun sqp-failure-callback
    (sqp)
  (let* ((config-task
          (send sqp :config-task))
         (kin-task-value
          (send config-task :kinematics-task-value :update? nil))
         (pos-diff-list
          (divide-vector kin-task-value :segment-length 3 :interval-length 3))
         (rot-diff-list
          (divide-vector (subseq kin-task-value 3) :segment-length 3 :interval-length 3))
         (thre
          (send config-task :get :thre))
         (rthre
          (send config-task :get :rthre))
         (eom-trans-diff
          (send config-task :eom-trans-task-value :update? nil))
         (eom-rot-diff
          (send config-task :eom-rot-task-value :update? nil))
         (torque-diff
          (send config-task :torque-task-value :update? nil))
         (eom-trans-thre
          (send config-task :get :eom-trans-thre))
         (eom-rot-thre
          (send config-task :get :eom-rot-thre))
         (torque-thre
          (send config-task :get :torque-thre))
         )
    (warning-message 3 "inverse-kinematics-optmotiongen failed.~%")
    (map
     nil
     #'(lambda (i pos-diff rot-diff pos-th rot-th)
         (warning-message
          (if (>= (* 1e3 (norm pos-diff)) pos-th) 1 3)
          "pos[~1d]  ~5,3f / ~5,3f  ~a~%" i (* 1e3 (norm pos-diff)) pos-th pos-diff) ;; [mm]
         (warning-message
          (if (>= (rad2deg (norm rot-diff)) rot-th) 1 3)
          "rot[~1d]  ~5,3f / ~5,3f  ~a~%" i (rad2deg (norm rot-diff)) rot-th rot-diff) ;; [deg]
         )
     (range (length pos-diff-list))
     pos-diff-list
     rot-diff-list
     thre
     rthre
     )
    (warning-message
     (if (>= (norm eom-trans-diff) eom-trans-thre) 1 3)
     "eom-trans  ~5,3f / ~5,3f  ~a~%" (norm eom-trans-diff) eom-trans-thre eom-trans-diff) ;; [N]
    (warning-message
     (if (>= (norm eom-rot-diff) eom-rot-thre) 1 3)
     "eom-rot  ~5,3f / ~5,3f  ~a~%" (norm eom-rot-diff) eom-rot-thre eom-rot-diff) ;; [Nm]
    (warning-message
     (if (>= (norm torque-diff) torque-thre) 1 3)
     "torque  ~5,3f / ~5,3f  ~a~%" (norm torque-diff) torque-thre torque-diff) ;; [Nm] [N]
    ))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; eus_qp/contact-optimization
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod default-contact-constraint
  (:init-from-support-polygon
   (&key
    (mu-trans 0.1)
    (mu-rot 0.001)
    (max-fz 1000)
    ((:support-polygon poly))
    ((:support-polygon-coords poly-coords))
    ((:support-polygon-margin margin) 20) ;; [mm]
    )
   (let* ((vs
           (if poly
               (mapcar
                #'(lambda (v) (send poly-coords :inverse-transform-vector v))
                (send poly :vertices)
                ))
           )
          )
     (send* self :init
            :mu-trans mu-trans
            :mu-rot mu-rot
            :max-fz max-fz
            (list :l-min-x (if poly (min (+ (elt (find-extream vs #'(lambda (v) (elt v 0)) #'<) 0) margin) 0) 0)
                  :l-max-x (if poly (max (- (elt (find-extream vs #'(lambda (v) (elt v 0)) #'>) 0) margin) 0) 0)
                  :l-min-y (if poly (min (+ (elt (find-extream vs #'(lambda (v) (elt v 1)) #'<) 1) margin) 0) 0)
                  :l-max-y (if poly (max (- (elt (find-extream vs #'(lambda (v) (elt v 1)) #'>) 1) margin) 0) 0)
                  ))
     ))
  )
