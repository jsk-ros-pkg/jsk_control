;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Wrench and torque distribution using QP
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; wrench + torque distribution functions
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun wrench-distribute-from-total-wrench
  (contact-coords-list contact-constraint-matrix-list
   &key (debug t) (robot)
        (total-wrench (concatenate float-vector (send (car (send robot :links)) :force) (send (car (send robot :links)) :moment)))
        (initial-state ;; calc initial state from pseudo-inverse
         (send robot :wrench-list->wrench-vector
               (send robot :calc-contact-wrenches-from-total-wrench
                     (send-all contact-coords-list :worldpos) :total-wrench total-wrench)))
        (wrench-optimize-weight-vector
         (fill (instantiate float-vector (* 6 (length contact-coords-list))) 1))
        (contact-constraint-vector-list)
;;        (qp-solver #'solve-eiquadprog))
;;        (qp-solver #'solve-octave-qp))
        (qp-solver #'solve-qpoases-qp)
        (check-status t)
        (solve-qp-mode :qp)
        (min-inequality-violation-weight) (min-inequality-violation-weight-vector)
        (equality-error-weight) (equality-error-weight-vector) ;; equality-error-weight is for example 1e8
        )
  "Wrench distribution function."
  (let* (;; inequality
         (inequality-matrix (apply #'concatenate-matrix-diagonal contact-constraint-matrix-list))
         (inequality-min-vector
          (cond
           (contact-constraint-vector-list
            (apply #'concatenate float-vector contact-constraint-vector-list))
           (inequality-matrix
            (instantiate float-vector (array-dimension inequality-matrix 0)))))
         ;; equality
         (equality-matrix (send robot :calc-grasp-matrix (send-all contact-coords-list :worldpos)))
         (equality-vector total-wrench) ;; convert to total ext wrench
         ;; optimize function
         (eval-weight-matrix (diagonal wrench-optimize-weight-vector))
         (eval-coeff-vector (instantiate float-vector (length wrench-optimize-weight-vector)))
         (state-dim (array-dimension inequality-matrix 1))
         (use-min-inequality-violation-p (or min-inequality-violation-weight min-inequality-violation-weight-vector))
         (use-equality-error-p (or equality-error-weight equality-error-weight-vector))
         (state-min-vector))
    (when use-min-inequality-violation-p
      (let* ((ineq-dim (length inequality-min-vector))
             (violation-state-weight-matrix
              (if min-inequality-violation-weight-vector
                  (diagonal min-inequality-violation-weight-vector)
                (scale-matrix min-inequality-violation-weight (unit-matrix ineq-dim)))))
        (setq equality-matrix (concatenate-matrix-row equality-matrix (make-matrix (length equality-vector) ineq-dim))
              inequality-matrix (concatenate-matrix-row inequality-matrix (unit-matrix ineq-dim))
              eval-weight-matrix (concatenate-matrix-diagonal eval-weight-matrix violation-state-weight-matrix)
              eval-coeff-vector (if (float-vector-p eval-coeff-vector)
                                    (concatenate float-vector eval-coeff-vector (instantiate float-vector ineq-dim))
                                  (concatenate-matrix-row eval-coeff-vector (make-matrix 1 ineq-dim (list (instantiate float-vector ineq-dim)))))
              state-min-vector (concatenate float-vector
                                            (fill (instantiate float-vector state-dim) -1e35)
                                            (instantiate float-vector ineq-dim)))))
    ;; Use equality error.
    ;;   For ex, Ax = b => (Ax - b)^T W (Ax - b) in evaluation function
    (when use-equality-error-p
      (let* ((equality-error-weight-matrix
              (if equality-error-weight-vector
                  (diagonal equality-error-weight-vector)
                (scale-matrix equality-error-weight (unit-matrix (array-dimension equality-matrix 0)))))
             (G^TW (m* (transpose equality-matrix) equality-error-weight-matrix)))
        (setq eval-weight-matrix (m+
                                  (m* G^TW equality-matrix)
                                  eval-weight-matrix)
              eval-coeff-vector (v+
                                 (scale -1 (transform G^TW equality-vector))
                                 eval-coeff-vector
                                 ))))
    (let ((ret
           (apply qp-solver
                  :initial-state initial-state
                  :eval-weight-matrix eval-weight-matrix
                  :eval-coeff-vector eval-coeff-vector
                  :inequality-matrix inequality-matrix
                  :inequality-min-vector inequality-min-vector
                  :check-status check-status
                  :solve-qp-mode solve-qp-mode
                  :debug debug
                  (append
                   (if use-min-inequality-violation-p
                       (list :state-min-vector state-min-vector))
                   (unless use-equality-error-p
                     (list :equality-matrix equality-matrix
                           :equality-vector equality-vector)
                     ))
                  )))
      (when (and use-equality-error-p ret (> (distance (transform equality-matrix ret) equality-vector) 1e-4)) ;; [N], [Nm]
        (when debug (warn ";; too large equality error in wrench torque opt ~A~%" (distance (transform equality-matrix ret) equality-vector)))
        (setq ret nil))
      (if ret
          (list :wrench-list (send robot :wrench-vector->wrench-list
                                   (if use-min-inequality-violation-p (subseq ret 0 state-dim) ret))))
      )))

#|
(defun wrench-torque-distribute-from-total-wrench-inertial-torque-old
  (contact-coords-list contact-constraint-matrix-list
   &key (robot)
        (link-list
         (mapcar #'(lambda (x) (send robot :link-list (send x :parent)))
                 contact-coords-list))
        (union-link-list (send robot :calc-union-link-list link-list))
        (total-wrench (concatenate float-vector (send (car (send robot :links)) :force) (send (car (send robot :links)) :moment)))
        (inertial-torque (concatenate float-vector (send-all union-link-list :joint :joint-torque)))
        (wrench-optimize-weight-vector
         (fill (instantiate float-vector (* 6 (length contact-coords-list))) 1))
        (torque-optimize-weight-vector
         (fill (instantiate float-vector (length inertial-torque)) 1))
        (initial-state
         (concatenate float-vector
                      (send robot :wrench-list->wrench-vector
                            (send robot :calc-contact-wrenches-from-total-wrench
                                  (send-all contact-coords-list :worldpos) :total-wrench total-wrench))
                      (instantiate float-vector (length inertial-torque))))
        (debug t)
;;        (qp-solver #'solve-eiquadprog))
;;        (qp-solver #'solve-octave-qp))
        (qp-solver #'solve-qpoases)
        (check-status t))
  "Deprecated wrench+torque distribution function."
  (let* ((contact-matrix (apply #'concatenate-matrix-diagonal contact-constraint-matrix-list))
         (torque-dim (length inertial-torque))
         (wrench-dim (array-dimension contact-matrix 1))
         ;; equality
         (equality-matrix
          (concatenate-matrix-column
           ;; [G 0]
           (concatenate-matrix-row
            (send robot :calc-grasp-matrix (send-all contact-coords-list :worldpos))
            (make-matrix 6 torque-dim))
           ;; [J^T E]
           (concatenate-matrix-row
            (transpose
             (send robot :calc-jacobian-from-link-list
                   link-list
                   :move-target contact-coords-list
                   :rotation-axis (mapcar #'(lambda (x) t) contact-coords-list)
                   :translation-axis (mapcar #'(lambda (x) t) contact-coords-list)
                   :transform-coords (mapcar #'(lambda (x) (make-coords)) contact-coords-list)))
            (unit-matrix torque-dim))))
         (equality-vector (concatenate float-vector total-wrench inertial-torque))
         ;; inequality
         (inequality-matrix
          (concatenate-matrix-column
           ;; [C 0]
           (concatenate-matrix-row contact-matrix (make-matrix (array-dimension contact-matrix 0) torque-dim))
           ;; [0 E]
           (concatenate-matrix-row (make-matrix torque-dim (array-dimension contact-matrix 1)) (unit-matrix torque-dim))
           ;; [0 -E]
           (concatenate-matrix-row (make-matrix torque-dim (array-dimension contact-matrix 1)) (scale-matrix -1 (unit-matrix torque-dim)))))
         (max-torque-vector (concatenate float-vector (send-all union-link-list :joint :max-joint-torque)))
         (inequality-min-vector
          (concatenate float-vector
                       ;; 0
                       (instantiate float-vector (array-dimension contact-matrix 0))
                       ;; tau_min
                       (v- max-torque-vector) ;; min
                       ;; tau_max
                       (v- max-torque-vector)))
         ;; optimize function
         (eval-weight-matrix
          (diagonal
           (concatenate float-vector wrench-optimize-weight-vector torque-optimize-weight-vector)))
         (eval-coeff-vector (instantiate float-vector (length initial-state))))
    (let* ((ret (funcall
                 qp-solver
                 :initial-state initial-state
                 :eval-weight-matrix eval-weight-matrix
                 :eval-coeff-vector eval-coeff-vector
                 :inequality-matrix inequality-matrix
                 :inequality-min-vector inequality-min-vector
                 :equality-matrix equality-matrix
                 :equality-vector equality-vector
                 :check-status check-status
                 :debug debug
                 )))
      (if ret
          (list :wrench-list (send robot :wrench-vector->wrench-list
                                   (subseq ret 0 wrench-dim))
                :torque-vector (subseq ret wrench-dim)))
      )))
|#

(defun wrench-torque-distribute-from-total-wrench-inertial-torque
  (contact-coords-list contact-constraint-matrix-list
   &key (robot)
        (link-list
         (mapcar #'(lambda (x) (send robot :link-list (send x :parent)))
                 contact-coords-list))
        (union-link-list (send robot :calc-union-link-list link-list))
        (total-wrench (concatenate float-vector (send (car (send robot :links)) :force) (send (car (send robot :links)) :moment)))
        (inertial-torque (concatenate float-vector (send-all union-link-list :joint :joint-torque)))
        (wrench-optimize-weight-vector
         (fill (instantiate float-vector (* 6 (length contact-coords-list))) 1))
        (torque-optimize-weight-vector
         (fill (instantiate float-vector (length inertial-torque)) 1))
        (initial-state
         (send robot :wrench-list->wrench-vector
               (send robot :calc-contact-wrenches-from-total-wrench
                     (send-all contact-coords-list :worldpos) :total-wrench total-wrench)))
        (contact-constraint-vector-list)
        (min-inequality-violation-weight) (min-inequality-violation-weight-vector)
        ((:jacobi tmp-jacobi))
        (debug t)
;;        (qp-solver #'solve-eiquadprog))
;;        (qp-solver #'solve-octave-qp))
        (qp-solver #'solve-qpoases-qp)
        (solve-qp-mode :qp)
        (equality-error-weight) (equality-error-weight-vector) ;; equality-error-weight is for example 1e8
        (check-status t))
  "Wrench + torque distribution function."
  (let* ((contact-matrix (apply #'concatenate-matrix-diagonal contact-constraint-matrix-list))
         (torque-dim (length inertial-torque))
         (wrench-dim (array-dimension contact-matrix 1))
         ;; equality
         (equality-matrix
          ;; G
          (send robot :calc-grasp-matrix (send-all contact-coords-list :worldpos)))
         (equality-vector total-wrench)
         (jacobi
          (if tmp-jacobi
              tmp-jacobi
            (send robot :calc-jacobian-from-link-list
                  link-list
                  :move-target contact-coords-list
                  :rotation-axis (mapcar #'(lambda (x) t) contact-coords-list)
                  :translation-axis (mapcar #'(lambda (x) t) contact-coords-list)
                  :transform-coords (mapcar #'(lambda (x) (make-coords)) contact-coords-list))))
         (jacobi^T (transpose jacobi))
         ;; inequality
         (inequality-matrix
          (concatenate-matrix-column
           ;; C
           contact-matrix
           ;; -J^T
           (scale-matrix -1 jacobi^T)
           ;; J^T
           jacobi^T))
         (max-torque-vector (concatenate float-vector (send-all union-link-list :joint :max-joint-torque)))
         (inequality-min-vector
          (concatenate float-vector
                       ;; 0
                       (cond
                        (contact-constraint-vector-list
                         (apply #'concatenate float-vector contact-constraint-vector-list))
                        (contact-matrix
                         (instantiate float-vector (array-dimension contact-matrix 0))))
                       ;; tau_min - \bar{tau}
                       (v- (v- max-torque-vector) inertial-torque) ;; min
                       ;; tau_max + \bar{tau}
                       (v+ (v- max-torque-vector) inertial-torque)))
         ;; optimize function
         (wr-weight-matrix (diagonal wrench-optimize-weight-vector))
         (tq-weight-matrix (diagonal torque-optimize-weight-vector))
         (eval-weight-matrix (m+ (m* (m* jacobi tq-weight-matrix) jacobi^T) wr-weight-matrix))
         (eval-coeff-vector (m* (make-matrix 1 torque-dim (list (scale -1 inertial-torque)))
                                (m* tq-weight-matrix jacobi^T)))
         (state-dim (array-dimension contact-matrix 1))
         (state-min-vector)
         (use-min-inequality-violation-p (or min-inequality-violation-weight min-inequality-violation-weight-vector))
         (use-equality-error-p (or equality-error-weight equality-error-weight-vector)))
    (when use-min-inequality-violation-p
      (let* ((ineq-dim (length inequality-min-vector))
             (violation-state-weight-matrix
              (if min-inequality-violation-weight-vector
                  (diagonal min-inequality-violation-weight-vector)
                (scale-matrix min-inequality-violation-weight (unit-matrix ineq-dim)))))
        (setq equality-matrix (concatenate-matrix-row equality-matrix (make-matrix (length equality-vector) ineq-dim))
              inequality-matrix (concatenate-matrix-row inequality-matrix (unit-matrix ineq-dim))
              eval-weight-matrix (concatenate-matrix-diagonal eval-weight-matrix violation-state-weight-matrix)
              eval-coeff-vector (if (float-vector-p eval-coeff-vector)
                                    (concatenate float-vector eval-coeff-vector (instantiate float-vector ineq-dim))
                                  (concatenate-matrix-row eval-coeff-vector (make-matrix 1 ineq-dim (list (instantiate float-vector ineq-dim)))))
              state-min-vector (concatenate float-vector
                                            (fill (instantiate float-vector state-dim) -1e35)
                                            (instantiate float-vector ineq-dim)))))
    (setq eval-coeff-vector (array-entity eval-coeff-vector)) ;; array->float-vector
    ;; Use equality error.
    ;;   For ex, Ax = b => (Ax - b)^T W (Ax - b) in evaluation function
    (when use-equality-error-p
      (let* ((equality-error-weight-matrix
              (if equality-error-weight-vector
                  (diagonal equality-error-weight-vector)
                (scale-matrix equality-error-weight (unit-matrix (array-dimension equality-matrix 0)))))
             (G^TW (m* (transpose equality-matrix) equality-error-weight-matrix)))
        (setq eval-weight-matrix (m+
                                  (m* G^TW equality-matrix)
                                  eval-weight-matrix)
              eval-coeff-vector (v+
                                 (scale -1 (transform G^TW equality-vector))
                                 eval-coeff-vector
                                 ))))
    (let* ((ret (apply
                 qp-solver
                 :initial-state initial-state
                 :eval-weight-matrix eval-weight-matrix
                 :eval-coeff-vector eval-coeff-vector
                 :inequality-matrix inequality-matrix
                 :inequality-min-vector inequality-min-vector
                 :check-status check-status
                 :solve-qp-mode solve-qp-mode
                 :debug debug
                 (append
                  (if use-min-inequality-violation-p
                      (list :state-min-vector state-min-vector))
                  (unless use-equality-error-p
                    (list :equality-matrix equality-matrix
                          :equality-vector equality-vector)))
                 )))
      (when (and use-equality-error-p ret (> (distance (transform equality-matrix ret) equality-vector) 1e-4)) ;; [N], [Nm]
        (when debug (warn ";; too large equality error in wrench torque opt ~A~%" (distance (transform equality-matrix ret) equality-vector)))
        (setq ret nil))
      (if ret
          (let ((ret-wrench (if use-min-inequality-violation-p (subseq ret 0 state-dim) ret)))
            (list :wrench-list (send robot :wrench-vector->wrench-list ret-wrench)
                  :torque-vector (v- inertial-torque (transform jacobi^T ret-wrench)))))
      )))

(defun wrench-distribute-from-total-wrench-and-given-wrench
  (ungiven-wrench-contact-coords-list
   given-wrench-contact-coords-list
   given-wrench-list
   &key (robot)
        (contact-constraint-matrix-list)
        (contact-constraint-vector-list))
  (wrench-distribute-from-total-wrench
   ungiven-wrench-contact-coords-list
   contact-constraint-matrix-list
   :robot robot
   :total-wrench (v- (concatenate float-vector (send (car (send robot :links)) :force) (send (car (send robot :links)) :moment))
                     (transform (send robot :calc-grasp-matrix (send-all given-wrench-contact-coords-list :worldpos))
                                (apply #'concatenate float-vector given-wrench-list)))
   :contact-constraint-vector-list contact-constraint-vector-list)
  )

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Classes and functions to represent contact constraints
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defclass contact-constraint
  :super propertied-object
  :slots (contact-coords drawing-object
          constraint-param-list
          constraint-matrix constraint-vector
          contact-constraint-list)
  )

(defmethod contact-constraint
  (:init
   (&key (name))
   (send self :name name)
   (when contact-constraint-list
     (setq constraint-param-list
           (list :matrix (apply #'append (mapcar #'(lambda (x) (cadr (memq :matrix x))) (send-all contact-constraint-list :constraint-param-list)))
                 :vector (apply #'append (mapcar #'(lambda (x) (cadr (memq :vector x))) (send-all contact-constraint-list :constraint-param-list))))))
   (let ((mat-list (cadr (memq :matrix constraint-param-list))))
     (setq constraint-matrix
           (make-matrix (length mat-list) (length (car mat-list)) mat-list)))
   (let ((vec-list (cadr (memq :vector constraint-param-list))))
     (setq constraint-vector (concatenate float-vector vec-list)))
   self)
  (:constraint-param-list () constraint-param-list)
  (:gen-drawing-object
   ()
   (setq drawing-object nil)
   )
  (:draw-on
   (&key (flush t))
   (unless drawing-object
     (send self :gen-drawing-object)
     (if drawing-object
         (send drawing-object :newcoords contact-coords)))
   (when drawing-object
     (send drawing-object :worldcoords)
     (send* drawing-object :draw-on :flush flush (send drawing-object :get :draw-on-args)))
   (if contact-constraint-list
       (send-all contact-constraint-list :draw-on :flush flush))
   )
  (:update-contact-coords
   (cc)
   (setq contact-coords (send cc :copy-worldcoords))
   (if drawing-object
       (send drawing-object :newcoords contact-coords))
   (if contact-constraint-list
       (send-all contact-constraint-list :update-contact-coords cc))
   )
  (:calc-constraint-matrix
   (cc)
   (send self :update-contact-coords cc)
   (m*
    constraint-matrix
    (transpose
     (concatenate-matrix-diagonal
      (send contact-coords :worldrot) (send contact-coords :worldrot))))
   )
  (:get-constraint-vector () constraint-vector)
  (:get-constraint-matrix () constraint-matrix)
  )

(defclass 2D-translational-friction-contact-constraint
  :super contact-constraint
  :slots (mu-trans norm-axis)
  )

(defmethod 2D-translational-friction-contact-constraint
  (:init
   (tmp-mu-trans &key ((:norm-axis tmp-norm-axis) :fz))
   "Calc conatraint param list for translational friction.
   mu-trans is friction coefficient.
   norm-axis is axis of normal (such as fz).
   fric-axis is axis of friction force (such as fx or fy)."
   (setq mu-trans tmp-mu-trans norm-axis tmp-norm-axis)
   (setq constraint-param-list
         (let ((ret
                (list (calc-constraint-param-list-for-translational-friction
                       mu-trans norm-axis :fx)
                      (calc-constraint-param-list-for-translational-friction
                       mu-trans norm-axis :fy))))
           (list :matrix (apply #'append (mapcar #'(lambda (x) (cadr (memq :matrix x))) ret))
                 :vector (apply #'append (mapcar #'(lambda (x) (cadr (memq :vector x))) ret)))
           ))
   (send-super :init))
  (:gen-drawing-object
   (&key (z-length 200))
   (let ((b
          (make-cone
           (float-vector 0 0 0)
           (list (float-vector (* -1 mu-trans z-length) (* -1 mu-trans z-length) z-length)
                 (float-vector (* -1 mu-trans z-length) (* 1 mu-trans z-length) z-length)
                 (float-vector (* 1 mu-trans z-length) (* 1 mu-trans z-length) z-length)
                 (float-vector (* 1 mu-trans z-length) (* -1 mu-trans z-length) z-length))
           )))
     (send b :worldcoords)
     (send b :put :draw-on-args (list :color #f(1 1 1) :width 3))
     (setq drawing-object b))
   )
  )

(defclass rotational-friction-contact-constraint
  :super contact-constraint
  :slots (mu-rot norm-axis fric-axis)
  )

(defmethod rotational-friction-contact-constraint
  (:init
   (tmp-mu-rot tmp-norm-axis &key ((:fric-axis tmp-fric-axis)))
   "Calc conatraint param list for translational friction.
   mu-trans is friction coefficient.
   norm-axis is axis of normal (such as fz).
   fric-axis is axis of friction force (such as fx or fy)."
   (setq mu-rot tmp-mu-rot norm-axis tmp-norm-axis fric-axis tmp-fric-axis)
   (setq constraint-param-list
         (calc-constraint-param-list-for-rotational-friction
          mu-rot norm-axis :fric-axis fric-axis))
   (send-super :init))
  )

(defclass 2D-cop-contact-constraint
  :super contact-constraint
  :slots (l-max-1 l-min-1 l-max-2 l-min-2
          force-axis)
  )

(defmethod 2D-cop-contact-constraint
  (:init
   (tmp-l-max-1 tmp-l-min-1 tmp-l-max-2 tmp-l-min-2 ;; [mm]
   &key ((:force-axis tmp-force-axis) :fz)
        (moment-axes (case tmp-force-axis
                       (:fz (list :ny :nx))
                       (:fy (list :nx :nz))
                       (:fx (list :nz :ny)))))
  "Calc two-dimensional rectangular COP constraint.
   l-*-? is all [mm].
   l-max-? and l-min-? are max and min direction for an axis.
   force-axis is axis of normal force (:fz by default).
   moment-axes are axes of moment term ( (list :ny :nx) by default)."
  (setq l-max-1 tmp-l-max-1 l-max-2 tmp-l-max-2
        l-min-1 tmp-l-min-1 l-min-2 tmp-l-min-2
        force-axis tmp-force-axis)
  (setq constraint-param-list
        (calc-constraint-param-list-for-2D-cop
         l-max-1 l-min-1 l-max-2 l-min-2
         :force-axis force-axis
         :moment-axes moment-axes))
  (send-super :init)
  )
  (:gen-drawing-object
   ()
   (setq drawing-object
         (make-prism
          (list
           (float-vector l-min-1 l-min-2 0)
           (float-vector l-min-1 l-max-2 0)
           (float-vector l-max-1 l-max-2 0)
           (float-vector l-max-1 l-min-2 0))
          1))
   (send drawing-object :put :draw-on-args (list :color #f(0 0 1) :width 3))
   )
  )

(defclass polygon-cop-contact-constraint
  :super contact-constraint
  :slots (contact-face contact-face-coords)
  )

(defmethod polygon-cop-contact-constraint
  (:init
   (tmp-contact-face)
  "Calc COP constraint with polygon contact region.
   contact-face is face class instance of contact region
   This class assumes that contact-face is vertical with z-axis."
  (setq contact-face tmp-contact-face)
  (let* ((x-vec (normalize-vector (v- (send (car (send contact-face :edges)) :direction))))
         (z-vec (v- (send contact-face :normal)))
         (y-vec (normalize-vector (v* z-vec x-vec)))
         )
    ;; x is parallel to first edge, z is parallel to face normal
    (setq contact-face-coords (make-coords :pos (cadr (send contact-face :centroid)) :rot (transpose (matrix x-vec y-vec z-vec))))
    )
  (setq constraint-param-list
        (calc-constraint-param-list-for-polygon-cop
         contact-face contact-face-coords))
  (send-super :init)
  )
  (:gen-drawing-object
   ()
   (setq drawing-object
         (make-prism
          (butlast
           (mapcar #'(lambda (p) (concatenate float-vector (subseq p 0 2) (float-vector 0)))
                   (mapcar #'(lambda (p) (send contact-face-coords :inverse-transform-vector p))
                           (send contact-face :vertices))))
          1))
   (send drawing-object :put :draw-on-args (list :color #f(0 0 1) :width 3))
   )
  )

(defclass norm-contact-constraint
  :super contact-constraint
  :slots (norm-axis)
  )

(defmethod norm-contact-constraint
  (:init
   (norm-axis &key (norm 1))
   "Calc constraint param for non-negative contact constraint.
   norm-axis is axis of normal (such as fz).
   norm = 1 is non-negative constraint. norm = -1 is non-positive constraint."
   (setq constraint-param-list
         (calc-constraint-param-list-for-norm
          norm-axis :norm norm))
   (send-super :init))
  )

(defclass min-max-contact-constraint
  :super contact-constraint
  :slots (axis limit-value)
  )

(defmethod min-max-contact-constraint
  (:init
   (v-axis v-limit-value &key (min/max :min))
   "Calc constraint param for min max constraint.
    axis is axis of normal (such as fz).
    min/max is :min => min value limitation. min/max is :max => max value limitation."
   (setq limit-value v-limit-value)
   (setq constraint-param-list
         (calc-constraint-param-list-for-min-max v-axis v-limit-value :min/max min/max))
   (send-super :init))
  )

(defclass 2D-translational-sliding-contact-constraint
  :super contact-constraint
  :slots (mu-trans norm-axis slide-axis)
  )

(defmethod 2D-translational-sliding-contact-constraint
  (:init
   (tmp-mu-trans &key ((:norm-axis tmp-norm-axis) :fz) ((:slide-axis tmp-slide-axis) :x))
   "Calc conatraint param list for translational sliding.
   mu-trans is friction coefficient.
   norm-axis is axis of normal (such as fz).
   slide-axis is axis with sign of sliding direction (such as x, y, -x, or -y).
   This constraint contains friction constraint for the other direction.
   e.g. When slide-axis is :x, this cnstarint consists of sliding constraint of x and friction constraint of y."
   (setq mu-trans tmp-mu-trans norm-axis tmp-norm-axis slide-axis tmp-slide-axis)
   (setq constraint-param-list
         (let ((ret
                (list (calc-constraint-param-list-for-translational-sliding
                       mu-trans norm-axis slide-axis)
                      (calc-constraint-param-list-for-translational-friction
                       mu-trans norm-axis (sliding-axis->no-sliding-axis slide-axis)))))
           (list :matrix (apply #'append (mapcar #'(lambda (x) (cadr (memq :matrix x))) ret))
                 :vector (apply #'append (mapcar #'(lambda (x) (cadr (memq :vector x))) ret)))
           ))
   (send-super :init))
  (:gen-drawing-object
   (&key (z-length 200))
   (setq drawing-object
         (make-prism
          (cond
           ((or (equal slide-axis :x) (equal slide-axis :-x))
            (list
             (float-vector 0 0 0)
             (float-vector (* -1 (axis->sgn slide-axis) mu-trans z-length) (* 1 mu-trans z-length) z-length)
             (float-vector (* -1 (axis->sgn slide-axis) mu-trans z-length) (* -1 mu-trans z-length) z-length))
            )
           ((or (equal slide-axis :y) (equal slide-axis :-y))
            (list
             (float-vector 0 0 0)
             (float-vector (* 1 mu-trans z-length) (* -1 (axis->sgn slide-axis) mu-trans z-length) z-length)
             (float-vector (* -1 mu-trans z-length) (* -1 (axis->sgn slide-axis) mu-trans z-length) z-length))
            ))
          1))
   (send drawing-object :put :draw-on-args (list :color #f(1 1 1) :width 3))
   )
  )

(defclass default-contact-constraint
  :super contact-constraint
  :slots ()
  )

(defmethod default-contact-constraint
  (:init
   (&key (mu-margin-ratio 1.0) (cop-margin-ratio 1.0)
         (mu-trans) (mu-rot) (slide-axis)
         (l-max-x) (l-max-y) (l-min-x) (l-min-y)
         (contact-face)
         (name) (max-fz))
   "Calc default constraint matrix.
    This is include 2D friction force, 1D rotational friction moment, non-negative normal force, and 2D-COP constraint,
    e.g., for foot constraint.
    mu-margin-ratio and cop-margin-ratio are in [0.0, 1.0] and used for reduce contact constraint boudaries.
    If xx-margin-ratio is 1.0, contact constraint boudaries are same as original one.
    If xx-margin-ratio is in (0.0, 1.0), contact constraint boudaries are reduced.
    If xx-margin-ratio is 0.0, no friction or point contact."
   (setq contact-constraint-list
         (append
          (list
           ;; friction
           (if slide-axis
               (instance 2D-translational-sliding-contact-constraint :init (* mu-margin-ratio mu-trans) :slide-axis slide-axis)
             (instance 2D-translational-friction-contact-constraint :init (* mu-margin-ratio mu-trans)))
           (instance rotational-friction-contact-constraint :init (* mu-margin-ratio mu-rot) :fz)
           ;; cop
           (if contact-face
               (instance polygon-cop-contact-constraint :init contact-face)
             (instance 2D-cop-contact-constraint :init (* cop-margin-ratio l-max-x) (* cop-margin-ratio l-min-x) (* cop-margin-ratio l-max-y) (* cop-margin-ratio l-min-y)))
           ;; fz
           (instance norm-contact-constraint :init :fz)
           )
          (if max-fz
              (list (instance min-max-contact-constraint :init :fz max-fz :min/max :max)))))
   (send-super :init :name name)
   )
  )

(defclass default-gripper-contact-constraint
  :super contact-constraint
  )

(defmethod default-gripper-contact-constraint
  (:init
   (&key (mu-trans) (mu-rot)
         (l-min) (l-max) (max-fx) (max-fy) (min-fy) (max-nx) (min-nx)
         (mu-margin-ratio 1.0) (cop-margin-ratio 1.0)
         (name))
   "Calc default gripper constraint matrix.
    Axes are set for default euslisp end-coords criteria, e.g,
     X : Hand forward.
     Y : Palm <-> back of the hand
     Z : Gripper axis.
    We linearlize friction constraints.
    Here we assume |Fx| >= |Fy|.
    For mu-margin-ratio and cop-margin-ratio, please see default-contact-constraint documentation."
   (let ((ret
          (list
           ;; Friction (mu fx >= fz >= -mu fx)
           (calc-constraint-param-list-for-translational-friction
            (* mu-trans mu-margin-ratio) :fx :fz)
           ;; Friction (mu fx >= nz >= -mu fx)
           (calc-constraint-param-list-for-rotational-friction
            (* mu-rot mu-margin-ratio) :fx :fric-axis :nz)
           ;; Cop (l-max fx >= ny >= l-min fx)
           (calc-constraint-param-list-for-1D-cop
            (* l-max cop-margin-ratio) (* l-min cop-margin-ratio)
            :force-axis :fx :moment-axis :ny)
           ;; Fx >= 0
           (calc-constraint-param-list-for-norm :fx)
           ;; Fx max limit
           (calc-constraint-param-list-for-min-max :fx max-fx :min/max :max)
           ;; Fy limit
           (calc-constraint-param-list-for-min-max :fy max-fy :min/max :max)
           (calc-constraint-param-list-for-min-max :fy min-fy :min/max :min)
           ;; Nx limit
           (calc-constraint-param-list-for-min-max :nx max-nx :min/max :max)
           (calc-constraint-param-list-for-min-max :nx min-nx :min/max :min)
           )))
     (setq constraint-param-list
           (list :matrix (apply #'append (mapcar #'(lambda (x) (cadr (memq :matrix x))) ret))
                 :vector (apply #'append (mapcar #'(lambda (x) (cadr (memq :vector x))) ret)))))
     (send-super :init :name name)
   )
  )

(defclass no-contact-constraint
  :super contact-constraint
  )

(defmethod no-contact-constraint
  (:init
   (&key (name))
   "Calc no-contact constraint matrix.
    Constraint is considered as C w = 0 <=> C w >=0 and C w <= 0.
    This can be used for swing foot phase for walking."
   (let ((ret (apply
               #'append
               (mapcar #'(lambda (axis)
                           (list (calc-constraint-param-list-for-min-max axis 0 :min/max :min)
                                 (calc-constraint-param-list-for-min-max axis 0 :min/max :max)))
                       '(:fx :fy :fz :nx :ny :nz)))))
     (setq constraint-param-list
           (list :matrix (apply #'append (mapcar #'(lambda (x) (cadr (memq :matrix x))) ret))
                 :vector (apply #'append (mapcar #'(lambda (x) (cadr (memq :vector x))) ret)))))
   (send-super :init :name name))
  )

(defclass 6d-min-max-contact-constraint
  :super contact-constraint
  :slots (max-wrench)
  )

(defmethod 6d-min-max-contact-constraint
  (:init
   (tmp-max-wrench &key (name))
   "Calc 6D wrench min-max constraint.
    tmp-max-wrench should be 6D vector of max wrench.
    Absolute values of min wrench and max wrench are same.
    This can be used for approximating the grasping contact."
   (setq max-wrench tmp-max-wrench)
   (let ((ret (apply
               #'append
               (mapcar #'(lambda (axis max-value)
                           (list (calc-constraint-param-list-for-min-max axis (- max-value) :min/max :min)
                                 (calc-constraint-param-list-for-min-max axis max-value :min/max :max)))
                       '(:fx :fy :fz :nx :ny :nz) (coerce max-wrench cons)))))
     (setq constraint-param-list
           (list :matrix (apply #'append (mapcar #'(lambda (x) (cadr (memq :matrix x))) ret))
                 :vector (apply #'append (mapcar #'(lambda (x) (cadr (memq :vector x))) ret)))))
   (send-super :init :name name))
  (:gen-drawing-object
   (&key (object-scale 0.1))
   (let ((top (float-vector 0 0 (elt max-wrench 2)))
         (middles (list (float-vector (elt max-wrench 0) 0 0)
                        (float-vector 0 (elt max-wrench 1) 0)
                        (float-vector (- (elt max-wrench 0)) 0 0)
                        (float-vector 0 (- (elt max-wrench 1)) 0)))
         (bottom (float-vector 0 0 (- (elt max-wrench 2))))
         upper-faces lower-faces)
     ;; apply object scale
     (setq top (scale object-scale top))
     (setq middles (mapcar #'(lambda (x) (list (scale object-scale x))) middles)) ;; middle elements should be list for make-face-from-vertices
     (setq bottom (scale object-scale bottom))
     ;; make faces
     (make-face-from-vertices middles)
     (setq upper-faces (geometry::make-conic-side-faces (list top) middles :upper)
           lower-faces (geometry::make-conic-side-faces (list bottom) (reverse middles) :lower))
     ;; make body instance
     (setq drawing-object (send (instantiate *body-class*) :init :faces (flatten (append upper-faces lower-faces)) :approximated t))
     (send drawing-object :put :draw-on-args (list :color #f(1 1 1) :width 2))
     ))
  )

(defun force-axis->index (ax)
  (case ax
    (:fx 0) (:fy 1) (:fz 2)
    (:nx 3) (:ny 4) (:nz 5)
    )
  )

(defun axis->index (ax)
  (case ax
    (:x 0) (:y 1) (:z 2)
    (:-x 0) (:-y 1) (:-z 2)
    )
  )

(defun axis->sgn (ax)
  (case ax
    (:x 1) (:y 1) (:z 1)
    (:-x -1) (:-y -1) (:-z -1)
    )
  )

(defun sliding-axis->no-sliding-axis (ax)
  (case ax
    (:x :fy) (:y :fx)
    (:-x :fy) (:-y :fx)
    )
  )

(defun calc-constraint-param-list-for-translational-friction
  (mu-trans norm-axis fric-axis)
  "Calc conatraint param list for translational friction.
   mu-trans is friction coefficient.
   norm-axis is axis of normal (such as fz).
   fric-axis is axis of friction force (such as fx or fy)."
  (let ((ret (mapcar #'(lambda (x) (make-list 6 :initial-element 0)) '(0 1)))
        (norm-idx (force-axis->index norm-axis))
        (fric-idx (force-axis->index fric-axis)))
    (setf (elt (car ret) norm-idx) mu-trans)
    (setf (elt (cadr ret) norm-idx) mu-trans)
    (setf (elt (car ret) fric-idx) -1)
    (setf (elt (cadr ret) fric-idx) 1)
    (list :matrix ret :vector (list 0 0))
    ))

(defun calc-constraint-param-list-for-rotational-friction
  (mu-rot norm-axis
   &key (fric-axis))
  "Calc conatraint param list for rotational friction.
   mu-rot trans is friction coefficient.
   norm-axis is axis of normal (such as fz).
   fric-axis is axis of friction moment (such as nz). This is same as norm-axis by default."
  (let* ((ret (mapcar #'(lambda (x) (make-list 6 :initial-element 0)) '(0 1)))
         (norm-idx (force-axis->index norm-axis))
         (fric-idx (if fric-axis
                       (force-axis->index fric-axis)
                     (force-axis->index
                      (case norm-axis
                        (:fx :nx) (:fy :ny) (:fz :nz))))))
    (setf (elt (car ret) norm-idx) mu-rot)
    (setf (elt (cadr ret) norm-idx) mu-rot)
    (setf (elt (car ret) fric-idx) -1)
    (setf (elt (cadr ret) fric-idx) 1)
    (list :matrix ret :vector (list 0 0))
    ))

(defun calc-constraint-param-list-for-min-max
  (axis limit-value &key (min/max :min))
  "Calc constraint param for min max constraint.
   axis is axis of normal (such as fz).
   min/max is :min => min value limitation. min/max is :max => max value limitation."
  (let* ((ret (list (make-list 6 :initial-element 0)))
         (axis-idx (force-axis->index axis)))
    (setf (elt (car ret) axis-idx) (case min/max (:min 1) (:max -1)))
    (list :matrix ret :vector (list (case min/max (:min limit-value) (:max (- limit-value)))))
    ))

(defun calc-constraint-param-list-for-norm
  (norm-axis &key (norm 1))
  "Calc constraint param for non-negative contact constraint.
   norm-axis is axis of normal (such as fz).
   norm = 1 is non-negative constraint. norm = -1 is non-positive constraint."
  (calc-constraint-param-list-for-min-max norm-axis 0.0 :min/max (if (= norm 1) :min :max)))

(defun calc-constraint-param-list-for-2D-cop
  (l-max-1 l-min-1 l-max-2 l-min-2 ;; [mm]
   &key (force-axis :fz)
        (moment-axes (case force-axis
                       (:fz (list :ny :nx))
                       (:fy (list :nx :nz))
                       (:fx (list :nz :ny)))))
  "Calc two-dimensional rectangular COP constraint.
   l-*-? is all [mm].
   l-max-? and l-min-? are max and min direction for an axis.
   force-axis is axis of normal force (:fz by default).
   moment-axes are axes of moment term ( (list :ny :nx) by default)."
  (let ((ret
         (mapcar
          #'(lambda (l-max l-min m-ax)
              (calc-constraint-param-list-for-1D-cop
               l-max l-min
               :force-axis force-axis
               :moment-axis m-ax))
          (list l-max-1 l-max-2)
          (list l-min-1 l-min-2)
          moment-axes)))
    (list :matrix (apply #'append (mapcar #'(lambda (x) (cadr (memq :matrix x))) ret))
          :vector (apply #'append (mapcar #'(lambda (x) (cadr (memq :vector x))) ret)))
    ))

(defun calc-constraint-param-list-for-1D-cop
  (l-max l-min ;; [mm]
   &key (force-axis :fz) (moment-axis :ny))
  "Calc onw-dimensional line COP constraint.
   l-* is all [mm].
   l-max and l-min are max and min direction for an axis.
   force-axis is axis of normal force (:fz by default).
   moment-axis are axis of moment term ( :ny by default)."
  (let* ((ret (mapcar #'(lambda (x) (make-list 6 :initial-element 0)) '(0 1)))
         (force-idx (force-axis->index force-axis))
         (moment-idx (force-axis->index moment-axis)))
    (setf (elt (car ret) force-idx) (* 1e-3 l-max))
    (setf (elt (cadr ret) force-idx) (* -1e-3 l-min))
    (case force-axis
      (:fz
       (if (eq moment-axis :ny)
           (progn
             (setf (elt (car ret) moment-idx) 1)
             (setf (elt (cadr ret) moment-idx) -1))
         (progn ;; :nx
           (setf (elt (car ret) moment-idx) -1)
           (setf (elt (cadr ret) moment-idx) 1))))
      (:fy
       (if (eq moment-axis :nx)
           (progn
             (setf (elt (car ret) moment-idx) 1)
             (setf (elt (cadr ret) moment-idx) -1))
         (progn ;; :nz
           (setf (elt (car ret) moment-idx) -1)
           (setf (elt (cadr ret) moment-idx) 1))))
      (:fx
       (if (eq moment-axis :nz)
           (progn
             (setf (elt (car ret) moment-idx) 1)
             (setf (elt (cadr ret) moment-idx) -1))
         (progn ;; :ny
           (setf (elt (car ret) moment-idx) -1)
           (setf (elt (cadr ret) moment-idx) 1)))))
    (list :matrix ret :vector (list 0 0))
    ))

(defun calc-constraint-param-list-for-polygon-cop
  (contact-face contact-face-coords)
  "Calc constraint param for COP constraint with polygon contact region.
   contact-face is face class instance of contact region
   contact-face-coords is coordinate of contact face (x is parallel to first edge, z is parallel to face normal)
   This function assumes that contact-face is vertical with z-axis."
  (let* ((edge-num (length (send contact-face :edges)))
         (ret (mapcar #'(lambda (x) (make-list 6 :initial-element 0))
                      (make-list edge-num))))
    (dotimes (i edge-num)
      (let* ((tmp-edge (elt (send contact-face :edges) i)) ;; be carefull that edge is class name
             (world-edge-dir (send tmp-edge :direction))
             (local-edge-dir (send contact-face-coords :inverse-rotate-vector world-edge-dir))
             (world-edge-point (send tmp-edge :point 0)) ;; [mm]
             (local-edge-point (scale 1e-3 (send contact-face-coords :inverse-transform-vector world-edge-point))) ;; [m]
             )
        (setf (elt (elt ret i) 2) (- (* (elt local-edge-point 0) (elt local-edge-dir 1)) (* (elt local-edge-point 1) (elt local-edge-dir 0)))) ;; fz
        (setf (elt (elt ret i) 3) (elt local-edge-dir 0)) ;; nx
        (setf (elt (elt ret i) 4) (elt local-edge-dir 1)) ;; ny
        (when (< (elt (elt ret i) 2) 0) ;; inequality is flipped depending on edge direction relative to centroid
          (setf (elt ret i) (mapcar #'(lambda (x) (- x)) (elt ret i))))
        ))
    (list :matrix ret :vector (make-list edge-num :initial-element 0))
    ))

(defun calc-constraint-param-list-for-translational-sliding
  (mu-trans norm-axis slide-axis)
  "Calc conatraint param list for translational sliding.
   mu-trans is friction coefficient.
   norm-axis is axis of normal (such as fz).
   slide-axis is axis of sliding direction (such as fx or fy)."
  (let ((ret (mapcar #'(lambda (x) (make-list 6 :initial-element 0)) '(0 1)))
        (norm-idx (force-axis->index norm-axis))
        (slide-idx (axis->index slide-axis))
        (slide-sgn (axis->sgn slide-axis))
        )
    (setf (elt (car ret) norm-idx) mu-trans)
    (setf (elt (cadr ret) norm-idx) (- mu-trans))
    (cond ((> slide-sgn 0)
           (setf (elt (car ret) slide-idx) 1)
           (setf (elt (cadr ret) slide-idx) -1))
          ((< slide-sgn 0)
           (setf (elt (car ret) slide-idx) -1)
           (setf (elt (cadr ret) slide-idx) 1)))
    (list :matrix ret :vector (list 0 0))
    ))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Utilities for old euslisp environment
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(unless (fboundp' concatenate-matrix-column)
(defun concatenate-matrix-column (&rest args)
  "Concatenate matrix in column direction."
  (let (m ml (size-of-column (array-dimension (car args) 1)))
    (dolist (mat args)
      (unless (= (array-dimension mat 1) size-of-column)
        (error ";; concatenate-matrix-column matrix size error (size=~A)~%" (mapcar #'(lambda (x) (array-dimension x 1)) args)))
      (setq m (if mat (length (matrix-column mat 0)) 0))
      (dotimes (i m)
	(push (matrix-row mat i) ml)))
    (when ml (apply #'matrix (reverse ml)))
    )))

(unless (fboundp' concatenate-matrix-row)
(defun concatenate-matrix-row (&rest args)
  "Concatenate matrix in row direction."
  (let (m ml (size-of-column (array-dimension (car args) 0)))
    (dolist (mat args)
      (unless (= (array-dimension mat 0) size-of-column)
        (error ";; concatenate-matrix-row matrix size error (size=~A)~%" (mapcar #'(lambda (x) (array-dimension x 0)) args)))
      (setq m (if mat (length (matrix-row mat 0)) 0))
      (dotimes (i m)
	(push (matrix-column mat i) ml)))
    (when ml (transpose (apply #'matrix (reverse ml))))
    )))

(unless (fboundp' concatenate-matrix-diagonal)
(defun concatenate-matrix-diagonal (&rest args)
  "Concatenate matrix in diagonal."
  (let (mat m ll ml vl)
    (dolist (mm args)
      (push (if mm (length (matrix-row mm 0)) 0) ll))
    (setq ll (reverse ll))
    (dotimes (i (length args))
      (setq mat (nth i args))
      (setq m (if mat (length (matrix-column mat 0)) 0))
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
    )))
