;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Wrench and torque distribution using QP
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; wrench + torque distribution functions
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun wrench-distribute-from-total-wrench
  (contact-coords-list contact-constraint-matrix-list
   &key (debug) (robot)
        (total-wrench (concatenate float-vector (send (car (send robot :links)) :force) (send (car (send robot :links)) :moment)))
        (initial-state ;; calc initial state from pseudo-inverse
         (send robot :wrench-list->wrench-vector
               (send robot :calc-contact-wrenches-from-total-wrench
                     (send-all contact-coords-list :worldpos) :total-wrench total-wrench)))
        (wrench-optimize-weight-vector
         (fill (instantiate float-vector (* 6 (length contact-coords-list))) 1))
;;        (qp-solver #'solve-eiquadprog))
;;        (qp-solver #'solve-octave-qp))
        (qp-solver #'solve-qpoases)
        (check-status t))
  "Wrench distribution function."
  (let* (;; inequality
         (inequality-matrix (apply #'concatenate-matrix-diagonal contact-constraint-matrix-list))
         (inequality-min-vector (if inequality-matrix (instantiate float-vector (array-dimension inequality-matrix 0))))
         ;; equality
         (equality-matrix (send robot :calc-grasp-matrix (send-all contact-coords-list :worldpos)))
         (equality-vector total-wrench) ;; convert to total ext wrench
         ;; optimize function
         (eval-weight-matrix (diagonal wrench-optimize-weight-vector))
         (eval-coeff-vector (instantiate float-vector (length wrench-optimize-weight-vector))))
    (let ((ret
           (funcall qp-solver
                    :initial-state initial-state
                    :eval-weight-matrix eval-weight-matrix
                    :eval-coeff-vector eval-coeff-vector
                    :equality-matrix equality-matrix
                    :equality-vector equality-vector
                    :inequality-matrix inequality-matrix
                    :inequality-min-vector inequality-min-vector
                    :check-status check-status
                    ;;:debug debug
                    )))
      (if ret
          (list :wrench-list (send robot :wrench-vector->wrench-list ret)))
      )))

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
        (debug)
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
                 ;;:debug debug
                 )))
      (if ret
          (list :wrench-list (send robot :wrench-vector->wrench-list
                                   (subseq ret 0 wrench-dim))
                :torque-vector (subseq ret wrench-dim)))
      )))

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
        (debug)
;;        (qp-solver #'solve-eiquadprog))
;;        (qp-solver #'solve-octave-qp))
        (qp-solver #'solve-qpoases)
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
          (send robot :calc-jacobian-from-link-list
                link-list
                :move-target contact-coords-list
                :rotation-axis (mapcar #'(lambda (x) t) contact-coords-list)
                :translation-axis (mapcar #'(lambda (x) t) contact-coords-list)
                :transform-coords (mapcar #'(lambda (x) (make-coords)) contact-coords-list)))
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
                       (instantiate float-vector (array-dimension contact-matrix 0))
                       ;; tau_min - \bar{tau}
                       (v- (v- max-torque-vector) inertial-torque) ;; min
                       ;; tau_max + \bar{tau}
                       (v+ (v- max-torque-vector) inertial-torque)))
         ;; optimize function
         (wr-weight-matrix (diagonal wrench-optimize-weight-vector))
         (tq-weight-matrix (diagonal torque-optimize-weight-vector))
         (eval-weight-matrix (m+ (m* (m* jacobi tq-weight-matrix) jacobi^T) wr-weight-matrix))
         (eval-coeff-vector (m* (make-matrix 1 torque-dim (list (scale -1 inertial-torque)))
                                (m* tq-weight-matrix jacobi^T))))
    (let* ((ret (funcall
                 qp-solver
                 :initial-state initial-state
                 :eval-weight-matrix eval-weight-matrix
                 :eval-coeff-vector (array-entity eval-coeff-vector)
                 :inequality-matrix inequality-matrix
                 :inequality-min-vector inequality-min-vector
                 :equality-matrix equality-matrix
                 :equality-vector equality-vector
                 :check-status check-status
                 ;;:debug debug
                 )))
      (if ret
          (list :wrench-list (send robot :wrench-vector->wrench-list ret)
                :torque-vector (v- inertial-torque (transform jacobi^T ret))))
      )))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Classes and functions to represent contact constraints
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defclass contact-constraint
  :super propertied-object
  :slots (contact-coords drawing-object constraint-param-list contact-constraint-list)
  )

(defmethod contact-constraint
  (:init
   (&key (name))
   (send self :name name)
   (if contact-constraint-list
       (setq constraint-param-list (apply #'append (send-all contact-constraint-list :constraint-param-list))))
   (send self :gen-drawing-object)
   self)
  (:constraint-param-list () constraint-param-list)
  (:gen-drawing-object
   ()
   (setq drawing-object nil)
   )
  (:draw-on
   (&key (flush t))
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
    (make-matrix
     (length constraint-param-list) (length (car constraint-param-list))
     constraint-param-list)
    (transpose
     (concatenate-matrix-diagonal
      (send contact-coords :worldrot) (send contact-coords :worldrot))))
   )
  )

(defclass 2D-translational-contact-constraint
  :super contact-constraint
  :slots (mu-trans norm-axis)
  )

(defmethod 2D-translational-contact-constraint
  (:init
   (tmp-mu-trans &key ((:norm-axis tmp-norm-axis) :fz))
   "Calc conatraint param list for translational friction.
   mu-trans is friction coefficient.
   norm-axis is axis of normal (such as fz).
   fric-axis is axis of friction force (such as fx or fy)."
   (setq mu-trans tmp-mu-trans norm-axis tmp-norm-axis)
   (setq constraint-param-list
         (append
          (calc-constraint-param-list-for-translational-friction
           mu-trans norm-axis :fx)
          (calc-constraint-param-list-for-translational-friction
           mu-trans norm-axis :fy)))
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

(defclass rotational-contact-constraint
  :super contact-constraint
  :slots (mu-rot norm-axis fric-axis)
  )

(defmethod rotational-contact-constraint
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
        (apply #'append
         (mapcar
          #'(lambda (l-max l-min m-ax)
              (calc-constraint-param-list-for-1D-cop
               l-max l-min
               :force-axis force-axis
               :moment-axis m-ax))
          (list l-max-1 l-max-2)
          (list l-min-1 l-min-2)
          moment-axes)))
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

(defclass default-contact-constraint
  :super contact-constraint
  :slots ()
  )

(defmethod default-contact-constraint
  (:init
   (&key (mu-trans) (mu-rot)
         (l-max-x) (l-max-y) (l-min-x) (l-min-y)
         (name))
   "Calc default constraint matrix.
    This is include 2D friction force, 1D rotational friction moment, non-negative normal force, and 2D-COP constraint,
    e.g., for foot constraint."
   (setq contact-constraint-list
         (list
           ;; friction
           (instance 2D-translational-contact-constraint :init mu-trans)
           (instance rotational-contact-constraint :init mu-rot :fz)
           ;; cop
           (instance 2D-cop-contact-constraint :init l-max-x l-min-x l-max-y l-min-y)
           ;; fz
           (instance norm-contact-constraint :init :fz)
           ))
   (send-super :init :name name)
   )
  )

(defun force-axis->index (ax)
  (case ax
    (:fx 0) (:fy 1) (:fz 2)
    (:nx 3) (:ny 4) (:nz 5)
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
    ret))

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
    ret))

(defun calc-constraint-param-list-for-norm
  (norm-axis &key (norm 1))
  "Calc constraint param for non-negative contact constraint.
   norm-axis is axis of normal (such as fz).
   norm = 1 is non-negative constraint. norm = -1 is non-positive constraint."
  (let* ((ret (list (make-list 6 :initial-element 0)))
         (norm-idx (force-axis->index norm-axis)))
    (setf (elt (car ret) norm-idx) norm)
    ret))

(defun calc-constraint-list-for-2D-cop
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
  (apply #'append
         (mapcar
          #'(lambda (l-max l-min m-ax)
              (calc-constraint-param-list-for-1D-cop
               l-max l-min
               :force-axis force-axis
               :moment-axis m-ax))
          (list l-max-1 l-max-2)
          (list l-min-1 l-min-2)
          moment-axes)))

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
    ret))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Utilities
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

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
    ))


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
    ))


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
    ))
