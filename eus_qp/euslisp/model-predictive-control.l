(load "package://eus_qp/euslisp/contact-optimization.l")
(load "package://eus_qpoases/euslisp/eus-qpoases.l")
;;(load "~/prog/euslib/jsk/gnuplotlib.l")

;; Model Predictive Control for linear system
;;   Linear time-variant system
;;     x_{k+1} = A_k x_k + B_k u_k + g_k
;;     C_k u_k >= D_k
;;     E_k u_k = F_k
;;     y_k = H_k x_k
;;   Total state equation
;;     X = Phi x0 + Psi U + Lambda G
;;     C U >= D
;;     E U = F
;;     Y = H X
;;       X = [x1^T, ..., x_N]^T
;;       U = [u_0^T, ..., u_{N-1}^T]^T
;;       G = [g_0^T, ..., g_{N-1}^T]^T
;;       C = diag(C_0, ..., C_{N-1})
;;       E = diag(E_0, ..., E_{N-1})
;;       D = [D_0, ..., D_{N-1}]
;;       F = [F_0, ..., F_{N-1}]
;;       H = diag(H_0, ..., H_{N-1})
;; Optimization
;;   min_U  (X-X^{ref})^T W1 (X-X^{ref}) + U^T W2 U
;;    (= min_U U^T (Psi^T W1 Psi + W2) U + 2 (Psi^T W1 (Phi x0 - X^{ref} + Lambda G))^T U )
;;   min_U  (X-X^{ref})^T W1 (X-X^{ref}) + U^T W2 U + (Y-Y^{ref})^T W3 (Y-Y^{ref})
;;    (= min_U U^T (Psi^T W1 Psi + W2 + Psi^T H^T W3 H Psi) U +
;;                  2 [ (Psi^T W1 (Phi x0 - X^{ref} + Lambda G))^T + (Psi^T H^T W3 (H Phi x0 - Y^{ref} + H Lambda G))^T ] U )
;;   s.t.
;;     C U >= D
;;     E U = F

(defclass model-predictive-control-param
  :super propertied-object
  :slots (dt ;; Control loop time [s]
          reference-state ;; Reference state X^{ref}
          reference-output ;; Reference output Y^{ref}. If no output, nil
          system-matrix ;; System matrix Ak
          input-matrix ;; Input matrix Bk. If no input, nil
          output-matrix ;; Output matrix Hk. If no output, nil
          feedforward-matrix ;; Feedforward matrix. If no output, nil
          inequality-matrix inequality-min-vector ;; Inequality constraints
          equality-matrix equality-vector ;; Equality constraints
          drift-coeff ;; Drift coefficient g_k. nil by default
          input-dim ;; Input dimension
          input-weight-matrix ;; Input weighing matrix W2_k
          )
  )

(defmethod model-predictive-control-param
  (:init
   (tmp-dt tmp-reference-state
    tmp-system-matrix tmp-input-matrix
    tmp-inequality-matrix tmp-inequality-min-vector
    tmp-equality-matrix tmp-equality-vector
    tmp-input-weight-vector
    &optional (tmp-drift-coeff) (tmp-reference-output) (tmp-output-matrix) (tmp-feedforward-matrix))
   "Parameters at the time k"
   (setq system-matrix tmp-system-matrix input-matrix tmp-input-matrix dt tmp-dt
         output-matrix tmp-output-matrix feedforward-matrix tmp-feedforward-matrix
         inequality-matrix tmp-inequality-matrix
         inequality-min-vector tmp-inequality-min-vector
         equality-matrix tmp-equality-matrix
         equality-vector tmp-equality-vector
         reference-state tmp-reference-state
         reference-output tmp-reference-output
         drift-coeff tmp-drift-coeff
         input-dim (if input-matrix (array-dimension input-matrix 1) 0)
         input-weight-matrix (if input-matrix (diagonal tmp-input-weight-vector)))
   self)
  (:system-matrix () system-matrix)
  (:input-matrix () input-matrix)
  (:output-matrix () output-matrix)
  (:feedforward-matrix () feedforward-matrix)
  (:inequality-matrix () inequality-matrix)
  (:inequality-min-vector () inequality-min-vector)
  (:equality-vector () equality-vector)
  (:equality-matrix () equality-matrix)
  (:reference-state () reference-state)
  (:reference-output () reference-output)
  (:drift-coeff () drift-coeff)
  (:input-dim () input-dim)
  (:input-weight-matrix () input-weight-matrix)
  )

(defclass model-predictive-control
  :super propertied-object
  :slots (preview-window ;; Preview window size
          state-dim ;; Dimension of state
          output-dim ;; Dimension of output
          param-list ;; Parameter list of constraints
          current-state ;; current state (x_0)
          input-value-list ;; Result input value list
          state-value-list ;; Result state value list
          error-weight-vector
          )
  )

(defmethod model-predictive-control
  (:init
   (tmp-preview-window tmp-state-dim
    &key (initial-state (instantiate float-vector tmp-state-dim))
         ((:output-dim tmp-output-dim) 0)
         (error-weight-vector (fill (instantiate float-vector tmp-state-dim) 1.0))
         (output-weight-vector (fill (instantiate float-vector tmp-output-dim) 1.0))
         )
   "Model Predictive Control initialization for linear time-variant system."
   (setq preview-window tmp-preview-window
         state-dim tmp-state-dim output-dim tmp-output-dim)
   (setq current-state initial-state)
   (setq error-weight-matrix (send self :calc-weight-matrix-for-preview-window error-weight-vector preview-window))
   (setq output-weight-matrix (send self :calc-weight-matrix-for-preview-window output-weight-vector preview-window))
   self)
  (:calc-phi-matrix
   (Ak-list) ;; Ak = system matrix
   (let ((prevM))
     (apply
      #'concatenate-matrix-column
      (mapcar
       #'(lambda (Ak)
           (if prevM
               (setq prevM (m* Ak prevM))
             (setq prevM Ak)))
       Ak-list))
     ))
  ;; Ak-list = [A0, A1, ..., AN-1], Ak-list[i] = Ai
  ;; Bk-list = [B0, B1, ..., BN-1], Bk-list[i] = Bi
  ;; (setq Ak-list (list (scale-matrix 1 (unit-matrix 3)) (scale-matrix 2 (unit-matrix 3)) (scale-matrix 3 (unit-matrix 3)) (scale-matrix 4 (unit-matrix 3))))
  ;; (setq Bk-list (list (scale-matrix 0.1 (unit-matrix 3)) (scale-matrix 0.2 (unit-matrix 3)) (scale-matrix 0.3 (unit-matrix 3)) (scale-matrix 0.4 (unit-matrix 3))))
  (:calc-psi-sub-matrix
   (Ak-list Bk-list i j &key (debug nil))
   (let* ((ret (if Bk-list (elt Bk-list j) (unit-matrix state-dim) ))) ;; Use unit matrix instead of Bk if Bk-list is not specified
     (unless ret (return-from :calc-psi-sub-matrix nil))
     (if debug (print ret))
     (mapcar #'(lambda (x)
                 (setq ret (m* x ret))
                 (if debug (print ret)))
             (if (> (length ak-list) (1+ j)) (subseq Ak-list (1+ j) (1+ i))))
     ret))
  (:calc-psi-matrix
   (Ak-list Bk-list)
   (let ((ret))
     (labels ((range
               (len)
               (let ((cnt -1)) (mapcar #'(lambda (x) (incf cnt)) (make-list len)))))
       (dotimes (idx (length Ak-list))
         (push (append
                (mapcar #'(lambda (x)
                            (send self :calc-psi-sub-matrix Ak-list Bk-list idx x))
                        (range (1+ idx)))
                (if (> (- preview-window (1+ idx)) 0)
                    (list
                     (if Bk-list
                         (make-matrix state-dim (reduce #'+ (mapcar #'(lambda (x) (send x :input-dim)) (subseq param-list (1+ idx)))))
                       (make-matrix state-dim (* state-dim (- preview-window (1+ idx))))
                       ))))
               ret))
       (apply #'concatenate-matrix-column (mapcar #'(lambda (x) (apply #'concatenate-matrix-row (remove nil x))) (reverse ret))))))
  (:calc-lambda-matrix
   (Ak-list)
   (send self :calc-psi-matrix Ak-list nil)
   )
  (:calc-weight-matrix-for-preview-window
   (weight-vector preview-window)
   (let* ((dim (length weight-vector))
          (mat (unit-matrix (* dim preview-window))))
     (dotimes (i preview-window)
       (dotimes (j dim)
         (let ((idx (+ (* dim i) j)))
           (setf (aref mat idx idx) (elt weight-vector j))
           )))
     mat))
  (:proc-model-predictive-control
   (&key (calc-state-value-list t))
   "Solve QP for one predictive control calculation."
   ;; Parameter list for all k = 0, ..., N-1
   (let* ((system-matrix-list (mapcar #'(lambda (x) (send x :system-matrix)) param-list))
          (input-matrix-list (mapcar #'(lambda (x) (send x :input-matrix)) param-list))
          (output-matrix-list (mapcar #'(lambda (x) (send x :output-matrix)) param-list))
          (feedforward-matrix-list (mapcar #'(lambda (x) (send x :feedforward-matrix)) param-list))
          (equality-matrix-list (mapcar #'(lambda (x) (send x :equality-matrix)) param-list))
          (equality-vector-list (mapcar #'(lambda (x) (send x :equality-vector)) param-list))
          (inequality-min-vector-list (mapcar #'(lambda (x) (send x :inequality-min-vector)) param-list))
          (inequality-matrix-list (mapcar #'(lambda (x) (send x :inequality-matrix)) param-list))
          (reference-state-list (mapcar #'(lambda (x) (send x :reference-state)) param-list))
          (reference-output-list (mapcar #'(lambda (x) (send x :reference-output)) param-list))
          (input-weight-matrix-list (mapcar #'(lambda (x) (send x :input-weight-matrix)) param-list))
          (drift-coeff-list
           (if (send (car param-list) :drift-coeff)
               (mapcar #'(lambda (x) (send x :drift-coeff)) param-list)))
          (use-drift-coeff (and drift-coeff-list (every #'identity drift-coeff-list)))
          (use-output-matrix (and (send (car param-list) :output-matrix) (send (car param-list) :reference-output)))
          )
     ;; Concatenate all parameter list for one matrix or one vector
     (let* (;; Phi
            (phi-matrix (send self :calc-phi-matrix system-matrix-list))
            ;; Lambda
            (lambda-matrix (if use-drift-coeff (send self :calc-lambda-matrix system-matrix-list)))
            ;; U
            (all-input-value-vector)
            ;; G
            (drift-coeff (if use-drift-coeff (apply #'concatenate float-vector drift-coeff-list)))
            ;; H
            (output-matrix (if use-output-matrix (apply #'concatenate-matrix-diagonal output-matrix-list)))
            ;; I
            (feedforward-matrix (if use-output-matrix (apply #'concatenate-matrix-diagonal feedforward-matrix-list)))
            (no-input-value (every #'null input-matrix-list)) ;; e.g., in the air
            (tmp-input-state-value))
       (unless no-input-value
         (let* (;; Psi
               (psi-matrix (send self :calc-psi-matrix system-matrix-list input-matrix-list))
               ;; X^{ref}
               (reference-state
                (if use-drift-coeff
                    (v- (apply #'concatenate float-vector reference-state-list) (transform lambda-matrix drift-coeff))
                  (apply #'concatenate float-vector reference-state-list)))
               ;; Y^{ref}
               (reference-output
                (if use-output-matrix
                    (if use-drift-coeff
                        (v- (apply #'concatenate float-vector reference-output-list) (transform output-matrix (transform lambda-matrix drift-coeff)))
                      (apply #'concatenate float-vector reference-output-list))))
               ;; C
               (inequality-matrix (if (every #'identity inequality-matrix-list)
                                      (apply #'concatenate-matrix-diagonal inequality-matrix-list)))
               ;; D
               (inequality-min-vector (if (every #'identity inequality-min-vector-list)
                                          (apply #'concatenate float-vector inequality-min-vector-list)))
               ;; E
               (equality-matrix (if (every #'identity equality-matrix-list)
                                    (apply #'concatenate-matrix-diagonal equality-matrix-list)))
               ;; F
               (equality-vector (if (every #'identity equality-vector-list)
                                    (apply #'concatenate float-vector equality-vector-list)))
               ;; W2
               (input-weight-matrix (apply #'concatenate-matrix-diagonal input-weight-matrix-list))
               ;; Psi^T W1
               (projected-error-weight-matrix (m* (transpose psi-matrix) error-weight-matrix))
               ;; Psi x0
               (projected-current-state (transform phi-matrix current-state))
               ;; error component in eval-coeff-vector
               (error-eval-coeff-vector (transform projected-error-weight-matrix (v- projected-current-state reference-state)))
               ;; error component in eval-weight-matrix
               (error-eval-weight-matrix (m* projected-error-weight-matrix psi-matrix))
               ;; (H Psi + I)^T
               (projected-output-weight-matrix
                (if use-output-matrix
                    (m* (transpose (m+ (m* output-matrix psi-matrix) feedforward-matrix)) output-weight-matrix)))
               ;; output component in eval-coeff-vector
               (output-eval-coeff-vector
                (if use-output-matrix
                    (transform projected-output-weight-matrix (v- (transform output-matrix projected-current-state) reference-output))))
               ;; output component in eval-weight-matrix
               (output-eval-weight-matrix
                (if use-output-matrix
                    (m* projected-output-weight-matrix (m+ (m* output-matrix psi-matrix) feedforward-matrix))))
               )
           (setq all-input-value-vector
                 (solve-qpoases-qp
                  :eval-weight-matrix
                  (if use-output-matrix
                      (m+ (m+ error-eval-weight-matrix input-weight-matrix) output-eval-weight-matrix)
                    (m+ error-eval-weight-matrix input-weight-matrix))
                  :eval-coeff-vector
                  (if use-output-matrix
                      (v+ error-eval-coeff-vector output-eval-coeff-vector)
                    error-eval-coeff-vector)
                  :inequality-matrix inequality-matrix
                  :inequality-min-vector inequality-min-vector
                  :equality-matrix equality-matrix
                  :equality-vector equality-vector
                  ;;:print-level :pl-low
                  )
                 tmp-input-state-value (transform psi-matrix all-input-value-vector))))
       ;; Calculate X and x_1,... and u_0, ...
       (when calc-state-value-list
         (let ((state-values (if no-input-value
                                 (transform phi-matrix current-state)
                               (v+ (transform phi-matrix current-state) tmp-input-state-value)))
               (tmp-count 0))
           (if use-drift-coeff
               (setq state-values (v+ state-values (transform lambda-matrix (apply #'concatenate float-vector drift-coeff-list)))))
           (setq input-value-list nil state-value-list nil)
           (dotimes (i preview-window)
             (let ((idim (send (elt param-list i) :input-dim)))
               (push (subseq all-input-value-vector tmp-count (+ tmp-count idim)) input-value-list)
               (push (subseq state-values (* i state-dim) (* (1+ i) state-dim)) state-value-list)
               (setq tmp-count (+ tmp-count idim))
               ))
           (setq input-value-list (reverse input-value-list) state-value-list (reverse state-value-list))
           (setq current-state (car state-value-list))
           ))
       all-input-value-vector
       )))
  (:set-param-list
   (prm-list)
   "Set parameter list."
   (setq param-list prm-list)
   )
  (:append-param
   (prm)
   "Append parameter."
   (setq param-list (append param-list (list prm)))
   (if (> (length param-list) preview-window) (pop param-list))
   (>= (length param-list) preview-window)
   )
  (:input-value-list () "List of input values." input-value-list)
  (:state-value-list () "List of state values." state-value-list)
  )

(defclass MPC-cog-motion-generator-param-base
  :super model-predictive-control-param
  :slots (all-limbs ;; Limbs for support and swing limb
          all-end-coords-list ;; End-coords list for support and swing limb
          contact-limbs ;; Limbs for support limb
          contact-coords-list ;; Contact coords list for support limb
          contact-constraint-list ;; Contact constraint list for support limb
          total-mass ;; Total robot mass [kg]
          )
  )

(defmethod MPC-cog-motion-generator-param-base
  (:init
   (tmp-dt tmp-reference-state
    all-ec-list c-const-l c-limbs a-limbs t-mass
    &key (wrench-dim 6)
         (input-force-weight 1e-4) (input-moment-weight 1e-2)
         (input-weight-vector (let ((ww (instantiate float-vector (* (length c-const-l) wrench-dim))))
                                (dotimes (i (length c-const-l))
                                  (dotimes (j 3) (setf (elt ww (+ (* i wrench-dim) j)) input-force-weight))
                                  (dotimes (j 3) (setf (elt ww (+ (* i wrench-dim) (+ 3 j))) input-moment-weight)))
                                ww))
         ((:reference-output tmp-reference-output)))
   (setq dt tmp-dt total-mass t-mass
         all-end-coords-list all-ec-list
         all-limbs a-limbs
         contact-limbs c-limbs
         contact-constraint-list c-const-l
         contact-coords-list (mapcar #'(lambda (l) (elt all-end-coords-list (position l all-limbs))) contact-limbs))
   (send-super :init
               tmp-dt
               tmp-reference-state
               (send self :calc-system-matrix)
               (send self :calc-input-matrix)
               (send self :calc-contact-constraint-matrix)
               (send self :calc-contact-constraint-vector)
               (send self :calc-equality-matrix)
               (send self :calc-equality-vector)
               input-weight-vector
               (send self :calc-drift-coeff)
               tmp-reference-output
               (send self :calc-output-matrix)
               (send self :calc-feedforward-matrix))
   self)
  (:calc-contact-constraint-matrix
   ()
   (apply #'concatenate-matrix-diagonal
          (mapcar #'(lambda (c-constraint)
                      (send c-constraint :get-constraint-matrix))
                  contact-constraint-list)))
  (:calc-contact-constraint-vector
   ()
   (apply #'concatenate float-vector
          (mapcar #'(lambda (c-constraint)
                      (send c-constraint :get-constraint-vector))
                  contact-constraint-list))
   )
  (:calc-input-matrix
   ()
   (apply
    #'concatenate-matrix-row
    (mapcar
     #'(lambda (ee)
         (send self :calc-input-matrix-for-one-contact-coords ee))
     contact-coords-list)
    ))
  (:all-end-coords-list () "All end coords list." all-end-coords-list)
  (:contact-constraint-list () "Contact constraint list." contact-constraint-list)
  (:calc-wrench-list-from-result-input-value
   (tmp-input-value)
   (let ((force-list) (moment-list))
     (dotimes (i (/ (length tmp-input-value) 6))
       (push (subseq tmp-input-value (* i 6) (+ (* i 6) 3)) force-list)
       (push (subseq tmp-input-value (+ (* i 6) 3) (+ (* i 6) 6)) moment-list))
     (setq force-list (reverse force-list) moment-list (reverse moment-list))
     (list
      (mapcar #'(lambda (l c)
                  (let ((idx (position l contact-limbs)))
                    (send c :rotate-vector
                          (if idx
                              (elt force-list idx)
                            (float-vector 0 0 0)))))
              all-limbs all-end-coords-list)
      (mapcar #'(lambda (l c)
                  (let ((idx (position l contact-limbs)))
                    (send c :rotate-vector
                          (if idx
                              (elt moment-list idx)
                            (float-vector 0 0 0)))))
              all-limbs all-end-coords-list))
     ))
  (:calc-output-matrix () nil)
  (:calc-feedforward-matrix () nil)
  )

(defclass MPC-horizontal-cog-motion-generator-param
  :super MPC-cog-motion-generator-param-base
  :slots (total-fz ;; Total force Z [N]
          cog-z ;; COG z [m]
          )
  )

(defmethod MPC-horizontal-cog-motion-generator-param
  (:init
   (tmp-dt tmp-reference-state
    all-ec-list c-const-l c-limbs t-force tmp-cog-z t-mass
    &key (wrench-dim 6)
         ((:all-limbs a-limbs) '(:rleg :lleg :rarm :larm))
         (input-force-weight 1e-4) (input-moment-weight 1e-2)
         (input-weight-vector (let ((ww (instantiate float-vector (* (length c-const-l) wrench-dim))))
                                (dotimes (i (length c-const-l))
                                  (dotimes (j 3) (setf (elt ww (+ (* i wrench-dim) j)) input-force-weight))
                                  (dotimes (j 3) (setf (elt ww (+ (* i wrench-dim) (+ 3 j))) input-moment-weight)))
                                ww))
         ((:reference-output tmp-reference-output)))
   (setq total-fz t-force
         cog-z tmp-cog-z)
   (send-super :init
               tmp-dt
               tmp-reference-state
               all-ec-list
               c-const-l
               c-limbs
               a-limbs
               t-mass
               :input-weight-vector input-weight-vector
               :reference-output tmp-reference-output)
   self)
  (:calc-fz-mat
   ()
   (apply #'concatenate-matrix-row
          (mapcar #'(lambda (tcc)
                      (concatenate-matrix-row
                       (m* (make-matrix 1 3 (list (list 0 0 1))) (send tcc :worldrot))
                       (make-matrix 1 3)))
                  contact-coords-list))
   )
  (:calc-equality-matrix () (send self :calc-fz-mat))
  (:calc-equality-vector () (float-vector total-fz))
  (:calc-drift-coeff () nil)
  (:calc-system-matrix
   ()
   (let ((dtFz/m (/ (* dt total-fz) total-mass)))
     (make-matrix
      6 6
      (list
       (list 1 dt 0 0 0 0)
       (list 0 1  0 0 0 0)
       (list 0 0  1 dt 0 0)
       (list 0 0  0 1 0 0)
       (list 0 0 (- dtFz/m) 0 1 0)
       (list dtFz/m 0 0 0 0 1)))
     ))
  (:calc-input-matrix-for-one-contact-coords
   (ee)
   (let* ((ee-pos (scale 1e-3 (send ee :worldpos)))
          (beta (* dt (- (elt ee-pos 2) cog-z))))
     (m*
      (make-matrix
       6 6
       (list
       (list (* 0.5 dt dt) 0 0 0 0 0)
       (list dt 0 0 0 0 0)
       (list 0 (* 0.5 dt dt) 0 0 0 0)
       (list 0 dt 0 0 0 0)
       (list 0 (- beta) (* dt (elt ee-pos 1)) dt 0 0)
       (list beta 0 (* -1 dt (elt ee-pos 0)) 0 dt 0)
       ))
      (concatenate-matrix-diagonal
       (send ee :worldrot)
       (send ee :worldrot))
     )))
  (:state->cog
   (astate) ;; [m] => [mm]
   (float-vector (* 1e3 (/ (elt astate 0) total-mass)) (* 1e3 (/ (elt astate 2) total-mass)) (* 1e3 cog-z))
   )
  )

(defclass MPC-vertical-cog-motion-generator-param
  :super MPC-cog-motion-generator-param-base
  :slots ()
  )

(defmethod MPC-vertical-cog-motion-generator-param
  (:init
   (tmp-dt tmp-reference-state
    all-ec-list c-const-l c-limbs t-mass
    &key (wrench-dim 6)
         ((:all-limbs a-limbs) '(:rleg :lleg :rarm :larm))
         (input-force-weight 1e-3) (input-moment-weight 1e-2)
         (input-weight-vector (let ((ww (instantiate float-vector (* (length c-const-l) wrench-dim))))
                                (dotimes (i (length c-const-l))
                                  (dotimes (j 3) (setf (elt ww (+ (* i wrench-dim) j)) input-force-weight))
                                  (dotimes (j 3) (setf (elt ww (+ (* i wrench-dim) (+ 3 j))) input-moment-weight)))
                                ww)))
   (send-super :init
               tmp-dt
               tmp-reference-state
               all-ec-list
               c-const-l
               c-limbs
               a-limbs
               t-mass
               :input-weight-vector input-weight-vector)
   self)
  (:calc-equality-matrix () nil)
  (:calc-equality-vector () nil)
  (:calc-drift-coeff
   ()
   (let ((mg (* 1e-3 total-mass (elt *g-vec* 2))))
     (float-vector (* -1 dt dt 0.5 mg) (* -1 dt mg))))
  (:calc-system-matrix
   ()
   (make-matrix
    2 2
    (list
     (list 1 dt)
     (list 0 1)))
   )
  (:calc-input-matrix-for-one-contact-coords
   (ee)
   (m*
    (make-matrix
     2 6
     (list
      (list 0 0 (* 0.5 dt dt) 0 0 0)
      (list 0 0 dt 0 0 0)))
    (concatenate-matrix-diagonal
     (send ee :worldrot)
     (send ee :worldrot))))
  (:state->cog-z
   (astate) ;; [m] => [mm]
   (* 1e3 (/ (elt astate 0) total-mass))
   )
  )

(defclass MPC-cog-motion-generator-base
  :super model-predictive-control
  :slots ()
  )

(defmethod MPC-cog-motion-generator-base
  (:get-all-end-coords-from-preview-index
   (&optional (preview-index 0))
   "Get all end-coords list from preview-index
    0 by default, e.g., current end-coords list."
   (send (elt param-list preview-index) :all-end-coords-list)
   )
  (:get-contact-constraint-list-from-preview-index
   (&optional (preview-index 0))
   "Get contact-constraint list from preview-index
    0 by default, e.g., current constraint list."
   (send (elt param-list preview-index) :contact-constraint-list)
   )
  (:calc-wrench-list-from-result-input-value
   (&optional (preview-index 0))
   (send (elt param-list preview-index) :calc-wrench-list-from-result-input-value (elt input-value-list preview-index))
   )
  )

(defclass MPC-horizontal-cog-motion-generator
  :super MPC-cog-motion-generator-base
  :slots ()
  )

(defmethod MPC-horizontal-cog-motion-generator
  (:init
   (tmp-preview-window
    &key (error-momentum-integral-weight 1.0e1) (error-momentum-weight 1e-1) (error-angular-momentum-weight 1e3)
         (initial-state (instantiate float-vector state-dim))
         (state-dim 6)
         (error-weight-vector (let ((ww (instantiate float-vector state-dim)))
                                (setf (elt ww 0) error-momentum-integral-weight)
                                (setf (elt ww 1) error-momentum-weight)
                                (setf (elt ww 2) error-momentum-integral-weight)
                                (setf (elt ww 3) error-momentum-weight)
                                (setf (elt ww 4) error-angular-momentum-weight)
                                (setf (elt ww 5) error-angular-momentum-weight)
                                ww))
         (output-dim 0) (output-weight-vector)
         )
   "Model Predictive Control for COG X Y (trans, rot) motion from Ngasaka 2012 paper.
    tmp-preview-window is preview window size."
   (send-super :init
               tmp-preview-window state-dim
               :initial-state initial-state
               :error-weight-vector error-weight-vector
               :output-dim output-dim
               :output-weight-vector output-weight-vector)
   )
  (:get-cog-from-preview-index
   (&optional (preview-index 0))
   "Get COG from preview-index.
    0 by default, e.g., current state."
   (send (elt param-list preview-index) :state->cog
         (elt (send self :state-value-list) preview-index))
   )
  (:get-ref-cog-from-preview-index
   (&optional (preview-index 0))
   "Get Reference COG from preview-index
    0 by default, e.g., current reference state."
   (send (elt param-list preview-index) :state->cog
         (send (elt param-list preview-index) :reference-state))
   )
  )

(defclass MPC-vertical-cog-motion-generator
  :super MPC-cog-motion-generator-base
  :slots ()
  )

(defmethod MPC-vertical-cog-motion-generator
  (:init
   (tmp-preview-window
    &key (error-momentum-integral-weight 1e1) (error-momentum-weight 1e-1)
         (initial-state (instantiate float-vector state-dim))
         (state-dim 2)
         (error-weight-vector (let ((ww (instantiate float-vector state-dim)))
                                (setf (elt ww 0) error-momentum-integral-weight)
                                (setf (elt ww 1) error-momentum-weight)
                                ww)))
   "Model Predictive Control for COG Z motion.
    tmp-preview-window is preview window size."
   (send-super :init
               tmp-preview-window state-dim
               :initial-state initial-state
               :error-weight-vector error-weight-vector)
   )
  (:get-cog-z-from-preview-index
   (&optional (preview-index 0))
   "Get COG Z from preview-index.
    0 by default, e.g., current state."
   (send (elt param-list preview-index) :state->cog-z (elt (send self :state-value-list) preview-index))
   )
  (:get-ref-cog-z-from-preview-index
   (&optional (preview-index 0))
   "Get Reference COG Z from preview-index.
    0 by default, e.g., current state."
   (send (elt param-list preview-index) :state->cog-z (send (elt param-list preview-index) :reference-state))
   )
  )
