(load "./instant-configuration-task.l")


(defun calc-jacobian-error-with-diff
    (&key
     (config-task)
     (config-method)
     (set-config-method)
     (task-value-method)
     (jacobian-method)
     (delta 1e-3)
     )
  (let* ((config-orig
          (copy-seq (send config-task config-method)))
         (config-temp+)
         (config-temp-)
         (task-value+)
         (task-value-)
         (jacobian)
         (jacobian-with-diff)
         (jacobian-with-diff-column-list)
         (jacobi-mat-error)
         (jacobi-row-vec-error)
         )
    (dotimes (i (length config-orig))
      (setq config-temp+ (copy-seq config-orig))
      (setf (elt config-temp+ i) (+ (elt config-temp+ i) delta))
      (send config-task set-config-method config-temp+)
      (setq task-value+ (send config-task task-value-method))

      (setq config-temp- (copy-seq config-orig))
      (setf (elt config-temp- i) (- (elt config-temp- i) delta))
      (send config-task set-config-method config-temp-)
      (setq task-value- (send config-task task-value-method))

      (push-back (scale (/ (* 2 delta)) (v- task-value+ task-value-))
                 jacobian-with-diff-column-list)
      )

    (setq jacobian (send config-task jacobian-method))
    (setq jacobian-with-diff (transpose (apply #'matrix jacobian-with-diff-column-list)))
    (setq jacobi-mat-error (m- jacobian jacobian-with-diff))
    (setq jacobi-row-vec-error
          (map float-vector #'(lambda (i) (norm (matrix-row jacobi-mat-error i)))
               (range (elt (array-dimensions jacobi-mat-error) 0))))

    (send config-task set-config-method config-orig)
    jacobi-row-vec-error
    ))

(defun validate-element-jacobian
    (&key
     (config-task)
     )
  (apply
   #'warning-message
   2
   "kinematics-task-jacobian-with-theta~%  ~a  (pos: ~a  rot: ~a)~%"
   (let* ((jacobi-diff
           (calc-jacobian-error-with-diff
             :config-task config-task
             :config-method :theta
             :set-config-method :set-theta
             :task-value-method :kinematics-task-value
             :jacobian-method :kinematics-task-jacobian-with-theta
             )
           )
          )
     (list (norm jacobi-diff)
           (norm (apply #'concatenate float-vector
                        (divide-vector jacobi-diff :segment-length 3 :interval-length 3)))
           (norm (apply #'concatenate float-vector
                        (divide-vector (subseq jacobi-diff 3) :segment-length 3 :interval-length 3)))
           )
     ))

  (when (> (send config-task :num-invariant-joint) 0)
    (apply
     #'warning-message
     2
     "kinematics-task-jacobian-with-phi~%  ~a  (pos: ~a  rot: ~a)~%"
     (let* ((jacobi-diff
             (calc-jacobian-error-with-diff
              :config-task config-task
              :config-method :phi
              :set-config-method :set-phi
              :task-value-method :kinematics-task-value
              :jacobian-method :kinematics-task-jacobian-with-phi
              )
             )
            )
       (list (norm jacobi-diff)
             (norm (apply #'concatenate float-vector
                          (divide-vector jacobi-diff :segment-length 3 :interval-length 3)))
             (norm (apply #'concatenate float-vector
                          (divide-vector (subseq jacobi-diff 3) :segment-length 3 :interval-length 3)))
             )
       ))
    )

  (unless (send config-task :only-kinematics?)
    (warning-message
     2
     "eom-trans-task-jacobian-with-wrench~%  ~a~%"
     (norm (calc-jacobian-error-with-diff
            :config-task config-task
            :config-method :wrench
            :set-config-method :set-wrench
            :task-value-method :eom-trans-task-value
            :jacobian-method :eom-trans-task-jacobian-with-wrench
            )))

    (warning-message
     2
     "eom-rot-task-jacobian-with-theta~%  ~a~%"
     (norm (calc-jacobian-error-with-diff
            :config-task config-task
            :config-method :theta
            :set-config-method :set-theta
            :task-value-method :eom-rot-task-value
            :jacobian-method :eom-rot-task-jacobian-with-theta
            )))

    (warning-message
     2
     "eom-rot-task-jacobian-with-wrench~%  ~a~%"
     (norm (calc-jacobian-error-with-diff
            :config-task config-task
            :config-method :wrench
            :set-config-method :set-wrench
            :task-value-method :eom-rot-task-value
            :jacobian-method :eom-rot-task-jacobian-with-wrench
            )))

    (when (> (send config-task :num-invariant-joint) 0)
      (warning-message
       2
       "eom-rot-task-jacobian-with-phi~%  ~a~%"
       (norm (calc-jacobian-error-with-diff
              :config-task config-task
              :config-method :phi
              :set-config-method :set-phi
              :task-value-method :eom-rot-task-value
              :jacobian-method :eom-rot-task-jacobian-with-phi
              )))
      )

    (when (> (send config-task :num-drive-joint) 0)
      (warning-message
       2
       "torque-task-jacobian-with-theta~%  ~a~%"
       (norm (calc-jacobian-error-with-diff
              :config-task config-task
              :config-method :theta
              :set-config-method :set-theta
              :task-value-method :torque-task-value
              :jacobian-method :torque-task-jacobian-with-theta
              )))

      (warning-message
       2
       "torque-task-jacobian-with-wrench~%  ~a~%"
       (norm (calc-jacobian-error-with-diff
              :config-task config-task
              :config-method :wrench
              :set-config-method :set-wrench
              :task-value-method :torque-task-value
              :jacobian-method :torque-task-jacobian-with-wrench
              )))

      (warning-message
       2
       "torque-task-jacobian-with-torque~%  ~a~%"
       (norm (calc-jacobian-error-with-diff
              :config-task config-task
              :config-method :torque
              :set-config-method :set-torque
              :task-value-method :torque-task-value
              :jacobian-method :torque-task-jacobian-with-torque
              )))

      (when (> (send config-task :num-invariant-joint) 0)
        (warning-message
         2
         "torque-task-jacobian-with-phi~%  ~a~%"
         (norm (calc-jacobian-error-with-diff
                :config-task config-task
                :config-method :phi
                :set-config-method :set-phi
                :task-value-method :torque-task-value
                :jacobian-method :torque-task-jacobian-with-phi
                ))))
      )
    )

  (when (> (send config-task :num-posture-joint) 0)
    (warning-message
     2
     "posture-task-jacobian-with-theta~%  ~a~%"
     (norm (calc-jacobian-error-with-diff
            :config-task config-task
            :config-method :theta
            :set-config-method :set-theta
            :task-value-method :posture-task-value
            :jacobian-method :posture-task-jacobian-with-theta
            )))
    )
  )

(defun validate-element-jacobian-manipulation
    (&key
     (config-task)
     )
  (validate-element-jacobian :config-task config-task)

  (warning-message
   2
   "eom-trans-obj-task-jacobian-with-wrench-obj~%  ~a~%"
   (norm (calc-jacobian-error-with-diff
          :config-task config-task
          :config-method :wrench-obj
          :set-config-method :set-wrench-obj
          :task-value-method :eom-trans-obj-task-value
          :jacobian-method :eom-trans-obj-task-jacobian-with-wrench-obj
          )))

  (warning-message
   2
   "eom-rot-obj-task-jacobian-with-theta~%  ~a~%"
   (norm (calc-jacobian-error-with-diff
          :config-task config-task
          :config-method :theta
          :set-config-method :set-theta
          :task-value-method :eom-rot-obj-task-value
          :jacobian-method :eom-rot-obj-task-jacobian-with-theta
          )))

  (warning-message
   2
   "eom-rot-obj-task-jacobian-with-wrench-obj~%  ~a~%"
   (norm (calc-jacobian-error-with-diff
          :config-task config-task
          :config-method :wrench-obj
          :set-config-method :set-wrench-obj
          :task-value-method :eom-rot-obj-task-value
          :jacobian-method :eom-rot-obj-task-jacobian-with-wrench-obj
          )))

  (when (> (send config-task :num-invariant-joint) 0)
    (warning-message
     2
     "eom-rot-obj-task-jacobian-with-phi~%  ~a~%"
     (norm (calc-jacobian-error-with-diff
            :config-task config-task
            :config-method :phi
            :set-config-method :set-phi
            :task-value-method :eom-rot-obj-task-value
            :jacobian-method :eom-rot-obj-task-jacobian-with-phi
            ))))
  )

(defun validate-element-jacobian-bspline
    (&key
     (config-task)
     )
  (apply
   #'warning-message
   2
   "kinematics-task-jacobian-with-control-vector~%  ~a  (pos: ~a  rot: ~a)~%"
   (let* ((jacobi-diff
           (calc-jacobian-error-with-diff
            :config-task config-task
            :config-method :control-vector
            :set-config-method :set-control-vector
            :task-value-method :kinematics-task-value
            :jacobian-method :kinematics-task-jacobian-with-control-vector
            )
           )
          )
     (list (norm jacobi-diff)
           (norm (apply #'concatenate float-vector
                        (divide-vector jacobi-diff :segment-length 3 :interval-length 3)))
           (norm (apply #'concatenate float-vector
                        (divide-vector (subseq jacobi-diff 3) :segment-length 3 :interval-length 3)))
           )
     ))

  (when (> (send config-task :num-timing) 0)
    (apply
     #'warning-message
     2
     "kinematics-task-jacobian-with-timing-vector~%  ~a  (pos: ~a  rot: ~a)~%"
     (let* ((jacobi-diff
             (calc-jacobian-error-with-diff
              :config-task config-task
              :config-method :timing-vector
              :set-config-method :set-timing-vector
              :task-value-method :kinematics-task-value
              :jacobian-method :kinematics-task-jacobian-with-timing-vector
              )
             )
            )
       (list (norm jacobi-diff)
             (norm (apply #'concatenate float-vector
                          (divide-vector jacobi-diff :segment-length 3 :interval-length 3)))
             (norm (apply #'concatenate float-vector
                          (divide-vector (subseq jacobi-diff 3) :segment-length 3 :interval-length 3)))
             )
       ))
    )

  (warning-message
   2
   "stationery-start-finish-task-jacobian-with-control-vector~%  ~a~%"
   (norm
    (calc-jacobian-error-with-diff
     :config-task config-task
     :config-method :control-vector
     :set-config-method :set-control-vector
     :task-value-method :stationery-start-finish-task-value
     :jacobian-method :stationery-start-finish-task-jacobian-with-control-vector
     )))

  (when (> (send config-task :num-timing) 0)
    (warning-message
     2
     "stationery-start-finish-task-jacobian-with-timing-vector~%  ~a~%"
     (norm
      (calc-jacobian-error-with-diff
       :config-task config-task
       :config-method :timing-vector
       :set-config-method :set-timing-vector
       :task-value-method :stationery-start-finish-task-value
       :jacobian-method :stationery-start-finish-task-jacobian-with-timing-vector
       )))
    )
  )

(defun validate-element-jacobian-bspline-dynamic
    (&key
     (config-task)
     )
  (apply
   #'warning-message
   2
   "kinematics-task-jacobian-with-theta-control-vector~%  ~a  (pos: ~a  rot: ~a)~%"
   (let* ((jacobi-diff
           (calc-jacobian-error-with-diff
            :config-task config-task
            :config-method :theta-control-vector
            :set-config-method :set-theta-control-vector
            :task-value-method :kinematics-task-value
            :jacobian-method :kinematics-task-jacobian-with-theta-control-vector
            )
           )
          )
     (list (norm jacobi-diff)
           (norm (apply #'concatenate float-vector
                        (divide-vector jacobi-diff :segment-length 3 :interval-length 3)))
           (norm (apply #'concatenate float-vector
                        (divide-vector (subseq jacobi-diff 3) :segment-length 3 :interval-length 3)))
           )
     ))

  (when (> (send config-task :num-invariant-joint) 0)
    (apply
     #'warning-message
     2
     "kinematics-task-jacobian-with-phi~%  ~a  (pos: ~a  rot: ~a)~%"
     (let* ((jacobi-diff
             (calc-jacobian-error-with-diff
              :config-task config-task
              :config-method :phi
              :set-config-method :set-phi
              :task-value-method :kinematics-task-value
              :jacobian-method :kinematics-task-jacobian-with-phi
              )
             )
            )
       (list (norm jacobi-diff)
             (norm (apply #'concatenate float-vector
                          (divide-vector jacobi-diff :segment-length 3 :interval-length 3)))
             (norm (apply #'concatenate float-vector
                          (divide-vector (subseq jacobi-diff 3) :segment-length 3 :interval-length 3)))
             )
       ))
    )

  (warning-message
   2
   "eom-trans-task-jacobian-with-cog-control-vector~%  ~a~%"
   (norm (calc-jacobian-error-with-diff
          :config-task config-task
          :config-method :cog-control-vector
          :set-config-method :set-cog-control-vector
          :task-value-method :eom-trans-task-value
          :jacobian-method :eom-trans-task-jacobian-with-cog-control-vector
          )))

  (warning-message
   2
   "eom-trans-task-jacobian-with-wrench-control-vector~%  ~a~%"
   (norm (calc-jacobian-error-with-diff
          :config-task config-task
          :config-method :wrench-control-vector
          :set-config-method :set-wrench-control-vector
          :task-value-method :eom-trans-task-value
          :jacobian-method :eom-trans-task-jacobian-with-wrench-control-vector
          )))

  (warning-message
   2
   "eom-rot-task-jacobian-with-theta-control-vector~%  ~a~%"
   (norm (calc-jacobian-error-with-diff
          :config-task config-task
          :config-method :theta-control-vector
          :set-config-method :set-theta-control-vector
          :task-value-method :eom-rot-task-value
          :jacobian-method :eom-rot-task-jacobian-with-theta-control-vector
          )))

  (warning-message
   2
   "eom-rot-task-jacobian-with-cog-control-vector~%  ~a~%"
   (norm (calc-jacobian-error-with-diff
          :config-task config-task
          :config-method :cog-control-vector
          :set-config-method :set-cog-control-vector
          :task-value-method :eom-rot-task-value
          :jacobian-method :eom-rot-task-jacobian-with-cog-control-vector
          )))

  (warning-message
   2
   "eom-rot-task-jacobian-with-ang-moment-control-vector~%  ~a~%"
   (norm (calc-jacobian-error-with-diff
          :config-task config-task
          :config-method :ang-moment-control-vector
          :set-config-method :set-ang-moment-control-vector
          :task-value-method :eom-rot-task-value
          :jacobian-method :eom-rot-task-jacobian-with-ang-moment-control-vector
          )))

  (warning-message
   2
   "eom-rot-task-jacobian-with-wrench-control-vector~%  ~a~%"
   (norm (calc-jacobian-error-with-diff
          :config-task config-task
          :config-method :wrench-control-vector
          :set-config-method :set-wrench-control-vector
          :task-value-method :eom-rot-task-value
          :jacobian-method :eom-rot-task-jacobian-with-wrench-control-vector
          )))

  (when (> (send config-task :num-invariant-joint) 0)
    (warning-message
     2
     "eom-rot-task-jacobian-with-phi~%  ~a~%"
     (norm (calc-jacobian-error-with-diff
            :config-task config-task
            :config-method :phi
            :set-config-method :set-phi
            :task-value-method :eom-rot-task-value
            :jacobian-method :eom-rot-task-jacobian-with-phi
            )))
    )

  (warning-message
   2
   "cog-task-jacobian-with-theta-control-vector~%  ~a~%"
   (norm (calc-jacobian-error-with-diff
          :config-task config-task
          :config-method :theta-control-vector
          :set-config-method :set-theta-control-vector
          :task-value-method :cog-task-value
          :jacobian-method :cog-task-jacobian-with-theta-control-vector
          )))

  (warning-message
   2
   "cog-task-jacobian-with-cog-control-vector~%  ~a~%"
   (norm (calc-jacobian-error-with-diff
          :config-task config-task
          :config-method :cog-control-vector
          :set-config-method :set-cog-control-vector
          :task-value-method :cog-task-value
          :jacobian-method :cog-task-jacobian-with-cog-control-vector
          )))

  (when (> (send config-task :num-invariant-joint) 0)
    (warning-message
     2
     "cog-task-jacobian-with-phi~%  ~a~%"
     (norm (calc-jacobian-error-with-diff
            :config-task config-task
            :config-method :phi
            :set-config-method :set-phi
            :task-value-method :cog-task-value
            :jacobian-method :cog-task-jacobian-with-phi
            )))
    )

  (warning-message
   2
   "ang-moment-task-jacobian-with-ang-moment-control-vector~%  ~a~%"
   (norm (calc-jacobian-error-with-diff
          :config-task config-task
          :config-method :ang-moment-control-vector
          :set-config-method :set-ang-moment-control-vector
          :task-value-method :ang-moment-task-value
          :jacobian-method :ang-moment-task-jacobian-with-ang-moment-control-vector
          )))

  (warning-message
   2
   "posture-task-jacobian-with-theta-control-vector~%  ~a~%"
   (norm (calc-jacobian-error-with-diff
          :config-task config-task
          :config-method :theta-control-vector
          :set-config-method :set-theta-control-vector
          :task-value-method :posture-task-value
          :jacobian-method :posture-task-jacobian-with-theta-control-vector
          )))
  )

(defun validate-jacobian
    (&key
     (config-task)
     )
  (warning-message
   2
   "task-jacobian~%  ~a~%"
   (norm
    (calc-jacobian-error-with-diff
     :config-task config-task
     :config-method :config-vector
     :set-config-method :set-config
     :task-value-method :task-value
     :jacobian-method :task-jacobian
     )))
  )
