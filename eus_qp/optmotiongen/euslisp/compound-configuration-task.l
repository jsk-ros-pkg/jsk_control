;;;;;;;;;;;;;;;;;;;;;;;;;
;; compound-configuration-task
;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass compound-configuration-task
  :super propertied-object
  :slots (;; config-task
          (_config-task-list "list of configuration-task instance")
          ;; buffer
          (_task-jacobi "buffer for $\frac{\partial \bm{e}}{\partial \bm{q}}$")
          (_ineq-mat "buffer for $\bm{C}$")
          (_eq-mat "buffer for $\bm{A}$")
          (_regular-mat "buffer for $\bm{W}_{\mathit{reg}}$")
          )
  :documentation
  "
複数のコンフィギュレーションとタスク関数から構成された複合コンフィギュレーション$\bm{q}$と複合タスク関数$\bm{e}(\bm{q})$のクラス．

コンフィギュレーション$\bm{q}$の取得・更新，タスク関数$\bm{e}(\bm{q})$の取得，タスク関数のヤコビ行列$\frac{\partial \bm{e}(\bm{q})}{\partial \bm{q}}$の取得，コンフィギュレーションの等式・不等式制約$\bm{A}, \bm{b}, \bm{C}, \bm{d}$の取得のためのメソッドが定義されている．
  "
  )

(defmethod compound-configuration-task
  (:init
   (&key
    (config-task-list)
    )
   "
Initialize instance
   "
   (setq _config-task-list config-task-list)
   )
  (:dim-config
   ()
   "
return $\mathit{dim}(\bm{q})$
   "
   (apply #'+ (send-all _config-task-list :dim-config))
   )
  (:config-vector
   ()
   "
return $\bm{q}$
   "
   (apply #'concatenate float-vector (send-all _config-task-list :config-vector))
   )
  (:set-config
   (config-new
    &key
    (relative? nil)
    )
   "
Set $\bm{q}$.
   "
   (dolist (config-task _config-task-list)
     (send config-task :set-config (subseq config-new 0 (send config-task :dim-config))
           :relative? relative?)
     (setq config-new (subseq config-new (send config-task :dim-config)))
     )
   )
  )

(defmethod compound-configuration-task
  (:task-value
   (&key (update? t))
   "
return $\bm{e}(\bm{q})$
   "
   (apply #'concatenate float-vector (send-all _config-task-list :task-value :update? update?))
   )
  (:task-jacobian
   ()
   "
return $\frac{\partial \bm{e}}{\partial \bm{q}}$
   "
   (cond (_task-jacobi
          (fill (array-entity _task-jacobi) 0)
          (let* ((block-mat) (i 0) (j 0))
            (dolist (config-task _config-task-list)
              (setq block-mat (send config-task :task-jacobian))
              (copy-block-matrix _task-jacobi block-mat i j)
              (setq i (+ i (length (matrix-column block-mat 0))))
              (setq j (+ j (length (matrix-row block-mat 0))))
              ))
          )
         (t
          (setq _task-jacobi
                (apply
                 #'concatenate-matrix-diagonal
                 (send-all _config-task-list :task-jacobian)
                 ))
          ))
   _task-jacobi
   )
  )

(defmethod compound-configuration-task
  (:config-inequality-constraint-matrix
   (&key (update? nil) (update-collision? nil))
   "
return $\bm{C}$
   "
   (cond (_ineq-mat
          (fill (array-entity _ineq-mat) 0)
          (let* ((block-mat) (i 0) (j 0))
            (dolist (config-task _config-task-list)
              (setq block-mat
                    (send config-task :config-inequality-constraint-matrix
                          :update? update? :update-collision? update-collision?))
              (copy-block-matrix _ineq-mat block-mat i j)
              (setq i (+ i (length (matrix-column block-mat 0))))
              (setq j (+ j (length (matrix-row block-mat 0))))
              ))
          )
         (t
          (setq _ineq-mat
                (apply
                 #'concatenate-matrix-diagonal
                 (send-all _config-task-list :config-inequality-constraint-matrix
                           :update? update? :update-collision? update-collision?)
                 ))
          ))
   _ineq-mat
   )
  (:config-inequality-constraint-vector
   (&key (update? t) (update-collision? nil))
   "
return $\bm{d}$
   "
   (apply
    #'concatenate
    float-vector
    (send-all _config-task-list :config-inequality-constraint-vector
              :update? update? :update-collision? update-collision?)
    ))
  (:config-equality-constraint-matrix
   (&key (update? nil))
   "
return $\bm{A}$
   "
   (cond (_eq-mat
          (fill (array-entity _eq-mat) 0)
          (let* ((block-mat) (i 0) (j 0))
            (dolist (config-task _config-task-list)
              (setq block-mat
                    (send config-task :config-equality-constraint-matrix
                          :update? update?))
              (copy-block-matrix _eq-mat block-mat i j)
              (setq i (+ i (length (matrix-column block-mat 0))))
              (setq j (+ j (length (matrix-row block-mat 0))))
              ))
          )
         (t
          (setq _eq-mat
                (apply
                 #'concatenate-matrix-diagonal
                 (send-all _config-task-list :config-equality-constraint-matrix
                           :update? update?)
                 ))
          ))
   _eq-mat
   )
  (:config-equality-constraint-vector
   (&key (update? t))
   "
return $\bm{b}$
   "
   (apply
    #'concatenate
    float-vector
    (send-all _config-task-list :config-equality-constraint-vector
              :update? update?)
    ))
  (:update-collision-inequality-constraint
   ()
   "
update inequality matrix $\bm{C}_{\mathit{col},\theta}, \bm{C}_{\mathit{col,\phi}}$ and inequality vector $\bm{d}_{\mathit{col}}$ for collision avoidance
   "
   (dolist (config-task _config-task-list)
     (when (member :update-collision-inequality-constraint (send config-task :methods))
       (send config-task :update-collision-inequality-constraint))
     )
   )
  )

(defmethod compound-configuration-task
  (:regular-matrix
   ()
   "
return $\bm{W}_{\mathit{reg}} \in \mathbb{R}^{\mathit{dim}(\bm{q}) \times \mathit{dim}(\bm{q})}$
   "
   (cond (_regular-mat
          (fill (array-entity _regular-mat) 0)
          (let* ((block-mat) (i 0) (j 0))
            (dolist (config-task _config-task-list)
              (setq block-mat (send config-task :regular-matrix))
              (copy-block-matrix _regular-mat block-mat i j)
              (setq i (+ i (length (matrix-column block-mat 0))))
              (setq j (+ j (length (matrix-row block-mat 0))))
              ))
          )
         (t
          (setq _regular-mat
                (apply
                 #'concatenate-matrix-diagonal
                 (send-all _config-task-list :regular-matrix)
                 ))
          ))
   _regular-mat
   )
  (:regular-vector
   ()
   "
return $\bm{v}_{\mathit{reg}} \in \mathbb{R}^{\mathit{dim}(\bm{q})}$
   "
   (apply
    #'concatenate
    float-vector
    (send-all _config-task-list :regular-vector)
    ))
  )

(defmethod compound-configuration-task
  (:update-viewer
   ()
   "
Update viewer.
   "
   (send-all _config-task-list :update-viewer)
   )
  (:print-status
   ()
   "
Print status.
   "
   (send-all _config-task-list :print-status)
   )
  (:print-setting-information
   ()
   "
Print setting information.
   "
   (dolist (config-task _config-task-list)
     (when (member :print-setting-information (send config-task :methods))
       (send config-task :print-setting-information))
     )
   )
  )
