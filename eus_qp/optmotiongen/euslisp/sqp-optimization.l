(load "./util.l")
(load "package://eus_qpoases/euslisp/eus-qpoases.l")


;;;;;;;;;;;;;;;;;;;;;;;;;
;; sqp-optimization
;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass sqp-optimization
  :super propertied-object
  :slots ((_config-task "instance of configuration-task")
          (_qp-retval "buffer for QP return value")
          (_qp-status "buffer for QP status")
          (_qp-int-status "QP status")
          (_task-value "buffer for task value $\bm{e}(\bm{q})$")
          (_task-jacobian "buffer for task jacobian $\frac{\partial \bm{e}}{\partial \bm{q}}$")
          (_dim-config-buf-matrix "matrix buffer")
          (_convergence-check-func "function to check convergence")
          (_failure-callback-func "callback function of failure")
          (_pre-process-func "pre-process function")
          (_post-process-func "post-process function")
          (_i "buffer for iteration count")
          (_status "status of sqp optimization")
          (_no-visualize? "whether to supress visualization")
          (_no-print? "whether to supress print")
          )
  :documentation
  "
逐次二次計画法のクラス．

instant-configuration-taskクラスやtrajectory-configuration-taskクラスのinstance (以降，configuration-taskと呼ぶ)が与えられた時に，
configuration-taskのタスク関数ノルム二乗$\|\bm{e}(\bm{q})\|^2$を最小にするコンフィギュレーション$\bm{q}$を反復計算により求める．
  "
  )

(defmethod sqp-optimization
  (:init
   (&key
    (config-task)
    (convergence-check-func)
    (failure-callback-func)
    (pre-process-func)
    (post-process-func)
    (no-visualize?)
    (no-print?)
    &allow-other-keys
    )
   "
Initialize instance
   "
   (setq _config-task config-task)
   (setq _dim-config-buf-matrix (make-matrix (send _config-task :dim-config) (send _config-task :dim-config)))
   (setq _convergence-check-func convergence-check-func)
   (setq _failure-callback-func failure-callback-func)
   (setq _pre-process-func pre-process-func)
   (setq _post-process-func post-process-func)
   (setq _no-visualize? no-visualize?)
   (setq _no-print? no-print?)
   )
  (:config-task
   ()
   "
Return configuration-task instance
   "
   _config-task
   )
  (:optimize
   (&key
    (loop-num 100)
    (loop-num-min)
    (update-viewer-interval 1) ;; nil for no visualization
    (print-status-interval 10) ;; nil for no print
    )
   "
Optimize

In each iteration, do following:
\begin{enumerate}
  \setlength{\parskip}{0cm}
  \setlength{\itemsep}{0cm}
  \item check convergence
  \item call pre-process function
  \item print status
  \item solve QP and update configuration
  \item call post-process function
\end{enumerate}

Solve following QP:
\begin{eqnarray}
  &&\min_{\Delta \bm{q}^{(k)}} \ \frac{1}{2} \Delta \bm{q}^{(k)T} \bm{W} \Delta \bm{q}^{(k)} + \bm{v}^T \Delta \bm{q}^{(k)} \\
  &&{\rm s.t. \ \ } \bm{A} \Delta \bm{q}^{(k)} = \bm{b} \\
  &&\phantom{\rm s.t. \ \ } \bm{C} \Delta \bm{q}^{(k)} \geq \bm{d} \\
  &&{\rm where \ \ }
  \bm{W} = \left( \frac{\partial \bm{e}(\bm{q}^{(k)})}{\partial \bm{q}^{(k)}} \right)^T \left( \frac{\partial \bm{e}(\bm{q}^{(k)})}{\partial \bm{q}^{(k)}} \right) + \bm{W}_{\mathit{reg}} \\
  &&\phantom{\rm where \ \ }
  \bm{v} = \left( \frac{\partial \bm{e}(\bm{q}^{(k)})}{\partial \bm{q}^{(k)}} \right)^T \bm{e}(\bm{q}^{(k)}) + \bm{v}_{\mathit{reg}}
\end{eqnarray}
and update configuration:
\begin{eqnarray}
\bm{q}^{(k+1)} = \bm{q}^{(k)} + \Delta \bm{q}^{(k)\ast}
\end{eqnarray}
   "
   (setq _status :optimizing)
   (when (member :print-setting-information (send _config-task :methods))
     (send _config-task :print-setting-information))
   (when loop-num-min
     (setq loop-num-min (min loop-num loop-num-min)))
   (setq _i 0)
   (when update-viewer-interval
     (send self :_update-viewer-once))
   (while t
     (incf _i)
     (when (and (or (not loop-num-min) (>= _i loop-num-min))
                _convergence-check-func
                (funcall _convergence-check-func self))
       (unless _no-print?
         (warning-message 2 "=== Optimization converged with ~a iteration ! ===~%" _i))
       (setq _status :success)
       (send self :_print-status-once :loop-num loop-num)
       (return-from nil nil)
       )
     (when _pre-process-func
       (funcall _pre-process-func self))
     (unless (send self :_optimize-iterate-once)
       (warning-message 2 "=== QP result is invalid. status: ~a ===~%" _qp-int-status)
       (setq _status :qp-error)
       (return-from :optimize _status)
       )
     (when (and print-status-interval
                (or (= (mod _i print-status-interval) 0) (= _i loop-num)))
       (send self :_print-status-once :loop-num loop-num))
     (when (and update-viewer-interval
                (or (= (mod _i update-viewer-interval) 0) (= _i loop-num)))
       (send self :_update-viewer-once))
     (when _post-process-func
       (funcall _post-process-func self))
     (when (>= _i loop-num)
       (return-from nil nil))
     )
   (when (equal _status :optimizing)
     (setq _status (if _convergence-check-func :not-converged :loop-finished))
     (when _failure-callback-func
       (funcall _failure-callback-func self))
     )
   (return-from :optimize _status)
   )
  (:iteration
   ()
   "
Return iteration index.
   "
   _i
   )
  (:status
   ()
   "
Return status of sqp optimization.
   "
   _status
   )
  (:_optimize-iterate-once
   ()
   (send _config-task :update-collision-inequality-constraint)
   (setq _task-value (send _config-task :task-value))
   (setq _task-jacobian (send _config-task :task-jacobian))
   ;; solve QP
   (setq _qp-status (float-vector 0))
   (setq _qp-retval
         (solve-qp-with-transformation-until-no-error
          :status _qp-status
          :initial-state
          (make-array (send _config-task :dim-config) :element-type :float :initial-element 0)
          :eval-weight-matrix
          (m+ (m* (transpose _task-jacobian) _task-jacobian _dim-config-buf-matrix)
              (send _config-task :regular-matrix)
              _dim-config-buf-matrix)
          :eval-coeff-vector
          (v+ (transform (transpose _task-jacobian) _task-value)
              (send _config-task :regular-vector))
          :equality-matrix
          (send _config-task :config-equality-constraint-matrix)
          :equality-vector
          (send _config-task :config-equality-constraint-vector)
          :inequality-matrix
          (send _config-task :config-inequality-constraint-matrix)
          :inequality-min-vector
          (send _config-task :config-inequality-constraint-vector)
          :solve-qp-mode :sqp-with-hotstart
          :debug t
          ))
   (setq _qp-int-status (round (elt _qp-status 0)))
   ;; update config
   (unless (float-vector-p _qp-retval)
     (return-from :_optimize-iterate-once nil))
   (send _config-task :set-config _qp-retval :relative? t)
   t
   )
  (:_update-viewer-once
   ()
   (when _no-visualize?
     (return-from :_update-viewer-once nil))
   (send *irtviewer* :draw-objects :flush nil)
   (send _config-task :update-viewer)
   (send *irtviewer* :viewer :flush)
   (x::window-main-one)
   )
  (:_print-status-once
   (&key
    (loop-num)
    )
   (when _no-print?
     (return-from :_print-status-once nil))
   (warning-message 2 "=== iteration ~a/~a ===~%" _i loop-num)
   (send _config-task :print-status)
   )
  )
