(load "./util.l")
(load "./extended-joint-link.l")
(load "./robot-environment.l")


;;;;;;;;;;;;;;;;;;;;;;;;;
;; bspline basis funciton
;;;;;;;;;;;;;;;;;;;;;;;;;

(defun get-bspline-knot
    (i n m x_min x_max h)
  "
\begin{eqnarray}
  t_i &=& \frac{i - n}{m - n} (t_f - t_s) + t_s \\
  &=& h i + \frac{m t_s - n t_f}{m - n}
\end{eqnarray}

return knot $t_i$ for B-spline function
  "
  (+ (* h i) (/ (- (* m x_min) (* n x_max)) (- m n)))
  )

(defun bspline-basis-func
    (x ;; target time
     i ;; target control point
     n ;; bspline order
     m ;; number of control point
     x_min ;; start time
     x_max ;; finish time
     &optional
     (n-orig n) ;; original bspline order (for recursive call)
     (m-orig m) ;; original number of control point (for recursive call)
     )
  "
\begin{eqnarray}
  b_{i, 0}(t) &=& \left\{ \begin{array}{ll} 1 & {\rm if \ \ } t_i \leq t < t_{i+1}\\ 0 & {\rm otherwise}\end{array}\right. \\
  b_{i, n}(t) &=& \frac{(t - t_i) b_{i, n-1}(t) + (t_{i+n+1} - t) b_{i+1, n-1}(t)}{n h}
\end{eqnarray}

return B-spline function value $b_{i, n}(t)$.
  "
  (let* ((h (/ (- x_max x_min) (- m-orig n-orig)))
         (x_i (get-bspline-knot i n-orig m-orig x_min x_max h))
         (x_i+1 (get-bspline-knot (+ i 1) n-orig m-orig x_min x_max h))
         (x_i+n+1 (get-bspline-knot (+ i n 1) n-orig m-orig x_min x_max h))
         )
    (when (= n 0)
      (cond ((and (<= x_i x) (< x x_i+1))
             (return-from bspline-basis-func 1.0)
             )
            (t
             (return-from bspline-basis-func 0.0)
             ))
      )
    (return-from
     bspline-basis-func
     (/
      (+ (* (- x x_i) (bspline-basis-func x i (- n 1) m x_min x_max n-orig m-orig))
         (* (- x_i+n+1 x) (bspline-basis-func x (+ i 1) (- n 1) m x_min x_max n-orig m-orig)))
      (* n h))
     )
    ))


;;;;;;;;;;;;;;;;;;;;;;;;;
;; bspline-configuration-task
;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass bspline-configuration-task
  :super propertied-object
  :slots ((_robot "robot instance")
          ;; configuration vector
          (_control-vector "$\bm{p}$")
          (_timing-vector "$\bm{t}$")
          ;; number
          (_num-kin "$N_{\mathit{kin}} := |\mathcal{T}^{\mathit{kin\mathchar`-trg}}| = |\mathcal{T}^{\mathit{kin\mathchar`-att}}|$")
          (_num-joint "$N_{\mathit{joint}} := |\mathcal{J}|$")
          (_num-control-point "$N_{\mathit{ctrl}}$")
          (_num-timing "$N_{\mathit{tm}}$")
          (_bspline-order "B-spline order, $n$")
          ;; dimensions
          (_dim-control-vector "$\mathit{dim}(\bm{p})$")
          (_dim-timing-vector "$\mathit{dim}(\bm{t})$")
          (_dim-config "$\mathit{dim}(\bm{q})$")
          (_dim-task "$\mathit{dim}(\bm{e})$")
          (_num-collision "$N_{\mathit{col}} :=$ number of collision check pairs")
          ;; scale
          (_stationery-start-finish-task-scale "$k_{\mathit{stat}}$")
          (_first-diff-square-integration-regular-scale "$k_{\mathit{sqr,1}}$")
          (_second-diff-square-integration-regular-scale "$k_{\mathit{sqr,2}}$")
          (_third-diff-square-integration-regular-scale "$k_{\mathit{sqr,3}}$")
          (_motion-duration-regular-scale "$k_{\mathit{duration}}$")
          (_norm-regular-scale-max "$k_{\mathit{max,p}}$")
          (_norm-regular-scale-offset "$k_{\mathit{off,p}}$")
          (_timing-norm-regular-scale-max "$k_{\mathit{max,t}}$")
          (_timing-norm-regular-scale-offset "$k_{\mathit{off,t}}$")
          ;; variables
          (_joint-list "$\mathcal{J}$")
          (_start-time "$t_s$")
          (_finish-time "$t_f$")
          (_kin-time-list "$\{ t_1^{\mathit{kin\mathchar`-tm}}, t_2^{\mathit{kin\mathchar`-tm}}, \cdots, t_{N_{\mathit{kin}}}^{\mathit{kin\mathchar`-tm}} \}$")
          (_kin-variable-timing-list "list of bool. t for variable timing.")
          (_kin-target-coords-list "$\mathcal{T}^{\mathit{kin\mathchar`-trg}}$")
          (_kin-attention-coords-list "$\mathcal{T}^{\mathit{kin\mathchar`-att}}$")
          (_kin-pos-tolerance-list "list of position tolerance $e_{\mathit{tol,pos}}$ [m]") ;; [m]
          (_kin-rot-tolerance-list "list of rotation tolerance $e_{\mathit{tol,rot}}$ [rad]") ;; [rad]
          (_joint-angle-margin "margin of $\bm{\theta}$ [deg] [mm]")
          (_collision-pair-list "list of bodyset-link or body pair")
          (_keep-timing-order? "whether to keep order of timing $\bm{t}$ or not")
          ;; buffer
          (_bspline-matrix "buffer for $\bm{B}_n(t)$")
          (_diff-mat "buffer for $\bm{D}^k$")
          (_diff-mat-list "buffer for $\{ \bm{D}^1, \bm{D}^2, \cdots, \bm{D}^K \}$")
          (_extended-diff-mat-list "buffer for $\{ \bm{\hat{D}}_1, \bm{\hat{D}}_2, \cdots, \bm{\hat{D}}_K \}$")
          (_task-jacobi "buffer for $\frac{\partial \bm{e}}{\partial \bm{q}}$")
          (_regular-mat "buffer for $\bm{W}_{\mathit{reg}}$")
          (_regular-vec "buffer for $\bm{v}_{\mathit{reg}}$")
          )
  :documentation
  "
Bスプラインを利用した軌道生成のためのコンフィギュレーション$\bm{q}$とタスク関数$\bm{e}(\bm{q})$のクラス．

コンフィギュレーション$\bm{q}$の取得・更新，タスク関数$\bm{e}(\bm{q})$の取得，タスク関数のヤコビ行列$\frac{\partial \bm{e}(\bm{q})}{\partial \bm{q}}$の取得，コンフィギュレーションの等式・不等式制約$\bm{A}, \bm{b}, \bm{C}, \bm{d}$の取得のためのメソッドが定義されている．


コンフィギュレーション・タスク関数を定めるために，初期化時に以下を与える
\begin{itemize}
\setlength{\itemindent}{10mm}
\setlength{\leftskip}{-10mm}
\item ロボット
\begin{description}
  \item[robot] ロボットのインスタンス
  \item[joint-list $\mathcal{J}$] 関節
\end{description}
\item Bスプラインのパラメータ
\begin{description}
  \item[start-time $t_s$] Bスプラインの使用区間の初期時刻
  \item[finish-time $t_f$] Bスプラインの使用区間の終端時刻
  \item[num-control-point $N_{\mathit{ctrl}}$] 制御点の個数
  \item[bspline-order $n$] Bスプラインの次数
\end{description}
\item 幾何拘束
\begin{description}
  \item[kin-target-coords-list $\mathcal{T}^{\mathit{kin\mathchar`-trg}}$] 幾何到達目標位置姿勢リスト
  \item[kin-attention-coords-list $\mathcal{T}^{\mathit{kin\mathchar`-att}}$] 幾何到達着目位置姿勢リスト
  \item[kin-time-list $\{ t_1^{\mathit{kin\mathchar`-tm}}, t_2^{\mathit{kin\mathchar`-tm}}, \cdots, t_{N_{\mathit{kin}}}^{\mathit{kin\mathchar`-tm}} \}$] 幾何到達タイミングリスト
  \item[kin-variable-timing-list] 幾何到達タイミングが可変か(t)，固定か(nil)のリスト．このリスト内のtの個数がタイミング$\bm{t}$の次元$N_{\mathit{tm}}$となる．
\end{description}
\end{itemize}

コンフィギュレーション$\bm{q}$は以下から構成される．
\begin{eqnarray}
  \bm{q} := \begin{pmatrix} \bm{p} \\ \bm{t} \end{pmatrix}
\end{eqnarray}
\begin{description}
  \setlength{\itemindent}{10mm}
  \item[$\bm{p} \in \mathbb{R}^{N_{\mathit{ctrl}} N_{\mathit{joint}}}$] 制御点 (Bスプライン基底関数の山の高さ) [rad] [m]
  \item[$\bm{t} \in \mathbb{R}^{N_{\mathit{tm}}}$] タイミング(幾何拘束タスクの課される時刻) [sec]
\end{description}

タスク関数$\bm{e}(\bm{q})$は以下から構成される．
\begin{eqnarray}
  \bm{e}(\bm{q}) := \begin{pmatrix} \bm{e}^{\mathit{kin}}(\bm{q}) \\ \bm{e}^{\mathit{stat}}(\bm{q}) \end{pmatrix} \in \mathbb{R}^{6 N_{\mathit{kin}} + 4 N_{\mathit{joint}}}
\end{eqnarray}
\begin{description}
  \setlength{\itemindent}{10mm}
  \item[$\bm{e}^{\mathit{kin}}(\bm{q}) \in \mathbb{R}^{6 N_{\mathit{kin}}}$] 幾何到達拘束 [rad] [m]
  \item[$\bm{e}^{\mathit{stat}}(\bm{q}) \in \mathbb{R}^{4 N_{\mathit{joint}}}$] 初期，終端時刻静止拘束 ${\rm [rad] [rad/s] [rad/s^2] [m] [m/s] [m/s^2]}$
\end{description}
  "
  )

(defmethod bspline-configuration-task
  (:init
   (&key
    (name)
    (robot)
    (joint-list (send robot :joint-list))
    (start-time 0.0) ;; [sec]
    (finish-time 10.0) ;; [sec]
    (num-control-point 10)
    (bspline-order 3)
    (kin-time-list)
    (kin-variable-timing-list
     (make-list (length kin-time-list) :initial-element nil))
    (kin-target-coords-list)
    (kin-attention-coords-list)
    (kin-pos-tolerance-list
     (make-list (length kin-time-list) :initial-element 0.0))
    (kin-rot-tolerance-list
     (make-list (length kin-time-list) :initial-element 0.0))
    (joint-angle-margin 3.0) ;; [deg] [mm]
    (collision-pair-list)
    (keep-timing-order? t)
    (stationery-start-finish-task-scale 0.0)
    (first-diff-square-integration-regular-scale 0.0)
    (second-diff-square-integration-regular-scale 0.0)
    (third-diff-square-integration-regular-scale 0.0)
    (motion-duration-regular-scale 0.0)
    (norm-regular-scale-max 1e-5)
    (norm-regular-scale-offset 1e-7)
    (timing-norm-regular-scale-max 1e-5)
    (timing-norm-regular-scale-offset 1e-7)
    )
   "
Initialize instance
   "
   (send self :name name)
   (setq _robot robot)
   (setq _start-time start-time)
   (setq _finish-time finish-time)
   (setq _joint-list joint-list)
   (setq _num-control-point num-control-point)
   (setq _bspline-order bspline-order)
   (setq _kin-time-list kin-time-list)
   (setq _kin-variable-timing-list kin-variable-timing-list)
   (setq _kin-target-coords-list kin-target-coords-list)
   (setq _kin-attention-coords-list kin-attention-coords-list)
   (setq _kin-pos-tolerance-list kin-pos-tolerance-list)
   (setq _kin-rot-tolerance-list kin-rot-tolerance-list)
   (setq _joint-angle-margin joint-angle-margin)
   (setq _collision-pair-list collision-pair-list)
   (setq _keep-timing-order? keep-timing-order?)
   (setq _stationery-start-finish-task-scale stationery-start-finish-task-scale)
   (setq _first-diff-square-integration-regular-scale first-diff-square-integration-regular-scale)
   (setq _second-diff-square-integration-regular-scale second-diff-square-integration-regular-scale)
   (setq _third-diff-square-integration-regular-scale third-diff-square-integration-regular-scale)
   (setq _motion-duration-regular-scale motion-duration-regular-scale)
   (setq _norm-regular-scale-max norm-regular-scale-max)
   (setq _norm-regular-scale-offset norm-regular-scale-offset)
   (setq _timing-norm-regular-scale-max timing-norm-regular-scale-max)
   (setq _timing-norm-regular-scale-offset timing-norm-regular-scale-offset)
   ;; sub methods
   (send self :_initialize-number)
   (send self :_initialize-dimension)
   (send self :_initialize-configuration-vector)
   )
  (:_initialize-number
   ()
   (setq _num-kin (length _kin-target-coords-list))
   (setq _num-joint (length _joint-list))
   (setq _num-timing (length (remove nil _kin-variable-timing-list)))
   (setq _num-collision (length _collision-pair-list))
   )
  (:_initialize-dimension
   ()
   (setq _dim-control-vector (* _num-joint _num-control-point))
   (setq _dim-timing-vector _num-timing)
   (setq _dim-config (+ _dim-control-vector _dim-timing-vector))
   (setq _dim-task (+ (* 6 _num-kin) (* 4 _num-joint)))
   )
  (:_initialize-configuration-vector
   ()
   (setq _control-vector
         (apply
          #'concatenate
          float-vector
          (map
           cons
           #'(lambda (j)
               (make-array _num-control-point
                           :element-type :float
                           :initial-element (send j :angle-to-speed (send j :joint-angle)))
               )
           _joint-list
           )))
   (setq _timing-vector
         (coerce
          (remove
           nil
           (mapcar
            #'(lambda (tm variable-tm)
                (if variable-tm (or tm 0.0))
                )
            _kin-time-list
            _kin-variable-timing-list
            ))
          float-vector
          ))
   )
  (:robot
   ()
   "
return robot instance
   "
   _robot
   )
  (:joint-list
   ()
   "
return $\mathcal{J}$
   "
   _joint-list
   )
  (:num-kin
   ()
   "
return $N_{\mathit{kin}} := |\mathcal{T}^{\mathit{kin\mathchar`-trg}}| = |\mathcal{T}^{\mathit{kin\mathchar`-att}}|$
   "
   _num-kin
   )
  (:num-joint
   ()
   "
return $N_{\mathit{joint}} := |\mathcal{J}|$
   "
   _num-joint
   )
  (:num-control-point
   ()
   "
return $N_{\mathit{ctrl}}$
   "
   _num-control-point
   )
  (:num-timing
   ()
   "
return $N_{\mathit{tm}}$
   "
   _num-timing
   )
  (:num-collision
   ()
   "
return $N_{\mathit{col}} :=$ number of collision check pairs
   "
   _num-collision
   )
  (:dim-config
   ()
   "
return $\mathit{dim}(\bm{q}) := \mathit{dim}(\bm{p}) + \mathit{dim}(\bm{t}) = N_{\mathit{ctrl}} N_{\mathit{joint}} + N_{tm}$
   "
   _dim-config
   )
  (:dim-task
   ()
   "
return $\mathit{dim}(\bm{e}) := \mathit{dim}(\bm{e}^{\mathit{kin}}) + \mathit{dim}(\bm{e}^{\mathit{stat}}) = 6 N_{\mathit{kin}} + 4 N_{\mathit{joint}}$
   "
   _dim-task
   )
  (:control-vector
   ()
   "
return control vector $\bm{p}$
   "
   _control-vector
   )
  (:timing-vector
   ()
   "
return timing vector $\bm{t}$
   "
   _timing-vector
   )
  (:config-vector
   ()
   "
return $\bm{q} := \begin{pmatrix} \bm{p} \\ \bm{t} \end{pmatrix}$
   "
   (concatenate
    float-vector
    _control-vector
    _timing-vector
    )
   )
  (:set-control-vector
   (control-vector-new
    &key
    (relative? nil)
    )
   "
Set $\bm{p}$.
   "
   (cond (relative?
          (v+ _control-vector control-vector-new _control-vector)
          )
         (t
          (setq _control-vector control-vector-new)
          ))
   )
  (:set-timing-vector
   (timing-vector-new
    &key
    (relative? nil)
    )
   "
Set $\bm{t}$.
   "
   (cond (relative?
          (v+ _timing-vector timing-vector-new _timing-vector)
          )
         (t
          (setq _timing-vector timing-vector-new)
          ))
   ;; update kin-time-list
   (let* ((kin-idx-mapping
           (remove
            nil
            (mapcar
             #'(lambda (variable-tm idx) (if variable-tm idx))
             _kin-variable-timing-list
             (range _num-kin)
             )))
          )
     (map
      nil
      #'(lambda (timing idx)
          (setf (elt _kin-time-list idx) timing)
          )
      _timing-vector
      kin-idx-mapping
      ))
   )
  (:set-config
   (config-new
    &key
    (relative? nil)
    )
   "
Set $\bm{q}$.
   "
   (send self :set-control-vector
         (subseq config-new 0 _dim-control-vector) :relative? relative?)
   (send self :set-timing-vector
         (subseq config-new _dim-control-vector (+ _dim-control-vector _dim-timing-vector)) :relative? relative?)
   )
  )

(defmethod bspline-configuration-task
  (:bspline-vector
   (tm
    &key
    (order-offset 0)
    )
   "
\begin{eqnarray}
  \bm{b}_n(t) &:=& \begin{pmatrix} b_{0,n}(t) \\ b_{1,n}(t) \\ \vdots \\ b_{N_{\mathit{ctrl}}-1,n}(t) \end{pmatrix} \in \mathbb{R}^{N_{\mathit{ctrl}}}
\end{eqnarray}

return $\bm{b}_n(t)$
   "
   (map float-vector
        #'(lambda (i)
            (bspline-basis-func
             tm i (+ _bspline-order order-offset) _num-control-point _start-time _finish-time
             _bspline-order _num-control-point))
        (range _num-control-point))
   )
  (:bspline-matrix
   (tm
    &key
    (order-offset 0)
    )
   "
\begin{eqnarray}
  \bm{B}_n(t) := \begin{pmatrix} \bm{b}_n^T(t)&&&\bm{O}\\&\bm{b}_n^T(t)&&\\&&\ddots&\\\bm{O}&&&\bm{b}_n^T(t) \end{pmatrix} \in \mathbb{R}^{N_{\mathit{joint}} \times N_{\mathit{ctrl}} N_{\mathit{joint}}}
\end{eqnarray}

return $\bm{B}_n(t)$
   "
   (cond (_bspline-matrix
          (fill (array-entity _bspline-matrix) 0)
          )
         (t
          (setq _bspline-matrix (make-matrix _num-joint _dim-control-vector))
          ))
   (dotimes (i _num-joint)
     (copy-block-matrix _bspline-matrix
                        (matrix (send self :bspline-vector tm :order-offset order-offset))
                        i (* i _num-control-point))
     )
   _bspline-matrix
   )
  (:differential-matrix
   (&key
    (diff-order 1)
    )
   "
\begin{eqnarray}
  \bm{D} &:=& \frac{1}{h} \begin{pmatrix} 1 & -1 &&&\bm{O}\\ & 1 & -1 &&\\&&\ddots&\ddots&\\&&&\ddots&-1\\\bm{O}&&&&1\end{pmatrix} \in \mathbb{R}^{N_{\mathit{ctrl}} \times N_{\mathit{ctrl}}}
\end{eqnarray}

return $\bm{D}^k$
   "
   (unless (assoc diff-order _diff-mat-list)
     (unless _diff-mat
       (setq _diff-mat (unit-matrix _num-control-point))
       (dotimes (i (1- _num-control-point))
         (setf (aref _diff-mat i (1+ i)) -1))
       (setq _diff-mat
             (scale-matrix
              (/ (- _num-control-point _bspline-order) (- _finish-time _start-time)) ;; 1/h
              _diff-mat))
       )
     (push
      (list diff-order (reduce #'m* (make-list diff-order :initial-element _diff-mat)))
      _diff-mat-list)
     )
   (cadr (assoc diff-order _diff-mat-list))
   )
  (:extended-differential-matrix
   (&key
    (diff-order 1)
    )
   "
\begin{eqnarray}
  \bm{\hat{D}}_k &:=& \begin{pmatrix} (\bm{D}^k)^T&&\bm{O}\\&\ddots&\\\bm{O}&&(\bm{D}^k)^T \end{pmatrix} \in \mathbb{R}^{N_{\mathit{ctrl}} N_{\mathit{joint}} \times N_{\mathit{ctrl}} N_{\mathit{joint}}}
\end{eqnarray}

return $\bm{\hat{D}}_k$
   "
   (unless (assoc diff-order _extended-diff-mat-list)
     (let* ((extended-diff-mat
             (make-matrix _dim-control-vector _dim-control-vector))
            (diff-mat
             (transpose (send self :differential-matrix :diff-order diff-order)))
            )
       (dotimes (i _num-joint)
         (copy-block-matrix extended-diff-mat
                            diff-mat
                            (* i _num-control-point) (* i _num-control-point))
         )
       (push (list diff-order extended-diff-mat) _extended-diff-mat-list)
       ))
   (cadr (assoc diff-order _extended-diff-mat-list))
   )
  (:bspline-differential-matrix
   (tm
    &key
    (diff-order 1)
    )
   "
return $\bm{B}_{n-k}(t) \bm{\hat{D}}_k \in \mathbb{R}^{N_{\mathit{joint}} \times N_{\mathit{ctrl}} N_{\mathit{joint}}}$
   "
   (m* (send self :bspline-matrix tm :order-offset (- diff-order))
       (send self :extended-differential-matrix :diff-order diff-order))
   )
  (:control-matrix
   ()
   "
\begin{eqnarray}
  \bm{P} := \begin{pmatrix} \bm{p}_1^T \\ \bm{p}_2^T \\ \vdots \\ \bm{p}_{n_{\mathit{joint}}}^T \end{pmatrix} \in \mathbb{R}^{N_{\mathit{joint}} \times N_{\mathit{ctrl}}}
\end{eqnarray}

return $\bm{P}$
   "
   (apply
    #'matrix
    (divide-vector
     _control-vector
     :segment-length _num-control-point
     ))
   )
  (:theta
   (tm)
   "
return $\bm{\theta}(t) = \bm{B}_{n}(t) \bm{p} \ {\rm [rad] [m]}$
   "
   (transform (send self :bspline-matrix tm) _control-vector)
   )
  (:theta-dot
   (tm
    &key
    (diff-order 1)
    )
   "
return $\bm{\theta}^{(k)}(t) = \dfrac{d^{(k)} \bm{\theta}(t)}{d t^{(k)}} = \bm{P} \bm{D}^k \bm{b}_{n-k}(t) \ {\rm [rad/s^k] [m/s^k]}$
   "
   (transform
    (send self :control-matrix)
    (transform
     (send self :differential-matrix :diff-order diff-order)
     (send self :bspline-vector tm :order-offset (- diff-order))
     ))
   )
  (:theta-dot-numerical
   (tm
    &key
    (diff-order 1)
    (delta-time 0.05) ;; [sec]
    )
   "
return $\bm{\theta}^{(k)}(t) = \dfrac{d^{(k)} \bm{\theta}(t)}{d t^{(k)}} = \dfrac{\bm{\theta}^{(k-1)}(t + \Delta t) - \bm{\theta}^{(k-1)}(t)}{\Delta t}\ {\rm [rad/s^k] [m/s^k]}$
   "
   (cond ((= diff-order 1)
          (scale (/ 1.0 delta-time)
                 (v- (send self :theta (+ tm delta-time))
                     (send self :theta tm)))
          )
         (t
          (scale (/ 1.0 delta-time)
                 (v- (send self :theta-dot-numerical (+ tm delta-time) :diff-order (1- diff-order))
                     (send self :theta-dot-numerical tm :diff-order (1- diff-order))))
          ))
   )
  (:apply-theta-to-robot
   (tm)
   "
apply \bm{\theta}(t) to robot.
   "
   (mapcar
    #'(lambda (j theta)
        (send j :joint-angle (send j :speed-to-angle theta))) ;; [mm] [deg]
    _joint-list
    (coerce (send self :theta tm) cons))
   )
  (:kin-target-coords-list
   ()
   "
\begin{eqnarray}
  T_m^{\mathit{kin\mathchar`-trg}} = \{\bm{p}_{l}^{\mathit{kin\mathchar`-trg}}, \bm{R}_{l}^{\mathit{kin\mathchar`-trg}}\} \ \ (l = 1,2,\cdots,N_{\mathit{kin}})
\end{eqnarray}

return $\mathcal{T}^{\mathit{kin\mathchar`-trg}} := \{T_1^{\mathit{kin\mathchar`-trg}}, T_2^{\mathit{kin\mathchar`-trg}}, \cdots, T_{N_\mathit{kin}}^{\mathit{kin\mathchar`-trg}} \}$
   "
   _kin-target-coords-list
   )
  (:kin-attention-coords-list
   ()
   "
\begin{eqnarray}
  T_m^{\mathit{kin\mathchar`-att}} = \{\bm{p}_{l}^{\mathit{kin\mathchar`-att}}, \bm{R}_{l}^{\mathit{kin\mathchar`-att}}\} \ \ (l = 1,2,\cdots,N_{\mathit{kin}})
\end{eqnarray}

return $\mathcal{T}^{\mathit{kin\mathchar`-att}} := \{T_1^{\mathit{kin\mathchar`-att}}, T_2^{\mathit{kin\mathchar`-att}}, \cdots, T_{N_\mathit{kin}}^{\mathit{kin\mathchar`-att}} \}$
   "
   _kin-attention-coords-list
   )
  )

(defmethod bspline-configuration-task
  (:kin-start-time
   ()
   "
return $t_s^{\mathit{kin}} := t_1^{\mathit{kin\mathchar`-tm}}$
   "
   (elt _kin-time-list 0)
   )
  (:kin-finish-time
   ()
   "
return $t_f^{\mathit{kin}} := t_{N_{\mathit{kin}}}^{\mathit{kin\mathchar`-tm}}$
   "
   (elt _kin-time-list (1- _num-kin))
   )
  (:motion-duration
   ()
   "
return $(t_{N_{\mathit{kin}}}^{\mathit{kin\mathchar`-tm}} - t_1^{\mathit{kin\mathchar`-tm}})$
   "
   (- (send self :kin-finish-time) (send self :kin-start-time))
   )
  (:kinematics-task-value
   (&key (update? t))
   "
\begin{eqnarray}
  \bm{e}^{\mathit{kin}}(\bm{q})
  &=& \bm{e}^{\mathit{kin}}(\bm{p}, \bm{t}) \\
  &=& \begin{pmatrix}
    \bm{e}_{1}^{\mathit{kin}}(\bm{p}, \bm{t}) \\ \bm{e}_{2}^{\mathit{kin}}(\bm{p}, \bm{t}) \\ \vdots \\ \bm{e}_{N_{\mathit{kin}}}^{\mathit{kin}}(\bm{p}, \bm{t})
  \end{pmatrix} \\
  \bm{e}_{l}^{\mathit{kin}}(\bm{p}, \bm{t}) &=&
  \begin{pmatrix}
    \bm{p}_{l}^{\mathit{kin\mathchar`-trg}} - \bm{p}_{l}^{\mathit{kin\mathchar`-att}}(\bm{p}, \bm{t}) \\
    \bm{a}\left(\bm{R}_{l}^{\mathit{kin\mathchar`-trg}} \bm{R}_{l}^{\mathit{kin\mathchar`-att}}(\bm{p}, \bm{t})^T\right)
  \end{pmatrix} \in \mathbb{R}^6 \ \ (l = 1,2,\cdots,N_{\mathit{kin}})
\end{eqnarray}
$\bm{a}(\bm{R})$は姿勢行列$\bm{R}$の等価角軸ベクトルを表す．

return $\bm{e}^{\mathit{kin}}(\bm{q}) \in \mathbb{R}^{6 N_{\mathit{kin}}}$
   "
   (when (or update? (not (send self :get :kinematics-task-value)))
     (send self :put :kinematics-task-value
           (apply
            #'concatenate
            float-vector
            (flatten
             (mapcar
              #'(lambda (tm att-coords trg-coords pos-tol rot-tol)
                  (send self :apply-theta-to-robot tm)
                  (list
                   (map
                    float-vector
                    #'(lambda (err)
                        (cond ((< err (- pos-tol))
                               (+ err pos-tol)
                               )
                              ((< pos-tol err)
                               (- err pos-tol)
                               )
                              (t
                               0.0
                               ))
                        )
                    (scale 1e-3 (v- (send trg-coords :worldpos) (send att-coords :worldpos))) ;; [m]
                    )
                   (map
                    float-vector
                    #'(lambda (err)
                        (cond ((< err (- rot-tol))
                               (+ err rot-tol)
                               )
                              ((< rot-tol err)
                               (- err rot-tol)
                               )
                              (t
                               0.0
                               ))
                        )
                    (matrix-log (m* (send trg-coords :worldrot) (transpose (send att-coords :worldrot)))) ;; [rad]
                    )
                   ))
              _kin-time-list
              _kin-attention-coords-list
              _kin-target-coords-list
              _kin-pos-tolerance-list
              _kin-rot-tolerance-list
              ))
            ))
     )
   (send self :get :kinematics-task-value)
   )
  (:stationery-start-finish-task-value
   (&key (update? t))
   "
\begin{eqnarray}
  \bm{e}^{\mathit{stat}}(\bm{q})
  &=& \bm{e}^{\mathit{stat}}(\bm{p}, \bm{t}) \\
  &=& \begin{pmatrix} \bm{e}^{\mathit{stat}}_{sv}(\bm{p}, \bm{t}) \\ \bm{e}^{\mathit{stat}}_{fv}(\bm{p}, \bm{t}) \\ \bm{e}^{\mathit{stat}}_{sa}(\bm{p}, \bm{t}) \\ \bm{e}^{\mathit{stat}}_{fa}(\bm{p}, \bm{t}) \end{pmatrix}
\end{eqnarray}
\begin{eqnarray}
  \bm{e}^{\mathit{stat}}_{sv}(\bm{p}, \bm{t})
  &:=& \bm{\dot{\theta}}(t_s^{\mathit{kin}}) \\
  \bm{e}^{\mathit{stat}}_{fv}(\bm{p}, \bm{t})
  &:=& \bm{\dot{\theta}}(t_f^{\mathit{kin}}) \\
  \bm{e}^{\mathit{stat}}_{sa}(\bm{p}, \bm{t})
  &:=& \bm{\ddot{\theta}}(t_s^{\mathit{kin}}) \\
  \bm{e}^{\mathit{stat}}_{fa}(\bm{p}, \bm{t})
  &:=& \bm{\ddot{\theta}}(t_f^{\mathit{kin}})
\end{eqnarray}

return $\bm{e}^{\mathit{stat}}(\bm{q}) \in \mathbb{R}^{4 N_{\mathit{joint}}}$
   "
   (when (or update? (not (send self :get :stationery-start-finish-task-value)))
     (send self :put :stationery-start-finish-task-value
           (concatenate
            float-vector
            (send self :theta-dot (send self :kin-start-time) :diff-order 1)
            (send self :theta-dot (send self :kin-finish-time) :diff-order 1)
            (send self :theta-dot (send self :kin-start-time) :diff-order 2)
            (send self :theta-dot (send self :kin-finish-time) :diff-order 2)
            )
           )
     )
   (send self :get :stationery-start-finish-task-value)
   )
  (:task-value
   (&key (update? t))
   "
return $\bm{e}(\bm{q})
:= \begin{pmatrix} \bm{e}^{\mathit{kin}}(\bm{q}) \\ k_{\mathit{stat}} \bm{e}^{\mathit{stat}}(\bm{q}) \end{pmatrix} \in \mathbb{R}^{6 N_{\mathit{kin}} + 4 N_{\mathit{joint}}}$
   "
   (concatenate
    float-vector
    (send self :kinematics-task-value :update? update?)
    (scale
     _stationery-start-finish-task-scale
     (send self :stationery-start-finish-task-value :update? update?))
    ))
  )

(defmethod bspline-configuration-task
  (:kinematics-task-jacobian-with-control-vector
   ()
   "
\eqref{eq:bspline-task-jacobian-with-control}より，タスク関数$\bm{e}^{\mathit{kin}}$を制御点$\bm{p}$で微分したヤコビ行列は次式で得られる．
\begin{eqnarray}
  \frac{\partial \bm{e}^{\mathit{kin}}}{\partial \bm{p}}
  &=& \begin{pmatrix}
    \frac{\partial \bm{e}_1^{\mathit{kin}}}{\partial \bm{p}} \\ \frac{\partial \bm{e}_2^{\mathit{kin}}}{\partial \bm{p}} \\ \vdots \\ \frac{\partial \bm{e}_{N_{\mathit{kin}}}^{\mathit{kin}}}{\partial \bm{p}}
  \end{pmatrix} \\
  \frac{\partial \bm{e}_l^{\mathit{kin}}}{\partial \bm{p}}
  &=& - \bm{J}^{\mathit{kin\mathchar`-att}}(\bm{\theta}(t_l^{\mathit{kin\mathchar`-tm}})) \bm{B}_n(t_l^{\mathit{kin\mathchar`-tm}}) \ \ (l = 1,2,\cdots,N_{\mathit{kin}})
\end{eqnarray}

return $\dfrac{\partial \bm{e}^{\mathit{kin}}}{\partial \bm{p}} \in \mathbb{R}^{6 N_{\mathit{kin}} \times N_{\mathit{ctrl}} N_{\mathit{joint}}}$
   "
   (apply
    #'concatenate-matrix-column
    (mapcar
     #'(lambda (tm att-coords)
         (send self :apply-theta-to-robot tm)
         (scale-matrix
          -1
          (m*
           (send _robot :calc-jacobian-from-joint-list
                 :union-joint-list _joint-list
                 :move-target (list att-coords)
                 :transform-coords (list (make-coords))
                 :translation-axis (list t)
                 :rotation-axis (list t)
                 )
           (send self :bspline-matrix tm)))
         )
     _kin-time-list
     _kin-attention-coords-list
     ))
   )
  (:kinematics-task-jacobian-with-timing-vector
   ()
   "
\eqref{eq:bspline-task-jacobian-with-timing}より，タスク関数$\bm{e}^{\mathit{kin}}$をタイミング$\bm{t}$で微分したヤコビ行列は次式で得られる．
\begin{eqnarray}
  \frac{\partial \bm{e}^{\mathit{kin}}}{\partial \bm{t}}
  &=& \begin{pmatrix}
    \frac{\partial \bm{e}_1^{\mathit{kin}}}{\partial \bm{t}} \\ \frac{\partial \bm{e}_2^{\mathit{kin}}}{\partial \bm{t}} \\ \vdots \\ \frac{\partial \bm{e}_{N_{\mathit{kin}}}^{\mathit{kin}}}{\partial \bm{t}}
  \end{pmatrix}
\end{eqnarray}
$\frac{\partial \bm{e}_l^{\mathit{kin}}}{\partial \bm{t}}$の$i$番目の列ベクトル
$\left[ \frac{\partial \bm{e}_l^{\mathit{kin}}}{\partial \bm{t}} \right]_{i} \in \mathbb{R}^{6}$は次式で表される$(i = 1,2,\cdots,N_{\mathit{tm}})$．
\begin{eqnarray}
  \left[ \frac{\partial \bm{e}_l^{\mathit{kin}}}{\partial \bm{t}} \right]_{i}
  &=&
  \left\{ \begin{array}{ll}
  - \bm{J}^{\mathit{kin\mathchar`-att}}(\bm{\theta}(t_l^{\mathit{kin\mathchar`-tm}})) \bm{P} \bm{D} \bm{b}_{n-1}(t_l^{\mathit{kin\mathchar`-tm}}) & t_l^{\mathit{kin\mathchar`-tm}} \ {\rm and} \ t_i \ {\rm is \ identical} \\ \bm{0} & {\rm otherwise} \end{array}\right.
\end{eqnarray}

return $\dfrac{\partial \bm{e}^{\mathit{kin}}}{\partial \bm{t}} \in \mathbb{R}^{6 N_{\mathit{kin}} \times N_\mathit{tm}}$
   "
   (let* ((mat
           (make-matrix (* 6 _num-kin) _dim-timing-vector))
          (kin-idx-mapping
           (remove
            nil
            (mapcar
             #'(lambda (variable-tm idx) (if variable-tm idx))
             _kin-variable-timing-list
             (range _num-kin)
             )))
          )
     (map
      nil
      #'(lambda (timing timing-idx kin-idx)
          (send self :apply-theta-to-robot timing)
          (copy-block-matrix
           mat
           (transpose
            (matrix
             (scale
              -1
              (transform
               (send _robot :calc-jacobian-from-joint-list
                     :union-joint-list _joint-list
                     :move-target (list (elt _kin-attention-coords-list kin-idx))
                     :transform-coords (list (make-coords))
                     :translation-axis (list t)
                     :rotation-axis (list t)
                     )
               (transform
                (send self :control-matrix)
                (transform
                 (send self :differential-matrix :diff-order 1)
                 (send self :bspline-vector timing :order-offset -1)
                 ))))))
           (* 6 kin-idx) timing-idx)
          )
      _timing-vector
      (range _num-timing)
      kin-idx-mapping
      )
     mat
     ))
  (:stationery-start-finish-task-jacobian-with-control-vector
   ()
   "
\eqref{eq:bspline-stationery-task-jacobian-with-control-sv},\eqref{eq:bspline-stationery-task-jacobian-with-control-fv},\eqref{eq:bspline-stationery-task-jacobian-with-control-sa},\eqref{eq:bspline-stationery-task-jacobian-with-control-fa}より，タスク関数$\bm{e}^{\mathit{stat}}$を制御点$\bm{p}$で微分したヤコビ行列は次式で得られる．
\begin{eqnarray}
  \frac{\partial \bm{e}^{\mathit{stat}}}{\partial \bm{p}}
  &=& \begin{pmatrix}
    \frac{\partial \bm{e}_{sv}^{\mathit{stat}}}{\partial \bm{p}} \\ \frac{\partial \bm{e}_{fv}^{\mathit{stat}}}{\partial \bm{p}} \\ \frac{\partial \bm{e}_{sa}^{\mathit{stat}}}{\partial \bm{p}} \\ \frac{\partial \bm{e}_{fa}^{\mathit{stat}}}{\partial \bm{p}}
  \end{pmatrix}
\end{eqnarray}
\begin{eqnarray}
  \frac{\partial \bm{e}^{\mathit{stat}}_{sv}(\bm{p}, \bm{t})}{\partial \bm{p}} &=& \bm{B}_{n-1}(t_s^{\mathit{kin}}) \bm{\hat{D}}_1 \\
  \frac{\partial \bm{e}^{\mathit{stat}}_{fv}(\bm{p}, \bm{t})}{\partial \bm{p}} &=& \bm{B}_{n-1}(t_f^{\mathit{kin}}) \bm{\hat{D}}_1 \\
  \frac{\partial \bm{e}^{\mathit{stat}}_{sa}(\bm{p}, \bm{t})}{\partial \bm{p}} &=& \bm{B}_{n-2}(t_s^{\mathit{kin}}) \bm{\hat{D}}_2 \\
  \frac{\partial \bm{e}^{\mathit{stat}}_{fa}(\bm{p}, \bm{t})}{\partial \bm{p}} &=& \bm{B}_{n-2}(t_f^{\mathit{kin}}) \bm{\hat{D}}_2
\end{eqnarray}

return $\dfrac{\partial \bm{e}^{\mathit{stat}}}{\partial \bm{p}} \in \mathbb{R}^{4 N_{\mathit{joint}} \times N_{\mathit{ctrl}} N_{\mathit{joint}}}$
   "
   (apply
    #'concatenate-matrix-column
    (list
     (send self :bspline-differential-matrix (send self :kin-start-time) :diff-order 1)
     (send self :bspline-differential-matrix (send self :kin-finish-time) :diff-order 1)
     (send self :bspline-differential-matrix (send self :kin-start-time) :diff-order 2)
     (send self :bspline-differential-matrix (send self :kin-finish-time) :diff-order 2)
     ))
   )
  (:stationery-start-finish-task-jacobian-with-timing-vector
   ()
   "
\eqref{eq:bspline-stationery-task-jacobian-with-timing-sv},\eqref{eq:bspline-stationery-task-jacobian-with-timing-fv},\eqref{eq:bspline-stationery-task-jacobian-with-timing-sa},\eqref{eq:bspline-stationery-task-jacobian-with-timing-fa}より，タスク関数$\bm{e}^{\mathit{stat}}$をタイミング$\bm{t}$で微分したヤコビ行列は次式で得られる．
\begin{eqnarray}
  \frac{\partial \bm{e}^{\mathit{stat}}}{\partial \bm{t}}
  &=& \begin{pmatrix}
    \frac{\partial \bm{e}_{sv}^{\mathit{stat}}}{\partial \bm{t}} \\ \frac{\partial \bm{e}_{fv}^{\mathit{stat}}}{\partial \bm{t}} \\ \frac{\partial \bm{e}_{sa}^{\mathit{stat}}}{\partial \bm{t}} \\ \frac{\partial \bm{e}_{fa}^{\mathit{stat}}}{\partial \bm{t}}
  \end{pmatrix}
\end{eqnarray}
$\frac{\partial \bm{e}_x^{\mathit{stat}}}{\partial \bm{t}}$の$i$番目の列ベクトル
$\left[ \frac{\partial \bm{e}_x^{\mathit{stat}}}{\partial \bm{t}} \right]_{i} \in \mathbb{R}^{N_\mathit{joint}}$は次式で表される$(x \in \{ sv, fv, sa, fa \}, i = 1,2,\cdots,N_{\mathit{tm}})$．
\begin{eqnarray}
  \left[\frac{\partial \bm{e}^{\mathit{stat}}_{sv}(\bm{p}, \bm{t})}{\partial \bm{t}} \right]_i &=& \left\{\begin{array}{ll} \bm{P} \bm{D}^2 \bm{b}_{n-2}(t_s^{\mathit{kin}}) & t_s^{\mathit{kin}} \ {\rm and} \ t_i \ {\rm is \ identical} \\ \bm{0} & {\rm otherwise} \end{array}\right. \\
  \left[\frac{\partial \bm{e}^{\mathit{stat}}_{fv}(\bm{p}, \bm{t})}{\partial \bm{t}} \right]_i &=& \left\{\begin{array}{ll} \bm{P} \bm{D}^2 \bm{b}_{n-2}(t_f^{\mathit{kin}}) & t_f^{\mathit{kin}} \ {\rm and} \ t_i \ {\rm is \ identical} \\ \bm{0} & {\rm otherwise} \end{array}\right. \\
  \left[\frac{\partial \bm{e}^{\mathit{stat}}_{sa}(\bm{p}, \bm{t})}{\partial \bm{t}} \right]_i &=& \left\{\begin{array}{ll} \bm{P} \bm{D}^3 \bm{b}_{n-3}(t_s^{\mathit{kin}}) & t_s^{\mathit{kin}} \ {\rm and} \ t_i \ {\rm is \ identical} \\ \bm{0} & {\rm otherwise} \end{array}\right. \\
  \left[\frac{\partial \bm{e}^{\mathit{stat}}_{fa}(\bm{p}, \bm{t})}{\partial \bm{t}} \right]_i &=& \left\{\begin{array}{ll} \bm{P} \bm{D}^3 \bm{b}_{n-3}(t_f^{\mathit{kin}}) & t_f^{\mathit{kin}} \ {\rm and} \ t_i \ {\rm is \ identical} \\ \bm{0} & {\rm otherwise} \end{array}\right.
\end{eqnarray}

return $\dfrac{\partial \bm{e}^{\mathit{stat}}}{\partial \bm{t}} \in \mathbb{R}^{4 N_{\mathit{joint}} \times N_{\mathit{tm}}}$
   "
   (let* ((mat (make-matrix (* 4 _num-joint) _dim-timing-vector))
          (control-mat
           (send self :control-matrix))
          (pd2-mat
           (m* control-mat (send self :differential-matrix :diff-order 2)))
          (pd3-mat
           (m* control-mat (send self :differential-matrix :diff-order 3)))
          )
     (when (elt _kin-variable-timing-list 0)
       (copy-block-matrix
        mat
        (transpose (matrix (transform pd2-mat (send self :bspline-vector (send self :kin-start-time) :order-offset -2))))
        0 0)
       )
     (when (elt _kin-variable-timing-list (1- _num-kin))
       (copy-block-matrix
        mat
        (transpose (matrix (transform pd2-mat (send self :bspline-vector (send self :kin-finish-time) :order-offset -2))))
        _num-joint (1- _dim-timing-vector))
       )
     (when (elt _kin-variable-timing-list 0)
       (copy-block-matrix
        mat
        (transpose (matrix (transform pd3-mat (send self :bspline-vector (send self :kin-start-time) :order-offset -3))))
        (* 2 _num-joint) 0)
       )
     (when (elt _kin-variable-timing-list (1- _num-kin))
       (copy-block-matrix
        mat
        (transpose (matrix (transform pd3-mat (send self :bspline-vector (send self :kin-finish-time) :order-offset -3))))
        (* 3 _num-joint) (1- _dim-timing-vector))
       )
     mat
     ))
  (:task-jacobian
   ()
   "
\begin{eqnarray}
  \frac{\partial \bm{e}}{\partial \bm{q}}
  &=&
  \bordermatrix{
    & N_{\mathit{ctrl}} N_{\mathit{joint}} & N_{\mathit{tm}} \cr
    6 N_{\mathit{kin}} & \frac{\partial \bm{e}^{\mathit{kin}}}{\partial \bm{p}} & \frac{\partial \bm{e}^{\mathit{kin}}}{\partial \bm{t}} \cr
    4 N_{\mathit{joint}} & k_{\mathit{stat}} \frac{\partial \bm{e}^{\mathit{stat}}}{\partial \bm{p}} & k_{\mathit{stat}} \frac{\partial \bm{e}^{\mathit{stat}}}{\partial \bm{t}} \cr
  }
\end{eqnarray}

return $\frac{\partial \bm{e}}{\partial \bm{q}} = \mathbb{R}^{(6 N_{\mathit{kin}} + 4 N_{\mathit{joint}}) \times (N_{\mathit{ctrl}} N_{\mathit{joint}} + N_{\mathit{tm}})}$
   "
   (cond (_task-jacobi
          (fill (array-entity _task-jacobi) 0)
          )
         (t
          (setq _task-jacobi (make-matrix _dim-task _dim-config))
          ))
   (copy-block-matrix _task-jacobi
                      (send self :kinematics-task-jacobian-with-control-vector)
                      0 0)
   (when (> _num-timing 0)
     (copy-block-matrix _task-jacobi
                        (send self :kinematics-task-jacobian-with-timing-vector)
                        0 _dim-control-vector)
     )
   (copy-block-matrix _task-jacobi
                      (scale-matrix
                       _stationery-start-finish-task-scale
                       (send self :stationery-start-finish-task-jacobian-with-control-vector))
                      (* 6 _num-kin) 0)
   (when (> _num-timing 0)
     (copy-block-matrix _task-jacobi
                        (scale-matrix
                         _stationery-start-finish-task-scale
                         (send self :stationery-start-finish-task-jacobian-with-timing-vector))
                        (* 6 _num-kin) _dim-control-vector)
     )
   _task-jacobi
   )
  )

(defmethod bspline-configuration-task
  (:theta-max-vector
   (&key (update? nil))
   "
return $\bm{\theta}_{\mathit{max}} \in \mathbb{R}^{N_{\mathit{joint}}}$
   "
   (when (or update? (not (send self :get :theta-max-vector)))
     (send self :put :theta-max-vector
           (map float-vector
                #'(lambda (j)
                    (cond ((< (- (send j :max-angle) (send j :min-angle)) _joint-angle-margin)
                           (send j :angle-to-speed (* 0.5 (+ (send j :max-angle) (send j :min-angle)))))
                          (t
                           (send j :angle-to-speed (- (send j :max-angle) _joint-angle-margin))))
                    )
                _joint-list
                ))
     )
   (send self :get :theta-max-vector)
   )
  (:theta-min-vector
   (&key (update? nil))
   "
return $\bm{\theta}_{\mathit{min}} \in \mathbb{R}^{N_{\mathit{joint}}}$
   "
   (when (or update? (not (send self :get :theta-min-vector)))
     (send self :put :theta-min-vector
           (map float-vector
                #'(lambda (j)
                    (cond ((< (- (send j :max-angle) (send j :min-angle)) _joint-angle-margin)
                           (send j :angle-to-speed (* 0.5 (+ (send j :max-angle) (send j :min-angle)))))
                          (t
                           (send j :angle-to-speed (+ (send j :min-angle) _joint-angle-margin))))
                    )
                _joint-list
                ))
     )
   (send self :get :theta-min-vector)
   )
  (:theta-inequality-constraint-matrix
   (&key (update? nil))
   "
\eqref{eq:bspline-theta-constraint}より，関節角度上下限制約は次式で表される．
\begin{eqnarray}
  &&\bm{\hat{E}} \bm{\theta}_{min} \leq \bm{p} + \Delta \bm{p} \leq \bm{\hat{E}} \bm{\theta}_{max} \\
  \Leftrightarrow&&
  \begin{pmatrix} \bm{I} \\ - \bm{I} \end{pmatrix} \Delta \bm{p} \geq \begin{pmatrix} \bm{\hat{E}} \bm{\theta}_{min} - \bm{p} \\ - \bm{\hat{E}} \bm{\theta}_{max} + \bm{p} \end{pmatrix} \\
  \Leftrightarrow&&
  \bm{C_{\theta}} \Delta \bm{p} \geq \bm{d_{\theta}}
\end{eqnarray}
ただし，
\begin{eqnarray}
  \bm{\hat{E}} := \begin{pmatrix} \bm{1}_{N_{\mathit{ctrl}}}&&&\bm{0}_{N_{\mathit{ctrl}}}\\&\bm{1}_{N_{\mathit{ctrl}}}&&\\&&\ddots&\\\bm{0}_{N_{\mathit{ctrl}}}&&&\bm{1}_{N_{\mathit{ctrl}}} \end{pmatrix} \in \mathbb{R}^{N_{\mathit{ctrl}} N_{\mathit{joint}} \times N_{\mathit{joint}}}
\end{eqnarray}
$\bm{1}_{N_{\mathit{ctrl}}} \in \mathbb{R}^{N_{\mathit{ctrl}}}$は全要素が$1$の$N_{\mathit{ctrl}}$次元ベクトルである．

return $\bm{C_{\theta}} := \begin{pmatrix} \bm{I} \\ - \bm{I}\end{pmatrix} \in \mathbb{R}^{2 N_{\mathit{ctrl}} N_{\mathit{joint}} \times N_{\mathit{ctrl}} N_{\mathit{joint}}}$
   "
   (when (or update? (not (send self :get :theta-inequality-constraint-matrix)))
     (send self :put :theta-inequality-constraint-matrix
           (concatenate-matrix-column
            (unit-matrix _dim-control-vector)
            (scale-matrix -1 (unit-matrix _dim-control-vector)))
           )
     )
   (send self :get :theta-inequality-constraint-matrix)
   )
  (:theta-inequality-constraint-vector
   (&key (update? t))
   "
return $\bm{d_{\theta}} := \begin{pmatrix} \bm{\hat{E}} \bm{\theta}_{min} - \bm{p} \\ - \bm{\hat{E}} \bm{\theta}_{max} + \bm{p} \end{pmatrix} \in \mathbb{R}^{2 N_{\mathit{ctrl}} N_{\mathit{joint}}}$
   "
   (when (or update? (not (send self :get :theta-inequality-constraint-vector)))
     (send self :put :theta-inequality-constraint-vector
           (concatenate
            float-vector
            (v-
             (apply #'concatenate float-vector
                    (map cons #'(lambda (x) (make-array _num-control-point :element-type :float :initial-element x))
                         (send self :theta-min-vector)))
             _control-vector)
            (v+
             (scale
              -1
              (apply #'concatenate float-vector
                     (map cons #'(lambda (x) (make-array _num-control-point :element-type :float :initial-element x))
                          (send self :theta-max-vector))))
             _control-vector)
            ))
     )
   (send self :get :theta-inequality-constraint-vector)
   )
  (:velocity-max-vector
   (&key (update? nil))
   "
return $\bm{v}_{\mathit{max}} \in \mathbb{R}^{N_{\mathit{joint}}}$
   "
   (when (or update? (not (send self :get :velocity-max-vector)))
     (send self :put :velocity-max-vector
           (map float-vector #'(lambda (j) (send j :max-joint-velocity)) _joint-list))
     )
   (send self :get :velocity-max-vector)
   )
  (:velocity-inequality-constraint-matrix
   (&key (update? nil))
   "
\eqref{eq:bspline-theta-dot-constraint}より，関節速度上下限制約は次式で表される．
\begin{eqnarray}
  &&- \bm{\hat{E}} \bm{v}_{max} \leq \bm{\hat{D}}_1 (\bm{p} + \Delta \bm{p}) \leq \bm{\hat{E}} \bm{v}_{max} \\
  \Leftrightarrow&&
  \begin{pmatrix} \bm{\hat{D}}_1 \\ - \bm{\hat{D}}_1 \end{pmatrix} \Delta \bm{p} \geq \begin{pmatrix} - \bm{\hat{E}} \bm{v}_{max} - \bm{\hat{D}}_1 \bm{p} \\ - \bm{\hat{E}} \bm{v}_{max} + \bm{\hat{D}}_1 \bm{p} \end{pmatrix} \\
  \Leftrightarrow&&
  \bm{C_{\dot{\theta}}} \Delta \bm{p} \geq \bm{d_{\dot{\theta}}}
\end{eqnarray}

return $\bm{C_{\dot{\theta}}} := \begin{pmatrix} \bm{\hat{D}}_1 \\ - \bm{\hat{D}}_1 \end{pmatrix} \in \mathbb{R}^{2 N_{\mathit{ctrl}} N_{\mathit{joint}} \times N_{\mathit{ctrl}} N_{\mathit{joint}}}$
   "
   (when (or update? (not (send self :get :velocity-inequality-constraint-matrix)))
     (let* ((mat
             (send self :extended-differential-matrix :diff-order 1))
            )
       (send self :put :velocity-inequality-constraint-matrix
             (concatenate-matrix-column mat (scale-matrix -1 mat)))
       ))
   (send self :get :velocity-inequality-constraint-matrix)
   )
  (:velocity-inequality-constraint-vector
   (&key (update? t))
   "
return $\bm{d_{\dot{\theta}}} := \begin{pmatrix} - \bm{\hat{E}} \bm{v}_{max} - \bm{\hat{D}}_1 \bm{p} \\ - \bm{\hat{E}} \bm{v}_{max} + \bm{\hat{D}}_1 \bm{p} \end{pmatrix} \in \mathbb{R}^{2 N_{\mathit{ctrl}} N_{\mathit{joint}}}$
   "
   (when (or update? (not (send self :get :velocity-inequality-constraint-vector)))
     (let* ((vec
             (apply #'concatenate float-vector
                    (map cons #'(lambda (x) (make-array _num-control-point :element-type :float :initial-element x))
                         (send self :velocity-max-vector))))
            (mat
             (send self :extended-differential-matrix :diff-order 1))
            )
       (send self :put :velocity-inequality-constraint-vector
             (concatenate
              float-vector
              (v- (scale -1 vec) (transform mat _control-vector))
              (v+ (scale -1 vec) (transform mat _control-vector))
              ))
       ))
   (send self :get :velocity-inequality-constraint-vector)
   )
  (:acceleration-max-vector
   (&key (update? nil))
   "
return $\bm{a}_{\mathit{max}} \in \mathbb{R}^{N_{\mathit{joint}}}$
   "
   (when (or update? (not (send self :get :acceleration-max-vector)))
     (send self :put :acceleration-max-vector
           (map float-vector #'(lambda (j) (send j :get :max-joint-acceleration)) _joint-list))
     )
   (send self :get :acceleration-max-vector)
   )
  (:acceleration-inequality-constraint-matrix
   (&key (update? nil))
   "
\eqref{eq:bspline-theta-ddot-constraint}より，関節加速度上下限制約は次式で表される．
\begin{eqnarray}
  &&- \bm{\hat{E}} \bm{a}_{max} \leq \bm{\hat{D}}_2 (\bm{p} + \Delta \bm{p}) \leq \bm{\hat{E}} \bm{a}_{max} \\
  \Leftrightarrow&&
  \begin{pmatrix} \bm{\hat{D}}_2 \\ - \bm{\hat{D}}_2 \end{pmatrix} \Delta \bm{p} \geq \begin{pmatrix} - \bm{\hat{E}} \bm{a}_{max} - \bm{\hat{D}}_2 \bm{p} \\ - \bm{\hat{E}} \bm{a}_{max} + \bm{\hat{D}}_2 \bm{p} \end{pmatrix} \\
  \Leftrightarrow&&
  \bm{C_{\ddot{\theta}}} \Delta \bm{p} \geq \bm{d_{\ddot{\theta}}}
\end{eqnarray}

return $\bm{C_{\ddot{\theta}}} := \begin{pmatrix} \bm{\hat{D}}_2 \\ - \bm{\hat{D}}_2 \end{pmatrix} \in \mathbb{R}^{2 N_{\mathit{ctrl}} N_{\mathit{joint}} \times N_{\mathit{ctrl}} N_{\mathit{joint}}}$
   "
   (when (or update? (not (send self :get :acceleration-inequality-constraint-matrix)))
     (let* ((mat
             (send self :extended-differential-matrix :diff-order 2))
            )
       (send self :put :acceleration-inequality-constraint-matrix
             (concatenate-matrix-column mat (scale-matrix -1 mat)))
       ))
   (send self :get :acceleration-inequality-constraint-matrix)
   )
  (:acceleration-inequality-constraint-vector
   (&key (update? t))
   "
return $\bm{d_{\ddot{\theta}}} := \begin{pmatrix} - \bm{\hat{E}} \bm{a}_{max} - \bm{\hat{D}}_2 \bm{p} \\ - \bm{\hat{E}} \bm{a}_{max} + \bm{\hat{D}}_2 \bm{p} \end{pmatrix} \in \mathbb{R}^{2 N_{\mathit{ctrl}} N_{\mathit{joint}}}$
   "
   (when (or update? (not (send self :get :acceleration-inequality-constraint-vector)))
     (let* ((vec
             (apply #'concatenate float-vector
                    (map cons #'(lambda (x) (make-array _num-control-point :element-type :float :initial-element x))
                         (send self :acceleration-max-vector))))
            (mat
             (send self :extended-differential-matrix :diff-order 2))
            )
       (send self :put :acceleration-inequality-constraint-vector
             (concatenate
              float-vector
              (v- (scale -1 vec) (transform mat _control-vector))
              (v+ (scale -1 vec) (transform mat _control-vector))
              ))
       ))
   (send self :get :acceleration-inequality-constraint-vector)
   )
  (:control-vector-inequality-constraint-matrix
   (&key (update? nil))
   "
\begin{eqnarray}
  &&
  \left\{
  \begin{array}{l}
  \bm{C_{{\theta}}} \Delta \bm{p} \geq \bm{d_{{\theta}}} \\
  \bm{C_{\dot{\theta}}} \Delta \bm{p} \geq \bm{d_{\dot{\theta}}} \\
  \bm{C_{\ddot{\theta}}} \Delta \bm{p} \geq \bm{d_{\ddot{\theta}}}
  \end{array}
  \right. \\
  \Leftrightarrow&&
  \begin{pmatrix}\bm{C_{{\theta}}}\\\bm{C_{\dot{\theta}}}\\\bm{C_{\ddot{\theta}}}\end{pmatrix}
  \Delta \bm{p}
  \geq
  \begin{pmatrix}\bm{d_{{\theta}}} \\ \bm{d_{\dot{\theta}}} \\ \bm{d_{\ddot{\theta}}}\end{pmatrix} \\
  \Leftrightarrow&&
  \bm{C_p} \Delta \bm{p} \geq \bm{d_p}
\end{eqnarray}

return $\bm{C_p} := \begin{pmatrix}\bm{C_{{\theta}}}\\\bm{C_{\dot{\theta}}}\\\bm{C_{\ddot{\theta}}}\end{pmatrix} \in \mathbb{R}^{N_{\mathit{p\mathchar`-ineq}} \times \mathit{dim}(\bm{p})}$
   "
   (concatenate-matrix-column
    (send self :theta-inequality-constraint-matrix :update? update?)
    (send self :velocity-inequality-constraint-matrix :update? update?)
    (send self :acceleration-inequality-constraint-matrix :update? update?))
   )
  (:control-vector-inequality-constraint-vector
   (&key (update? t))
   "
return $\bm{d_p} := \begin{pmatrix}\bm{d_{{\theta}}} \\ \bm{d_{\dot{\theta}}} \\ \bm{d_{\ddot{\theta}}}\end{pmatrix} \in \mathbb{R}^{N_{\mathit{p\mathchar`-ineq}}}$
   "
   (concatenate
    float-vector
    (send self :theta-inequality-constraint-vector :update? update?)
    (send self :velocity-inequality-constraint-vector :update? update?)
    (send self :acceleration-inequality-constraint-vector :update? update?))
   )
  (:timing-vector-inequality-constraint-matrix
   (&key (update? nil))
   "
\eqref{eq:bspline-timing-min-max-constraint}より，タイミングがBスプラインの初期，終端時刻の間に含まれる制約は次式で表される．
\begin{eqnarray}
  &&t_s \bm{1} \leq \bm{t} + \Delta \bm{t} \leq t_f \bm{1} \\
  \Leftrightarrow&&
  \begin{pmatrix} \bm{I} \\ - \bm{I} \end{pmatrix} \Delta \bm{t} \geq \begin{pmatrix} t_s \bm{1} - \bm{t} \\ - t_f \bm{1} + \bm{t} \end{pmatrix}
\end{eqnarray}
また，\eqref{eq:bspline-timing-order-constraint}より，タイミングの順序が入れ替わることを許容しない場合，その制約は次式で表される．
\begin{eqnarray}
  &&\bm{D}_{\mathit{tm}} (\bm{t} + \Delta \bm{t}) \geq \bm{0} \\
  \Leftrightarrow&&
  \bm{D}_{\mathit{tm}} \Delta \bm{t} \geq - \bm{D}_{\mathit{tm}} \bm{t}
\end{eqnarray}
ただし，
\begin{eqnarray}
  \bm{D}_{\mathit{tm}} = \begin{pmatrix} -1 & 1 &&&& \bm{O} \\ & -1 & 1 &&& \\ &&&\ddots& \\ \bm{O} &&&& -1 & 1 \end{pmatrix} \in \mathbb{R}^{(N_{\mathit{tm}}-1) \times N_{\mathit{tm}}}
\end{eqnarray}
これらを合わせると，
\begin{eqnarray}
  \begin{pmatrix} \bm{I} \\ - \bm{I} \\ \bm{D}_{\mathit{tm}} \end{pmatrix} \Delta \bm{t} \geq \begin{pmatrix} t_s \bm{1} - \bm{t} \\ - t_f \bm{1} + \bm{t} \\ - \bm{D}_{\mathit{tm}} \bm{t} \end{pmatrix}
  \Leftrightarrow&&
  \bm{C_t} \Delta \bm{p} \geq \bm{d_t}
\end{eqnarray}


return $\bm{C_t} := \begin{pmatrix} \bm{I} \\ - \bm{I} \\ \bm{D}_{\mathit{tm}} \end{pmatrix} \in \mathbb{R}^{(3 N_{\mathit{tm}}-1) \times \mathit{dim}(\bm{t})}$
   "
   (when (or update? (not (send self :get :timing-vector-inequality-constraint-matrix)))
     (let* ((keep-order-matrix)
            )
       (when _keep-timing-order?
         (setq keep-order-matrix (make-matrix _num-timing _num-timing))
         (dotimes (i _num-timing)
           (setf (aref keep-order-matrix i i) -1))
         (dotimes (i (1- _num-timing))
           (setf (aref keep-order-matrix i (1+ i)) 1))
         (setq keep-order-matrix
               (block-matrix keep-order-matrix 0 0 (1- _num-timing) _num-timing))
         )
       (send self :put :timing-vector-inequality-constraint-matrix
             (apply
              #'concatenate-matrix-column
              (remove
               nil
               (list
                (unit-matrix _dim-timing-vector)
                (scale-matrix -1 (unit-matrix _dim-timing-vector))
                (if (and (>= _num-timing 2) _keep-timing-order?)
                    keep-order-matrix)
                )))
             )
       ))
   (send self :get :timing-vector-inequality-constraint-matrix)
   )
  (:timing-vector-inequality-constraint-vector
   (&key (update? t))
   "
return $\bm{d_t} := \begin{pmatrix} t_s \bm{1} - \bm{t} \\ - t_f \bm{1} + \bm{t} \\ - \bm{D}_{\mathit{tm}} \bm{t} \end{pmatrix} \in \mathbb{R}^{(3 N_{\mathit{tm}}-1)}$
   "
   (when (or update? (not (send self :get :timing-vector-inequality-constraint-vector)))
     (send self :put :timing-vector-inequality-constraint-vector
           (apply
            #'concatenate
            float-vector
            (remove
             nil
             (list
              (v-
               (make-array _dim-timing-vector :element-type :float :initial-element _start-time)
               _timing-vector)
              (v+
               (scale -1 (make-array _dim-timing-vector :element-type :float :initial-element _finish-time))
               _timing-vector)
              (if (and (>= _num-timing 2) _keep-timing-order?)
                  (map float-vector #'- (subseq _timing-vector 0 (1- _num-timing)) (subseq _timing-vector 1)))
              ))))
     )
   (send self :get :timing-vector-inequality-constraint-vector)
   )
  (:config-inequality-constraint-matrix
   (&key (update? nil) (update-collision? nil))
   "
\begin{eqnarray}
  &&
  \left\{
  \begin{array}{l}
  \bm{C_p} \Delta \bm{p} \geq \bm{d_p} \\
  \bm{C_t} \Delta \bm{t} \geq \bm{d_t}
  \end{array}
  \right. \\
  \Leftrightarrow&&
  \begin{pmatrix}\bm{C_p}& \\ &\bm{C_t} \end{pmatrix}
  \begin{pmatrix}\Delta \bm{p}\\\Delta \bm{t}\end{pmatrix}
  \geq
  \begin{pmatrix}\bm{d_p}\\\bm{d_t}\end{pmatrix} \\
  \Leftrightarrow&&
  \bm{C} \Delta \bm{q} \geq \bm{d}
\end{eqnarray}

return $\bm{C} := \begin{pmatrix}\bm{C_p}& \\ &\bm{C_t} \end{pmatrix} \in \mathbb{R}^{N_{\mathit{ineq}} \times \mathit{dim}(\bm{q})}$
   "
   (concatenate-matrix-diagonal
    (send self :control-vector-inequality-constraint-matrix :update? update?)
    (send self :timing-vector-inequality-constraint-matrix :update? update?))
   )
  (:config-inequality-constraint-vector
   (&key (update? t) (update-collision? nil))
   "
return $\bm{d} := \begin{pmatrix}\bm{d_p}\\\bm{d_t}\end{pmatrix} \in \mathbb{R}^{N_{\mathit{ineq}}}$
   "
   (concatenate
    float-vector
    (send self :control-vector-inequality-constraint-vector :update? update?)
    (send self :timing-vector-inequality-constraint-vector :update? update?))
   )
  )

(defmethod bspline-configuration-task
  (:config-equality-constraint-matrix
   (&key (update? nil))
   "
return $\bm{A} \in \mathbb{R}^{0 \times \mathit{dim}(\bm{q})}$ (no equality constraint)
   "
   (make-matrix 0 _dim-config)
   )
  (:config-equality-constraint-vector
   (&key (update? t))
   "
return $\bm{b} \in \mathbb{R}^{0}$ (no equality constraint)
   "
   (float-vector)
   )
  )

(defmethod bspline-configuration-task
  (:_dim-control-vector-unit-matrix
   ()
   (or (send self :get :_dim-control-vector-unit-matrix)
       (send self :put :_dim-control-vector-unit-matrix (unit-matrix _dim-control-vector)))
   )
  (:_dim-control-vector-buf-matrix
   ()
   (or (send self :get :_dim-control-vector-buf-matrix)
       (send self :put :_dim-control-vector-buf-matrix (make-matrix _dim-control-vector _dim-control-vector)))
   )
  (:square-integration-regular-matrix
   (&key
    (diff-order 1)
    (delta-time
     (/ (- _finish-time _start-time) 100.0)) ;; [sec]
    )
   "
\eqref{eq:bspline-square-integration}より，関節角微分の二乗積分は次式で得られる．
\begin{eqnarray}
  F_{sqr,k}(\bm{p})
  &=& \int_{t_s}^{t_f} \left\| \bm{\theta}^{(k)}(t) \right\|^2 dt \\
  &=& \bm{p}^T \bm{H}_{\mathit{sqr,k}} \bm{p}
\end{eqnarray}
ただし，
\begin{eqnarray}
  \bm{H}_{\mathit{sqr,k}}
  &=& \int_{t_s}^{t_f} \left( \bm{B}_{n-k}(t) \bm{\hat{D}}_k \right)^T \bm{B}_{n-k}(t) \bm{\hat{D}}_k dt \\
  &=& \int_{t_s}^{t_f} \begin{pmatrix} \left( \bm{D}^k \bm{b}_{n-k}(t) \right) \left( \bm{D}^k \bm{b}_{n-k}(t) \right)^T && \bm{O} \\ &\ddots& \\ \bm{O} && \left( \bm{D}^k \bm{b}_{n-k}(t) \right) \left( \bm{D}^k \bm{b}_{n-k}(t) \right)^T \end{pmatrix} dt
\end{eqnarray}
これは二次形式の正則化項である．

return $\bm{H}_{\mathit{sqr,k}} \in \mathbb{R}^{\mathit{dim}(\bm{p}) \times \mathit{dim}(\bm{p})}$
   "
   (let* ((reg-mat (make-matrix _num-control-point _num-control-point))
          (tm _start-time)
          (tmp-vec)
          (finish? nil)
          )
     (while t
       (when (>= (+ tm delta-time) _finish-time)
         (setq finish? t)
         (setq delta-time (- _finish-time tm))
         )
       (setq tmp-vec
             (transform (send self :differential-matrix :diff-order diff-order)
                        (send self :bspline-vector (+ tm (* 0.5 delta-time)) :order-offset (- diff-order))))
       (m+ reg-mat (scale-matrix delta-time (m* (transpose (matrix tmp-vec)) (matrix tmp-vec))) reg-mat)
       (setq tm (+ tm delta-time))
       (when finish?
         (return-from nil nil))
       )
     (apply #'concatenate-matrix-diagonal
            (make-list _num-joint :initial-element reg-mat))
     ))
  (:first-differential-square-integration-regular-matrix
   (&key
    (delta-time
     (/ (- _finish-time _start-time) 100.0)) ;; [sec]
    )
   "
return $\bm{H}_{\mathit{sqr,1}} \in \mathbb{R}^{\mathit{dim}(\bm{p}) \times \mathit{dim}(\bm{p})}$
   "
   (or (send self :get :first-differential-square-integration-regular-matrix)
       (send self :put :first-differential-square-integration-regular-matrix
             (send self :square-integration-regular-matrix :diff-order 1 :delta-time delta-time)))
   )
  (:second-differential-square-integration-regular-matrix
   (&key
    (delta-time
     (/ (- _finish-time _start-time) 100.0)) ;; [sec]
    )
   "
return $\bm{H}_{\mathit{sqr,2}} \in \mathbb{R}^{\mathit{dim}(\bm{p}) \times \mathit{dim}(\bm{p})}$
   "
   (or (send self :get :second-differential-square-integration-regular-matrix)
       (send self :put :second-differential-square-integration-regular-matrix
             (send self :square-integration-regular-matrix :diff-order 2 :delta-time delta-time)))
   )
  (:third-differential-square-integration-regular-matrix
   (&key
    (delta-time
     (/ (- _finish-time _start-time) 100.0)) ;; [sec]
    )
   "
return $\bm{H}_{\mathit{sqr,3}} \in \mathbb{R}^{\mathit{dim}(\bm{p}) \times \mathit{dim}(\bm{p})}$
   "
   (or (send self :get :third-differential-square-integration-regular-matrix)
       (send self :put :third-differential-square-integration-regular-matrix
             (send self :square-integration-regular-matrix :diff-order 3 :delta-time delta-time)))
   )
  (:control-vector-regular-matrix
   ()
   "
\begin{eqnarray}
  \bm{W}_{\mathit{reg,p}} := \min(k_{\mathit{max,p}}, \|\bm{e}\|^2+k_{\mathit{off,p}}) \bm{I} + k_{\mathit{sqr,1}} \bm{H}_{\mathit{sqr,1}} + k_{\mathit{sqr,2}} \bm{H}_{\mathit{sqr,2}} + k_{\mathit{sqr,3}} \bm{H}_{\mathit{sqr,3}}
\end{eqnarray}

return $ \bm{W}_{\mathit{reg,p}} \in \mathbb{R}^{\mathit{dim}(\bm{p}) \times \mathit{dim}(\bm{p})}$
   "
   (reduce
    #'(lambda (m1 m2) (m+ m1 m2 (send self :_dim-control-vector-buf-matrix)))
    (remove
     nil
     (list
      (scale-matrix (min _norm-regular-scale-max (+ (norm2 (send self :task-value :update? nil)) _norm-regular-scale-offset))
                    (send self :_dim-control-vector-unit-matrix))
      (if (not (= _first-diff-square-integration-regular-scale 0.0))
          (scale-matrix _first-diff-square-integration-regular-scale
                        (send self :first-differential-square-integration-regular-matrix)))
      (if (not (= _second-diff-square-integration-regular-scale 0.0))
          (scale-matrix _second-diff-square-integration-regular-scale
                        (send self :second-differential-square-integration-regular-matrix)))
      (if (not (= _third-diff-square-integration-regular-scale 0.0))
          (scale-matrix _third-diff-square-integration-regular-scale
                        (send self :third-differential-square-integration-regular-matrix)))
      )))
   )
  (:control-vector-regular-vector
   ()
   "
\begin{eqnarray}
   \bm{v}_{\mathit{reg,p}} := (k_{\mathit{sqr,1}} \bm{H}_{\mathit{sqr,1}} + k_{\mathit{sqr,2}} \bm{H}_{\mathit{sqr,2}} + k_{\mathit{sqr,3}} \bm{H}_{\mathit{sqr,3}}) \bm{p}
\end{eqnarray}

return $\bm{v}_{\mathit{reg,p}} \in \mathbb{R}^{\mathit{dim}(\bm{p})}$
   "
   (apply
    #'v++
    (remove
     nil
     (list
      (make-array _dim-control-vector :element-type :float :initial-element 0.0)
      (if (not (= _first-diff-square-integration-regular-scale 0.0))
          (scale _first-diff-square-integration-regular-scale
                 (transform (send self :first-differential-square-integration-regular-matrix) _control-vector)))
      (if (not (= _second-diff-square-integration-regular-scale 0.0))
          (scale _second-diff-square-integration-regular-scale
                 (transform (send self :second-differential-square-integration-regular-matrix) _control-vector)))
      (if (not (= _third-diff-square-integration-regular-scale 0.0))
          (scale _third-diff-square-integration-regular-scale
                 (transform (send self :third-differential-square-integration-regular-matrix) _control-vector)))
      )))
   )
  (:motion-duration-regular-matrix
   ()
   "
\eqref{eq:bspline-motion-duration}より，動作期間の二乗は次式で得られる．
\begin{eqnarray}
  F_{\mathit{duration}}(\bm{t})
  &=& \left| t_1 - t_{N_{\mathit{tm}}} \right|^2 \\
  &=& \bm{t}^T \begin{pmatrix} 1 & & -1 \\ & & \\ -1 & & 1 \end{pmatrix} \bm{t} \\
  &=& \bm{t}^T \bm{H}_{\mathit{duration}} \bm{t}
\end{eqnarray}
これは二次形式の正則化項である．

return $\bm{H}_{\mathit{duration}} \in \mathbb{R}^{\mathit{dim}(\bm{t}) \times \mathit{dim}(\bm{t})}$

   "
   (or (send self :get :motion-duration-regular-matrix)
       (send self :put :motion-duration-regular-matrix
             (let* ((mat (make-matrix _dim-timing-vector _dim-timing-vector))
                    )
               (when (and (elt _kin-variable-timing-list 0) (elt _kin-variable-timing-list (1- _num-kin)))
                 (setf (aref mat 0 0) 1)
                 (setf (aref mat 0 (1- _dim-timing-vector)) -1)
                 (setf (aref mat (1- _dim-timing-vector) 0) -1)
                 (setf (aref mat (1- _dim-timing-vector) (1- _dim-timing-vector)) 1)
                 )
               mat
               )))
   )
  (:timing-vector-regular-matrix
   ()
   "
\begin{eqnarray}
  \bm{W}_{\mathit{reg,t}} := \min(k_{\mathit{max,t}}, \|\bm{e}\|^2+k_{\mathit{off,t}}) \bm{I} + k_{\mathit{duration}} \bm{H}_{\mathit{duration}}
\end{eqnarray}

return $ \bm{W}_{\mathit{reg,t}} \in \mathbb{R}^{\mathit{dim}(\bm{t}) \times \mathit{dim}(\bm{t})}$
   "
   (reduce
    #'m+
    (list
     (scale-matrix (min _timing-norm-regular-scale-max (+ (norm2 (send self :task-value :update? nil)) _timing-norm-regular-scale-offset))
                   (unit-matrix _dim-timing-vector))
     (scale-matrix _motion-duration-regular-scale
                   (send self :motion-duration-regular-matrix))
     ))
   )
  (:timing-vector-regular-vector
   ()
   "
\begin{eqnarray}
  \bm{v}_{\mathit{reg,t}} := k_{\mathit{duration}} \bm{H}_{\mathit{duration}} \bm{t}
\end{eqnarray}

return $ \bm{v}_{\mathit{reg,t}} \in \mathbb{R}^{\mathit{dim}(\bm{t})}$
   "
   (scale _motion-duration-regular-scale
          (transform (send self :motion-duration-regular-matrix) _timing-vector))
   )
  (:regular-matrix
   ()
   "
\begin{eqnarray}
  \bm{W}_{\mathit{reg}} := \begin{pmatrix} \bm{W}_{\mathit{reg,p}} & \\ & \bm{W}_{\mathit{reg,t}} \end{pmatrix}
\end{eqnarray}

return $\bm{W}_{\mathit{reg}} \in \mathbb{R}^{\mathit{dim}(\bm{q}) \times \mathit{dim}(\bm{q})}$
   "
   (cond (_regular-mat
          (fill (array-entity _regular-mat) 0)
          )
         (t
          (setq _regular-mat (make-matrix _dim-config _dim-config))
          ))
   (copy-block-matrix
    _regular-mat
    (send self :control-vector-regular-matrix)
    0 0)
   (copy-block-matrix
    _regular-mat
    (send self :timing-vector-regular-matrix)
    _dim-control-vector _dim-control-vector)
   )
  (:regular-vector
   ()
   "
\begin{eqnarray}
  \bm{v}_{\mathit{reg}} := \begin{pmatrix} \bm{v}_{\mathit{reg,p}} \\ \bm{v}_{\mathit{reg,t}} \end{pmatrix}
\end{eqnarray}

return $ \bm{v}_{\mathit{reg}} \in \mathbb{R}^{\mathit{dim}(\bm{q}) \times \mathit{dim}(\bm{q})}$
   "
   (cond (_regular-vec
          (fill _regular-vec 0)
          )
         (t
          (setq _regular-vec
                (make-array _dim-config :element-type :float :initial-element 0))
          ))
   (copy-block-vec
    _regular-vec
    (send self :control-vector-regular-vector)
    0)
   (copy-block-vec
    _regular-vec
    (send self :timing-vector-regular-vector)
    _dim-control-vector)
   )
  )

(defmethod bspline-configuration-task
  (:update-collision-inequality-constraint
   ()
   "
Not implemented yet.
   "
   )
  ;;   (:update-collision-inequality-constraint
  ;;    (&key
  ;;     (distance-margin 0.01) ;; [m]
  ;;     )
  ;;    "
  ;;    "
  ;;    (unless _collision-pair-list
  ;;      (setq _collision-theta-inequality-constraint-matrix
  ;;            (make-matrix 0 _dim-theta))
  ;;      (setq _collision-phi-inequality-constraint-matrix
  ;;            (make-matrix 0 _dim-phi))
  ;;      (setq _collision-inequality-constraint-vector
  ;;            (float-vector))
  ;;      (return-from :update-collision-inequality-constraint nil)
  ;;      )
  ;;    (let* ((variant-mat-vector-list)
  ;;           (invariant-mat-vector-list)
  ;;           (dist-list)
  ;;           )
  ;;      (send self :put :collision-line-list nil)
  ;;      (dolist (pair _collision-pair-list)
  ;;        (let* ((b1 (elt pair 0))
  ;;               (b2 (elt pair 1))
  ;;               (col-ret (pqp-collision-distance b1 b2 :qsize 2))
  ;;               (p1 (elt col-ret 1))
  ;;               (p2 (elt col-ret 2))
  ;;               (c1 (make-cascoords :pos p1 :parent b1))
  ;;               (c2 (make-cascoords :pos p2 :parent b2))
  ;;               (dir (normalize-vector (v- p1 p2)))
  ;;               (dist (- (* 1e-3 (elt col-ret 0)) distance-margin)) ;; [m]
  ;;               (variant-mat-vector
  ;;                (transform
  ;;                 dir
  ;;                 (m-
  ;;                  (send _robot :calc-jacobian-from-joint-list
  ;;                        :union-joint-list _variant-joint-list
  ;;                        :move-target (list c1)
  ;;                        :transform-coords (list (make-coords))
  ;;                        :translation-axis (list t)
  ;;                        :rotation-axis (list nil)
  ;;                        )
  ;;                  (send _robot :calc-jacobian-from-joint-list
  ;;                        :union-joint-list _variant-joint-list
  ;;                        :move-target (list c2)
  ;;                        :transform-coords (list (make-coords))
  ;;                        :translation-axis (list t)
  ;;                        :rotation-axis (list nil)
  ;;                        ))))
  ;;               (invariant-mat-vector
  ;;                (transform
  ;;                 dir
  ;;                 (m-
  ;;                  (send _robot :calc-jacobian-from-joint-list
  ;;                        :union-joint-list _invariant-joint-list
  ;;                        :move-target (list c1)
  ;;                        :transform-coords (list (make-coords))
  ;;                        :translation-axis (list t)
  ;;                        :rotation-axis (list nil)
  ;;                        )
  ;;                  (send _robot :calc-jacobian-from-joint-list
  ;;                        :union-joint-list _invariant-joint-list
  ;;                        :move-target (list c2)
  ;;                        :transform-coords (list (make-coords))
  ;;                        :translation-axis (list t)
  ;;                        :rotation-axis (list nil)
  ;;                        ))))
  ;;               )
  ;;          (send self :put :collision-line-list
  ;;                (append (list (make-line p1 p2)) (send self :get :collision-line-list)))
  ;;          (push variant-mat-vector variant-mat-vector-list)
  ;;          (push invariant-mat-vector invariant-mat-vector-list)
  ;;          (push dist dist-list)
  ;;          (send b1 :dissoc c1)
  ;;          (send b2 :dissoc c2)
  ;;          ))
  ;;      (setq _collision-theta-inequality-constraint-matrix
  ;;            (apply #'matrix variant-mat-vector-list))
  ;;      (setq _collision-phi-inequality-constraint-matrix
  ;;            (apply #'matrix invariant-mat-vector-list))
  ;;      (setq _collision-inequality-constraint-vector
  ;;            (scale -1 (coerce dist-list float-vector)))
  ;;      ))
  ;;   (:collision-theta-inequality-constraint-matrix
  ;;    ()
  ;;    "
  ;;    "
  ;;    _collision-theta-inequality-constraint-matrix
  ;;    )
  ;;   (:collision-phi-inequality-constraint-matrix
  ;;    ()
  ;;    "
  ;;    "
  ;;    _collision-phi-inequality-constraint-matrix
  ;;    )
  ;;   (:collision-inequality-constraint-matrix
  ;;    (&key (update? nil))
  ;;    "
  ;;    "
  ;;    (when update?
  ;;      (send self :update-collision-inequality-constraint))
  ;;    (concatenate-matrix-row
  ;;     (send self :collision-theta-inequality-constraint-matrix)
  ;;     (make-matrix _num-collision (+ _dim-wrench _dim-torque))
  ;;     (send self :collision-phi-inequality-constraint-matrix))
  ;;    )
  ;;   (:collision-inequality-constraint-vector
  ;;    (&key (update? nil))
  ;;    "
  ;;    "
  ;;    (when update?
  ;;      (send self :update-collision-inequality-constraint))
  ;;    _collision-inequality-constraint-vector
  ;;    )
  )

(defmethod bspline-configuration-task
  (:update-viewer
   (&key
    (trajectory-delta-time
     (/ (- _finish-time _start-time) 10.0)) ;; [sec]
    )
   "
Update viewer.
   "
   ;; visualize kinematics task
   (mapcar
    #'(lambda (tm trg-coords att-coords)
        (send self :apply-theta-to-robot tm)
        (send trg-coords :draw-on :flush nil :color #f(0 1 0) :size 50 :width 10)
        (send att-coords :draw-on :flush nil :color #f(1 0 0) :size 100 :width 2)
        )
    _kin-time-list
    _kin-target-coords-list
    _kin-attention-coords-list
    )
   ;; visualize attention coords trajectory
   (when (> trajectory-delta-time 0)
     (let* ((attention-coords-list
             (remove-duplicates _kin-attention-coords-list :test #'equal))
            (attention-pos-list)
            (prev-attention-pos-list)
            (tm _start-time)
            (finish? nil)
            )
       (while t
         (when attention-pos-list
           (setq prev-attention-pos-list (mapcar #'copy-seq attention-pos-list)))
         (send self :apply-theta-to-robot tm)
         (setq attention-pos-list (send-all attention-coords-list :worldpos))
         (when prev-attention-pos-list
           (mapcar
            #'(lambda (prev-att-pos att-pos)
                (send (make-line prev-att-pos att-pos)
                      :draw-on :flush nil :color #f(0.9 0.2 0.2) :width 5))
            prev-attention-pos-list
            attention-pos-list
            ))
         (when finish?
           (return-from nil nil))
         (setq tm (+ tm trajectory-delta-time))
         (when (> tm _finish-time)
           (setq finish? t)
           (setq tm (min tm _finish-time))
           )
         )))
   ;; visualize collision
   (send-all (send self :get :collision-line-list) :draw-on :flush nil :color #f(1 0 0) :width 2)
   )
  (:print-status
   ()
   "
Print status.
   "
   (warning-message 9 "|task|=~a~%" (norm (send self :task-value :update? nil)))
   (warning-message 9 "|kin-task|=~a:~%" (norm (send self :kinematics-task-value :update? nil)))
   (mapcar
    #'(lambda (kin-task-i)
        (warning-message
         9 "  |kin-task-i|=~a (~a)~%" (norm kin-task-i) kin-task-i))
    (divide-vector (send self :kinematics-task-value :update? nil) :segment-length 6)
    )
   (warning-message 9 "|stationery-start-finish-task|=~a~%"
                    (norm
                     (scale _stationery-start-finish-task-scale
                            (send self :stationery-start-finish-task-value :update? nil))))
   (unless (= _first-diff-square-integration-regular-scale 0.0)
     (warning-message
      9 "|first-differential-square-integration-reg|=~a~%"
      (v. _control-vector
          (transform
           (scale-matrix _first-diff-square-integration-regular-scale
                         (send self :first-differential-square-integration-regular-matrix))
           _control-vector))))
   (unless (= _second-diff-square-integration-regular-scale 0.0)
     (warning-message
      9 "|second-differential-square-integration-reg|=~a~%"
      (v. _control-vector
          (transform
           (scale-matrix _second-diff-square-integration-regular-scale
                         (send self :second-differential-square-integration-regular-matrix))
           _control-vector))))
   (unless (= _third-diff-square-integration-regular-scale 0.0)
     (warning-message
      9 "|third-differential-square-integration-reg|=~a~%"
      (v. _control-vector
          (transform
           (scale-matrix _third-diff-square-integration-regular-scale
                         (send self :third-differential-square-integration-regular-matrix))
           _control-vector))))
   (unless (= _motion-duration-regular-scale 0.0)
     (warning-message
      9 "|motion-duration-reg|=~a~%"
      (v. _timing-vector
          (transform
           (scale-matrix _motion-duration-regular-scale
                         (send self :motion-duration-regular-matrix))
           _timing-vector))))
   (send self :print-motion-information)
   )
  (:print-motion-information
   ()
   "
Print motion information.
   "
   (warning-message
    2 "|motion-duration|=~a (start-time=~a, finish-time=~a)~%"
    (- (send self :kin-finish-time) (send self :kin-start-time))
    (send self :kin-start-time)
    (send self :kin-finish-time))
   (warning-message
    2 "|start-vel|=~a~%"
    (norm (send self :theta-dot (send self :kin-start-time) :diff-order 1)))
   (warning-message
    2 "|start-acc|=~a~%"
    (norm (send self :theta-dot (send self :kin-start-time) :diff-order 2)))
   (warning-message
    2 "|finish-vel|=~a~%"
    (norm (send self :theta-dot (send self :kin-finish-time) :diff-order 1)))
   (warning-message
    2 "|finish-acc|=~a~%"
    (norm (send self :theta-dot (send self :kin-finish-time) :diff-order 2)))
   )
  (:play-animation
   (&key
    (robot)
    (delta-time
     (/ (- _finish-time _start-time) 100.0)) ;; [sec]
    (only-motion-duration? t)
    (loop? t)
    (visualize-callback-func)
    )
   "
Play motion animation.
   "
   (do-until-key
    (let* ((tm (if only-motion-duration? (send self :kin-start-time) _start-time))
           (finish? nil)
           )
      (while t
        (send self :apply-theta-to-robot tm)
        (send robot :angle-vector (send _robot :angle-vector))
        ;; draw
        (x::window-main-one)
        (send *irtviewer* :draw-objects :flush nil)
        (send *irtviewer* :viewer :flush)
        (unix::usleep (round (* delta-time 1000 1000)))
        (when visualize-callback-func
          (funcall visualize-callback-func))
        ;; update time
        (when finish?
          (return-from nil nil))
        (setq tm (+ tm delta-time))
        (when (> tm (if only-motion-duration? (send self :kin-finish-time) _finish-time))
          (setq finish? t)
          (setq tm (min tm _finish-time))
          )
        ))
    (unless loop?
      (return-from nil nil))
    ))
  (:plot-theta-graph
   (&key
    (joint-id nil)
    (divide-num 200)
    (plot-numerical? nil)
    (only-motion-duration? t)
    (dat-filename
     "/tmp/bspline-configuration-task-plot-theta-graph.dat")
    (dump-pdf? nil)
    (dump-filename
     (ros::resolve-ros-path "package://eus_qp/optmotiongen/logs/bspline-configuration-task-graph.pdf"))
    )
   "
Plot graph.
   "
   (let* ((start-time
           (if only-motion-duration?
               (send self :kin-start-time) _start-time)
           )
          (finish-time
           (if only-motion-duration?
               (send self :kin-finish-time) _finish-time)
           )
          (tm-list
           (mapcar
            #'(lambda (i) (+ (* (/ (float i) divide-num) (- finish-time start-time)) start-time))
            (range divide-num)
            ))
          (theta-list
           (mapcar
            #'(lambda (tm) (send self :theta tm))
            tm-list
            ))
          (theta-dot-list
           (mapcar
            #'(lambda (tm) (send self :theta-dot tm))
            tm-list
            ))
          (theta-dot-numerical-list
           (if plot-numerical?
               (mapcar
                #'(lambda (tm) (send self :theta-dot-numerical tm))
                tm-list
                )))
          (theta-dot2-list
           (mapcar
            #'(lambda (tm) (send self :theta-dot tm :diff-order 2))
            tm-list
            ))
          (theta-dot2-numerical-list
           (if plot-numerical?
               (mapcar
                #'(lambda (tm) (send self :theta-dot-numerical tm :diff-order 2))
                tm-list
                )))
          (control-vec-list
           (divide-vector _control-vector :segment-length _num-control-point))
          (plot-trg-list
           (list "theta" "theta-dot" "theta-dot2" "bspline"))
          )
     (dolist (j (if joint-id (list joint-id) (range _num-joint)))
       (with-open-file
        (f dat-filename :direction :output :if-exists :new-version :if-does-not-exist :create)
        (dotimes (i divide-num)
          (format f "~a ~a ~a ~a ~a ~a ~a ~a ~a ~a ~a ~a"
                  (elt tm-list i)
                  (elt (elt theta-list i) j)
                  (elt (send self :theta-max-vector) j)
                  (elt (send self :theta-min-vector) j)
                  (elt (elt theta-dot-list i) j)
                  (if plot-numerical? (elt (elt theta-dot-numerical-list i) j) 0)
                  (elt (send self :velocity-max-vector) j)
                  (- (elt (send self :velocity-max-vector) j))
                  (elt (elt theta-dot2-list i) j)
                  (if plot-numerical? (elt (elt theta-dot2-numerical-list i) j) 0)
                  (elt (send self :acceleration-max-vector) j)
                  (- (elt (send self :acceleration-max-vector) j))
                  )
          (let* ((bspline-vec
                  (send self :bspline-vector (elt tm-list i)))
                 )
            (dotimes (m _num-control-point)
              (format f " ~a"
                      (* (elt (elt control-vec-list j) m) (elt bspline-vec m)))
              ))
          (format f " ~%")
          ))
       (cond (dump-pdf?
              (let* ((joint-name
                      (send (elt _joint-list j) :name))
                     (plt-filename
                      (ros::resolve-ros-path "package://eus_qp/optmotiongen/scripts/bspline-configuration-task-plot-theta-graph-dump-pdf.plt"))
                     (pdf-filename-src
                      "/tmp/bspline-configuration-task-plot-theta-graph-~a-~a.pdf")
                     )
                (dolist (plot-trg plot-trg-list)
                  (unix:system
                   (format nil "gnuplot -e \"plot_trg='~a'; joint_name='~a'; num_control=~a; pdf_file='~a'\" ~a"
                           plot-trg joint-name _num-control-point (format nil pdf-filename-src "theta" joint-name) plt-filename
                           ))
                  )
                ))
             (t
              (unix:system
               (format nil "gnuplot -e \"plot_numerical=~a\" ~a"
                       (if plot-numerical? 1 0)
                       (ros::resolve-ros-path "package://eus_qp/optmotiongen/scripts/bspline-configuration-task-plot-theta-graph-display-window.plt")
                       ))
              ))
       )
     (when dump-pdf?
       (let* ((pdf-filename-src
               "/tmp/bspline-configuration-task-plot-theta-graph-~a-~a.pdf")
              (multipage-filename-src
               "/tmp/bspline-configuration-task-plot-theta-graph-~a-multipage.pdf")
              (singlepage-filename-src
               "/tmp/bspline-configuration-task-plot-theta-graph-~a-singlepage.pdf")
              )
         (dolist (plot-trg plot-trg-list)
           (unix:system
            (format
             nil
             "pdfunite ~a ~a"
             (reduce
              #'(lambda (x y) (format nil "~a ~a" x y))
              (mapcar
               #'(lambda (jnt) (format nil pdf-filename-src plot-trg (send jnt :name)))
               _joint-list))
             (format nil multipage-filename-src plot-trg)
             ))
           (unix:system
            (format
             nil
             "pdfnup --nup 4x~a ~a --outfile ~a"
             (ceiling (/ _num-joint 4.0))
             (format nil multipage-filename-src plot-trg)
             (format nil singlepage-filename-src plot-trg)
             ))
           )
         (unix:system
          (format nil "DIRNAME=`dirname ~a`; if [ ! -d $DIRNAME ]; then mkdir -p $DIRNAME; fi" dump-filename))
         (unix:system
          (format
           nil
           "pdfunite ~a ~a"
           (reduce
            #'(lambda (x y) (format nil "~a ~a" x y))
            (mapcar
             #'(lambda (plot-trg) (format nil singlepage-filename-src plot-trg))
             plot-trg-list
             ))
           dump-filename
           ))
         ))
     ))
  )

(defmethod bspline-configuration-task
  (:generate-angle-vector-sequence
   (&key
    (divide-num 100)
    (start-time
     (send self :kin-start-time))
    (finish-time
     (send self :kin-finish-time))
    (delta-time
     (/ (float (- finish-time start-time)) divide-num)) ;; [sec]
    )
   "
Generate angle-vector-sequence.
   "
   (let* ((tm start-time)
          (finish? nil)
          (av-list)
          )
     (while t
       (send self :apply-theta-to-robot tm)
       (push-back (send _robot :angle-vector) av-list)
       ;; update time
       (when finish?
         (return-from nil nil))
       (setq tm (+ tm delta-time))
       (when (> tm finish-time)
         (setq finish? t)
         (setq tm (min tm finish-time))
         )
       )
     av-list
     ))
  )
