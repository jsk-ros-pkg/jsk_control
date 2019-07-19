(load "./instant-configuration-task.l")


;;;;;;;;;;;;;;;;;;;;;;;;;
;; trajectory-configuration-task
;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass trajectory-configuration-task
  :super propertied-object
  :slots ((_instant-config-task-list "list of instant-config-task instance")
          ;; number
          (_num-instant-config-task "L")
          ;; dimensions
          (_dim-variant-config "$\mathit{dim}(\bm{q}_{\mathit{var}})$")
          (_dim-invariant-config "$\mathit{dim}(\bm{q}_{\mathit{invar}})$")
          (_dim-config "$\mathit{dim}(\bm{q})$")
          (_dim-task "$\mathit{dim}(\bm{e})$")
          ;; scale
          (_norm-regular-scale-max "$k_{\mathit{max}}$")
          (_norm-regular-scale-offset "$k_{\mathit{off}}$")
          (_adjacent-regular-scale-list "$k^{(1)}_{\mathit{adj}},k^{(2)}_{\mathit{adj}},\cdots,k^{(L-1)}_{\mathit{adj}}$")
          (_torque-regular-scale "$k_{\mathit{trq}}$")
          ;; buffer
          (_task-jacobi "buffer for $\frac{\partial \bm{e}}{\partial \bm{q}}$")
          )
  :documentation
  "
軌道コンフィギュレーション$\bm{q}$と軌道タスク関数$\bm{e}(\bm{q})$のクラス．

以降では，説明文やメソッド名で，``軌道''や``trajectory''を省略する．

コンフィギュレーション$\bm{q}$の取得・更新，タスク関数$\bm{e}(\bm{q})$の取得，タスク関数のヤコビ行列$\frac{\partial \bm{e}(\bm{q})}{\partial \bm{q}}$の取得，コンフィギュレーションの等式・不等式制約$\bm{A}, \bm{b}, \bm{C}, \bm{d}$の取得のためのメソッドが定義されている．

コンフィギュレーション・タスク関数を定めるために，初期化時に以下を与える
\begin{itemize}
\setlength{\itemindent}{10mm}
\setlength{\leftskip}{-10mm}
\item 瞬時のコンフィギュレーション・タスクのリスト
\begin{description}
  \item[instant-config-task-list] instant-configuration-taskのリスト
\end{description}
\item 目的関数の重み
\begin{description}
  \item[norm-regular-scale-max $k_{\mathit{max}}$] コンフィギュレーション更新量正則化の重み最大値
  \item[norm-regular-scale-offset $k_{\mathit{off}}$] コンフィギュレーション更新量正則化の重みオフセット
  \item[adjacent-regular-scale-list $k_{\mathit{adj}}^{(l)}$] 隣接コンフィギュレーション正則化の重みのリスト
  \item[torque-regular-scale $k_{\mathit{trq}}$] トルク正則化の重み
\end{description}
\end{itemize}

コンフィギュレーション$\bm{q}$は以下から構成される．
\begin{eqnarray}
  \bm{q} := \begin{pmatrix} \bm{q}_{\mathit{var}}^{(1) T} & \bm{q}_{\mathit{var}}^{(2) T} & \cdots & \bm{q}_{\mathit{var}}^{(L) T} & \bm{q}_{\mathit{invar}}^T \end{pmatrix}^T
\end{eqnarray}
ここで，
\begin{eqnarray}
  \bm{q}_{\mathit{invar}} := \bm{q}_{\mathit{invar}}^{(1)} = \bm{q}_{\mathit{invar}}^{(2)} = \cdots = \bm{q}_{\mathit{invar}}^{(L)}
\end{eqnarray}
$\bm{q}_{\mathit{var}}^{(l)}, \bm{q}_{\mathit{invar}}^{(l)} \ (l=1,2,\cdots,L)$は$l$番目の瞬時の時変，時不変コンフィギュレーションを表す．

タスク関数$\bm{e}(\bm{q})$は以下から構成される．
\begin{eqnarray}
  \bm{e}(\bm{q}) := \begin{pmatrix} \bm{e}^{(1) T}(\bm{q}_{\mathit{var}}^{(1)}, \bm{q}_{\mathit{invar}}) & \bm{e}^{(2) T}(\bm{q}_{\mathit{var}}^{(2)}, \bm{q}_{\mathit{invar}}) & \cdots & \bm{e}^{(L) T}(\bm{q}_{\mathit{var}}^{(L)}, \bm{q}_{\mathit{invar}}) \end{pmatrix}^T
\end{eqnarray}
$\bm{e}^{(l)}(\bm{q}_{\mathit{var}}^{(l)}, \bm{q}_{\mathit{invar}}) \ (l=1,2,\cdots,L)$は$l$番目の瞬時のタスク関数を表す．
  "
  )

(defmethod trajectory-configuration-task
  (:init
   (&key
    (name)
    (instant-config-task-list)
    (norm-regular-scale-max 1e-4)
    (norm-regular-scale-offset 1e-7)
    (adjacent-regular-scale 5e-3)
    (adjacent-regular-scale-list)
    (torque-regular-scale 1e-3)
    )
   "
Initialize instance
   "
   (send self :name name)
   (setq _instant-config-task-list instant-config-task-list)
   (setq _norm-regular-scale-max norm-regular-scale-max)
   (setq _norm-regular-scale-offset norm-regular-scale-offset)
   (cond (adjacent-regular-scale-list
          (setq _adjacent-regular-scale-list adjacent-regular-scale-list)
          )
         (t
          (setq _adjacent-regular-scale-list
                (make-list (1- (length instant-config-task-list)) :initial-element adjacent-regular-scale))
          ))
   (setq _torque-regular-scale torque-regular-scale)
   ;; sub methods
   (send self :_initialize-number)
   (send self :_initialize-dimension)
   )
  (:_initialize-number
   ()
   (setq _num-instant-config-task (length _instant-config-task-list))
   )
  (:_initialize-dimension
   ()
   (setq _dim-variant-config (apply #'+ (send-all _instant-config-task-list :dim-variant-config)))
   (setq _dim-invariant-config (send (car _instant-config-task-list) :dim-invariant-config))
   (setq _dim-config (+ _dim-variant-config _dim-invariant-config))
   (setq _dim-task (apply #'+ (send-all _instant-config-task-list :dim-task)))
   )
  (:instant-config-task-list
   ()
   "
return instant-config-task-list
   "
   _instant-config-task-list
   )
  (:dim-variant-config
   ()
   "
return $\mathit{dim}(\bm{q_{\mathit{var}}}) := \sum_{l=1}^{L} dim(\bm{q}_{\mathit{var}}^{(l)})$
   "
   _dim-variant-config
   )
  (:dim-invariant-config
   ()
   "
return $\mathit{dim}(\bm{q_{\mathit{invar}}}) := dim(\bm{q}_{\mathit{invar}}^{(l)}) \ (l = 1,2,\cdots,Lで同じ)$
   "
   _dim-invariant-config
   )
  (:dim-config
   ()
   "
return $\mathit{dim}(\bm{q}) := \mathit{dim}(\bm{q_{\mathit{var}}}) + \mathit{dim}(\bm{q_{\mathit{invar}}})$
   "
   _dim-config
   )
  (:dim-task
   ()
   "
return $\mathit{dim}(\bm{e}) := \sum_{l=1}^{L} dim(\bm{e}^{(l)})$
   "
   _dim-task
   )
  (:variant-config-vector
   ()
   "
return $\bm{q_{\mathit{var}}} := \begin{pmatrix} \bm{q}_{\mathit{var}}^{(1)} \\ \bm{q}_{\mathit{var}}^{(2)} \\ \vdots \\ \bm{q}_{\mathit{var}}^{(L)}\end{pmatrix}$
   "
   (apply #'concatenate float-vector (send-all _instant-config-task-list :variant-config-vector))
   )
  (:invariant-config-vector
   ()
   "
return $\bm{q_{\mathit{invar}}} := \bm{q}_{\mathit{invar}}^{(l)} \ (l = 1,2,\cdots,Lで同じ)$
   "
   (send (car _instant-config-task-list) :invariant-config-vector)
   )
  (:config-vector
   ()
   "
return $\bm{q} := \begin{pmatrix} \bm{q_{\mathit{var}}} \\ \bm{q_{\mathit{invar}}} \end{pmatrix} = \begin{pmatrix} \bm{q}_{\mathit{var}}^{(1)} \\ \bm{q}_{\mathit{var}}^{(2)} \\ \vdots \\ \bm{q}_{\mathit{var}}^{(L)} \\ \bm{q}_{\mathit{invar}} \end{pmatrix}$
   "
   (concatenate float-vector
                (send self :variant-config-vector)
                (send self :invariant-config-vector))
   )
  (:set-variant-config
   (variant-config-new
    &key
    (relative? nil)
    (apply-to-robot? t)
    )
   "
Set $\bm{q_{\mathit{var}}}$.
   "
   (dolist (config-task _instant-config-task-list)
     (send config-task
           :set-variant-config (subseq variant-config-new 0 (send config-task :dim-variant-config))
           :relative? relative?
           :apply-to-robot? apply-to-robot?)
     (setq variant-config-new (subseq variant-config-new (send config-task :dim-variant-config)))
     )
   )
  (:set-invariant-config
   (invariant-config-new
    &key
    (relative? nil)
    (apply-to-robot? t)
    )
   "
Set $\bm{q_{\mathit{invar}}}$.
   "
   (send-all _instant-config-task-list
             :set-invariant-config invariant-config-new
             :relative? relative?
             :apply-to-robot? apply-to-robot?)
   )
  (:set-config
   (config-new
    &key
    (relative? nil)
    (apply-to-robot? t)
    )
   "
Set $\bm{q}$.
   "
   (send self :set-variant-config
         (subseq config-new 0 _dim-variant-config) :relative? relative? :apply-to-robot? apply-to-robot?)
   (send self :set-invariant-config
         (subseq config-new _dim-variant-config) :relative? relative? :apply-to-robot? apply-to-robot?)
   )
  )

(defmethod trajectory-configuration-task
  (:task-value
   (&key (update? t))
   "
return $\bm{e}(\bm{q}) := \begin{pmatrix} \bm{e}^{(1)}(\bm{q}_{\mathit{var}}^{(1)}, \bm{q}_{\mathit{invar}}) \\ \bm{e}^{(2)}(\bm{q}_{\mathit{var}}^{(2)}, \bm{q}_{\mathit{invar}}) \\ \vdots \\ \bm{e}^{(L)}(\bm{q}_{\mathit{var}}^{(L)}, \bm{q}_{\mathit{invar}}) \end{pmatrix}$
   "
   (apply #'concatenate float-vector (send-all _instant-config-task-list :task-value :update? update?))
   )
  (:variant-task-jacobian
   ()
   "
\begin{eqnarray}
  \frac{\partial \bm{e}}{\partial \bm{q}_{\mathit{var}}}
  =
  \begin{pmatrix}
    \frac{\partial \bm{e}^{(1)}}{\partial \bm{q}_{\mathit{var}}^{(1)}} & & & \bm{O} \\
    & \frac{\partial \bm{e}^{(2)}}{\partial \bm{q}_{\mathit{var}}^{(2)}} & & \\
    & & \ddots & \\
    \bm{O} & & & \frac{\partial \bm{e}^{(L)}}{\partial \bm{q}_{\mathit{var}}^{(L)}} \\
  \end{pmatrix}
\end{eqnarray}

return $\frac{\partial \bm{e}}{\partial \bm{q}_{\mathit{var}}} \in \mathbb{R}^{dim(\bm{e}) \times dim(\bm{q}_{\mathit{var}})}$
   "
   ()
   (apply #'concatenate-matrix-diagonal
          (mapcar #'(lambda (config-task) (send config-task :variant-task-jacobian)) _instant-config-task-list))
   )
  (:invariant-task-jacobian
   ()
   "
\begin{eqnarray}
  \frac{\partial \bm{e}}{\partial \bm{q}_{\mathit{invar}}}
  =
  \begin{pmatrix}
    \frac{\partial \bm{e}^{(1)}}{\partial \bm{q}_{\mathit{invar}}} \\
    \frac{\partial \bm{e}^{(2)}}{\partial \bm{q}_{\mathit{invar}}} \\
    \vdots \\
    \frac{\partial \bm{e}^{(L)}}{\partial \bm{q}_{\mathit{invar}}}
  \end{pmatrix}
\end{eqnarray}

return $\frac{\partial \bm{e}}{\partial \bm{q}_{\mathit{invar}}} \in \mathbb{R}^{dim(\bm{e}) \times dim(\bm{q}_{\mathit{invar}})}$
   "
   ()
   (apply #'concatenate-matrix-column
          (mapcar #'(lambda (config-task) (send config-task :invariant-task-jacobian)) _instant-config-task-list))
   )
  (:task-jacobian
   ()
   "
\begin{eqnarray}
  \frac{\partial \bm{e}}{\partial \bm{q}}
  &=&
  \begin{pmatrix}
    \frac{\partial \bm{e}}{\partial \bm{q}_{\mathit{var}}} & \frac{\partial \bm{e}}{\partial \bm{q}_{\mathit{invar}}}
  \end{pmatrix} \\
  &=&
  \begin{pmatrix}
    \frac{\partial \bm{e}^{(1)}}{\partial \bm{q}_{\mathit{var}}^{(1)}} & & & \bm{O} & \frac{\partial \bm{e}^{(1)}}{\partial \bm{q}_{\mathit{invar}}} \\
    & \frac{\partial \bm{e}^{(2)}}{\partial \bm{q}_{\mathit{var}}^{(2)}} & & & \frac{\partial \bm{e}^{(2)}}{\partial \bm{q}_{\mathit{invar}}} \\
    & & \ddots & & \\
    \bm{O} & & & \frac{\partial \bm{e}^{(L)}}{\partial \bm{q}_{\mathit{var}}^{(L)}} & \frac{\partial \bm{e}^{(L)}}{\partial \bm{q}_{\mathit{invar}}} \\
  \end{pmatrix}
\end{eqnarray}

return $\frac{\partial \bm{e}}{\partial \bm{q}} \in \mathbb{R}^{dim(\bm{e}) \times dim(\bm{q})}$
   "

   (cond (_task-jacobi
          (fill (array-entity _task-jacobi) 0)
          )
         (t
          (setq _task-jacobi (make-matrix _dim-task _dim-config))
          ))
   (copy-block-matrix _task-jacobi
                      (send self :variant-task-jacobian)
                      0 0)

   (when (> (send (car _instant-config-task-list) :num-invariant-joint) 0)
     (copy-block-matrix _task-jacobi
                        (send self :invariant-task-jacobian)
                        0 _dim-variant-config)
     )
   _task-jacobi
   )
  )

(defmethod trajectory-configuration-task
  (:variant-config-inequality-constraint-matrix
   (&key (update? nil))
   "
\begin{eqnarray}
  \bm{C}_{\mathit{var}} :=
  \begin{pmatrix}
    \bm{C}_{\mathit{var}}^{(1)} & & & \bm{O} \\
    & \bm{C}_{\mathit{var}}^{(2)} & & \\
    & & \ddots & \\
    \bm{O} & & & \bm{C}_{\mathit{var}}^{(L)} \\
  \end{pmatrix}
\end{eqnarray}

return $\bm{C}_{\mathit{var}} \in \mathbb{R}^{N_{\mathit{var\mathchar`-ineq}} \times \mathit{dim}(\bm{q}_{\mathit{var}})}$
   "
   (apply #'concatenate-matrix-diagonal
          (send-all _instant-config-task-list :variant-config-inequality-constraint-matrix :update? update?))
   )
  (:variant-config-inequality-constraint-vector
   (&key (update? t))
   "
\begin{eqnarray}
  \bm{d}_{\mathit{var}} :=
  \begin{pmatrix}
    \bm{d}_{\mathit{var}}^{(1)} \\
    \bm{d}_{\mathit{var}}^{(2)} \\
    \vdots \\
    \bm{d}_{\mathit{var}}^{(L)} \\
  \end{pmatrix}
\end{eqnarray}

return $\bm{d}_{\mathit{var}} \in \mathbb{R}^{N_{\mathit{var\mathchar`-ineq}}}$
   "
   (apply #'concatenate float-vector
          (send-all _instant-config-task-list :variant-config-inequality-constraint-vector :update? update?))
   )
  (:invariant-config-inequality-constraint-matrix
   (&key (update? nil))
   "
\begin{eqnarray}
  \bm{C}_{\mathit{invar}} := \bm{C}_{\mathit{invar}}^{(l)} \ (l = 1,2,\cdots,Lで同じ)
\end{eqnarray}

return $\bm{C}_{\mathit{invar}} \in \mathbb{R}^{N_{\mathit{invar\mathchar`-ineq}} \times \mathit{dim}(\bm{q}_{\mathit{invar}})}$
   "
   (send (car _instant-config-task-list) :invariant-config-inequality-constraint-matrix :update? update?)
   )
  (:invariant-config-inequality-constraint-vector
   (&key (update? t))
   "
\begin{eqnarray}
  \bm{d}_{\mathit{invar}} := \bm{d}_{\mathit{invar}}^{(l)} \ (l = 1,2,\cdots,Lで同じ)
\end{eqnarray}

return $\bm{d}_{\mathit{invar}} \in \mathbb{R}^{N_{\mathit{invar\mathchar`-ineq}}}$
   "
   (send (car _instant-config-task-list) :invariant-config-inequality-constraint-vector :update? update?)
   )
  (:config-inequality-constraint-matrix
   (&key (update? nil) (update-collision? nil))
   "
\begin{eqnarray}
  \bm{C} := \begin{pmatrix}\bm{C}_{\mathit{var}}& \\ &\bm{C}_{\mathit{invar}} \\ \hdashline[2pt/2pt] \multicolumn{2}{c}{\bm{C}_{\mathit{col}}} \end{pmatrix} \in \mathbb{R}^{N_{\mathit{ineq}} \times \mathit{dim}(\bm{q})}
\end{eqnarray}

return $\bm{C} \in \mathbb{R}^{N_{\mathit{ineq}} \times \mathit{dim}(\bm{q})}$
   "
   (concatenate-matrix-column
    (concatenate-matrix-diagonal
     (send self :variant-config-inequality-constraint-matrix :update? update?)
     (send self :invariant-config-inequality-constraint-matrix :update? update?))
    (send self :collision-inequality-constraint-matrix :update? update-collision?))
   )
  (:config-inequality-constraint-vector
   (&key (update? t) (update-collision? nil))
   "
\begin{eqnarray}
  \bm{d} := \begin{pmatrix}\bm{d}_{\mathit{var}}\\\bm{d}_{\mathit{invar}}\\\bm{d}_{\mathit{col}}\end{pmatrix}
\end{eqnarray}

return $\bm{d} \in \mathbb{R}^{N_{\mathit{ineq}}}$
   "
   (concatenate
    float-vector
    (send self :variant-config-inequality-constraint-vector :update? update?)
    (send self :invariant-config-inequality-constraint-vector :update? update?)
    (send self :collision-inequality-constraint-vector :update? update-collision?)
    ))
  )

(defmethod trajectory-configuration-task
  (:variant-config-equality-constraint-matrix
   (&key (update? nil))
   "
\begin{eqnarray}
  \bm{A}_{\mathit{var}} :=
  \begin{pmatrix}
    \bm{A}_{\mathit{var}}^{(1)} & & & \bm{O} \\
    & \bm{A}_{\mathit{var}}^{(2)} & & \\
    & & \ddots & \\
    \bm{O} & & & \bm{A}_{\mathit{var}}^{(L)} \\
  \end{pmatrix}
\end{eqnarray}

return $\bm{A}_{\mathit{var}} \in \mathbb{R}^{N_{\mathit{var\mathchar`-eq}} \times \mathit{dim}(\bm{q}_{\mathit{var}})}$
   "
   (apply #'concatenate-matrix-diagonal
          (send-all _instant-config-task-list :variant-config-equality-constraint-matrix :update? update?))
   )
  (:variant-config-equality-constraint-vector
   (&key (update? t))
   "
\begin{eqnarray}
  \bm{b}_{\mathit{var}} :=
  \begin{pmatrix}
    \bm{b}_{\mathit{var}}^{(1)} \\
    \bm{b}_{\mathit{var}}^{(2)} \\
    \vdots \\
    \bm{b}_{\mathit{var}}^{(L)} \\
  \end{pmatrix}
\end{eqnarray}

return $\bm{b}_{\mathit{var}} \in \mathbb{R}^{N_{\mathit{var\mathchar`-eq}}}$
   "
   (apply #'concatenate float-vector
          (send-all _instant-config-task-list :variant-config-equality-constraint-vector :update? update?))
   )
  (:invariant-config-equality-constraint-matrix
   (&key (update? nil))
   "
\begin{eqnarray}
  \bm{A}_{\mathit{invar}} := \bm{A}_{\mathit{invar}}^{(l)} \ (l = 1,2,\cdots,Lで同じ)
\end{eqnarray}

return $\bm{A}_{\mathit{invar}} \in \mathbb{R}^{N_{\mathit{invar\mathchar`-eq}} \times \mathit{dim}(\bm{q}_{\mathit{invar}})}$
   "
   (send (car _instant-config-task-list) :invariant-config-equality-constraint-matrix :update? update?)
   )
  (:invariant-config-equality-constraint-vector
   (&key (update? t))
   "
\begin{eqnarray}
  \bm{b}_{\mathit{invar}} := \bm{b}_{\mathit{invar}}^{(l)} \ (l = 1,2,\cdots,Lで同じ)
\end{eqnarray}

return $\bm{b}_{\mathit{invar}} \in \mathbb{R}^{N_{\mathit{invar\mathchar`-eq}}}$
   "
   (send (car _instant-config-task-list) :invariant-config-equality-constraint-vector :update? update?)
   )
  (:config-equality-constraint-matrix
   (&key (update? nil))
   "
\begin{eqnarray}
  \bm{A} := \begin{pmatrix}\bm{A}_{\mathit{var}}& \\ &\bm{A}_{\mathit{invar}}\end{pmatrix} \in \mathbb{R}^{N_{\mathit{eq}} \times \mathit{dim}(\bm{q})}
\end{eqnarray}

return $\bm{A} \in \mathbb{R}^{N_{\mathit{eq}} \times \mathit{dim}(\bm{q})}$
   "
   (concatenate-matrix-diagonal
    (send self :variant-config-equality-constraint-matrix :update? update?)
    (send self :invariant-config-equality-constraint-matrix :update? update?))
   )
  (:config-equality-constraint-vector
   (&key (update? t))
   "
\begin{eqnarray}
  \bm{b} := \begin{pmatrix}\bm{b}_{\mathit{var}}\\\bm{b}_{\mathit{invar}}\\\end{pmatrix}
\end{eqnarray}

return $\bm{b} \in \mathbb{R}^{N_{\mathit{eq}}}$
   "
   (concatenate
    float-vector
    (send self :variant-config-equality-constraint-vector :update? update?)
    (send self :invariant-config-equality-constraint-vector :update? update?)
    ))
  )

(defmethod trajectory-configuration-task
  (:update-collision-inequality-constraint
   ()
   "
update inequality matrix $\bm{C}_{\mathit{col},\theta}^{(l)}, \bm{C}_{\mathit{col,\phi}}^{(l)}$ and inequality vector $\bm{d}_{\mathit{col}}^{(l)}$ for collision avoidance $(l = 1,2,\cdots,L)$
   "
   (send-all _instant-config-task-list :update-collision-inequality-constraint)
   )
  (:collision-inequality-constraint-matrix
   (&key (update? nil))
   "
\begin{eqnarray}
  \bm{\hat{C}}_{\mathit{col},\theta}^{(l)} &:=&
  \bordermatrix{
    & \mathit{dim}(\bm{\theta}^{(l)}) & \mathit{dim}(\bm{\hat{w}}^{(l)}) & \mathit{dim}(\bm{\tau}^{(l)}) \cr
    N_{\mathit{col}}^{(l)} & \bm{C}_{\mathit{col},\theta}^{(l)} & \bm{O} & \bm{O}
  }\\
  \bm{C}_{\mathit{col}} &:=&
  \begin{pmatrix}
    \bm{\hat{C}}_{\mathit{col},\theta}^{(1)} & & & & \bm{C}_{\mathit{col},\phi}^{(1)} \\
    & \bm{\hat{C}}_{\mathit{col},\theta}^{(2)} & & & \bm{C}_{\mathit{col},\phi}^{(2)} \\
    & & \ddots & & \vdots \\
    & & & \bm{\hat{C}}_{\mathit{col},\theta}^{(L)} & \bm{C}_{\mathit{col},\phi}^{(L)} \\
  \end{pmatrix}
\end{eqnarray}

return $\bm{C}_{\mathit{col}} \in \mathbb{R}^{N_{\mathit{col}} \times dim(\bm{q})}$
   "
   (when update?
     (send self :update-collision-inequality-constraint))
   (concatenate-matrix-row
    (apply #'concatenate-matrix-diagonal
           (mapcar #'(lambda (config-task)
                       (copy-block-matrix (make-matrix (send config-task :num-collision) (send config-task :dim-variant-config))
                                          (send config-task :collision-theta-inequality-constraint-matrix)
                                          0 0))
                   _instant-config-task-list))
    (apply #'concatenate-matrix-column
           (send-all _instant-config-task-list :collision-phi-inequality-constraint-matrix))
    )
   )
  (:collision-inequality-constraint-vector
   (&key (update? nil))
   "
\begin{eqnarray}
  \bm{d}_{\mathit{col}} &:=&
  \begin{pmatrix}
    \bm{d}_{\mathit{col}}^{(1)} \\
    \bm{d}_{\mathit{col}}^{(2)} \\
    \vdots \\
    \bm{d}_{\mathit{col}}^{(L)}
  \end{pmatrix}
\end{eqnarray}

return $\bm{d}_{\mathit{col}} \in \mathbb{R}^{N_{\mathit{col}}}$
   "
   (when update?
     (send self :update-collision-inequality-constraint))
   (apply
    #'concatenate
    float-vector
    (send-all _instant-config-task-list :collision-inequality-constraint-vector))
   )
  )

(defmethod trajectory-configuration-task
  (:_dim-config-unit-matrix
   ()
   (or (send self :get :_dim-config-unit-matrix)
       (send self :put :_dim-config-unit-matrix (unit-matrix _dim-config)))
   )
  (:_dim-config-buf-matrix
   ()
   (or (send self :get :_dim-config-buf-matrix)
       (send self :put :_dim-config-buf-matrix (make-matrix _dim-config _dim-config)))
   )
  (:adjacent-regular-matrix
   (&key (update? nil))
   "
二次形式の正則化項として次式を考える．
\begin{eqnarray}
  F_{\mathit{adj}}(\bm{q}) &=& \sum_{l=1}^{L-1} k_{\mathit{adj}}^{(l)} \| \bm{\theta}_{l+1} - \bm{\theta}_l \|^2 \\
  &=& \bm{q}^T \bm{W}_{\mathit{adj}} \bm{q}
\end{eqnarray}
ここで，
\begin{eqnarray}
  \bm{\bar{I}}^{(l)}_{\mathit{adj}}
  &:=&
  \bordermatrix{
    & \mathit{dim}(\bm{\theta}^{(l)}) & \mathit{dim}(\bm{\hat{w}}^{(l)}) & \mathit{dim}(\bm{\tau}^{(l)}) \cr
    \mathit{dim}(\bm{\theta}^{(l)}) & k_{\mathit{adj}}^{(l)} \bm{I} && \cr
    \mathit{dim}(\bm{\hat{w}}^{(l)}) &&& \cr
    \mathit{dim}(\bm{\tau}^{(l)}) &&& \cr
  }
  \in \mathbb{R}^{\mathit{dim}(\bm{q}^{(l)}_{\mathit{var}}) \times \mathit{dim}(\bm{q}^{(l)}_{\mathit{var}})}
  \\
  \bm{\bar{W}}_{\mathit{adj}}
  &:=&
  \begin{pmatrix}
    \bm{\bar{I}}^{(1)}_{\mathit{adj}} & - \bm{\bar{I}}^{(1)}_{\mathit{adj}} & & & & \bm{O}\\
    - \bm{\bar{I}}^{(1)}_{\mathit{adj}} & \bm{\bar{I}}^{(1)}_{\mathit{adj}} + \bm{\bar{I}}^{(2)}_{\mathit{adj}} & - \bm{\bar{I}}^{(2)}_{\mathit{adj}} & & & \\
    & &  \ddots & & & \\
    & & & & \bm{\bar{I}}^{(L-2)}_{\mathit{adj}} + \bm{\bar{I}}^{(L-1)}_{\mathit{adj}} & - \bm{\bar{I}}^{(L-1)}_{\mathit{adj}}\\
    \bm{O} & & & & - \bm{\bar{I}}^{(L-1)}_{\mathit{adj}} & \bm{\bar{I}}^{(L-1)}_{\mathit{adj}}
  \end{pmatrix} \in \mathbb{R}^{dim(\bm{q}_{\mathit{var}}) \times dim(\bm{q}_{\mathit{var}})} \\
  \bm{W}_{\mathit{adj}}
  &:=&
  \begin{pmatrix}
    \bm{\bar{W}}_{\mathit{adj}}&\\&\bm{O}
  \end{pmatrix}
\end{eqnarray}

return $\bm{W}_{\mathit{adj}} \in \mathbb{R}^{dim(\bm{q}) \times dim(\bm{q})}$
   "
   (when (or update? (not (send self :get :adjacent-regular-matrix)))
     (send self :put :adjacent-regular-matrix
           (make-matrix _dim-config _dim-config))
     (assert
      (= (1- (length _instant-config-task-list)) (length _adjacent-regular-scale-list))
      (format nil "A length of adjacent regular scale should be the same as a length of instant config task list minus 1
Now the length of adjacent-regular-scale-list: ~a, the length of instant-config-task-list is ~a ~%"
              (length _adjacent-regular-scale-list) (length _instant-config-task-list)))
     (let* ((idx 0)
            (config-idx 0)
            )
       (dolist (config-task _instant-config-task-list)
         (copy-block-matrix (send self :get :adjacent-regular-matrix)
                            (scale-matrix
                             (cond
                              ((= config-idx 0)
                               (elt _adjacent-regular-scale-list config-idx))
                              ((= config-idx (length _adjacent-regular-scale-list))
                               (elt _adjacent-regular-scale-list (1- config-idx)))
                              (t
                               (+ (elt _adjacent-regular-scale-list (1- config-idx))
                                  (elt _adjacent-regular-scale-list config-idx))))
                             (unit-matrix (send config-task :num-variant-joint)))
                            idx idx
                            )
         (unless (= config-idx (1- (length _instant-config-task-list)))
           (copy-block-matrix (send self :get :adjacent-regular-matrix)
                              (scale-matrix (* -1 (elt _adjacent-regular-scale-list config-idx))
                                            (unit-matrix (min (send (elt _instant-config-task-list (1+ config-idx)) :num-variant-joint)
                                                              (send config-task :num-variant-joint))))
                              idx (+ idx (send config-task :dim-variant-config))
                              )
           (copy-block-matrix (send self :get :adjacent-regular-matrix)
                              (scale-matrix (* -1 (elt _adjacent-regular-scale-list config-idx))
                                            (unit-matrix (min (send (elt _instant-config-task-list (1+ config-idx)) :num-variant-joint)
                                                              (send config-task :num-variant-joint))))
                              (+ idx (send config-task :dim-variant-config)) idx
                              )
           )
         (incf idx (send config-task :dim-variant-config))
         (incf config-idx 1)
         )
       )
     )
   (send self :get :adjacent-regular-matrix)
   )
  (:adjacent-regular-vector
   (&key (update? t))
   "
\begin{eqnarray}
  \bm{v}_{\mathit{adj}}
  &:=&
  \bm{W}_{\mathit{adj}} \bm{q}
\end{eqnarray}

return $\bm{v}_{\mathit{adj}} \in \mathbb{R}^{dim(\bm{q})}$
   "
   (when (or update? (not (send self :get :adjacent-regular-vector)))
     (send self :put :adjacent-regular-vector
           (transform (send self :adjacent-regular-matrix) (send self :config-vector)))
     )
   (send self :get :adjacent-regular-vector)
   )
  (:torque-regular-matrix
   (&key (update? nil))
   "
\begin{eqnarray}
  \bm{\bar{W}}_{trq} &:=&
  \begin{pmatrix}
    \bm{W}_{trq}^{(1)} & & & \bm{O} \\
    & \bm{W}_{trq}^{(2)} & & \\
    & & \ddots & \\
    \bm{O} & & & \bm{W}_{trq}^{(L)} \\
  \end{pmatrix}
  \in \mathbb{R}^{dim(\bm{q}_{\mathit{var}}) \times dim(\bm{q}_{\mathit{var}})} \\
  \bm{W}_{trq} &:=&
  \begin{pmatrix}
    \bm{\bar{W}}_{trq} & \\
    & \bm{O}
  \end{pmatrix}
\end{eqnarray}

return $\bm{W}_{trq} \in \mathbb{R}^{dim(\bm{q}) \times dim(\bm{q})}$
   "
   (when (or update? (not (send self :get :torque-regular-matrix)))
     (send self :put :torque-regular-matrix
           (make-matrix _dim-config _dim-config))
     (let* ((idx 0)
            )
       (dolist (config-task _instant-config-task-list)
         (copy-block-matrix (send self :get :torque-regular-matrix)
                            (send config-task :torque-regular-matrix :update? update? :only-variant? t)
                            idx idx
                            )
         (incf idx (send config-task :dim-variant-config))
         ))
     )
   (send self :get :torque-regular-matrix)
   )
  (:torque-regular-vector
   (&key (update? t))
   "
\begin{eqnarray}
  \bm{\bar{v}}_{trq} &:=&
  \begin{pmatrix}
    \bm{v}_{trq}^{(1)} \\
    \bm{v}_{trq}^{(2)} \\
    \vdots \\
    \bm{v}_{trq}^{(L)}
  \end{pmatrix}
  \in \mathbb{R}^{dim(\bm{q}_{\mathit{var}})} \\
  \bm{v}_{trq} &:=&
  \begin{pmatrix}
    \bm{\bar{v}}_{trq} \\
    \bm{0}
  \end{pmatrix}
\end{eqnarray}

return $\bm{v}_{trq} \in \mathbb{R}^{dim(\bm{q})}$
   "
   (when (or update? (not (send self :get :torque-regular-vector)))
     (send self :put :torque-regular-vector
           (make-array _dim-config :element-type :float :initial-element 0))
     (let* ((idx 0)
            )
       (dolist (config-task _instant-config-task-list)
         (copy-block-vec (send self :get :torque-regular-vector)
                         (send config-task :torque-regular-vector :update? update? :only-variant? t)
                         idx
                         )
         (incf idx (send config-task :dim-variant-config))
         ))
     )
   (send self :get :torque-regular-vector)
   )
  (:regular-matrix
   ()
   "
\begin{eqnarray}
  \bm{W}_{\mathit{reg}} := \min(k_{\mathit{max}}, \|\bm{e}\|^2+k_{\mathit{off}}) \bm{I} + \bm{W}_{\mathit{adj}} + k_{\mathit{trq}} \bm{W}_{trq}
\end{eqnarray}

return $\bm{W}_{\mathit{reg}} \in \mathbb{R}^{\mathit{dim}(\bm{q}) \times \mathit{dim}(\bm{q})}$
   "
   (m+
    (m+
     (scale-matrix (min _norm-regular-scale-max (+ (norm2 (send self :task-value :update? nil)) _norm-regular-scale-offset))
                   (send self :_dim-config-unit-matrix))
     (send self :adjacent-regular-matrix)
     (send self :_dim-config-buf-matrix)
     )
    (scale-matrix _torque-regular-scale (send self :torque-regular-matrix))
    (send self :_dim-config-buf-matrix)
    )
   )
  (:regular-vector
   ()
   "
\begin{eqnarray}
  \bm{v}_{\mathit{reg}} := \bm{v}_{\mathit{adj}} + k_{\mathit{trq}} \bm{v}_{trq}
\end{eqnarray}

return $\bm{v}_{\mathit{reg}}\in \mathbb{R}^{\mathit{dim}(\bm{q})}$
   "
   (v+
    (send self :adjacent-regular-vector)
    (scale _torque-regular-scale (send self :torque-regular-vector)))
   )
  )

(defmethod trajectory-configuration-task
  (:update-viewer
   ()
   "
Update viewer.
   "
   (send-all _instant-config-task-list :update-viewer)
   )
  (:print-status
   ()
   "
Print status.
   "
   (warning-message 9 "|task-all|=~a~%" (norm (send self :task-value :update? nil)))
   (dotimes (i _num-instant-config-task)
     (warning-message 9 "== config ~a [~a] ==~%" i (send (elt _instant-config-task-list i) :name))
     (send (elt _instant-config-task-list i) :print-status)
     )
   )
  (:play-animation
   (&key
    (robot-env)
    (loop? t)
    (visualize-callback-func)
    (visualize-force? nil)
    (force-color #f(0.8 0.2 0.2))
    )
   "
Play motion.
   "
   (when (= (length _instant-config-task-list) 1)
     (send robot-env :angle-vector
           (send (send (car _instant-config-task-list) :robot-env) :angle-vector))
     (send *irtviewer* :draw-objects)
     (return-from :play-animation nil)
     )
   (let* ((reverse? nil)
          )
     (do-until-key
      (dotimes (i (1- (length _instant-config-task-list)))
        (let* ((instant-config-task-current
                (elt (if reverse? (reverse _instant-config-task-list) _instant-config-task-list) i))
               (instant-config-task-next
                (elt (if reverse? (reverse _instant-config-task-list) _instant-config-task-list) (1+ i)))
               (name-set)
               (current-name-list)
               (next-name-list)
               (current-coords-idx)
               (next-coords-idx)
               )
          (dotimes (j 10)
            (send robot-env :angle-vector
                  (midpoint (/ (float (1+ j)) 10)
                            (send (send instant-config-task-current :robot-env) :angle-vector)
                            (send (send instant-config-task-next :robot-env) :angle-vector)))
            (send *irtviewer* :draw-objects :flush nil)
            (when visualize-force?
              ;; draw force and moment of the robot
              (setq name-set
                    (intersection
                     (setq current-name-list (send-all (send instant-config-task-current :contact-attention-coords-list) :name))
                     (setq next-name-list (send-all (send instant-config-task-next :contact-attention-coords-list) :name))))
              (dolist (name name-set)
                (setq current-coords-idx
                      (concatenate cons current-coords-idx
                                   (list (position-if #'(lambda (nm) (string= nm name)) current-name-list))))
                (setq next-coords-idx
                      (concatenate cons next-coords-idx
                                   (list (position-if #'(lambda (nm) (string= nm name)) next-name-list)))))
              (draw-force-value
               :force-list
               (mapcar
                #'(lambda (vec1 vec2) (midpoint (/ (float (1+ j)) 10) vec1 vec2))
                (mapcar #'(lambda (idx) (elt (send instant-config-task-current :force-list) idx))
                        current-coords-idx)
                (mapcar #'(lambda (idx) (elt (send instant-config-task-next :force-list) idx))
                        next-coords-idx)
                )
               :moment-list
               (mapcar
                #'(lambda (vec1 vec2) (midpoint (/ (float (1+ j)) 10) vec1 vec2))
                (mapcar #'(lambda (idx) (elt (send instant-config-task-current :moment-list) idx))
                        current-coords-idx)
                (mapcar #'(lambda (idx) (elt (send instant-config-task-next :moment-list) idx))
                        next-coords-idx)
                )
               :coords-list
               (mapcar
                #'(lambda (coords1 coords2) (midcoords (/ (float (1+ j)) 10) coords1 coords2))
                (mapcar #'(lambda (idx) (elt (send instant-config-task-current :contact-target-coords-list) idx))
                        current-coords-idx)
                (mapcar #'(lambda (idx) (elt (send instant-config-task-next :contact-target-coords-list) idx))
                        next-coords-idx))
               :force-color force-color
               :draw-cop? nil
               ))
            (x::window-main-one)
            (send *irtviewer* :viewer :flush)
            (unix::usleep (* 40 1000))
            (when visualize-callback-func
              (funcall visualize-callback-func))
            )
          ))
      (unless loop?
        (return-from nil nil))
      (unix::usleep (* 1000 1000))
      (setq reverse? (not reverse?))
      ))
   )
  (:generate-robot-state-list
   (&key
    (robot-env)
    (joint-name-list
     (send-all (send robot-env :robot :joint-list) :name))
    (root-link-name
     (send (car (send robot-env :robot :links)) :name))
    (step-time 0.004) ;; [sec]
    (divide-num 100)
    (limb-list
     (list :rleg :lleg :rarm :larm))
    )
   "
Generate and return robot state list.
   "
   (let* ((rs-list)
          (instant-config-task)
          (instant-config-task-next)
          (contact-map)
          (contact-map-next)
          (av-current)
          (av-next)
          (wl-current)
          (wl-next)
          (tm 0.0)
          (av)
          (rc)
          (zmp)
          (wl)
          (cs)
          )
     (dotimes (i (1- (length _instant-config-task-list)))
       (setq instant-config-task
             (elt _instant-config-task-list i))
       (setq instant-config-task-next
             (elt _instant-config-task-list (1+ i)))
       (setq av-current
             (send (send instant-config-task :robot-env) :angle-vector))
       (setq av-next
             (send (send instant-config-task-next :robot-env) :angle-vector))
       ;; (= (elt contact-map i) j) means that (elt limb-list i) corresponds to (elt contact-attention-coords-list j)
       (setq contact-map
             (mapcar
              #'(lambda (limb)
                  (send instant-config-task :_get-contact-index-from-limb :limb limb))
              limb-list
              ))
       (setq contact-map-next
             (mapcar
              #'(lambda (limb)
                  (send instant-config-task-next :_get-contact-index-from-limb :limb limb))
              limb-list
              ))
       (setq wl-current
             (mapcar
              #'(lambda (cm)
                  (if (numberp cm)
                      (elt (send instant-config-task :wrench-list) cm)
                    (float-vector 0 0 0 0 0 0)
                    ))
              contact-map
              ))
       (setq wl-next
             (mapcar
              #'(lambda (cm)
                  (if (numberp cm)
                      (elt (send instant-config-task-next :wrench-list) cm)
                    (float-vector 0 0 0 0 0 0)
                    ))
              contact-map-next
              ))
       (setq cs
             (mapcar
              #'(lambda (cm) (if (numberp cm) :support :swing))
              contact-map
              ))
       (dotimes (j (if (= i (- (length _instant-config-task-list) 2)) (1+ divide-num) divide-num))
         (send robot-env :angle-vector (midpoint (/ (float j) divide-num) av-current av-next))
         (setq av
               (map float-vector
                    #'(lambda (joint-name) (send (send robot-env :robot :joint joint-name) :joint-angle))
                    joint-name-list))
         (setq rc
               (send (send robot-env :robot :link root-link-name) :copy-worldcoords))
         (setq wl
               (mapcar
                #'(lambda (w-current w-next) (midpoint (/ (float j) divide-num) w-current w-next))
                wl-current wl-next))
         (setq zmp
               (scale
                (/ 1.0 ;; denominator of zmp
                   (apply
                    #'+
                    (remove
                     nil
                     (mapcar
                      #'(lambda (cm cm-next w)
                          (if (or (numberp cm) (numberp cm-next)) (elt w 2)))
                      contact-map
                      contact-map-next
                      wl
                      ))))
                (reduce ;; numerator of zmp
                 #'v+
                 (remove
                  nil
                  (mapcar
                   #'(lambda (cm cm-next w)
                       (if (or (numberp cm) (numberp cm-next))
                           (let* ((fx (elt w 0))
                                  (fy (elt w 1))
                                  (fz (elt w 2))
                                  (nx (elt w 3))
                                  (ny (elt w 4))
                                  (nz (elt w 5))
                                  (trg-coords ;; actually need to interpolate target-coords
                                   (cond ((numberp cm)
                                          (elt (send instant-config-task :contact-target-coords-list) cm)
                                          )
                                         ((numberp cm-next)
                                          (elt (send instant-config-task-next :contact-target-coords-list) cm-next)
                                          )))
                                  (px (elt (send trg-coords :worldpos) 0))
                                  (py (elt (send trg-coords :worldpos) 1))
                                  (pz (elt (send trg-coords :worldpos) 2))
                                  (zmp-z 0)
                                  )
                             (float-vector
                              (+ (- ny) (* -1 (- pz zmp-z) fx) (* px fz))
                              (+ nx (* -1 (- pz zmp-z) fy) (* py fz))
                              zmp-z)
                             )))
                   contact-map
                   contact-map-next
                   wl
                   ))))
               )
         (push-back
          (list :time tm
                :angle-vector av
                :root-coords rc
                :zmp zmp
                :wrench-list ;; order of limb-list
                (list (mapcar #'(lambda (w) (subseq w 0 3)) wl)
                      (mapcar #'(lambda (w) (subseq w 3 6)) wl))
                :contact-state cs ;; order of limb-list
                )
          rs-list)
         (setq tm (+ tm step-time))
         ))
     rs-list
     ))
  )
