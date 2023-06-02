(load "./util.l")
(load "./smooth-kinematics.l")


;;;;;;;;;;;;;;;;;;;;;;;;;
;; surface-configuration-task
;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass surface-configuration-task
  :super propertied-object
  :slots ((_body "body")
          (_surface-coords "cascaded-coords of surface point $\bm{p} \in \mathbb{R}^{3}$ [mm]")
          (_pos-normal-tangent "$\bm{p}, \bm{\xi}, (\bm{\zeta}, \bm{\eta}) \in \mathbb{R}^{3}$ surface position, normal, and tangents")
          ;; dimensions
          (_dim-u "$\mathit{dim}(\bm{u}) := 2$")
          (_dim-config "$\mathit{dim}(\bm{q})$")
          (_dim-task "$\mathit{dim}(\bm{e})$")
          ;; scale
          (_normal-task-scale "$k_{\mathit{nrm}}$")
          (_norm-regular-scale-max "$k_{\mathit{max}}$")
          (_norm-regular-scale-offset "$k_{\mathit{off}}$")
          ;; variables
          (_kin-target-coords "$\mathcal{T}^{\mathit{kin\mathchar`-trg}}$")
          (_delta-u "$\Delta \bm{u}_{\mathit{limit}} \in \mathbb{R}^{2}$")
          (_flip-normal? "whether to flip the normal or not")
          )
  :documentation
  "
ボディ表面上の着目点を扱うためのコンフィギュレーションとタスク関数のクラス．

コンフィギュレーション$\bm{q}$の取得・更新，タスク関数$\bm{e}(\bm{q})$の取得，タスク関数のヤコビ行列$\frac{\partial \bm{e}(\bm{q})}{\partial \bm{q}}$の取得，コンフィギュレーションの等式・不等式制約$\bm{A}, \bm{b}, \bm{C}, \bm{d}$の取得のためのメソッドが定義されている．

ボディ表面の着目点の位置$\bm{p} \in \mathbb{R}^{3}$は，二次元コンフィギュレーション$\bm{q} \in \mathbb{R}^{2}$で表現できる．
しかし，単一の二次元パラメータをボディ表面上の点に網羅的に対応させる写像を得ることは困難である．
したがって，本クラスではコンフィギュレーション$\bm{q}$を明示的に保持することはしない．
代わりに，保持している物体表面上の点$\bm{p} \in \mathbb{R}^{3}$の周りに，局所的な座標系$(\bm{\zeta}, \bm{\eta}, \bm{\xi})$を構築し，この座標系で定義された局所二次元コンフィギュレーション$\bm{u}$を構築する．
$\bm{\zeta}, \bm{\eta}$は点$\bm{p}$におけるボディの接ベクトルを表し，$\bm{\xi}$は法線ベクトルを表す．
このコンフィギュレーションは，$\bm{u}=\bm{0}$で$\bm{p}$に対応し，$\bm{u}$の第1,2要素がそれぞれ，接ベクトル$\bm{\zeta}, \bm{\eta} \in \mathbb{R}^{3}$方向に対応するものとする．
  "
  )

(defmethod surface-configuration-task
  (:init
   (&key
    (name)
    ((:body tmp-body))
    (initial-surface-pos)
    (kin-target-coords)
    (delta-u 0.02)
    (flip-normal? nil)
    (normal-task-scale 1)
    (norm-regular-scale-max 1e10)
    (norm-regular-scale-offset 1e0)
    &allow-other-keys
    )
   "
Initialize instance
   "
   (send self :name name)
   (setq _body tmp-body)
   (send self :_initialize-surface-pos initial-surface-pos)
   (setq _kin-target-coords kin-target-coords)
   (setq _delta-u delta-u)
   (setq _flip-normal? flip-normal?)
   (setq _normal-task-scale normal-task-scale)
   (setq _norm-regular-scale-max norm-regular-scale-max)
   (setq _norm-regular-scale-offset norm-regular-scale-offset)
   ;; sub methods
   (send self :_initialize-dimension)
   (send self :_initialize-btmodel)
   )
  (:_initialize-surface-pos
   (initial-surface-pos)
   (setq _surface-coords
         (make-cascoords :pos (send _body :project initial-surface-pos) :parent _body))
   (setq _pos-normal-tangent
         (append (list (send self :surface-pos))
                 (send _body :calc-normal-tangent (send self :surface-pos) :flip-normal? _flip-normal?)))
   )
  (:_initialize-dimension
   ()
   (setq _dim-u 2)
   (setq _dim-config _dim-u)
   (setq _dim-task 4)
   )
  (:_initialize-btmodel
   ()
   (when (member (send _body :methods) :make-btmodel)
     (send _body :make-btmodel)
     (geo::_bt-set-margin (send _body :get :btmodel) -0.04)
     )
   )
  (:body
   ()
   "
return body instance
   "
   _body
   )

  (:dim-config
   ()
   "
return $\mathit{dim}(\bm{q}) := 2$
   "
   _dim-config
   )
  (:dim-task
   ()
   "
return $\mathit{dim}(\bm{e}) := 3$
   "
   _dim-task
   )
  (:config-vector
   ()
   "
Not supported because this class does not have configuration $\bm{q}$ explicitly.

return $\bm{q}$
   "
   (warning-message 1 ":config-vector methods is not supported in surface-configuration-task.~%")
   (float-vector 0 0)
   )
  (:set-config
   (delta-config
    &key
    (relative? t)
    )
   "
Set $\bm{q}$.

The surface point $\bm{p}$ is updated from $\bm{q}$.
   "
   (unless relative?
     (warning-message 1 ":set-config with :relative? nil is not supported.~%"))
   (send self :_update-surface-pos-from-u delta-config)
   )
  (:_update-surface-pos-from-u
   (delta-config
    &aux
    (tangent (elt _pos-normal-tangent 2))
    )
   (send self :put :surface-pos-before-projection
         (reduce
          #'v+
          (list ;; [mm]
           (send self :surface-pos)
           (scale (* 1e3 (elt delta-config 0)) (elt tangent 0))
           (scale (* 1e3 (elt delta-config 1)) (elt tangent 1))
           )))
   (when _surface-coords
     (send _body :dissoc _surface-coords))
   (setq _surface-coords
         (make-cascoords
          :pos
          (send _body :project (send self :get :surface-pos-before-projection))
          ;; (send _body :project (send (send self :kin-target-coords) :worldpos)) ;; for compare with baseline
          :parent _body))
   (setq _pos-normal-tangent
         (append (list (send self :surface-pos))
                 (send _body :calc-normal-tangent (send self :surface-pos) :flip-normal? _flip-normal?)))
   )
  (:surface-pos
   ()
   "
get surface point

return $\bm{p} \in \mathbb{R}^{3}$
   "
   (send _surface-coords :worldpos)
   )
  (:surface-coords
   (&key
    (surface-pos (elt _pos-normal-tangent 0))
    (surface-normal (elt _pos-normal-tangent 1))
    )
   "
get surface coords. z-axis is parallel with surface normal.
   "
   (orient-coords-to-axis (make-coords :pos surface-pos) surface-normal)
   )
  (:kin-target-coords
   ()
   "
 get kinematics target coords.
   "
   (cond ((functionp _kin-target-coords)
          (funcall _kin-target-coords)
          )
         (t
          _kin-target-coords
          ))
   )
  )

(defmethod surface-configuration-task
  (:position-task-value
   (&key (update? t))
   "
\begin{eqnarray}
  \bm{e}^{\mathit{pos}}(\bm{q}) &=& \bm{p}^{\mathit{kin\mathchar`-trg}} - \bm{p}(\bm{u})
\end{eqnarray}

return $\bm{e}^{\mathit{pos}}(\bm{q}) \in \mathbb{R}^{3}$
   "
   (when (or update? (not (send self :get :position-task-value)))
     (send self :put :position-task-value
           (scale 1e-3 (v- (send (send self :kin-target-coords) :worldpos) (send self :surface-pos)))) ;; [m]
     )
   (send self :get :position-task-value)
   )
  (:normal-task-value
   (&key (update? t))
   "
\begin{eqnarray}
  \bm{e}^{\mathit{nrm}}(\bm{q}) &=& 1 - \bm{\xi}^{\mathit{kin\mathchar`-trg},T} \bm{\xi}(\bm{u})
\end{eqnarray}
$\bm{\xi}^{\mathit{kin\mathchar`-trg}}, \bm{\xi}(\bm{u})$はそれぞれ，目標点，着目点の法線ベクトルを表す．

return $\bm{e}^{\mathit{nrm}}(\bm{q}) \in \mathbb{R}$
   "
   (when (or update? (not (send self :get :normal-task-value)))
     (send self :put :normal-task-value
           (float-vector
            (- 1.0
               (v. (send (send self :kin-target-coords) :rotate-vector (float-vector 0 0 1))
                   (elt _pos-normal-tangent 1))
               )))
   )
   (send self :get :normal-task-value)
   )
  (:task-value
   (&key (update? t))
   "
return $\bm{e}(\bm{q}) := \begin{pmatrix} \bm{e}^{\mathit{pos}}(\bm{q}) \\ k_{\mathit{nrm}} \bm{e}^{\mathit{nrm}}(\bm{q}) \end{pmatrix} \in \mathbb{R}^{4}$
   "
   (concatenate
    float-vector
    (send self :position-task-value :update? update?)
    (scale _normal-task-scale (send self :normal-task-value :update? update?))
    )
   )
  )

(defmethod surface-configuration-task
  (:position-task-jacobian-with-u
   ()
   "
\begin{eqnarray}
  \frac{\partial \bm{e}^{\mathit{pos}}}{\partial \bm{u}} &=& - \begin{pmatrix} \bm{\zeta} & \bm{\eta} \end{pmatrix}
\end{eqnarray}

return $\frac{\partial \bm{e}^{\mathit{pos}}}{\partial \bm{u}} \in \mathbb{R}^{3 \times 2}$
   "
   (scale-matrix -1 (transpose (matrix (elt (elt _pos-normal-tangent 2) 0) (elt (elt _pos-normal-tangent 2) 1))))
   )
  (:normal-task-jacobian-with-u
   ()
   "
\begin{eqnarray}
  \frac{\partial \bm{e}^{\mathit{nrm}}}{\partial \bm{u}}
  &=&
  - \bm{\xi}^{\mathit{kin\mathchar`-trg},T} \frac{\partial \bm{\xi}(\bm{u})}{\partial \bm{u}}
\end{eqnarray}

return $\frac{\partial \bm{e}^{\mathit{nrm}}}{\partial \bm{u}} \in \mathbb{R}^{1 \times 2}$
   "
   (let* ((target-norm
           (send (send self :kin-target-coords) :rotate-vector (float-vector 0 0 1)))
          )
     (scale-matrix
      -1
      (m*
       (matrix target-norm)
       (send self :_normal-jacobian-by-finite-difference)))
     ))
  (:_normal-jacobian-by-finite-difference
   (&aux
    (normal (elt _pos-normal-tangent 1))
    (tangent (elt _pos-normal-tangent 2))
    (eps 1e-2) ;; [m]
    )
   "
return $\frac{\partial \bm{\xi}(\bm{u})}{\partial \bm{u}} \in \mathbb{R}^{3 \times 2}$
   "
   (labels ((calc-normal-from-u
             (u)
             (elt
              (send _body :calc-normal-tangent
                    (send _body :project
                          (reduce
                           #'v+
                           (list
                            (send self :surface-pos)
                            (scale (elt u 0) (elt tangent 0))
                            (scale (elt u 1) (elt tangent 1))
                            )))
                    :flip-normal? _flip-normal?
                    )
              0))
            )
     (transpose
      (matrix
       (scale (/ 1.0 (* 2 eps))
              (v- (calc-normal-from-u (float-vector eps 0)) (calc-normal-from-u (float-vector (- eps) 0))))
       (scale (/ 1.0 (* 2 eps))
              (v- (calc-normal-from-u (float-vector 0 eps)) (calc-normal-from-u (float-vector 0 (- eps)))))
       ))
     ))
  (:task-jacobian
   ()
   "
\begin{eqnarray}
  \frac{\partial \bm{e}}{\partial \bm{q}} &=&
  \begin{pmatrix}
  \frac{\partial \bm{e}^{\mathit{pos}}}{\partial \bm{u}} \\ k_{\mathit{nrm}} \frac{\partial \bm{e}^{\mathit{nrm}}}{\partial \bm{u}}
  \end{pmatrix}
\end{eqnarray}

return $\frac{\partial \bm{e}}{\partial \bm{q}} \in \mathbb{R}^{4 \times 2}$
   "
   (concatenate-matrix-column
    (send self :position-task-jacobian-with-u)
    (scale-matrix _normal-task-scale (send self :normal-task-jacobian-with-u))
    )
   )
  )

(defmethod surface-configuration-task
  (:delta-u-limit-vector
   (&key
    (update? nil)
    (delta-u1 _delta-u) ;; [m]
    (delta-u2 _delta-u) ;; [m]
    )
   "
get trust region of $\bm{u}$

return $\Delta \bm{u}_{\mathit{limit}} \in \mathbb{R}^{2}$
   "
   (when (or update? (not (send self :get :delta-u-limit-vector)))
     (send self :put :delta-u-limit-vector
           (float-vector delta-u1 delta-u2))
     )
   (send self :get :delta-u-limit-vector)
   )
  (:u-inequality-constraint-matrix
   (&key (update? nil))
   "
\begin{eqnarray}
&& - \Delta \bm{u}_{\mathit{limit}} \leq \Delta \bm{u} \leq \Delta \bm{u}_{\mathit{limit}} \\
\Leftrightarrow&&\begin{pmatrix} \bm{I} \\ - \bm{I}\end{pmatrix} \Delta \bm{u}
\geq \begin{pmatrix} - \Delta \bm{u}_{\mathit{limit}} \\ - \Delta \bm{u}_{\mathit{limit}} \end{pmatrix} \\
\Leftrightarrow&&\bm{C_{u}} \Delta \bm{u} \geq \bm{d_{u}}
\end{eqnarray}

return $\bm{C_{u}} := \begin{pmatrix} \bm{I} \\ - \bm{I}\end{pmatrix} \in \mathbb{R}^{4 \times 2}$
   "
   (when (or update? (not (send self :get :u-inequality-constraint-matrix)))
     (send self :put :u-inequality-constraint-matrix
           (concatenate-matrix-column
            (unit-matrix _dim-u)
            (scale-matrix -1 (unit-matrix _dim-u)))
           )
     )
   (send self :get :u-inequality-constraint-matrix)
   )
  (:u-inequality-constraint-vector
   (&key (update? t))
   "
return $\bm{d_{u}} := \begin{pmatrix} - \Delta \bm{u}_{\mathit{limit}} \\ - \Delta \bm{u}_{\mathit{limit}} \end{pmatrix} \in \mathbb{R}^{4}$
   "
   (when (or update? (not (send self :get :u-inequality-constraint-vector)))
     (send self :put :u-inequality-constraint-vector
           (concatenate
            float-vector
            (scale -1 (send self :delta-u-limit-vector))
            (scale -1 (send self :delta-u-limit-vector)))
           )
     )
   (send self :get :u-inequality-constraint-vector)
   )
  (:config-inequality-constraint-matrix
   (&key (update? nil) (update-collision? nil))
   "
\begin{eqnarray}
  &&
  \bm{C} \Delta \bm{q} \geq \bm{d}
\end{eqnarray}

return $\bm{C} := \bm{C_u}$
   "
   (send self :u-inequality-constraint-matrix :update? update?)
   )
  (:config-inequality-constraint-vector
   (&key (update? t) (update-collision? nil))
   "
return $\bm{d} := \bm{d_u}$
   "
   (send self :u-inequality-constraint-vector :update? update?)
   )
  )

(defmethod surface-configuration-task
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

(defmethod surface-configuration-task
  (:regular-matrix
   ()
   "
\begin{eqnarray}
  \bm{W}_{\mathit{reg}} := \min(k_{\mathit{max}}, \|\bm{e}\|^2+k_{\mathit{off}}) \bm{I}
\end{eqnarray}

return $\bm{W}_{\mathit{reg}} \in \mathbb{R}^{\mathit{dim}(\bm{q}) \times \mathit{dim}(\bm{q})}$
   "
   (scale-matrix (min _norm-regular-scale-max (+ (norm2 (send self :task-value :update? nil)) _norm-regular-scale-offset))
                 (unit-matrix _dim-config))
   )
  (:regular-vector
   ()
   "
\begin{eqnarray}
  \bm{v}_{\mathit{reg}} := \bm{0}
\end{eqnarray}

return $\bm{v}_{\mathit{reg}} \in \mathbb{R}^{\mathit{dim}(\bm{q})}$
   "
   (make-array _dim-config :element-type :float)
   )
  )

(defmethod surface-configuration-task
  (:distance-vector
   (config-task)
   "
\begin{eqnarray}
  \bm{d} := \bm{p}(\bm{u}) - \bm{p}_{\mathit{other}}(\bm{u}_{\mathit{other}})
\end{eqnarray}
Distance $\bm{d}$ is used for sqp-msc (sqp with multiple solution candidates).

return $\bm{d} \in \mathbb{R}^3$
   "
   (v- (send self :surface-pos) (send config-task :surface-pos))
   )
  (:distance-jacobian
   ()
   "
\begin{eqnarray}
  \bm{J}_{d} := \frac{\partial \bm{d}}{\partial \bm{u}} = \begin{pmatrix} \bm{\zeta} & \bm{\eta} \end{pmatrix}
\end{eqnarray}
Distance jacobian $\bm{J}_{d}$ is used for sqp-msc (sqp with multiple solution candidates).

return $\bm{J}_{d} \in \mathbb{R}^{3 \times 2}$
   "
   (transpose (matrix (elt (elt _pos-normal-tangent 2) 0) (elt (elt _pos-normal-tangent 2) 1)))
   )
  )

(defmethod surface-configuration-task
  (:update-viewer
   ()
   "
Update viewer.
   "
   (send (send self :kin-target-coords) :draw-on :flush nil :color #f(0 1 0) :size 50 :width 10)
   (send (send self :surface-pos) :draw-on :flush nil :color #f(1 0 0) :size 100 :width 4)
   (when (send self :get :surface-pos-before-projection)
     (send (send self :get :surface-pos-before-projection) :draw-on :flush nil :color #f(1.0 0.5 0.2) :size 10 :width 4))
   ;; tangent
   (when _pos-normal-tangent
     (let* ((tangent-origin (elt _pos-normal-tangent 0))
            (normal (elt _pos-normal-tangent 1))
            (tangent (elt _pos-normal-tangent 2))
            )
       (send tangent-origin :draw-on :flush nil :color #f(0 1 1) :size 10 :width 10)
       (send _body :draw-on :flush nil :color #f(0.5 0 0) :width 1)
       (with-set-color-line-width
        *viewer* #f(0.8 0.3 0.5) 4
        (send *irtviewer* :viewer :draw-arrow
              tangent-origin
              (v+ tangent-origin (scale 50 normal))
              nil))
       (with-set-color-line-width
        *viewer* #f(0.5 0.8 0.3) 4
        (send *irtviewer* :viewer :draw-arrow
              tangent-origin
              (v+ tangent-origin (scale 50 (elt tangent 0)))
              nil)
        (send *irtviewer* :viewer :draw-arrow
              tangent-origin
              (v+ tangent-origin (scale 50 (elt tangent 1)))
              nil)
        )
       ))
   )
  (:print-status
   ()
   "
Print status.
   "
   (when (= _dim-task 0)
     (return-from :print-status nil))
   (warning-message 9 "|task|=~a~%" (norm (send self :task-value :update? nil)))
   (warning-message 9 "  |pos-task|=~a:~%" (norm (send self :position-task-value :update? nil)))
   (warning-message 9 "  |nrm-task|=~a:~%" (norm (send self :normal-task-value :update? nil)))
   )
  )
