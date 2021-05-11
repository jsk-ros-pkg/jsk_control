(load "./instant-configuration-task.l")
(load "./robot-object-environment.l")

(unless (boundp 'instant-configuration-task) ;; dummy class definition for document
  (defclass instant-configuration-task :super object))


;;;;;;;;;;;;;;;;;;;;;;;;;
;; instant-manipulation-configuration-task
;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass instant-manipulation-configuration-task
  :super instant-configuration-task
  :slots ((_robot-obj-env "robot-object-environment instance")
          ;; configuration vector
          (_wrench-obj-vector "$\bm{\hat{w}}_{\mathit{obj}}$ [N] [Nm]")
          ;; number
          (_num-contact-obj "$N_{\mathit{cnt\mathchar`-obj}} := |\mathcal{T}^{\mathit{cnt\mathchar`-trg\mathchar`-obj}}|$")
          (_num-act-react "$N_{\mathit{act\mathchar`-react}} := |\mathcal{P}^{\mathit{act\mathchar`-react}}|$")
          ;; dimensions
          (_dim-wrench-obj "$\mathit{dim}(\bm{\hat{w}}_{\mathit{obj}}) = 6 N_{\mathit{cnt\mathchar`-obj}}$")
          ;; variables
          (_contact-target-coords-obj-list "$\mathcal{T}^{\mathit{cnt\mathchar`-trg\mathchar`-obj}}$")
          (_contact-constraint-obj-list "list of contact-constraint instance for object")
          (_act-react-pair-list "$\mathcal{P}^{\mathit{act\mathchar`-react}}$")
          )
  :documentation
  "
マニピュレーションにける瞬時コンフィギュレーション$\bm{q}^{(l)}$と瞬時タスク関数$\bm{e}^{(l)}(\bm{q}^{(l)})$のクラス．
マニピュレーション対象の物体の瞬時コンフィギュレーションや瞬時タスク関数を含む．

このクラスの説明で用いる全ての変数は，時間ステップ$l$を表す添字をつけて$\bm{x}^{(l)}$と表されるべきだが，このクラス内の説明では省略して$\bm{x}$と表す．
また，以降では，説明文やメソッド名で，``瞬時''や``instant''を省略する．

コンフィギュレーション$\bm{q}$の取得・更新，タスク関数$\bm{e}(\bm{q})$の取得，タスク関数のヤコビ行列$\frac{\partial \bm{e}(\bm{q})}{\partial \bm{q}}$の取得，コンフィギュレーションの等式・不等式制約$\bm{A}, \bm{b}, \bm{C}, \bm{d}$の取得のためのメソッドが定義されている．

コンフィギュレーション・タスク関数を定めるために，instant-configuration-taskの設定に加えて，初期化時に以下を与える
\begin{itemize}
\setlength{\itemindent}{10mm}
\setlength{\leftskip}{-10mm}
\item ロボット・物体・環境
\begin{description}
  \item[robot-object-environment] ロボット・物体・環境を表すrobot-object-environmentクラスのインスタンス
\end{description}
\item 物体の接触拘束
\begin{description}
  \item[contact-target-coords-obj-list $\mathcal{T}^{\mathit{cnt\mathchar`-trg\mathchar`-obj}}$] 物体の接触目標位置姿勢リスト
  \item[contact-constraint-obj-list] 物体の接触レンチ制約リスト
\end{description}
\item 作用・反作用の拘束
\begin{description}
  \item[act-react-pair-list $\mathcal{P}^{\mathit{act\mathchar`-react}}$] 作用・反作用の関係にあるロボット・物体の接触目標位置姿勢ペアのリスト
\end{description}
\end{itemize}

コンフィギュレーション$\bm{q}$は以下から構成される．
\begin{description}
  \setlength{\itemindent}{10mm}
  \item[$\bm{\theta} \in \mathbb{R}^{N_\mathit{var\mathchar`-joint}}$] 時変関節角度 [rad] [m]
  \item[$\bm{\hat{w}} \in \mathbb{R}^{6 N_{\mathit{cnt}}}$] ロボットの接触レンチ [N] [Nm]
  \item[$\bm{\hat{w}}_{\mathit{obj}} \in \mathbb{R}^{6 N_{\mathit{cnt\mathchar`-obj}}}$] 物体の接触レンチ [N] [Nm]
  \item[$\bm{\tau} \in \mathbb{R}^{N_\mathit{drive\mathchar`-joint}}$] 関節駆動トルク [Nm] [N]
  \item[$\bm{\phi} \in \mathbb{R}^{N_\mathit{invar\mathchar`-joint}}$] 時不変関節角度 [rad] [m]
\end{description}
$\bm{\hat{w}}$は次式のように，全接触部位でのワールド座標系での力・モーメントを並べたベクトルである．
\begin{eqnarray}
  \bm{\hat{w}} &=& \begin{pmatrix} \bm{w}_1^T & \bm{w}_2^T & \cdots & \bm{w}_{N_{\mathit{cnt}}}^T \end{pmatrix}^T \\
  &=& \begin{pmatrix} \bm{f}_1^T &  \bm{n}_1^T &  \bm{f}_2^T &  \bm{n}_2^T & \cdots & \bm{f}_{N_{\mathit{cnt}}}^T &  \bm{n}_{N_{\mathit{cnt}}}^T \end{pmatrix}^T
\end{eqnarray}

タスク関数$\bm{e}(\bm{q})$は以下から構成される．
\begin{description}
  \setlength{\itemindent}{10mm}
  \item[$\bm{e}^{\mathit{kin}}(\bm{q}) \in \mathbb{R}^{6 N_{\mathit{kin}}}$] 幾何到達拘束 [rad] [m]
  \item[$\bm{e}^{\mathit{eom\mathchar`-trans}}(\bm{q}) \in \mathbb{R}^3$] ロボットの力の釣り合い [N]
  \item[$\bm{e}^{\mathit{eom\mathchar`-rot}}(\bm{q}) \in \mathbb{R}^3$] ロボットのモーメントの釣り合い [Nm]
  \item[$\bm{e}^{\mathit{eom\mathchar`-trans\mathchar`-obj}}(\bm{q}) \in \mathbb{R}^3$] 物体の力の釣り合い [N]
  \item[$\bm{e}^{\mathit{eom\mathchar`-rot\mathchar`-obj}}(\bm{q}) \in \mathbb{R}^3$] 物体のモーメントの釣り合い [Nm]
  \item[$\bm{e}^{\mathit{trq}}(\bm{q}) \in \mathbb{R}^{N_{\mathit{drive\mathchar`-joint}}}$] 関節トルクの釣り合い [rad] [m]
  \item[$\bm{e}^{\mathit{posture}}(\bm{q}) \in \mathbb{R}^{N_{\mathit{posture\mathchar`-joint}}}$] 関節角目標 [rad] [m]
\end{description}

  "
  )

(defmethod instant-manipulation-configuration-task
  (:init
   (&rest
    args
    &key
    (contact-target-coords-obj-list)
    (contact-constraint-obj-list)
    (act-react-pair-list)
    &allow-other-keys
    )
   "
Initialize instance
   "
   (send-super* :init args)
   (setq _robot-obj-env _robot-env) ;; rename variable for easy understanding
   (setq _contact-target-coords-obj-list contact-target-coords-obj-list)
   (setq _contact-constraint-obj-list contact-constraint-obj-list)
   (setq _act-react-pair-list act-react-pair-list)
   ;; sub methods
   (send self :_initialize-number)
   (send self :_initialize-configuration-vector)
   (send self :_initialize-dimension)
   (send self :_contact-constraint-matrix-list)
   (send self :_contact-constraint-obj-matrix-list)
   )
  (:_initialize-number
   ()
   (send-super :_initialize-number)
   (setq _num-contact-obj (length _contact-target-coords-obj-list))
   (setq _num-act-react (length _act-react-pair-list))
   )
  (:_initialize-configuration-vector
   ()
   (send-super :_initialize-configuration-vector)
   (setq _wrench-obj-vector (make-array (* 6 _num-contact-obj) :element-type :float))
   )
  (:_initialize-dimension
   ()
   (send-super :_initialize-dimension)
   (setq _dim-wrench-obj (* 6 _num-contact-obj))
   (setq _dim-variant-config (+ _dim-theta _dim-wrench _dim-wrench-obj _dim-torque))
   (setq _dim-config (+ _dim-variant-config _dim-invariant-config))
   (setq _dim-task (+ (* 6 _num-kin) 3 3 3 3 _num-drive-joint _num-posture-joint))
   )
  (:robot-obj-env
   ()
   "
return robot-object-environment instance
   "
   _robot-obj-env
   )
  (:wrench-obj
   ()
   "
return $\bm{\hat{w}}_{\mathit{obj}}$
   "
   _wrench-obj-vector
   )
  (:num-contact-obj
   ()
   "
return $N_{\mathit{cnt\mathchar`-obj}} := |\mathcal{T}^{\mathit{cnt\mathchar`-trg\mathchar`-obj}}|$
   "
   _num-contact-obj
   )
  (:dim-variant-config
   ()
   "
\begin{eqnarray}
\mathit{dim}(\bm{q_{\mathit{var}}})
&:=& \mathit{dim}(\bm{\theta}) + \mathit{dim}(\bm{\hat{w}}) + \mathit{dim}(\bm{\hat{w}_{\mathit{obj}}}) + \mathit{dim}(\bm{\tau}) \\
&=& N_\mathit{var\mathchar`-joint} + 6 N_{\mathit{cnt}} + 6 N_{\mathit{cnt\mathchar`-obj}} + N_\mathit{drive\mathchar`-joint}
\end{eqnarray}

return $\mathit{dim}(\bm{q_{\mathit{var}}})$

   "
   (send-super :dim-variant-config)
   )
  (:dim-task
   ()
   "
\begin{eqnarray}
\mathit{dim}(\bm{e})
&:=& \mathit{dim}(\bm{e}^{\mathit{kin}}) + \mathit{dim}(\bm{e}^{\mathit{eom\mathchar`-trans}}) + \mathit{dim}(\bm{e}^{\mathit{eom\mathchar`-rot}}) + \mathit{dim}(\bm{e}^{\mathit{eom\mathchar`-trans\mathchar`-obj}}) \nonumber \\ && + \mathit{dim}(\bm{e}^{\mathit{eom\mathchar`-rot\mathchar`-obj}})+ \mathit{dim}(\bm{e}^{\mathit{trq}}) + \mathit{dim}(\bm{e}^{\mathit{posture}}) \\
&=& 6 N_{\mathit{kin}} + 3 + 3 + 3 + 3 + N_{\mathit{drive\mathchar`-joint}} + N_{\mathit{posture\mathchar`-joint}}
\end{eqnarray}

return $\mathit{dim}(\bm{e})$
   "
   (send-super :dim-task)
   )
  (:variant-config-vector
   ()
   "
return $\bm{q_{\mathit{var}}} := \begin{pmatrix} \bm{\theta} \\ \bm{\hat{w}} \\ \bm{\hat{w}_{\mathit{obj}}} \\ \bm{\tau} \end{pmatrix}$
   "
   (concatenate
    float-vector
    _theta-vector
    _wrench-vector
    _wrench-obj-vector
    _torque-vector
    )
   )
  (:config-vector
   ()
   "
return $\bm{q} := \begin{pmatrix} \bm{q_{\mathit{var}}} \\ \bm{q_{\mathit{invar}}} \end{pmatrix} = \begin{pmatrix} \bm{\theta} \\ \bm{\hat{w}} \\ \bm{\hat{w}}_{\mathit{obj}} \\ \bm{\tau} \\ \bm{\phi} \end{pmatrix}$
   "
   (send-super :config-vector)
   )
  (:set-wrench-obj
   (wrench-obj-new
    &key
    (relative? nil)
    )
   "
Set $\bm{\hat{w}}_{\mathit obj}$.
   "
   (cond (relative?
          (v+ _wrench-obj-vector wrench-obj-new _wrench-obj-vector)
          )
         (t
          (setq _wrench-obj-vector wrench-obj-new)
          ))
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
   (send self :set-theta
         (subseq variant-config-new 0 _dim-theta) :relative? relative? :apply-to-robot? apply-to-robot?)
   (send self :set-wrench
         (subseq variant-config-new _dim-theta (+ _dim-theta _dim-wrench)) :relative? relative?)
   (send self :set-wrench-obj
         (subseq variant-config-new (+ _dim-theta _dim-wrench) (+ _dim-theta _dim-wrench _dim-wrench-obj)) :relative? relative?)
   (send self :set-torque
         (subseq variant-config-new (+ _dim-theta _dim-wrench _dim-wrench-obj) (+ _dim-theta _dim-wrench _dim-wrench-obj _dim-torque)) :relative? relative?)
   )
  )

(defmethod instant-manipulation-configuration-task
  (:contact-target-coords-obj-list
   ()
   "
\begin{eqnarray}
  T_m^{\mathit{cnt\mathchar`-trg\mathchar`-obj}} = \{\bm{p}_{m}^{\mathit{cnt\mathchar`-trg\mathchar`-obj}}, \bm{R}_{m}^{\mathit{cnt\mathchar`-trg\mathchar`-obj}}\} \ \ (m = 1,2,\cdots,N_{\mathit{cnt\mathchar`-obj}})
\end{eqnarray}

return $\mathcal{T}^{\mathit{cnt\mathchar`-trg\mathchar`-obj}} := \{T_1^{\mathit{cnt\mathchar`-trg\mathchar`-obj}}, T_2^{\mathit{cnt\mathchar`-trg\mathchar`-obj}}, \cdots, T_{N_\mathit{cnt\mathchar`-obj}}^{\mathit{cnt\mathchar`-trg\mathchar`-obj}} \}$
   "
   _contact-target-coords-obj-list
   )
  (:contact-constraint-obj-list
   ()
   "
return list of contact-constraint instance for object
   "
   _contact-constraint-obj-list
   )
  (:wrench-obj-list
   ()
   "
return $\{ \bm{w}_{\mathit{obj},1}, \bm{w}_{\mathit{obj},2}, \cdots, \bm{w}_{\mathit{obj},{N_{obj}}} \}$
   "
   (divide-vector _wrench-obj-vector :segment-length 6)
   )
  (:force-obj-list
   ()
   "
return $\{ \bm{f}_{\mathit{obj},1}, \bm{f}_{\mathit{obj},2}, \cdots, \bm{f}_{\mathit{obj},{N_{cnt\mathchar`-obj}}} \}$
   "
   (divide-vector _wrench-obj-vector :segment-length 3 :interval-length 3)
   )
  (:moment-obj-list
   ()
   "
return $\{ \bm{n}_{\mathit{obj},1}, \bm{n}_{\mathit{obj},2}, \cdots, \bm{n}_{\mathit{obj},{N_{cnt\mathchar`-obj}}} \}$
   "
   (divide-vector (subseq _wrench-obj-vector 3) :segment-length 3 :interval-length 3)
   )
  (:mg-obj-vec
   ()
   "
return $m_{\mathit{obj}} \bm{g}$
   "
   (or (send self :get :mg-obj-vec)
       (send self :put :mg-obj-vec
             (scale (* 1e-3 (send _robot-obj-env :object :weight)) (scale 1e-3 *g-vec*)))) ;; [N]
   )
  (:cog-obj
   (&key (update? t))
   "
return $\bm{p}_{\mathit{Gobj}}(\bm{q})$
   "
   (scale 1e-3 (send _robot-obj-env :object :centroid update?))
   )
  )

(defmethod instant-manipulation-configuration-task
  (:eom-trans-obj-task-value
   (&key (update? t))
   "
\begin{eqnarray}
  \bm{e}^{\mathit{eom\mathchar`-trans\mathchar`-obj}}(\bm{q})
  &=& \bm{e}^{\mathit{eom\mathchar`-trans\mathchar`-obj}}(\bm{\hat{w}}_{\mathit{obj}}) \\
  &=& \sum_{m=1}^{N_{\mathit{cnt\mathchar`-obj}}} \bm{f}_{\mathit{obj, m}} - m_{\mathit{obj}} \bm{g}
\end{eqnarray}

return $\bm{e}^{\mathit{eom\mathchar`-trans\mathchar`-obj}}(\bm{q}) \in \mathbb{R}^3$
   "
   (when (or update? (not (send self :get :eom-trans-obj-task-value)))
     (send self :put :eom-trans-obj-task-value
           (apply
            #'v++
            (append
             (send self :force-obj-list)
             (list (scale -1 (send self :mg-obj-vec)))
             ))
           ))
   (send self :get :eom-trans-obj-task-value)
   )
  (:eom-rot-obj-task-value
   (&key (update? t))
   "
\begin{eqnarray}
  \bm{e}^{\mathit{eom\mathchar`-rot\mathchar`-obj}}(\bm{q})
  &=& \bm{e}^{\mathit{eom\mathchar`-rot\mathchar`-obj}}(\bm{\theta}, \bm{\hat{w}}_{\mathit{obj}}, \bm{\phi}) \\
  &=& \sum_{m=1}^{N_{\mathit{cnt\mathchar`-obj}}} \left\{ \left(\bm{p}_m^{\mathit{cnt\mathchar`-trg\mathchar`-obj}}(\bm{\theta}, \bm{\phi}) - \bm{p}_{\mathit{Gobj}}(\bm{\theta}, \bm{\phi})\right) \times \bm{f}_{\mathit{obj, m}} + \bm{n}_{\mathit{obj, m}} \right\}
\end{eqnarray}

return $\bm{e}^{\mathit{eom\mathchar`-rot\mathchar`-obj}}(\bm{q}) \in \mathbb{R}^3$
   "
   (when (or update? (not (send self :get :eom-rot-obj-task-value)))
     (send self :put :eom-rot-obj-task-value
           (apply
            #'v++
            (append
             (mapcar
              #'(lambda (f n p) (v+ (v* (v- p (send self :cog-obj)) f) n))
              (send self :force-obj-list)
              (send self :moment-obj-list)
              (mapcar #'(lambda (c) (scale 1e-3 (send c :worldpos))) _contact-target-coords-obj-list) ;; [m]
              ))
            )
           ))
   (send self :get :eom-rot-obj-task-value)
   )
  (:task-value
   (&key (update? t))
   "
return $\bm{e}(\bm{q})
:= \begin{pmatrix} \bm{e}^{\mathit{kin}}(\bm{q}) \\ \bm{e}^{\mathit{eom\mathchar`-trans}}(\bm{q}) \\ \bm{e}^{\mathit{eom\mathchar`-rot}}(\bm{q}) \\ \bm{e}^{\mathit{eom\mathchar`-trans\mathchar`-obj}}(\bm{q}) \\ \bm{e}^{\mathit{eom\mathchar`-rot\mathchar`-obj}}(\bm{q}) \\ \bm{e}^{\mathit{trq}}(\bm{q}) \\ \bm{e}^{\mathit{posture}}(\bm{q}) \end{pmatrix}
= \begin{pmatrix} \bm{e}^{\mathit{kin}}(\bm{\theta}, \bm{\phi}) \\ \bm{e}^{\mathit{eom\mathchar`-trans}}(\bm{\hat{w}}) \\ \bm{e}^{\mathit{eom\mathchar`-rot}}(\bm{\theta}, \bm{\hat{w}}, \bm{\phi}) \\ \bm{e}^{\mathit{eom\mathchar`-trans\mathchar`-obj}}(\bm{\hat{w}}_{\mathit obj}) \\ \bm{e}^{\mathit{eom\mathchar`-rot\mathchar`-obj}}(\bm{\theta}, \bm{\hat{w}}_{\mathit obj}, \bm{\phi}) \\ \bm{e}^{\mathit{trq}}(\bm{\theta}, \bm{\hat{w}}, \bm{\tau}, \bm{\phi}) \\ \bm{e}^{\mathit{posture}}(\bm{\theta}) \end{pmatrix}$
   "
   (concatenate
    float-vector
    (send self :kinematics-task-value :update? update?)
    (send self :eom-trans-task-value :update? update?)
    (send self :eom-rot-task-value :update? update?)
    (send self :eom-trans-obj-task-value :update? update?)
    (send self :eom-rot-obj-task-value :update? update?)
    (if (> _num-drive-joint 0)
        (send self :torque-task-value :update? update?))
    (if (> _num-posture-joint 0)
        (send self :posture-task-value :update? update?))
    )
   )
  )

(defmethod instant-manipulation-configuration-task
  (:eom-trans-obj-task-jacobian-with-wrench-obj
   ()
   "
\begin{eqnarray}
  \frac{\partial \bm{e}^{\mathit{eom\mathchar`-trans\mathchar`-obj}}}{\partial \bm{\hat{w}}_{\mathit{obj}}}
  &=& \begin{pmatrix}
    \frac{\partial \bm{e}^{\mathit{eom\mathchar`-trans\mathchar`-obj}}}{\partial \bm{f}_{\mathit{obj,1}}} & \frac{\partial \bm{e}^{\mathit{eom\mathchar`-trans\mathchar`-obj}}}{\partial \bm{n}_{\mathit{obj,1}}} & \cdots & \frac{\partial \bm{e}^{eom\mathchar`-trans\mathchar`-obj}}{\partial \bm{f}_{\mathit{obj, N_{\mathit{cnt\mathchar`-obj}}}}} & \frac{\partial \bm{e}^{\mathit{eom\mathchar`-trans\mathchar`-obj}}}{\partial \bm{n}_{\mathit{obj, N_{cnt\mathchar`-obj}}}}
  \end{pmatrix} \\
  &=&\begin{pmatrix}
    \bm{I}_3 & \bm{O}_3 & \cdots & \bm{I}_3 & \bm{O}_3
  \end{pmatrix}
\end{eqnarray}

return $\frac{\partial \bm{e}^{\mathit{eom\mathchar`-trans\mathchar`-obj}}}{\partial \bm{\hat{w}}_{\mathit{obj}}} \in \mathbb{R}^{3 \times 6N_{\mathit{cnt\mathchar`-obj}}}$
   "
   (apply #'concatenate-matrix-row
          (flatten (mapcar #'(lambda (x) (list (unit-matrix 3) (make-matrix 3 3))) (make-list _num-contact-obj))))
   )
  (:eom-rot-obj-task-jacobian-with-theta
   ()
   "
\begin{eqnarray}
  \frac{\partial \bm{e}^{\mathit{eom\mathchar`-rot\mathchar`-obj}}}{\partial \bm{\theta}}
  &=& \sum_{m=1}^{N_{\mathit{cnt\mathchar`-obj}}} \left\{ - [\bm{f}_{\mathit{obj, m}} \times] \left( \bm{J}_{\theta,m}^{\mathit{cnt\mathchar`-trg\mathchar`-obj}}(\bm{\theta}, \bm{\phi}) - \bm{J}_{\mathit{Gobj}\theta}(\bm{\theta}, \bm{\phi}) \right) \right\} \\
  &=& \left[\left( \sum_{m=1}^{N_{\mathit{cnt\mathchar`-obj}}} \bm{f}_{\mathit{obj, m}} \right) \times \right] \bm{J}_{\mathit{Gobj}\theta}(\bm{\theta}, \bm{\phi})
  - \sum_{m=1}^{N_{\mathit{cnt\mathchar`-obj}}} [\bm{f}_{\mathit{obj, m}} \times] \bm{J}_{\theta,m}^{\mathit{cnt\mathchar`-trg\mathchar`-obj}}(\bm{\theta}, \bm{\phi})
\end{eqnarray}
$\sum_{m=1}^{N_{\mathit{cnt\mathchar`-obj}}} \bm{f}_{\mathit{obj,m}} = m_{\mathit{obj}} \bm{g}$つまり，eom-trans-obj-taskが成立すると仮定すると次式が成り立つ．
\begin{eqnarray}
  \frac{\partial \bm{e}^{\mathit{eom\mathchar`-rot\mathchar`-obj}}}{\partial \bm{\theta}}
  &=& \left[ m_{\mathit{obj}} \bm{g} \times \right] \bm{J}_{\mathit{Gobj}\theta}(\bm{\theta}, \bm{\phi})
  - \sum_{m=1}^{N_{\mathit{cnt\mathchar`-obj}}} [\bm{f}_{\mathit{obj, m}} \times] \bm{J}_{\theta,m}^{\mathit{cnt\mathchar`-trg\mathchar`-obj}}(\bm{\theta}, \bm{\phi})
\end{eqnarray}

return $\frac{\partial \bm{e}^{\mathit{eom\mathchar`-rot\mathchar`-obj}}}{\partial \bm{\theta}} \in \mathbb{R}^{3 \times N_{\mathit{var\mathchar`-joint}}}$
   "
   (reduce
    #'m+
    (remove
     nil
     (flatten
      (list
       (m* (outer-product-matrix (send self :mg-obj-vec))
           (send _robot-obj-env :object-with-root-virtual :calc-cog-jacobian-from-joint-list
                 :union-joint-list _variant-joint-list
                 :translation-axis t
                 ))
       (mapcar
        #'(lambda (f c)
            (if (derivedp c cascaded-coords)
                (scale-matrix
                 -1
                 (m*
                  (outer-product-matrix f)
                  (send _robot-obj-env :calc-jacobian-from-joint-list
                        :union-joint-list _variant-joint-list
                        :move-target (list c)
                        :transform-coords (list (make-coords))
                        :translation-axis (list t)
                        :rotation-axis (list nil)
                        )
                  ))))
        (send self :force-obj-list)
        _contact-target-coords-obj-list
        )
       ))
     ))
   )
  (:eom-rot-obj-task-jacobian-with-wrench-obj
   ()
   "
\begin{eqnarray}
  \frac{\partial \bm{e}^{\mathit{eom\mathchar`-rot\mathchar`-obj}}}{\partial \bm{\hat{w}_{\mathit{obj}}}}
  &=& \begin{pmatrix}
    \frac{\partial \bm{e}^{\mathit{eom\mathchar`-rot\mathchar`-obj}}}{\partial \bm{f}_{\mathit{obj,1}}} & \frac{\partial \bm{e}^{\mathit{eom\mathchar`-rot\mathchar`-obj}}}{\partial \bm{n}_{\mathit{obj,1}}} & \cdots & \frac{\partial \bm{e}^{\mathit{eom\mathchar`-rot\mathchar`-obj}}}{\partial \bm{f}_{\mathit{obj,N_{cnt\mathchar`-obj}}}} & \frac{\partial \bm{e}^{\mathit{eom\mathchar`-rot\mathchar`-obj}}}{\partial \bm{n}_{\mathit{obj,N_{cnt\mathchar`-obj}}}}
  \end{pmatrix} \\
  \frac{\partial \bm{e}^{\mathit{eom\mathchar`-rot\mathchar`-obj}}}{\partial \bm{f}_{\mathit{obj,m}}}
  &=& \left[ \left(\bm{p}_m^{\mathit{cnt\mathchar`-trg\mathchar`-obj}}(\bm{\theta}, \bm{\phi}) - \bm{p}_{\mathit{Gobj}}(\bm{\theta}, \bm{\phi})\right) \times \right] \ \ (m = 1,2,\cdots,N_{\mathit{cnt\mathchar`-obj}}) \\
  \frac{\partial \bm{e}^{\mathit{eom\mathchar`-rot\mathchar`-obj}}}{\partial \bm{n}_{\mathit{obj,m}}}
  &=& \bm{I}_3 \ \ (m = 1,2,\cdots,N_{\mathit{cnt\mathchar`-obj}})
\end{eqnarray}

return $\frac{\partial \bm{e}^{\mathit{eom\mathchar`-rot\mathchar`-obj}}}{\partial \bm{\hat{w}_{\mathit{obj}}}} \in \mathbb{R}^{3 \times 6N_{\mathit{cnt\mathchar`-obj}}}$
   "
   (apply #'concatenate-matrix-row
          (flatten
           (mapcar
            #'(lambda (p)
                (list (outer-product-matrix (v- p (send self :cog-obj))) (unit-matrix 3)))
            (mapcar #'(lambda (c) (scale 1e-3 (send c :worldpos))) _contact-target-coords-obj-list) ;; [m]
            )))
   )
  (:eom-rot-obj-task-jacobian-with-phi
   ()
   "
\begin{eqnarray}
  \frac{\partial \bm{e}^{\mathit{eom\mathchar`-rot\mathchar`-obj}}}{\partial \bm{\phi}}
  &=& \sum_{m=1}^{N_{\mathit{cnt\mathchar`-obj}}} \left\{ - [\bm{f}_{\mathit{obj,m}} \times] \left( \bm{J}_{\phi, m}^{\mathit{cnt\mathchar`-trg\mathchar`-obj}}(\bm{\theta}, \bm{\phi}) - \bm{J}_{\mathit{Gobj}\phi}(\bm{\theta}, \bm{\phi}) \right) \right\} \\
  &=& \left[\left( \sum_{m=1}^{N_{\mathit{cnt\mathchar`-obj}}} \bm{f}_{\mathit{obj,m}} \right) \times \right] \bm{J}_{\mathit{Gobj}\phi}(\bm{\theta}, \bm{\phi})
  - \sum_{m=1}^{N_{\mathit{cnt\mathchar`-obj}}}  [\bm{f}_{\mathit{obj,m}} \times] \bm{J}_{\phi, m}^{\mathit{cnt\mathchar`-trg\mathchar`-obj}}(\bm{\theta}, \bm{\phi})
\end{eqnarray}
$\sum_{m=1}^{N_{\mathit{cnt\mathchar`-obj}}} \bm{f}_{\mathit{obj,m}} = m_{\mathit{obj}} \bm{g}$つまり，eom-trans-obj-taskが成立すると仮定すると次式が成り立つ．
\begin{eqnarray}
  \frac{\partial \bm{e}^{\mathit{eom\mathchar`-rot\mathchar`-obj}}}{\partial \bm{\phi}}
  &=& \left[ m_{\mathit{obj}} \bm{g} \times \right] \bm{J}_{\mathit{Gobj}\phi}(\bm{\theta}, \bm{\phi}) - \sum_{m=1}^{N_{\mathit{cnt\mathchar`-obj}}} [\bm{f}_{\mathit{obj,m}} \times] \bm{J}_{\phi, m}^{\mathit{cnt\mathchar`-trg\mathchar`-obj}}(\bm{\theta}, \bm{\phi})
\end{eqnarray}

return $\frac{\partial \bm{e}^{\mathit{eom\mathchar`-rot\mathchar`-obj}}}{\partial \bm{\phi}} \in \mathbb{R}^{3 \times N_{\mathit{invar\mathchar`-joint}}}$
   "
   (reduce
    #'m+
    (remove
     nil
     (flatten
      (list
       (m* (outer-product-matrix (send self :mg-obj-vec))
           (send _robot-obj-env :object-with-root-virtual :calc-cog-jacobian-from-joint-list
                 :union-joint-list _invariant-joint-list
                 :translation-axis t
                 ))
       (mapcar
        #'(lambda (f c)
            (if (derivedp c cascaded-coords)
                (scale-matrix
                 -1
                 (m*
                  (outer-product-matrix f)
                  (send _robot-obj-env :calc-jacobian-from-joint-list
                        :union-joint-list _invariant-joint-list
                        :move-target (list c)
                        :transform-coords (list (make-coords))
                        :translation-axis (list t)
                        :rotation-axis (list nil)
                        )
                  ))))
        (send self :force-obj-list)
        _contact-target-coords-obj-list
        )
       ))
     ))
   )
  (:variant-task-jacobian
   ()
   "
\begin{eqnarray}
  \frac{\partial \bm{e}}{\partial \bm{q}_{\mathit{var}}}
  &=&
  \bordermatrix{
    & N_{\mathit{var\mathchar`-joint}} & 6 N_{\mathit{cnt}} & 6 N_{\mathit{cnt\mathchar`-obj}} & N_{\mathit{drive\mathchar`-joint}} \cr
    6 N_{\mathit{kin}} & \frac{\partial \bm{e}^{\mathit{kin}}}{\partial \bm{\theta}} & & & \cr
    3 & & \frac{\partial \bm{e}^{\mathit{eom\mathchar`-trans}}}{\partial \bm{\hat{w}}} & & & \cr
    3 & \frac{\partial \bm{e}^{\mathit{eom\mathchar`-rot}}}{\partial \bm{\theta}} & \frac{\partial \bm{e}^{\mathit{eom\mathchar`-rot}}}{\partial \bm{\hat{w}}} & & & \cr
    3 & & & \frac{\partial \bm{e}^{\mathit{eom\mathchar`-trans\mathchar`-obj}}}{\partial \bm{\hat{w}}_{\mathit{obj}}} & & \cr
    3 & \frac{\partial \bm{e}^{\mathit{eom\mathchar`-rot\mathchar`-obj}}}{\partial \bm{\theta}} & & \frac{\partial \bm{e}^{\mathit{eom\mathchar`-rot\mathchar`-obj}}}{\partial \bm{\hat{w}}_{\mathit{obj}}} & & \cr
    N_{\mathit{drive\mathchar`-joint}} & \frac{\partial \bm{e}^{\mathit{trq}}}{\partial \bm{\theta}} & \frac{\partial \bm{e}^{\mathit{trq}}}{\partial \bm{\hat{w}}} & & \frac{\partial \bm{e}^{\mathit{trq}}}{\partial \bm{\tau}} \cr
    N_{\mathit{posture\mathchar`-joint}} & \frac{\partial \bm{e}^{\mathit{posture}}}{\partial \bm{\theta}} & & & \cr
  }
\end{eqnarray}

return $\frac{\partial \bm{e}}{\partial \bm{q}_{\mathit{var}}} \in \mathbb{R}^{(6 N_{\mathit{kin}} + 3 + 3 + 3 + 3 + N_{\mathit{drive\mathchar`-joint}} + N_{\mathit{posture\mathchar`-joint}}) \times (N_{\mathit{var\mathchar`-joint}} + 6 N_{\mathit{cnt}} + 6 N_{\mathit{cnt\mathchar`-obj}} + N_{\mathit{drive\mathchar`-joint}})}$
   "
   (cond (_variant-task-jacobi
          (fill (array-entity _variant-task-jacobi) 0)
          )
         (t
          (setq _variant-task-jacobi (make-matrix _dim-task _dim-variant-config))
          ))
   (copy-block-matrix _variant-task-jacobi
                      (send self :kinematics-task-jacobian-with-theta)
                      0 0)
   (copy-block-matrix _variant-task-jacobi
                      (send self :eom-trans-task-jacobian-with-wrench)
                      (* 6 _num-kin) _num-variant-joint)
   (copy-block-matrix _variant-task-jacobi
                      (send self :eom-rot-task-jacobian-with-theta)
                      (+ (* 6 _num-kin) 3) 0)
   (copy-block-matrix _variant-task-jacobi
                      (send self :eom-rot-task-jacobian-with-wrench)
                      (+ (* 6 _num-kin) 3) _num-variant-joint)
   (copy-block-matrix _variant-task-jacobi
                      (send self :eom-trans-obj-task-jacobian-with-wrench-obj)
                      (+ (* 6 _num-kin) 3 3) (+ _num-variant-joint (* 6 _num-contact)))
   (copy-block-matrix _variant-task-jacobi
                      (send self :eom-rot-obj-task-jacobian-with-theta)
                      (+ (* 6 _num-kin) 3 3 3) 0)
   (copy-block-matrix _variant-task-jacobi
                      (send self :eom-rot-obj-task-jacobian-with-wrench-obj)
                      (+ (* 6 _num-kin) 3 3 3) (+ _num-variant-joint (* 6 _num-contact)))
   (when (> _num-drive-joint 0)
     (copy-block-matrix _variant-task-jacobi
                        (send self :torque-task-jacobian-with-theta)
                        (+ (* 6 _num-kin) 3 3 3 3) 0)
     (copy-block-matrix _variant-task-jacobi
                        (send self :torque-task-jacobian-with-wrench)
                        (+ (* 6 _num-kin) 3 3 3 3) _num-variant-joint)
     (copy-block-matrix _variant-task-jacobi
                        (send self :torque-task-jacobian-with-torque)
                        (+ (* 6 _num-kin) 3 3 3 3) (+ _num-variant-joint (* 6 _num-contact) (* 6 _num-contact-obj)))
     )
   (when (> _num-posture-joint 0)
     (copy-block-matrix _variant-task-jacobi
                        (send self :posture-task-jacobian-with-theta)
                        (+ (* 6 _num-kin) 3 3 3 3 _num-drive-joint) 0)
     )
   _variant-task-jacobi
   )
  (:invariant-task-jacobian
   ()
   "
\begin{eqnarray}
  \frac{\partial \bm{e}}{\partial \bm{q}_{\mathit{invar}}}
  &=&
  \bordermatrix{
    & N_{\mathit{invar\mathchar`-joint}} \cr
    6 N_{\mathit{kin}} & \frac{\partial \bm{e}^{\mathit{kin}}}{\partial \bm{\phi}} \cr
    3 & \cr
    3 & \frac{\partial \bm{e}^{\mathit{eom\mathchar`-rot}}}{\partial \bm{\phi}} \cr
    3 & \cr
    3 & \frac{\partial \bm{e}^{\mathit{eom\mathchar`-rot\mathchar`-obj}}}{\partial \bm{\phi}} \cr
    N_{\mathit{drive\mathchar`-joint}} & \frac{\partial \bm{e}^{\mathit{trq}}}{\partial \bm{\phi}} \cr
    N_{\mathit{posture\mathchar`-joint}} & \cr
  }
\end{eqnarray}

return $\frac{\partial \bm{e}}{\partial \bm{q}_{\mathit{invar}}} \in \mathbb{R}^{(6 N_{\mathit{kin}} + 3 + 3 + 3 + 3 + N_{\mathit{drive\mathchar`-joint}} + N_{\mathit{posture\mathchar`-joint}}) \times N_{\mathit{invar\mathchar`-joint}}}$
   "
   (cond (_invariant-task-jacobi
          (fill (array-entity _invariant-task-jacobi) 0)
          )
         (t
          (setq _invariant-task-jacobi (make-matrix _dim-task _dim-invariant-config))
          ))
   (copy-block-matrix _invariant-task-jacobi
                      (send self :kinematics-task-jacobian-with-phi)
                      0 0)
   (copy-block-matrix _invariant-task-jacobi
                      (send self :eom-rot-task-jacobian-with-phi)
                      (+ (* 6 _num-kin) 3) 0)
   (copy-block-matrix _invariant-task-jacobi
                      (send self :eom-rot-obj-task-jacobian-with-phi)
                      (+ (* 6 _num-kin) 3 3 3) 0)
   (when (> _num-drive-joint 0)
     (copy-block-matrix _invariant-task-jacobi
                        (send self :torque-task-jacobian-with-phi)
                        (+ (* 6 _num-kin) 3 3 3 3) 0)
     )
   _invariant-task-jacobi
   )
  (:task-jacobian
   ()
   "
\begin{eqnarray}
  \frac{\partial \bm{e}}{\partial \bm{q}}
  &=& \begin{pmatrix}
    \frac{\partial \bm{e}}{\partial \bm{q}_{\mathit{var}}} & \frac{\partial \bm{e}}{\partial \bm{q}_{\mathit{invar}}}
  \end{pmatrix} \\
  &=&
  \bordermatrix{
    & N_{\mathit{var\mathchar`-joint}} & 6 N_{\mathit{cnt}} & 6 N_{\mathit{cnt\mathchar`-obj}} & N_{\mathit{drive\mathchar`-joint}} & N_{\mathit{invar\mathchar`-joint}} \cr
    6 N_{\mathit{kin}} & \frac{\partial \bm{e}^{\mathit{kin}}}{\partial \bm{\theta}} & & & & \frac{\partial \bm{e}^{\mathit{kin}}}{\partial \bm{\phi}} \cr
    3 & & \frac{\partial \bm{e}^{\mathit{eom\mathchar`-trans}}}{\partial \bm{\hat{w}}} & & & &  \cr
    3 & \frac{\partial \bm{e}^{\mathit{eom\mathchar`-rot}}}{\partial \bm{\theta}} & \frac{\partial \bm{e}^{\mathit{eom\mathchar`-rot}}}{\partial \bm{\hat{w}}} & & & \frac{\partial \bm{e}^{\mathit{eom\mathchar`-rot}}}{\partial \bm{\phi}} \cr
    3 & & & \frac{\partial \bm{e}^{\mathit{eom\mathchar`-trans\mathchar`-obj}}}{\partial \bm{\hat{w}}_{\mathit{obj}}} & &  \cr
    3 & \frac{\partial \bm{e}^{\mathit{eom\mathchar`-rot\mathchar`-obj}}}{\partial \bm{\theta}} & & \frac{\partial \bm{e}^{\mathit{eom\mathchar`-rot\mathchar`-obj}}}{\partial \bm{\hat{w}}_{\mathit{obj}}} & & \frac{\partial \bm{e}^{\mathit{eom\mathchar`-rot\mathchar`-obj}}}{\partial \bm{\phi}} \cr
    N_{\mathit{drive\mathchar`-joint}} & \frac{\partial \bm{e}^{\mathit{trq}}}{\partial \bm{\theta}} & \frac{\partial \bm{e}^{\mathit{trq}}}{\partial \bm{\hat{w}}} & & \frac{\partial \bm{e}^{\mathit{trq}}}{\partial \bm{\tau}} & \frac{\partial \bm{e}^{\mathit{trq}}}{\partial \bm{\phi}}\cr
    N_{\mathit{posture\mathchar`-joint}} & \frac{\partial \bm{e}^{\mathit{posture}}}{\partial \bm{\theta}} & & & & \cr
  }
\end{eqnarray}

return $\frac{\partial \bm{e}}{\partial \bm{q}} \in \mathbb{R}^{(6 N_{\mathit{kin}} + 3 + 3 + 3 + 3 + N_{\mathit{drive\mathchar`-joint}} + N_{\mathit{posture\mathchar`-joint}}) \times (N_{\mathit{var\mathchar`-joint}} + 6 N_{\mathit{cnt}} + 6 N_{\mathit{cnt\mathchar`-obj}} + N_{\mathit{drive\mathchar`-joint}} + N_{\mathit{invar\mathchar`-joint}})}$
   "
   (send-super :task-jacobian)
   )
  )

(defmethod instant-manipulation-configuration-task
  (:_contact-constraint-obj-matrix-list
   (&key (update? nil)) ;; this should be true
   "
return $\{\bm{C}_{w_{\mathit{obj}},1}, \bm{C}_{w_{\mathit{obj}},2}, \cdots, \bm{C}_{w_{\mathit{obj}},N_{\mathit{cnt\mathchar`-obj}}}\}$
   "
   (when (or update? (not (send self :get :_contact-constraint-obj-matrix-list)))
     (send self :put :_contact-constraint-obj-matrix-list
           (mapcar
            #'(lambda (const coords) (send const :calc-constraint-matrix coords))
            _contact-constraint-obj-list
            _contact-target-coords-obj-list
            ))
     )
   (send self :get :_contact-constraint-obj-matrix-list)
   )
  (:_contact-constraint-obj-vector-list
   (&key (update? nil))
   "
return $\{\bm{d}_{w_{\mathit{obj}},1}, \bm{d}_{w_{\mathit{obj}},2}, \cdots, \bm{d}_{w_{\mathit{obj}},N_{\mathit{cnt\mathchar`-obj}}}\}$
   "
   (when (or update? (not (send self :get :_contact-constraint-obj-vector-list)))
     (send self :put :_contact-constraint-obj-vector-list
           (mapcar
            #'(lambda (const) (send const :get-constraint-vector))
            _contact-constraint-obj-list
            ))
     )
   (send self :get :_contact-constraint-obj-vector-list)
   )
  (:wrench-obj-inequality-constraint-matrix
   (&key (update? t))
   "
物体の接触レンチ$\bm{w}_{\mathit{obj}} \in \mathbb{R}^{6}$が満たすべき制約（非負制約，摩擦制約，圧力中心制約）が次式のように表されるとする．
\begin{eqnarray}
  \bm{C}_{w_{\mathit{obj}}} \bm{w}_{\mathit{obj}} \geq \bm{d}_{w_{\mathit{obj}}}
\end{eqnarray}
$N_{\mathit{cnt\mathchar`-obj}}$箇所の接触部位の接触レンチを並べたベクトル$\bm{\hat{w}}_{\mathit{obj}}$の不等式制約は次式で表される．
\begin{eqnarray}
  &&\bm{C}_{w_{\mathit{obj}},m} (\bm{w}_{\mathit{obj,m}} + \Delta \bm{w}_{\mathit{obj,m}}) \geq \bm{d}_{w_{\mathit{obj}},m} \ \ (m=1,2,\cdots,N_{\mathit{cnt\mathchar`-obj}}) \\
  \Leftrightarrow&&\bm{C}_{w_{\mathit{obj}},m} \Delta \bm{w}_{\mathit{obj,m}} \geq \bm{d}_{w_{\mathit{obj}},m} - \bm{C}_{w_{\mathit{obj}},m} \bm{w}_{\mathit{obj,m}} \ \ (m=1,2,\cdots,N_{\mathit{cnt\mathchar`-obj}}) \\
  \Leftrightarrow&&
  \begin{pmatrix}\bm{C}_{w_{\mathit{obj}},1}&&&\\&\bm{C}_{w_{\mathit{obj}},2}&&\\&&\ddots&\\&&&\bm{C}_{w_{\mathit{obj}},N_{\mathit{cnt\mathchar`-obj}}}\end{pmatrix}
\begin{pmatrix}\Delta \bm{w}_{\mathit{obj},1} \\ \Delta \bm{w}_{\mathit{obj},2} \\ \vdots \\ \Delta \bm{w}_{\mathit{obj},N_{cnt\mathchar`-obj}}\end{pmatrix}
  \geq
  \begin{pmatrix}\bm{d}_{w_{\mathit{obj},1}}-\bm{C}_{w_{\mathit{obj}},1}\bm{w}_{\mathit{obj},1} \\ \bm{d}_{w_{\mathit{obj},2}}-\bm{C}_{w_{\mathit{obj}},2}\bm{w}_{\mathit{obj},2} \\ \vdots \\ \bm{d}_{w_{\mathit{obj},N_{\mathit{cnt\mathchar`-obj}}}}-\bm{C}_{w_{\mathit{obj}},N_{\mathit{cnt\mathchar`-obj}}}\bm{w}_{\mathit{obj},N_{cnt\mathchar`-obj}}\end{pmatrix} \\
  \Leftrightarrow&&\bm{C}_{\hat{w}_{\mathit{obj}}} \Delta \bm{\hat{w}}_{\mathit{obj}} \geq \bm{d}_{\hat{w}_{\mathit{obj}}}
\end{eqnarray}

return $\bm{C}_{\hat{w}_{\mathit{obj}}} := \begin{pmatrix}\bm{C}_{w_{\mathit{obj}},1}&&&\\&\bm{C}_{w_{\mathit{obj}},2}&&\\&&\ddots&\\&&&\bm{C}_{w_{\mathit{obj}},N_{\mathit{cnt\mathchar`-obj}}}\end{pmatrix} \in \mathbb{R}^{N_{\mathit{wrench\mathchar`-obj\mathchar`-ineq}} \times 6 N_{\mathit{cnt\mathchar`-obj}}}$
   "
   (when (or update? (not (send self :get :wrench-obj-inequality-constraint-matrix)))
     (send self :put :wrench-obj-inequality-constraint-matrix
           (apply #'concatenate-matrix-diagonal (send self :_contact-constraint-obj-matrix-list)))
     )
   (send self :get :wrench-obj-inequality-constraint-matrix)
   )
  (:wrench-obj-inequality-constraint-vector
   (&key (update? t))
   "
return $\bm{d}_{\hat{w}_{\mathit{obj}}} := \begin{pmatrix}\bm{d}_{w_{\mathit{obj},1}}-\bm{C}_{w_{\mathit{obj}},1}\bm{w}_{\mathit{obj,1}} \\ \bm{d}_{w_{\mathit{obj},2}}-\bm{C}_{w_{\mathit{obj}},2}\bm{w}_{\mathit{obj,2}} \\ \vdots \\ \bm{d}_{w_{\mathit{obj},N_{\mathit{cnt\mathchar`-obj}}}}-\bm{C}_{w_{\mathit{obj}},N_{\mathit{cnt\mathchar`-obj}}}\bm{w}_{\mathit{obj,N_{cnt\mathchar`-obj}}}\end{pmatrix} \in \mathbb{R}^{N_{\mathit{wrench\mathchar`-obj\mathchar`-ineq}}}$
   "
   (when (or update? (not (send self :get :wrench-obj-inequality-constraint-vector)))
     (send self :put :wrench-obj-inequality-constraint-vector
           (apply #'concatenate
                  float-vector
                  (mapcar
                   #'(lambda (v m w) (v- v (transform m w)))
                   (send self :_contact-constraint-obj-vector-list)
                   (send self :_contact-constraint-obj-matrix-list)
                   (send self :wrench-obj-list)
                   )))
     )
   (send self :get :wrench-obj-inequality-constraint-vector)
   )
  (:variant-config-inequality-constraint-matrix
   (&key (update? nil))
   "
\begin{eqnarray}
  &&
  \left\{
  \begin{array}{l}
  \bm{C}_{\theta} \Delta \bm{\theta} \geq \bm{d}_{\theta} \\
  \bm{C}_{\hat{w}} \Delta \bm{\hat{w}} \geq \bm{d}_{\hat{w}} \\
  \bm{C}_{\hat{w}_{\mathit{obj}}} \Delta \bm{\hat{w}_{\mathit{obj}}} \geq \bm{d}_{\hat{w}_{\mathit{obj}}} \\
  \bm{C}_{\tau} \Delta \bm{\tau} \geq \bm{d}_{\tau}
  \end{array}
  \right. \\
  \Leftrightarrow&&
  \begin{pmatrix}\bm{C}_{\theta}&&&\\&\bm{C}_{\hat{w}}&&\\&&\bm{C}_{\hat{w}_{\mathit{obj}}}&\\&&&\bm{C}_{\tau}\end{pmatrix}
  \begin{pmatrix}\Delta \bm{\theta}\\\Delta \bm{\hat{w}}\\\Delta \bm{\hat{w}}_{\mathit{obj}}\\\Delta \bm{\tau}\end{pmatrix}
  \geq
  \begin{pmatrix}\bm{d}_{\theta}\\\bm{d}_{\hat{w}}\\\bm{d}_{\hat{w}_{\mathit{obj}}}\\\bm{d}_{\tau}\end{pmatrix} \\
  \Leftrightarrow&&
  \bm{C}_{\mathit{var}} \Delta \bm{q}_{\mathit{var}} \geq \bm{d}_{\mathit{var}}
\end{eqnarray}

return $\bm{C}_{\mathit{var}} := \begin{pmatrix}\bm{C}_{\theta}&&&\\&\bm{C}_{\hat{w}}&&\\&&\bm{C}_{\hat{w}_{\mathit{obj}}}&\\&&&\bm{C}_{\tau}\end{pmatrix} \in \mathbb{R}^{N_{\mathit{var\mathchar`-ineq}} \times \mathit{dim}(\bm{q}_{\mathit{var}})}$
   "
   (concatenate-matrix-diagonal
    (send self :theta-inequality-constraint-matrix :update? update?)
    (send self :wrench-inequality-constraint-matrix :update? update?)
    (send self :wrench-obj-inequality-constraint-matrix :update? update?)
    (send self :torque-inequality-constraint-matrix :update? update?)
    )
   )
  (:variant-config-inequality-constraint-vector
   (&key (update? t))
   "
return $\bm{d}_{\mathit{var}} := \begin{pmatrix}\bm{d}_{\theta}\\\bm{d}_{\hat{w}}\\\bm{d}_{\hat{w}_{\mathit{obj}}}\\\bm{d}_{\tau}\end{pmatrix} \in \mathbb{R}^{N_{\mathit{var\mathchar`-ineq}}}$
   "
   (concatenate
    float-vector
    (send self :theta-inequality-constraint-vector :update? update?)
    (send self :wrench-inequality-constraint-vector :update? update?)
    (send self :wrench-obj-inequality-constraint-vector :update? update?)
    (send self :torque-inequality-constraint-vector :update? update?)
    )
   )
  )

(defmethod instant-manipulation-configuration-task
  (:act-react-equality-constraint-matrix
   (&key (update? nil))
   "
ロボット・物体間の接触レンチに関する作用・反作用の法則は次式のように表される．
\begin{eqnarray}
  &&\bm{\hat{w}}_{i(m)} + \bm{\hat{w}}_{\mathit{obj},j(m)} = \bm{0} \ \ (m = 1,2,\cdots,N_{\mathit{act\mathchar`-react}}) \\
  \Leftrightarrow&&\bm{A}_{\mathit{act\mathchar`-react,robot,m}} \bm{\hat{w}} + \bm{A}_{\mathit{act\mathchar`-react,obj,m}} \bm{\hat{w}}_{\mathit{obj}} = \bm{0} \ \ (m = 1,2,\cdots,N_{\mathit{act\mathchar`-react}}) \\
  && {\rm where} \ \ \bm{A}_{\mathit{act\mathchar`-react,robot,m}} =
  \bordermatrix{
    & & & & i(m)番目 & & & \cr
    & \bm{O}_6 & \bm{O}_6 & \cdots & \bm{I}_6 & \cdots & \bm{O}_6 & \bm{O}_6 \cr
  } \in \mathbb{R}^{6 \times 6 N_{\mathit{cnt}}} \\
  && \phantom{\rm where} \ \ \bm{A}_{\mathit{act\mathchar`-react,obj,m}} =
  \bordermatrix{
    & & & & j(m)番目 & & & \cr
    & \bm{O}_6 & \bm{O}_6 & \cdots & \bm{I}_6 & \cdots & \bm{O}_6 & \bm{O}_6 \cr
  } \in \mathbb{R}^{6 \times 6 N_{\mathit{cnt\mathchar`-obj}}} \\
  \Leftrightarrow&&\bm{A}_{\mathit{act\mathchar`-react,robot}} \bm{\hat{w}} + \bm{A}_{\mathit{act\mathchar`-react,obj}} \bm{\hat{w}}_{\mathit{obj}} = \bm{0} \\
  && {\rm where} \ \ \bm{A}_{\mathit{act\mathchar`-react,robot}} =
  \begin{pmatrix}
    \bm{A}_{\mathit{act\mathchar`-react,robot,1}} \\ \vdots \\ \bm{A}_{\mathit{act\mathchar`-react,robot,N_{\mathit{act\mathchar`-react}}}}
  \end{pmatrix}
  \in \mathbb{R}^{6 N_{\mathit{act\mathchar`-react}} \times 6 N_{\mathit{cnt}}} \\
  && \phantom{\rm where} \ \ \bm{A}_{\mathit{act\mathchar`-react,obj}} =
  \begin{pmatrix}
    \bm{A}_{\mathit{act\mathchar`-react,obj,1}} \\ \vdots \\ \bm{A}_{\mathit{act\mathchar`-react,obj,N_{\mathit{act\mathchar`-react}}}}
  \end{pmatrix}
  \in \mathbb{R}^{6 N_{\mathit{act\mathchar`-react}} \times 6 N_{\mathit{cnt\mathchar`-obj}}} \\
  \Leftrightarrow&&\bm{A}_{\mathit{act\mathchar`-react}} \begin{pmatrix} \bm{\hat{w}} \\ \bm{\hat{w}}_{\mathit{obj}} \end{pmatrix} = \bm{0} \in \mathbb{R}^{6 N_{\mathit{act\mathchar`-react}}} \\
  && {\rm where} \ \ \bm{A}_{\mathit{act\mathchar`-react}} =
  \begin{pmatrix}
    \bm{A}_{\mathit{act\mathchar`-react,robot}} & \bm{A}_{\mathit{act\mathchar`-react,obj}}
  \end{pmatrix}
  \in \mathbb{R}^{6 N_{\mathit{act\mathchar`-react}} \times (6 N_{\mathit{cnt}} + 6 N_{\mathit{cnt\mathchar`-obj}})} \\
  \Leftrightarrow&&\bm{A}_{\mathit{act\mathchar`-react}} \begin{pmatrix} \bm{\hat{w}} + \Delta \bm{\hat{w}} \\ \bm{\hat{w}}_{\mathit{obj}} + \Delta \bm{\hat{w}}_{\mathit{obj}} \end{pmatrix} = \bm{0} \\
  \Leftrightarrow&&\bm{A}_{\mathit{act\mathchar`-react}} \begin{pmatrix} \Delta \bm{\hat{w}} \\ \Delta \bm{\hat{w}}_{\mathit{obj}} \end{pmatrix} = \bm{b}_{\mathit{act\mathchar`-react}} \\
  &&{\rm where} \ \ \bm{b}_{\mathit{act\mathchar`-react}} = - \bm{A}_{\mathit{act\mathchar`-react}} \begin{pmatrix} \bm{\hat{w}} \\ \bm{\hat{w}}_{\mathit{obj}} \end{pmatrix}
\end{eqnarray}

$i(m), j(m)$は作用・反作用の関係にある接触レンチの$m$番目の対におけるロボット，物体の接触レンチのインデックスである．

return $\bm{A}_{\mathit{act\mathchar`-react}} \in \mathbb{R}^{6 N_{\mathit{act\mathchar`-react}} \times (6 N_{\mathit{cnt}} + 6 N_{\mathit{cnt\mathchar`-obj}})} $
   "
   (when (or update? (not (send self :get :act-react-equality-constraint-matrix)))
     (cond ((= _num-act-react 0)
            (send self :put :act-react-equality-constraint-matrix
                  (make-matrix 0 (+ (* 6 _num-contact) (* 6 _num-contact-obj))))
            )
           (t
            (send self :put :act-react-equality-constraint-matrix
                  (apply
                   #'concatenate-matrix-column
                   (mapcar
                    #'(lambda (pair)
                        (let ((mat
                               (make-matrix 6 (+ (* 6 _num-contact) (* 6 _num-contact-obj))))
                              )
                          (copy-block-matrix mat
                                             (unit-matrix 6)
                                             0
                                             (* 6 (position (elt pair 0) _contact-target-coords-list)))
                          (copy-block-matrix mat
                                             (unit-matrix 6)
                                             0
                                             (+ (* 6 _num-contact) (* 6 (position (elt pair 1) _contact-target-coords-obj-list))))
                          mat
                          ))
                    _act-react-pair-list
                    )))
            ))
     )
   (send self :get :act-react-equality-constraint-matrix)
   )
  (:act-react-equality-constraint-vector
   (&key (update? t))
   "
return $\bm{b}_{\mathit{act\mathchar`-react}} \in \mathbb{R}^{6 N_{\mathit{act\mathchar`-react}}}$
   "
   (when (or update? (not (send self :get :act-react-equality-constraint-vector)))
     (cond ((= _num-act-react 0)
            (send self :put :act-react-equality-constraint-vector
                  (float-vector))
            )
           (t
            (send self :put :act-react-equality-constraint-vector
                  (scale
                   -1
                   (transform
                    (send self :act-react-equality-constraint-matrix)
                    (concatenate float-vector _wrench-vector _wrench-obj-vector)))
                  )
            ))
     )
   (send self :get :act-react-equality-constraint-vector)
   )
  (:variant-config-equality-constraint-matrix
   (&key (update? nil))
   "
\begin{eqnarray}
  &&
  \bm{A}_{\mathit{act\mathchar`-react}} \begin{pmatrix} \Delta \bm{\hat{w}} \\ \Delta \bm{\hat{w}}_{\mathit{obj}} \end{pmatrix} = \bm{b}_{\mathit{act\mathchar`-react}} \\
  \Leftrightarrow&&
  \begin{pmatrix}\bm{O} & \bm{A}_{\mathit{act\mathchar`-react}} & \bm{O}\end{pmatrix}
  \begin{pmatrix}\Delta \bm{\theta}\\\Delta \bm{\hat{w}}\\\Delta \bm{\hat{w}}_{\mathit{obj}}\\\Delta \bm{\tau}\end{pmatrix}
  =
  \bm{b}_{\mathit{act\mathchar`-react}} \\
  \Leftrightarrow&&
  \bm{A}_{\mathit{var}} \Delta \bm{q}_{\mathit{var}} = \bm{b}_{\mathit{var}}
\end{eqnarray}

return $\bm{A}_{\mathit{var}} := \begin{pmatrix}\bm{O} & \bm{A}_{\mathit{act\mathchar`-react}} & \bm{O}\end{pmatrix} \in \mathbb{R}^{6 N_{\mathit{act\mathchar`-react}} \times \mathit{dim}(\bm{q}_{\mathit{var}})}$
   "
   (concatenate-matrix-row
    (make-matrix (* 6 _num-act-react) _dim-theta)
    (send self :act-react-equality-constraint-matrix :update? update?)
    (make-matrix (* 6 _num-act-react) _dim-torque)
    )
   )
  (:variant-config-equality-constraint-vector
   (&key (update? t))
   "
return $\bm{b}_{\mathit{var}} := \bm{b}_{\mathit{act\mathchar`-react}} \in \mathbb{R}^{6 N_{\mathit{act\mathchar`-react}}}$
   "
   (send self :act-react-equality-constraint-vector :update? update?)
   )
  (:invariant-config-equality-constraint-matrix
   (&key (update? nil))
   "
return $\bm{A}_{\mathit{invar}} \in \mathbb{R}^{0 \times \mathit{dim}(\bm{q}_{\mathit{invar}})}$ (no equality constraint)
   "
   (make-matrix 0 _dim-invariant-config)
   )
  (:invariant-config-equality-constraint-vector
   (&key (update? t))
   "
return $\bm{b}_{\mathit{invar}} \in \mathbb{R}^{0}$ (no equality constraint)
   "
   (float-vector)
   )
  (:config-equality-constraint-matrix
   (&key (update? nil))
   "
\begin{eqnarray}
  &&
  \bm{A}_{\mathit{var}} \Delta \bm{q}_{\mathit{var}} = \bm{b}_{\mathit{var}} \\
  \Leftrightarrow&&
  \begin{pmatrix} \bm{A}_{\mathit{var}} & \bm{O} \end{pmatrix}
  \begin{pmatrix}\Delta \bm{q}_{\mathit{var}}\\\Delta \bm{q}_{\mathit{invar}}\end{pmatrix}
  =
  \bm{b}_{\mathit{var}} \\
  \Leftrightarrow&&
  \bm{A} \Delta \bm{q} = \bm{b}
\end{eqnarray}

return $\bm{A} := \begin{pmatrix} \bm{A}_{\mathit{var}} & \bm{O} \end{pmatrix} \in \mathbb{R}^{N_{\mathit{eq}} \times \mathit{dim}(\bm{q})}$
   "
   (concatenate-matrix-diagonal
    (send self :variant-config-equality-constraint-matrix :update? update?)
    (send self :invariant-config-equality-constraint-matrix :update? update?)
    )
   )
  (:config-equality-constraint-vector
   (&key (update? t))
   "
return $\bm{b} := \bm{b}_{\mathit{var}} \in \mathbb{R}^{N_{\mathit{eq}}}$
   "
   (concatenate
    float-vector
    (send self :variant-config-equality-constraint-vector :update? update?)
    (send self :invariant-config-equality-constraint-vector :update? update?)
    )
   )
  )

(defmethod instant-manipulation-configuration-task
  (:torque-regular-matrix
   (&key (update? nil) (only-variant? nil))
   "
二次形式の正則化項として次式を考える．
\begin{eqnarray}
  F_{\mathit{tau}}(\bm{q}) &=& \left\| \frac{\bm{\tau}}{\bm{\tau}_{\mathit{max}}} \right\|^2 \ \ (ベクトルの要素ごとの割り算を表す) \\
  &=& \bm{\tau}^T \bm{\bar{W}}_{trq} \bm{\tau}
\end{eqnarray}
ここで，
\begin{eqnarray}
  \bm{\bar{W}}_{trq} := \begin{pmatrix} \frac{1}{\tau_{\mathit{max,1}}^2} & & & \\ & \frac{1}{\tau_{\mathit{max,2}}^2} & & \\ & & \ddots & \\ & & & \frac{1}{\tau_{\mathit{max,N_{\mathit{drive\mathchar`-joint}}}}^2} \end{pmatrix}
  \in \mathbb{R}^{\mathit{dim}(\bm{\tau}) \times \mathit{dim}(\bm{\tau})}
\end{eqnarray}

\it{only-variant?} is true:
\begin{eqnarray}
  \bm{W}_{trq} :=
  \bordermatrix{
    & \mathit{dim}(\bm{\theta}) & \mathit{dim}(\bm{\hat{w}}) &\mathit{dim}(\bm{\hat{w}}_{\mathit{obj}}) & \mathit{dim}(\bm{\tau}) \cr
    \mathit{dim}(\bm{\theta}) &&&& \cr
    \mathit{dim}(\bm{\hat{w}}) &&&& \cr
   \mathit{dim}(\bm{\hat{w}}_{\mathit{obj}}) &&&& \cr
    \mathit{dim}(\bm{\tau}) &&&&\bm{\bar{W}}_{trq} \cr
  }
  \in \mathbb{R}^{\mathit{dim}(\bm{q}_{\mathit{var}}) \times \mathit{dim}(\bm{q}_{\mathit{var}})}
\end{eqnarray}

otherwise:
\begin{eqnarray}
  \bm{W}_{trq} :=
  \bordermatrix{
    & \mathit{dim}(\bm{\theta}) & \mathit{dim}(\bm{\hat{w}}) &\mathit{dim}(\bm{\hat{w}}_{\mathit{obj}}) & \mathit{dim}(\bm{\tau}) & \mathit{dim}(\bm{\phi}) \cr
    \mathit{dim}(\bm{\theta}) &&&&& \cr
    \mathit{dim}(\bm{\hat{w}}) &&&&& \cr
   \mathit{dim}(\bm{\hat{w}}_{\mathit{obj}}) &&&&& \cr
    \mathit{dim}(\bm{\tau}) &&&&\bm{\bar{W}}_{trq}& \cr
    \mathit{dim}(\bm{\phi}) &&&&& \cr
  }
  \in \mathbb{R}^{\mathit{dim}(\bm{q}) \times \mathit{dim}(\bm{q})}
\end{eqnarray}

return $\bm{W}_{trq}$
   "
   (when (or update? (not (send self :get :torque-regular-matrix)))
     (send self :put :torque-regular-matrix
           (if only-variant?
               (make-matrix _dim-variant-config _dim-variant-config)
             (make-matrix _dim-config _dim-config)))
     (mapcar
      #'(lambda (i tq-max)
          (setf (aref (send self :get :torque-regular-matrix) (+ _dim-theta _dim-wrench _dim-wrench-obj i) (+ _dim-theta _dim-wrench _dim-wrench-obj i))
                (/ 1.0 (* tq-max tq-max))))
      (range _dim-torque)
      (coerce (send self :torque-max-vector) cons))
     )
   (send self :get :torque-regular-matrix)
   )
  (:torque-regular-vector
   (&key (update? t) (only-variant? nil))
   "
\begin{eqnarray}
  \bm{\bar{v}}_{trq} &:=& \bm{\bar{W}}_{trq} \bm{\tau} \\
  &=& \begin{pmatrix} \frac{\tau_1}{\tau_{\mathit{max,1}}^2} \\ \frac{\tau_2}{\tau_{\mathit{max,2}}^2} \\ \vdots \\ \frac{\tau_{\mathit{dim}(\bm{\tau})}}{\tau_{\mathit{max,\mathit{dim}(\bm{\tau})}}^2} \end{pmatrix}
  \in \mathbb{R}^{\mathit{dim}(\bm{\tau})}
\end{eqnarray}

\it{only-variant?} is true:
\begin{eqnarray}
  \bm{v}_{trq} :=
  \bordermatrix{
    & 1 \cr
    \mathit{dim}(\bm{\theta}) & \cr
    \mathit{dim}(\bm{\hat{w}}) & \cr
    \mathit{dim}(\bm{\hat{w}}_{\mathit{obj}}) & \cr
    \mathit{dim}(\bm{\tau}) & \bm{\bar{v}}_{trq} \cr
  }
  \in \mathbb{R}^{\mathit{dim}(\bm{q}_{\mathit{var}})}
\end{eqnarray}

otherwise:
\begin{eqnarray}
  \bm{v}_{trq} :=
  \bordermatrix{
    & 1 \cr
    \mathit{dim}(\bm{\theta}) & \cr
    \mathit{dim}(\bm{\hat{w}}) & \cr
    \mathit{dim}(\bm{\hat{w}}_{\mathit{obj}}) & \cr
    \mathit{dim}(\bm{\tau}) & \bm{\bar{v}}_{trq} \cr
    \mathit{dim}(\bm{\phi}) & \cr
  }
  \in \mathbb{R}^{\mathit{dim}(\bm{q})}
\end{eqnarray}

return $\bm{v}_{trq}$
   "
   (when (or update? (not (send self :get :torque-regular-vector)))
     (send self :put :torque-regular-vector
           (if only-variant?
               (make-array _dim-variant-config :element-type :float :initial-element 0)
             (make-array _dim-config :element-type :float :initial-element 0)))
     (mapcar
      #'(lambda (i tq tq-max)
          (setf (elt (send self :get :torque-regular-vector) (+ _dim-theta _dim-wrench _dim-wrench-obj i))
                (/ tq (* tq-max tq-max))))
      (range _dim-torque)
      (coerce _torque-vector cons)
      (coerce (send self :torque-max-vector) cons))
     )
   (send self :get :torque-regular-vector)
   )
  )

(defmethod instant-manipulation-configuration-task
  (:collision-inequality-constraint-matrix
   (&key (update? nil))
   "
\begin{eqnarray}
  \bm{C}_{\mathit{col}} :=
  \bordermatrix{
    & \mathit{dim}(\bm{\theta}) & \mathit{dim}(\bm{\hat{w}}) & \mathit{dim}(\bm{\hat{w}}_{\mathit{obj}}) & \mathit{dim}(\bm{\tau}) & \mathit{dim}(\bm{\phi}) \cr
    N_{\mathit{col}} & \bm{C}_{\mathit{col},\theta} & \bm{O} & \bm{O} & \bm{O} & \bm{C}_{\mathit{col,\phi}}
  }
\end{eqnarray}

return $\bm{C}_{\mathit{col}} \in \mathbb{R}^{N_{\mathit{col}} \times \mathit{dim}(\bm{q})}$
   "
   (when update?
     (send self :update-collision-inequality-constraint))
   (concatenate-matrix-row
    (send self :collision-theta-inequality-constraint-matrix)
    (make-matrix _num-collision (+ _dim-wrench _dim-wrench-obj _dim-torque))
    (send self :collision-phi-inequality-constraint-matrix))
   )
  )

(defmethod instant-manipulation-configuration-task
  (:update-viewer
   ()
   "
Update viewer.
   "
   (send-super :update-viewer)
   (send-all _contact-constraint-obj-list :draw-on :flush nil)
   (draw-force-value
    :force-list (send self :force-obj-list)
    :moment-list (send self :moment-obj-list)
    :coords-list _contact-target-coords-obj-list
    :draw-cop? nil
    :force-color (float-vector 1.0 0.7 0.3)
    )
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
        (warning-message 9 "  |kin-task-i|=~a~%" (norm kin-task-i)))
    (divide-vector (send self :kinematics-task-value :update? nil) :segment-length 6)
    )
   (warning-message 9 "|eom-trans-task|=~a~%" (norm (send self :eom-trans-task-value :update? nil)))
   (warning-message 9 "|eom-rot-task|=~a~%" (norm (send self :eom-rot-task-value :update? nil)))
   (warning-message 9 "|eom-trans-obj-task|=~a~%" (norm (send self :eom-trans-obj-task-value :update? nil)))
   (warning-message 9 "|eom-rot-obj-task|=~a~%" (norm (send self :eom-rot-obj-task-value :update? nil)))
   (warning-message 9 "|torque-task|=~a~%" (norm (send self :torque-task-value :update? nil)))
   (warning-message 9 "|posture-task|=~a~%" (norm (send self :posture-task-value :update? nil)))
   )
  (:_get-contact-index-from-limb ;; used in trajectory-configuration-task
   (&key (limb))
   (position-if
    #'(lambda (att-trg-coords-list)
        ;; if target-coords is contained in act-react-pair-list, exclude the contact
        (if (not (find (elt att-trg-coords-list 1) (mapcar #'(lambda (x) (elt x 0)) _act-react-pair-list)))
            (some #'(lambda (limb-link)
                      (string= (send (send (elt att-trg-coords-list 0) :parent) :name) (send limb-link :name)))
                  (send _robot-env :robot limb))
          ))
    (mapcar
     #'list
     _contact-attention-coords-list
     _contact-target-coords-list)
    ))
  )
