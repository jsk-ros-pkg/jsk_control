(load "./util.l")
(load "./extended-joint-link.l")
(load "./robot-environment.l")
(load "./torque-gradient.l")
(load "package://eus_qp/euslisp/contact-optimization.l")


;;;;;;;;;;;;;;;;;;;;;;;;;
;; instant-configuration-task
;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass instant-configuration-task
  :super propertied-object
  :slots ((_robot-env "robot-environment instance")
          ;; configuration vector
          (_theta-vector "$\bm{\theta}$ [rad] [m]")
          (_wrench-vector "$\bm{\hat{w}}$ [N] [Nm]")
          (_torque-vector "$\bm{\tau}$ [Nm]")
          (_phi-vector "$\bm{\phi}$ [rad] [m]")
          ;; number
          (_num-kin "$N_{\mathit{kin}} := |\mathcal{T}^{\mathit{kin\mathchar`-trg}}| = |\mathcal{T}^{\mathit{kin\mathchar`-att}}|$")
          (_num-contact "$N_{\mathit{cnt}} := |\mathcal{T}^{\mathit{cnt\mathchar`-trg}}| = |\mathcal{T}^{\mathit{cnt\mathchar`-att}}|$")
          (_num-variant-joint "$N_\mathit{var\mathchar`-joint} := |\mathcal{J}_{\mathit{var}}|$")
          (_num-invariant-joint "$N_\mathit{invar\mathchar`-joint} := |\mathcal{J}_{\mathit{invar}}|$")
          (_num-drive-joint "$N_\mathit{drive\mathchar`-joint} := |\mathcal{J}_{\mathit{drive}}|$")
          (_num-posture-joint "$N_{\mathit{posture\mathchar`-joint}} := |\mathcal{J}_{\mathit{posture}}|$")
          (_num-external "$N_{\mathit{ex}} :=$ number of external wrenches")
          (_num-collision "$N_{\mathit{col}} :=$ number of collision check pairs")
          ;; dimensions
          (_dim-theta "$\mathit{dim}(\bm{\theta}) = N_\mathit{var\mathchar`-joint}$")
          (_dim-wrench "$\mathit{dim}(\bm{\hat{w}}) = 6 N_{\mathit{cnt}}$")
          (_dim-torque "$\mathit{dim}(\bm{\tau}) = N_\mathit{drive\mathchar`-joint}$")
          (_dim-phi "$\mathit{dim}(\bm{\phi}) = N_\mathit{invar\mathchar`-joint}$")
          (_dim-variant-config "$\mathit{dim}(\bm{q}_{\mathit{var}})$")
          (_dim-invariant-config "$\mathit{dim}(\bm{q}_{\mathit{invar}})$")
          (_dim-config "$\mathit{dim}(\bm{q})$")
          (_dim-task "$\mathit{dim}(\bm{e})$")
          ;; scale
          (_kin-scale-mat-list "$K_{\mathit{kin}}$")
          (_target-posture-scale-list "$k_{\mathit{posture}}$")
          (_norm-regular-scale-max "$k_{\mathit{max}}$")
          (_norm-regular-scale-coeff "$k_{\mathit{coeff}}$")
          (_norm-regular-scale-offset "$k_{\mathit{off}}$")
          (_torque-regular-scale "$k_{\mathit{trq}}$")
          (_wrench-maximize-scale "$k_{\mathit{w\mathchar`-max}}$")
          ;; variables
          (_variant-joint-list "$\mathcal{J}_{\mathit{var}}$")
          (_invariant-joint-list "$\mathcal{J}_{\mathit{invar}}$")
          (_drive-joint-list "$\mathcal{J}_{\mathit{drive}}$")
          (_kin-target-coords-list "$\mathcal{T}^{\mathit{kin\mathchar`-trg}}$")
          (_kin-attention-coords-list "$\mathcal{T}^{\mathit{kin\mathchar`-att}}$")
          (_contact-target-coords-list "$\mathcal{T}^{\mathit{cnt\mathchar`-trg}}$")
          (_contact-attention-coords-list "$\mathcal{T}^{\mathit{cnt\mathchar`-att}}$")
          (_variant-joint-angle-margin "margin of $\bm{\theta}$ [deg] [mm]")
          (_invariant-joint-angle-margin "margin of $\bm{\phi}$ [deg] [mm]")
          (_delta-linear-joint "trust region of linear joint configuration [mm]")
          (_delta-rotational-joint "trust region of rotational joint configuration [deg]")
          (_contact-constraint-list "list of contact-constraint instance")
          (_posture-joint-list "$\mathcal{J}_{\mathit{posture}}$")
          (_posture-joint-angle-list "$\bm{\bar{\theta}}^{\mathit{trg}}$")
          (_external-wrench-list "$\{ \bm{w}_1^{\mathit{ex}}, \bm{w}_2^{\mathit{ex}}, \cdots, \bm{w}_{N_{ex}}^{\mathit{ex}} \}$")
          (_external-coords-list "$\{T_1^{\mathit{ex}}, T_2^{\mathit{ex}}, \cdots, T_{N_{\mathit{ex}}}^{\mathit{ex}} \}$")
          (_wrench-maximize-direction-list "$\{ \bm{d}_1^{\mathit{w\mathchar`-max}}, \bm{d}_2^{\mathit{w\mathchar`-max}}, \cdots, \bm{d}_{N_{\mathit{cnt}}}^{\mathit{w\mathchar`-max}} \}$")
          (_collision-pair-list "list of bodyset-link or body pair")
          (_collision-distance-margin-list "list of collision distance margin")
          (_only-kinematics? "whether to consider only kinematics or not")
          ;; buffer
          (_variant-task-jacobi "buffer for $\frac{\partial \bm{e}}{\partial \bm{q}_{\mathit{var}}}$")
          (_invariant-task-jacobi "buffer for $\frac{\partial \bm{e}}{\partial \bm{q}_{\mathit{invar}}}$")
          (_task-jacobi "buffer for $\frac{\partial \bm{e}}{\partial \bm{q}}$")
          (_collision-theta-inequality-constraint-matrix "buffer for $\bm{C}_{\mathit{col},\theta}$")
          (_collision-phi-inequality-constraint-matrix "buffer for $\bm{C}_{\mathit{col,\phi}}$")
          (_collision-inequality-constraint-vector "buffer for $\bm{C}_{\mathit{col}}$")
          )
  :documentation
  "
瞬時コンフィギュレーション$\bm{q}^{(l)}$と瞬時タスク関数$\bm{e}^{(l)}(\bm{q}^{(l)})$のクラス．

このクラスの説明で用いる全ての変数は，時間ステップ$l$を表す添字をつけて$\bm{x}^{(l)}$と表されるべきだが，このクラス内の説明では省略して$\bm{x}$と表す．
また，以降では，説明文やメソッド名で，``瞬時''や``instant''を省略する．

コンフィギュレーション$\bm{q}$の取得・更新，タスク関数$\bm{e}(\bm{q})$の取得，タスク関数のヤコビ行列$\frac{\partial \bm{e}(\bm{q})}{\partial \bm{q}}$の取得，コンフィギュレーションの等式・不等式制約$\bm{A}, \bm{b}, \bm{C}, \bm{d}$の取得のためのメソッドが定義されている．

コンフィギュレーション・タスク関数を定めるために，初期化時に以下を与える
\begin{itemize}
\setlength{\itemindent}{10mm}
\setlength{\leftskip}{-10mm}
\item ロボット・環境
\begin{description}
  \item[robot-environment] ロボット・環境を表すrobot-environmentクラスのインスタンス
  \item[variant-joint-list $\mathcal{J}_{\mathit{var}}$] 時変関節
  \item[invariant-joint-list $\mathcal{J}_{\mathit{invar}}$] 時不変関節(与えなければ時不変関節は考慮されない)
  \item[drive-joint-list $\mathcal{J}_{\mathit{drive}}$] 駆動関節(与えなければ関節駆動トルクは考慮されない)
\end{description}
\item 幾何拘束
\begin{description}
  \item[kin-target-coords-list $\mathcal{T}^{\mathit{kin\mathchar`-trg}}$] 幾何到達目標位置姿勢リスト
  \item[kin-attention-coords-list $\mathcal{T}^{\mathit{kin\mathchar`-att}}$] 幾何到達着目位置姿勢リスト
  \item[kin-scale-mat-list $K_{\mathit{kin}}$] 幾何拘束の座標系，重みを表す変換行列のリスト
\end{description}
\item 接触拘束
\begin{description}
  \item[contact-target-coords-list $\mathcal{T}^{\mathit{cnt\mathchar`-trg}}$] 接触目標位置姿勢リスト
  \item[contact-attention-coords-list $\mathcal{T}^{\mathit{cnt\mathchar`-att}}$] 接触着目位置姿勢リスト
  \item[contact-constraint-list] 接触レンチ制約リスト
\end{description}
\item コンフィギュレーション拘束(必要な場合のみ)
\begin{description}
  \item[posture-joint-list $\mathcal{J}_{\mathit{posture}}$] 着目関節リスト
  \item[posture-joint-angle-list $\bm{\bar{\theta}}^{\mathit{trg}}$] 着目関節の目標関節角
  \item[target-posture-scale $k_{\mathit{posture}}$] コンフィギュレーション拘束の重み
\end{description}
\item 干渉回避拘束(必要な場合のみ)
\begin{description}
  \item[collision-pair-list] 干渉回避するbodyset-linkもしくはbodyのペアのリスト
  \item[collision-distance-margin] 干渉回避の距離マージン(全てのペアで同じ値の場合)
  \item[collision-distance-margin-list] 干渉回避の距離マージンのリスト(ペアごとに異なる値の場合)
\end{description}
\item 外レンチ(必要な場合のみ)
\begin{description}
  \item[external-wrench-list] 外レンチのリスト(ワールド座標系で表す)
  \item[external-coords-list] 外レンチの作用点座標のリスト(位置のみを使用)
\end{description}
\item 接触力最大化(必要な場合のみ)
\begin{description}
  \item[wrench-maximize-direction-list] 接触レンチ最大化方向(ワールド座標系で表す)
\end{description}
\item 目的関数の重み
\begin{description}
  \item[norm-regular-scale-max $k_{\mathit{max}}$] コンフィギュレーション更新量正則化の重み最大値
  \item[norm-regular-scale-coeff $k_{\mathit{coeff}}$] コンフィギュレーション更新量正則化の係数
  \item[norm-regular-scale-offset $k_{\mathit{off}}$] コンフィギュレーション更新量正則化の重みオフセット
  \item[torque-regular-scale $k_{\mathit{trq}}$] トルク正則化の重み
  \item[wrench-maximize-scale $k_{\mathit{w\mathchar`-max}}$] 接触レンチ最大化の重み
\end{description}
\end{itemize}

コンフィギュレーション$\bm{q}$は以下から構成される．
\begin{eqnarray}
  \bm{q} := \begin{pmatrix} \bm{\theta}^T & \bm{\hat{w}}^T & \bm{\tau}^T & \bm{\phi}^T \end{pmatrix}^T
\end{eqnarray}
\begin{description}
  \setlength{\itemindent}{10mm}
  \item[$\bm{\theta} \in \mathbb{R}^{N_\mathit{var\mathchar`-joint}}$] 時変関節角度 [rad] [m]
  \item[$\bm{\hat{w}} \in \mathbb{R}^{6 N_{\mathit{cnt}}}$] 接触レンチ [N] [Nm]
  \item[$\bm{\tau} \in \mathbb{R}^{N_\mathit{drive\mathchar`-joint}}$] 関節駆動トルク [Nm] [N]
  \item[$\bm{\phi} \in \mathbb{R}^{N_\mathit{invar\mathchar`-joint}}$] 時不変関節角度 [rad] [m]
\end{description}
$\bm{\hat{w}}$は次式のように，全接触部位でのワールド座標系での力・モーメントを並べたベクトルである．
\begin{eqnarray}
  \bm{\hat{w}} &=& \begin{pmatrix} \bm{w}_1^T & \bm{w}_2^T & \cdots & \bm{w}_{N_{\mathit{cnt}}}^T \end{pmatrix}^T \\
  &=& \begin{pmatrix} \bm{f}_1^T &  \bm{n}_1^T &  \bm{f}_2^T &  \bm{n}_2^T & \cdots & \bm{f}_{N_{\mathit{cnt}}}^T &  \bm{n}_{N_{\mathit{cnt}}}^T \end{pmatrix}^T
\end{eqnarray}

タスク関数$\bm{e}(\bm{q})$は以下から構成される．
\begin{eqnarray}
  \bm{e}(\bm{q}) := \begin{pmatrix} \bm{e}^{\mathit{kin} T}(\bm{q}) & \bm{e}^{\mathit{eom\mathchar`-trans} T}(\bm{q}) & \bm{e}^{\mathit{eom\mathchar`-rot} T}(\bm{q}) & \bm{e}^{\mathit{trq} T}(\bm{q}) & \bm{e}^{\mathit{posture} T}(\bm{q}) \end{pmatrix}^T
\end{eqnarray}
\begin{description}
  \setlength{\itemindent}{10mm}
  \item[$\bm{e}^{\mathit{kin}}(\bm{q}) \in \mathbb{R}^{6 N_{\mathit{kin}}}$] 幾何到達拘束 [rad] [m]
  \item[$\bm{e}^{\mathit{eom\mathchar`-trans}}(\bm{q}) \in \mathbb{R}^3$] 力の釣り合い [N]
  \item[$\bm{e}^{\mathit{eom\mathchar`-rot}}(\bm{q}) \in \mathbb{R}^3$] モーメントの釣り合い [Nm]
  \item[$\bm{e}^{\mathit{trq}}(\bm{q}) \in \mathbb{R}^{N_{\mathit{drive\mathchar`-joint}}}$] 関節トルクの釣り合い [rad] [m]
  \item[$\bm{e}^{\mathit{posture}}(\bm{q}) \in \mathbb{R}^{N_{\mathit{posture\mathchar`-joint}}}$] 関節角目標 [rad] [m]
\end{description}
  "
  )

(defmethod instant-configuration-task
  (:init
   (&key
    (name)
    (robot-env)
    (variant-joint-list (send robot-env :variant-joint-list))
    (invariant-joint-list (send robot-env :invariant-joint-list))
    (drive-joint-list (send robot-env :drive-joint-list))
    (only-kinematics?)
    (kin-target-coords-list)
    (kin-attention-coords-list)
    (contact-target-coords-list)
    (contact-attention-coords-list)
    (variant-joint-angle-margin 3.0) ;; [deg] [mm]
    (invariant-joint-angle-margin 3.0) ;; [deg] [mm]
    (delta-linear-joint) ;; [mm]
    (delta-rotational-joint) ;; [deg]
    (contact-constraint-list
     (send-all contact-attention-coords-list :get :contact-constraint))
    (posture-joint-list)
    (posture-joint-angle-list) ;; [deg] [mm]
    (external-wrench-list)
    (external-coords-list)
    (wrench-maximize-direction-list)
    (collision-pair-list)
    (collision-distance-margin 0.01) ;; [m]
    (collision-distance-margin-list) ;; [m]
    (kin-scale 1.0)
    (kin-scale-list)
    (kin-scale-mat-list)
    (target-posture-scale 1e-3)
    (target-posture-scale-list)
    (norm-regular-scale-max (if only-kinematics? 1e-3 1e-5))
    (norm-regular-scale-coeff 1.0)
    (norm-regular-scale-offset 1e-7)
    (torque-regular-scale 1e-4)
    (wrench-maximize-scale 0)
    &allow-other-keys
    )
   "
Initialize instance
   "
   (send self :name name)
   (setq _robot-env robot-env)
   (setq _variant-joint-list variant-joint-list)
   (setq _invariant-joint-list invariant-joint-list)
   (setq _drive-joint-list drive-joint-list)
   (setq _only-kinematics? only-kinematics?)
   (setq _kin-target-coords-list kin-target-coords-list)
   (setq _kin-attention-coords-list kin-attention-coords-list)
   (setq _contact-target-coords-list contact-target-coords-list)
   (setq _contact-attention-coords-list contact-attention-coords-list)
   (setq _variant-joint-angle-margin variant-joint-angle-margin)
   (setq _invariant-joint-angle-margin invariant-joint-angle-margin)
   (setq _delta-linear-joint delta-linear-joint)
   (setq _delta-rotational-joint delta-rotational-joint)
   (setq _contact-constraint-list contact-constraint-list)
   (setq _posture-joint-list posture-joint-list)
   (setq _posture-joint-angle-list posture-joint-angle-list)
   (setq _external-wrench-list external-wrench-list)
   (setq _external-coords-list external-coords-list)
   (setq _wrench-maximize-direction-list wrench-maximize-direction-list)
   (setq _collision-pair-list collision-pair-list)
   (setq _collision-distance-margin-list
         (or collision-distance-margin-list
             (make-list (length _collision-pair-list) :initial-element collision-distance-margin)))
   (cond (kin-scale-mat-list
          (setq _kin-scale-mat-list kin-scale-mat-list)
          )
         (kin-scale-list
          (setq _kin-scale-mat-list
                (mapcar
                 #'(lambda (sc)
                     (cond ((numberp sc)
                            (scale-matrix sc (unit-matrix 6))
                            )
                           (t
                            (diagonal sc)
                            )))
                 kin-scale-list))
          )
         (kin-scale
          (setq _kin-scale-mat-list
                (make-list (length kin-target-coords-list) :initial-element (scale-matrix kin-scale (unit-matrix 6))))
          ))
   (cond (target-posture-scale-list
          (setq _target-posture-scale-list target-posture-scale-list)
          )
         (t
          (setq _target-posture-scale-list
                (make-list (length posture-joint-list) :initial-element target-posture-scale))
          ))
   (setq _norm-regular-scale-max norm-regular-scale-max)
   (setq _norm-regular-scale-coeff norm-regular-scale-coeff)
   (setq _norm-regular-scale-offset norm-regular-scale-offset)
   (setq _torque-regular-scale torque-regular-scale)
   (setq _wrench-maximize-scale wrench-maximize-scale)
   ;; sub methods
   (send self :_initialize-number)
   (send self :_initialize-configuration-vector)
   (send self :_initialize-dimension)
   (send self :_contact-constraint-matrix-list)
   )
  (:_initialize-number
   ()
   (setq _num-kin (length _kin-target-coords-list))
   (setq _num-contact (length _contact-target-coords-list))
   (setq _num-variant-joint (length _variant-joint-list))
   (setq _num-invariant-joint (length _invariant-joint-list))
   (setq _num-drive-joint (length _drive-joint-list))
   (setq _num-posture-joint (length _posture-joint-list))
   (setq _num-external (length _external-wrench-list))
   (setq _num-collision (length _collision-pair-list))
   )
  (:_initialize-configuration-vector
   ()
   (setq _theta-vector (send self :_get-theta-from-model))
   (setq _wrench-vector (make-array (* 6 _num-contact) :element-type :float))
   (setq _torque-vector (make-array _num-drive-joint :element-type :float))
   (setq _phi-vector (send self :_get-phi-from-model))
   )
  (:_initialize-dimension
   ()
   (setq _dim-theta _num-variant-joint)
   (setq _dim-wrench (* 6 _num-contact))
   (setq _dim-torque _num-drive-joint)
   (setq _dim-phi _num-invariant-joint)
   (setq _dim-variant-config (+ _dim-theta _dim-wrench _dim-torque))
   (setq _dim-invariant-config _dim-phi)
   (setq _dim-config (+ _dim-variant-config _dim-invariant-config))
   (cond (_only-kinematics?
          (setq _dim-task (+ (* 6 _num-kin) _num-posture-joint))
          )
         (t
          (setq _dim-task (+ (* 6 _num-kin) 3 3 _num-drive-joint _num-posture-joint))
          ))
   )
  (:robot-env
   ()
   "
return robot-environment instance
   "
   _robot-env
   )
  (:variant-joint-list
   ()
   "
return $\mathcal{J}_{\mathit{var}}$
   "
   _variant-joint-list
   )
  (:invariant-joint-list
   ()
   "
return $\mathcal{J}_{\mathit{invar}}$
   "
   _invariant-joint-list
   )
  (:drive-joint-list
   ()
   "
return $\mathcal{J}_{\mathit{drive}}$
   "
   _drive-joint-list
   )
  (:only-kinematics?
   ()
   "
return whether to consider only kinematics or not
   "
   _only-kinematics?
   )
  (:theta
   ()
   "
return $\bm{\theta}$
   "
   _theta-vector
   )
  (:wrench
   ()
   "
return $\bm{\hat{w}}$
   "
   _wrench-vector
   )
  (:torque
   ()
   "
return $\bm{\tau}$
   "
   _torque-vector
   )
  (:phi
   ()
   "
return $\bm{\phi}$
   "
   _phi-vector
   )
  (:num-kin
   ()
   "
return $N_{\mathit{kin}} := |\mathcal{T}^{\mathit{kin\mathchar`-trg}}| = |\mathcal{T}^{\mathit{kin\mathchar`-att}}|$
   "
   _num-kin
   )
  (:num-contact
   ()
   "
return $N_{\mathit{cnt}} := |\mathcal{T}^{\mathit{cnt\mathchar`-trg}}| = |\mathcal{T}^{\mathit{cnt\mathchar`-att}}|$
   "
   _num-contact
   )
  (:num-variant-joint
   ()
   "
return $N_\mathit{var\mathchar`-joint} := |\mathcal{J}_{\mathit{var}}|$
   "
   _num-variant-joint
   )
  (:num-invariant-joint
   ()
   "
return $N_\mathit{invar\mathchar`-joint} := |\mathcal{J}_{\mathit{invar}}|$
   "
   _num-invariant-joint
   )
  (:num-drive-joint
   ()
   "
return $N_\mathit{drive\mathchar`-joint} := |\mathcal{J}_{\mathit{drive}}|$
   "
   _num-drive-joint
   )
  (:num-posture-joint
   ()
   "
return $N_\mathit{target\mathchar`-joint} := |\mathcal{J}_{\mathit{target}}|$
   "
   _num-posture-joint
   )
  (:num-external
   ()
   "
return $N_{\mathit{ex}} :=$ number of external wrench
   "
   _num-external
   )
  (:num-collision
   ()
   "
return $N_{\mathit{col}} :=$ number of collision check pairs
   "
   _num-collision
   )
  (:dim-variant-config
   ()
   "
\begin{eqnarray}
\mathit{dim}(\bm{q_{\mathit{var}}})
&:=& \mathit{dim}(\bm{\theta}) + \mathit{dim}(\bm{\hat{w}}) + \mathit{dim}(\bm{\tau}) \\
&=& N_\mathit{var\mathchar`-joint} + 6 N_{\mathit{cnt}} + N_\mathit{drive\mathchar`-joint}
\end{eqnarray}

return $\mathit{dim}(\bm{q_{\mathit{var}}})$
   "
   _dim-variant-config
   )
  (:dim-invariant-config
   ()
   "
return $\mathit{dim}(\bm{q_{\mathit{invar}}}) := \mathit{dim}(\bm{\phi}) = N_\mathit{invar\mathchar`-joint}$
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
\begin{eqnarray}
\mathit{dim}(\bm{e})
&:=& \mathit{dim}(\bm{e}^{\mathit{kin}}) + \mathit{dim}(\bm{e}^{\mathit{eom\mathchar`-trans}}) + \mathit{dim}(\bm{e}^{\mathit{eom\mathchar`-rot}}) + \mathit{dim}(\bm{e}^{\mathit{trq}}) + \mathit{dim}(\bm{e}^{\mathit{posture}}) \\
&=& 6 N_{\mathit{kin}} + 3 + 3 + N_{\mathit{drive\mathchar`-joint}} + N_{\mathit{posture\mathchar`-joint}}
\end{eqnarray}

return $\mathit{dim}(\bm{e})$
   "
   _dim-task
   )
  (:variant-config-vector
   ()
   "
return $\bm{q_{\mathit{var}}} := \begin{pmatrix} \bm{\theta} \\ \bm{\hat{w}} \\ \bm{\tau} \end{pmatrix}$
   "
   (concatenate
    float-vector
    _theta-vector
    _wrench-vector
    _torque-vector
    )
   )
  (:invariant-config-vector
   ()
   "
return $\bm{q_{\mathit{invar}}} := \bm{\phi}$
   "
   _phi-vector
   )
  (:config-vector
   ()
   "
return $\bm{q} := \begin{pmatrix} \bm{q_{\mathit{var}}} \\ \bm{q_{\mathit{invar}}} \end{pmatrix} = \begin{pmatrix} \bm{\theta} \\ \bm{\hat{w}} \\ \bm{\tau} \\ \bm{\phi} \end{pmatrix}$
   "
   (concatenate
    float-vector
    (send self :variant-config-vector)
    (send self :invariant-config-vector)
    )
   )
  (:set-theta
   (theta-new
    &key
    (relative? nil)
    (apply-to-robot? t)
    )
   "
Set $\bm{\theta}$.
   "
   (cond (relative?
          (v+ _theta-vector theta-new _theta-vector)
          )
         (t
          (setq _theta-vector theta-new)
          ))
   (when apply-to-robot?
     (send self :_apply-theta-to-model))
   )
  (:set-wrench
   (wrench-new
    &key
    (relative? nil)
    )
   "
Set $\bm{\hat{w}}$.
   "
   (cond (relative?
          (v+ _wrench-vector wrench-new _wrench-vector)
          )
         (t
          (setq _wrench-vector wrench-new)
          ))
   )
  (:set-torque
   (torque-new
    &key
    (relative? nil)
    )
   "
Set $\bm{\tau}$.
   "
   (cond (relative?
          (v+ _torque-vector torque-new _torque-vector)
          )
         (t
          (setq _torque-vector torque-new)
          ))
   )
  (:set-phi
   (phi-new
    &key
    (relative? nil)
    (apply-to-robot? t)
    )
   "
Set $\bm{\phi}$.
   "
   (cond (relative?
          (v+ _phi-vector phi-new _phi-vector)
          )
         (t
          (setq _phi-vector phi-new)
          ))
   (when apply-to-robot?
     (send self :_apply-phi-to-model))
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
   (send self :set-torque
         (subseq variant-config-new (+ _dim-theta _dim-wrench) (+ _dim-theta _dim-wrench _dim-torque)) :relative? relative?)
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
   (send self :set-phi
         invariant-config-new :relative? relative? :apply-to-robot? apply-to-robot?)
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
  (:_get-theta-from-model
   ()
   (map float-vector
        #'(lambda (j)
            (send j :angle-to-speed (send j :joint-angle)))
        _variant-joint-list)
   )
  (:_get-phi-from-model
   ()
   (map float-vector
        #'(lambda (j)
            (send j :angle-to-speed (send j :joint-angle)))
        _invariant-joint-list)
   )
  (:_apply-theta-to-model
   ()
   (mapcar
    #'(lambda (j theta)
        (send j :joint-angle (send j :speed-to-angle theta))) ;; [mm] [deg]
    _variant-joint-list
    (coerce _theta-vector cons))
   )
  (:_apply-phi-to-model
   ()
   (mapcar
    #'(lambda (j phi)
        (send j :joint-angle (send j :speed-to-angle phi))) ;; [mm] [deg]
    _invariant-joint-list
    (coerce _phi-vector cons))
   )
  )

(defmethod instant-configuration-task
  (:kin-target-coords-list
   ()
   "
\begin{eqnarray}
  T_m^{\mathit{kin\mathchar`-trg}} = \{\bm{p}_{m}^{\mathit{kin\mathchar`-trg}}, \bm{R}_{m}^{\mathit{kin\mathchar`-trg}}\} \ \ (m = 1,2,\cdots,N_{\mathit{kin}})
\end{eqnarray}

return $\mathcal{T}^{\mathit{kin\mathchar`-trg}} := \{T_1^{\mathit{kin\mathchar`-trg}}, T_2^{\mathit{kin\mathchar`-trg}}, \cdots, T_{N_\mathit{kin}}^{\mathit{kin\mathchar`-trg}} \}$
   "
   _kin-target-coords-list
   )
  (:kin-attention-coords-list
   ()
   "
\begin{eqnarray}
  T_m^{\mathit{kin\mathchar`-att}} = \{\bm{p}_{m}^{\mathit{kin\mathchar`-att}}, \bm{R}_{m}^{\mathit{kin\mathchar`-att}}\} \ \ (m = 1,2,\cdots,N_{\mathit{kin}})
\end{eqnarray}

return $\mathcal{T}^{\mathit{kin\mathchar`-att}} := \{T_1^{\mathit{kin\mathchar`-att}}, T_2^{\mathit{kin\mathchar`-att}}, \cdots, T_{N_\mathit{kin}}^{\mathit{kin\mathchar`-att}} \}$
   "
   _kin-attention-coords-list
   )
  (:contact-target-coords-list
   ()
   "
\begin{eqnarray}
  T_m^{\mathit{cnt\mathchar`-trg}} = \{\bm{p}_{m}^{\mathit{cnt\mathchar`-trg}}, \bm{R}_{m}^{\mathit{cnt\mathchar`-trg}}\} \ \ (m = 1,2,\cdots,N_{\mathit{cnt}})
\end{eqnarray}

return $\mathcal{T}^{\mathit{cnt\mathchar`-trg}} := \{T_1^{\mathit{cnt\mathchar`-trg}}, T_2^{\mathit{cnt\mathchar`-trg}}, \cdots, T_{N_\mathit{cnt}}^{\mathit{cnt\mathchar`-trg}} \}$
   "
   _contact-target-coords-list
   )
  (:contact-attention-coords-list
   ()
   "
\begin{eqnarray}
  T_m^{\mathit{cnt\mathchar`-att}} = \{\bm{p}_{m}^{\mathit{cnt\mathchar`-att}}, \bm{R}_{m}^{\mathit{cnt\mathchar`-att}}\} \ \ (m = 1,2,\cdots,N_{\mathit{cnt}})
\end{eqnarray}

return $\mathcal{T}^{\mathit{cnt\mathchar`-att}} := \{T_1^{\mathit{cnt\mathchar`-att}}, T_2^{\mathit{cnt\mathchar`-att}}, \cdots, T_{N_\mathit{cnt}}^{\mathit{cnt\mathchar`-att}} \}$
   "
   _contact-attention-coords-list
   )
  (:contact-constraint-list
   ()
   "
return list of contact-constraint instance
   "
   _contact-constraint-list
   )
  (:wrench-list
   ()
   "
return $\{ \bm{w}_1, \bm{w}_2, \cdots, \bm{w}_{N_{\mathit{cnt}}} \}$
   "
   (divide-vector _wrench-vector :segment-length 6)
   )
  (:force-list
   ()
   "
return $\{ \bm{f}_1, \bm{f}_2, \cdots, \bm{f}_{N_{\mathit{cnt}}} \}$
   "
   (divide-vector _wrench-vector :segment-length 3 :interval-length 3)
   )
  (:moment-list
   ()
   "
return $\{ \bm{n}_1, \bm{n}_2, \cdots, \bm{n}_{N_{\mathit{cnt}}} \}$
   "
   (divide-vector (subseq _wrench-vector 3) :segment-length 3 :interval-length 3)
   )
  (:external-wrench-list
   (&optional (new-external-wrench-list :nil))
   "
set / get $\{ \bm{w}_1^{\mathit{ex}}, \bm{w}_2^{\mathit{ex}}, \cdots, \bm{w}_{N_{\mathit{ex}}}^{\mathit{ex}} \}$
"
   (cond ((equal new-external-wrench-list :nil)
          _external-wrench-list
          )
         (t
          (setq _external-wrench-list new-external-wrench-list)
          ))
   )
  (:external-force-list
   ()
"
return $\{ \bm{f}_1^{\mathit{ex}}, \bm{f}_2^{\mathit{ex}}, \cdots, \bm{f}_{N_{\mathit{ex}}}^{\mathit{ex}} \}$
"
   (mapcar #'(lambda (w) (subseq w 0 3)) _external-wrench-list)
   )
  (:external-moment-list
   ()
"
return $\{ \bm{n}_1^{\mathit{ex}}, \bm{n}_2^{\mathit{ex}}, \cdots, \bm{n}_{N_{\mathit{ex}}}^{\mathit{ex}} \}$
"
   (mapcar #'(lambda (w) (subseq w 3 6)) _external-wrench-list)
   )
  (:mg-vec
   ()
   "
return $m \bm{g}$
   "
   (or (send self :get :mg-vec)
       (send self :put :mg-vec
             (scale (* 1e-3 (send _robot-env :robot :weight)) (scale 1e-3 *g-vec*)))) ;; [N]
   )
  (:cog
   (&key (update? t))
   "
return $\bm{p}_G(\bm{q})$
   "
   (scale 1e-3 (send _robot-env :robot :centroid update?))
   )
  )

(defmethod instant-configuration-task
  (:kinematics-task-value
   (&key (update? t))
   "
\begin{eqnarray}
  \bm{e}^{\mathit{kin}}(\bm{q})
  &=& \bm{e}^{\mathit{kin}}(\bm{\theta}, \bm{\phi}) \\
  &=& \begin{pmatrix}
    \bm{e}_{1}^{\mathit{kin}}(\bm{\theta}, \bm{\phi}) \\ \bm{e}_{2}^{\mathit{kin}}(\bm{\theta}, \bm{\phi}) \\ \vdots \\ \bm{e}_{N_{\mathit{kin}}}^{\mathit{kin}}(\bm{\theta}, \bm{\phi})
  \end{pmatrix} \\
  \bm{e}_{m}^{\mathit{kin}}(\bm{\theta}, \bm{\phi}) &=&
  K_{\mathit{kin}}
  \begin{pmatrix}
    \bm{p}_{m}^{\mathit{kin\mathchar`-trg}}(\bm{\theta}, \bm{\phi}) - \bm{p}_{m}^{\mathit{kin\mathchar`-att}}(\bm{\theta}, \bm{\phi}) \\
    \bm{a}\left(\bm{R}_{m}^{\mathit{kin\mathchar`-trg}}(\bm{\theta}, \bm{\phi}) \bm{R}_{m}^{\mathit{kin\mathchar`-att}}(\bm{\theta}, \bm{\phi})^T\right)
  \end{pmatrix} \in \mathbb{R}^6 \ \ (m = 1,2,\cdots,N_{\mathit{kin}})
\end{eqnarray}
$\bm{a}(\bm{R})$は姿勢行列$\bm{R}$の等価角軸ベクトルを表す．

return $\bm{e}^{\mathit{kin}}(\bm{q}) \in \mathbb{R}^{6 N_{\mathit{kin}}}$
   "
   (when (or update? (not (send self :get :kinematics-task-value)))
     (send self :put :kinematics-task-value
           (apply #'concatenate
                  float-vector
                  (mapcar
                   #'(lambda (trg-coords att-coords sc-mat)
                       (let* ((pos-diff ;; [m]
                               (cond ((equal att-coords :cog)
                                      (v- (scale 1e-3 (send trg-coords :worldpos)) (send self :cog))
                                      )
                                     (t
                                      (scale 1e-3 (v- (send trg-coords :worldpos) (send att-coords :worldpos)))
                                      )))
                              (rot-diff ;; [rad]
                               (cond ((equal att-coords :cog)
                                      (float-vector 0 0 0)
                                      )
                                     (t
                                      (matrix-log (m* (send trg-coords :worldrot) (transpose (send att-coords :worldrot))))
                                      )))
                              )
                         (transform sc-mat (concatenate float-vector pos-diff rot-diff))
                         ))
                   _kin-target-coords-list
                   _kin-attention-coords-list
                   _kin-scale-mat-list
                   ))
           ))
   (send self :get :kinematics-task-value)
   )
  (:eom-trans-task-value
   (&key (update? t))
   "
\begin{eqnarray}
  \bm{e}^{\mathit{eom\mathchar`-trans}}(\bm{q})
  &=& \bm{e}^{\mathit{eom\mathchar`-trans}}(\bm{\hat{w}}) \\
  &=& \sum_{m=1}^{N_{\mathit{cnt}}} \bm{f}_m - m \bm{g} + \sum_{m=1}^{N_{\mathit{ex}}} \bm{f}_{m}^{\mathit{ex}}
\end{eqnarray}

return $\bm{e}^{\mathit{eom\mathchar`-trans}}(\bm{q}) \in \mathbb{R}^3$
   "
   (when (or update? (not (send self :get :eom-trans-task-value)))
     (send self :put :eom-trans-task-value
           (apply
            #'v++
            (append
             (send self :force-list)
             (list (scale -1 (send self :mg-vec)))
             (send self :external-force-list)
             ))
           ))
   (send self :get :eom-trans-task-value)
   )
  (:eom-rot-task-value
   (&key (update? t))
   "
\begin{eqnarray}
  \bm{e}^{\mathit{eom\mathchar`-rot}}(\bm{q})
  &=& \bm{e}^{\mathit{eom\mathchar`-rot}}(\bm{\theta}, \bm{\hat{w}}, \bm{\phi}) \\
  &=& \sum_{m=1}^{N_{\mathit{cnt}}} \left\{ \left(\bm{p}_m^{\mathit{cnt\mathchar`-trg}}(\bm{\theta}, \bm{\phi}) - \bm{p}_G(\bm{\theta}, \bm{\phi})\right) \times \bm{f}_m + \bm{n}_m \right\} \nonumber\\
  && + \sum_{m=1}^{N_{\mathit{ex}}} \left\{ \left(\bm{p}_m^{\mathit{ex}}(\bm{\theta}, \bm{\phi}) - \bm{p}_G(\bm{\theta}, \bm{\phi})\right) \times \bm{f}_m^{\mathit{ex}} + \bm{n}_m^{\mathit{ex}} \right\}
\end{eqnarray}

return $\bm{e}^{\mathit{eom\mathchar`-rot}}(\bm{q}) \in \mathbb{R}^3$
   "
   (when (or update? (not (send self :get :eom-rot-task-value)))
     (send self :put :eom-rot-task-value
           (apply
            #'v++
            (append
             (mapcar
              #'(lambda (f n p) (v+ (v* (v- p (send self :cog)) f) n))
              (send self :force-list)
              (send self :moment-list)
              (mapcar #'(lambda (c) (scale 1e-3 (send c :worldpos))) _contact-target-coords-list) ;; [m]
              )
             (mapcar
              #'(lambda (f n p) (v+ (v* (v- p (send self :cog)) f) n))
              (send self :external-force-list)
              (send self :external-moment-list)
              (mapcar #'(lambda (c) (scale 1e-3 (send c :worldpos))) _external-coords-list) ;; [m]
              ))
            )
           ))
   (send self :get :eom-rot-task-value)
   )
  (:torque-task-value
   (&key (update? t))
   "
\begin{eqnarray}
  \bm{e}^{\mathit{trq}}(\bm{q})
  &=& \bm{e}^{\mathit{trq}}(\bm{\theta}, \bm{\hat{w}}, \bm{\tau}, \bm{\phi}) \\
  &=& \bm{\tau} + \sum_{m=1}^{N_{\mathit{cnt}}} \bm{\tau}^{\mathit{cnt}}_m(\bm{\theta}, \bm{\phi}) - \bm{\tau}^{\mathit{grav}}(\bm{\theta}, \bm{\phi}) + \sum_{m=1}^{N_{\mathit{ex}}} \bm{\tau}^{\mathit{ex}}_m(\bm{\theta}, \bm{\phi}) \\
  &=& \bm{\tau} + \sum_{m=1}^{N_{\mathit{cnt}}} \bm{J}^{\mathit{cnt\mathchar`-trg}}_{\mathit{drive\mathchar`-joint},m}(\bm{\theta}, \bm{\phi})^T \bm{w}_m - \bm{\tau}^{\mathit{grav}}(\bm{\theta}, \bm{\phi}) + \sum_{m=1}^{N_{\mathit{ex}}} \bm{J}^{\mathit{ex}}_{\mathit{drive\mathchar`-joint},m}(\bm{\theta}, \bm{\phi})^T \bm{w}_m^{\mathit{ex}}
\end{eqnarray}
$\bm{\tau}^{\mathit{cnt}}_m(\bm{\theta}, \bm{\phi})$は$m$番目の接触部位にかかる接触レンチ$\bm{w}_m$による関節トルク，
$\bm{\tau}^{\mathit{grav}}_m(\bm{\theta}, \bm{\phi})$は自重による関節トルクを表す．

return $\bm{e}^{\mathit{trq}}(\bm{q}) \in \mathbb{R}^{N_{\mathit{drive\mathchar`-joint}}}$
   "
   (when (or update? (not (send self :get :torque-task-value)))
     (send self :put :torque-task-value
           (apply
            #'v++
            (remove
             nil
             (list
              _torque-vector
              (get-contact-torque
               :robot _robot-env
               :drive-joint-list _drive-joint-list
               :wrench-list (send self :wrench-list)
               :contact-target-coords-list _contact-target-coords-list
               :contact-attention-coords-list _contact-attention-coords-list
               )
              (scale
               -1
               (get-gravity-torque
                :robot _robot-env
                :drive-joint-list _drive-joint-list
                :gravity-link-list (send _robot-env :robot :links)
                ))
              (if (> _num-external 0)
                  (get-contact-torque
                   :robot _robot-env
                   :drive-joint-list _drive-joint-list
                   :wrench-list (send self :external-wrench-list)
                   :contact-target-coords-list _external-coords-list
                   :contact-attention-coords-list _external-coords-list
                   ))
              )))
           ))
   (send self :get :torque-task-value)
   )
  (:posture-task-value
   (&key (update? t))
   "
\begin{eqnarray}
  \bm{e}^{\mathit{posture}}(\bm{q})
  &=& \bm{e}^{\mathit{posture}}(\bm{\theta}) \\
  &=& k_{\mathit{posture}} \left( \bm{\bar{\theta}}^{\mathit{trg}} - \bm{\bar{\theta}} \right)
\end{eqnarray}
$\bm{\bar{\theta}}^{\mathit{trg}}, \bm{\bar{\theta}}$は着目関節リスト$\mathcal{J}_{\mathit{posture}}$の目標関節角と現在の関節角．

return $\bm{e}^{\mathit{posture}}(\bm{q}) \in \mathbb{R}^{N_{\mathit{posture\mathchar`-joint}}}$
   "
   (map float-vector
        #'(lambda (j a sc)
            (* sc (send j :angle-to-speed (- a (send j :joint-angle)))))
        _posture-joint-list
        _posture-joint-angle-list
        _target-posture-scale-list
        )
   )
  (:task-value
   (&key (update? t))
   "
return $\bm{e}(\bm{q})
:= \begin{pmatrix} \bm{e}^{\mathit{kin}}(\bm{q}) \\ \bm{e}^{\mathit{eom\mathchar`-trans}}(\bm{q}) \\ \bm{e}^{\mathit{eom\mathchar`-rot}}(\bm{q}) \\ \bm{e}^{\mathit{trq}}(\bm{q}) \\ \bm{e}^{\mathit{posture}}(\bm{q}) \end{pmatrix}
= \begin{pmatrix} \bm{e}^{\mathit{kin}}(\bm{\theta}, \bm{\phi}) \\ \bm{e}^{\mathit{eom\mathchar`-trans}}(\bm{\hat{w}}) \\ \bm{e}^{\mathit{eom\mathchar`-rot}}(\bm{\theta}, \bm{\hat{w}}, \bm{\phi}) \\ \bm{e}^{\mathit{trq}}(\bm{\theta}, \bm{\hat{w}}, \bm{\tau}, \bm{\phi}) \\ \bm{e}^{\mathit{posture}}(\bm{\theta}) \end{pmatrix}$
   "
   (concatenate
    float-vector
    (send self :kinematics-task-value :update? update?)
    (if (not _only-kinematics?) (send self :eom-trans-task-value :update? update?))
    (if (not _only-kinematics?) (send self :eom-rot-task-value :update? update?))
    (if (and (not _only-kinematics?) (> _num-drive-joint 0))
        (send self :torque-task-value :update? update?))
    (if (> _num-posture-joint 0)
        (send self :posture-task-value :update? update?))
    )
   )
  )

(defmethod instant-configuration-task
  (:kinematics-task-jacobian-with-theta
   ()
   "
\begin{eqnarray}
  \frac{\partial \bm{e}^{\mathit{kin}}}{\partial \bm{\theta}}
  &=& \begin{pmatrix}
    \frac{\partial \bm{e}_1^{\mathit{kin}}}{\partial \bm{\theta}} \\ \frac{\partial \bm{e}_2^{\mathit{kin}}}{\partial \bm{\theta}} \\ \vdots \\ \frac{\partial \bm{e}_{N_{\mathit{kin}}}^{\mathit{kin}}}{\partial \bm{\theta}}
  \end{pmatrix} \\
  \frac{\partial \bm{e}_m^{\mathit{kin}}}{\partial \bm{\theta}}
  &=& K_{\mathit{kin}} \left\{ \bm{J}_{\theta,m}^{\mathit{kin\mathchar`-trg}}(\bm{\theta}, \bm{\phi}) - \bm{J}_{\theta,m}^{\mathit{kin\mathchar`-att}}(\bm{\theta}, \bm{\phi}) \right\} \ \ (m = 1,2,\cdots,N_{\mathit{kin}})
\end{eqnarray}

return $\dfrac{\partial \bm{e}^{\mathit{kin}}}{\partial \bm{\theta}} \in \mathbb{R}^{6 N_{\mathit{kin}} \times N_\mathit{var\mathchar`-joint}}$
   "
   (apply
    #'concatenate-matrix-column
    (mapcar
     #'(lambda (trg-coords att-coords sc-mat)
         (let* ((mat
                 (reduce
                  #'m+
                  (remove
                   nil
                   (list
                    (if (derivedp trg-coords cascaded-coords)
                        (send _robot-env :calc-jacobian-from-joint-list
                              :union-joint-list _variant-joint-list
                              :move-target (list trg-coords)
                              :transform-coords (list (make-coords))
                              :translation-axis (list t)
                              :rotation-axis (list t)
                              ))
                    (scale-matrix
                     -1
                     (cond ((equal att-coords :cog)
                            (concatenate-matrix-column
                             (send _robot-env :robot-with-root-virtual :calc-cog-jacobian-from-joint-list
                                   :union-joint-list _variant-joint-list
                                   :translation-axis t
                                   )
                             (make-matrix 3 (length _variant-joint-list))
                             )
                            )
                           (t
                            (send _robot-env :calc-jacobian-from-joint-list
                                  :union-joint-list _variant-joint-list
                                  :move-target (list att-coords)
                                  :transform-coords (list (make-coords))
                                  :translation-axis (list t)
                                  :rotation-axis (list t)
                                  )
                            ))
                     )
                    )
                   )))
                )
           (m* sc-mat mat)
           ))
     _kin-target-coords-list
     _kin-attention-coords-list
     _kin-scale-mat-list
     ))
   )
  (:kinematics-task-jacobian-with-phi
   ()
   "
\begin{eqnarray}
  \frac{\partial \bm{e}^{\mathit{kin}}}{\partial \bm{\phi}}
  &=& \begin{pmatrix}
    \frac{\partial \bm{e}_1^{\mathit{kin}}}{\partial \bm{\phi}} \\ \frac{\partial \bm{e}_2^{\mathit{kin}}}{\partial \bm{\phi}} \\ \vdots \\ \frac{\partial \bm{e}_{N_{\mathit{kin}}}^{\mathit{kin}}}{\partial \bm{\phi}}
  \end{pmatrix} \\
  \frac{\partial \bm{e}_m^{\mathit{kin}}}{\partial \bm{\phi}}
  &=& K_{\mathit{kin}} \left\{ \bm{J}_{\phi,m}^{\mathit{kin\mathchar`-trg}}(\bm{\theta}, \bm{\phi}) - \bm{J}_{\phi,m}^{\mathit{kin\mathchar`-att}}(\bm{\theta}, \bm{\phi}) \right\} \ \ (m = 1,2,\cdots,N_{\mathit{kin}})
\end{eqnarray}

return $\dfrac{\partial \bm{e}^{\mathit{kin}}}{\partial \bm{\phi}} \in \mathbb{R}^{6 N_{\mathit{kin}} \times N_\mathit{invar\mathchar`-joint}}$
   "
   (apply
    #'concatenate-matrix-column
    (mapcar
     #'(lambda (trg-coords att-coords sc-mat)
         (let* ((mat
                 (reduce
                  #'m+
                  (remove
                   nil
                   (list
                    (if (derivedp trg-coords cascaded-coords)
                        (send _robot-env :calc-jacobian-from-joint-list
                              :union-joint-list _invariant-joint-list
                              :move-target (list trg-coords)
                              :transform-coords (list (make-coords))
                              :translation-axis (list t)
                              :rotation-axis (list t)
                              ))
                    (scale-matrix
                     -1
                     (cond ((equal att-coords :cog)
                            (concatenate-matrix-column
                             (send _robot-env :robot-with-root-virtual :calc-cog-jacobian-from-joint-list
                                   :union-joint-list _invariant-joint-list
                                   :translation-axis t
                                   )
                             (make-matrix 3 (length _invariant-joint-list))
                             )
                            )
                           (t
                            (send _robot-env :calc-jacobian-from-joint-list
                                  :union-joint-list _invariant-joint-list
                                  :move-target (list att-coords)
                                  :transform-coords (list (make-coords))
                                  :translation-axis (list t)
                                  :rotation-axis (list t)
                                  )
                            ))
                     )
                    )
                   )))
                )
           (m* sc-mat mat)
           ))
     _kin-target-coords-list
     _kin-attention-coords-list
     _kin-scale-mat-list
     ))
   )
  (:eom-trans-task-jacobian-with-wrench
   ()
   "
\begin{eqnarray}
  \frac{\partial \bm{e}^{\mathit{eom\mathchar`-trans}}}{\partial \bm{\hat{w}}}
  &=& \begin{pmatrix}
    \frac{\partial \bm{e}^{\mathit{eom\mathchar`-trans}}}{\partial \bm{f}_1} & \frac{\partial \bm{e}^{\mathit{eom\mathchar`-trans}}}{\partial \bm{n}_1} & \cdots & \frac{\partial \bm{e}^{\mathit{eom\mathchar`-trans}}}{\partial \bm{f}_{N_{\mathit{cnt}}}} & \frac{\partial \bm{e}^{\mathit{eom\mathchar`-trans}}}{\partial \bm{n}_{N_{\mathit{cnt}}}}
  \end{pmatrix} \\
  &=&\begin{pmatrix}
    \bm{I}_3 & \bm{O}_3 & \cdots & \bm{I}_3 & \bm{O}_3
  \end{pmatrix}
\end{eqnarray}

return $\frac{\partial \bm{e}^{\mathit{eom\mathchar`-trans}}}{\partial \bm{\hat{w}}} \in \mathbb{R}^{3 \times 6N_{\mathit{cnt}}}$
   "
   (apply #'concatenate-matrix-row
          (flatten (mapcar #'(lambda (x) (list (unit-matrix 3) (make-matrix 3 3))) (make-list _num-contact))))
   )
  (:eom-rot-task-jacobian-with-theta
   ()
   "
\begin{eqnarray}
  \frac{\partial \bm{e}^{\mathit{eom\mathchar`-rot}}}{\partial \bm{\theta}}
  &=& \sum_{m=1}^{N_{\mathit{cnt}}} \left\{ - [\bm{f}_m \times] \left( \bm{J}_{\theta,m}^{\mathit{cnt\mathchar`-trg}}(\bm{\theta}, \bm{\phi}) - \bm{J}_{G\theta}(\bm{\theta}, \bm{\phi}) \right) \right\} \nonumber \\
  &&+ \sum_{m=1}^{N_{\mathit{ex}}} \left\{ - [\bm{f}_m^{\mathit{ex}} \times] \left( \bm{J}_{\theta, m}^{\mathit{ex}}(\bm{\theta}, \bm{\phi}) - \bm{J}_{G\theta}(\bm{\theta}, \bm{\phi}) \right) \right\} \\
  &=& \left[\left( \sum_{m=1}^{N_{\mathit{cnt}}} \bm{f}_m + \sum_{m=1}^{N_{\mathit{ex}}} \bm{f}_m^{\mathit{ex}} \right) \times \right] \bm{J}_{G\theta}(\bm{\theta}, \bm{\phi}) \nonumber \\
  &&- \sum_{m=1}^{N_{\mathit{cnt}}} [\bm{f}_m \times] \bm{J}_{\theta,m}^{\mathit{cnt\mathchar`-trg}}(\bm{\theta}, \bm{\phi})
  - \sum_{m=1}^{N_{\mathit{ex}}} [\bm{f}_m^{\mathit{ex}} \times] \bm{J}_{\theta, m}^{\mathit{ex}}(\bm{\theta}, \bm{\phi})
\end{eqnarray}
$\sum_{m=1}^{N_{\mathit{cnt}}} \bm{f}_m + \sum_{m=1}^{N_{\mathit{ex}}} \bm{f}_m^{\mathit{ex}} = m \bm{g}$つまり，eom-trans-taskが成立すると仮定すると次式が成り立つ．
\begin{eqnarray}
  \frac{\partial \bm{e}^{\mathit{eom\mathchar`-rot}}}{\partial \bm{\theta}}
  &=& \left[ m \bm{g} \times \right] \bm{J}_{G\theta}(\bm{\theta}, \bm{\phi})
  - \sum_{m=1}^{N_{\mathit{cnt}}} [\bm{f}_m \times] \bm{J}_{\theta,m}^{\mathit{cnt\mathchar`-trg}}(\bm{\theta}, \bm{\phi})
  - \sum_{m=1}^{N_{\mathit{ex}}} [\bm{f}_m^{\mathit{ex}} \times] \bm{J}_{\theta, m}^{\mathit{ex}}(\bm{\theta}, \bm{\phi})
\end{eqnarray}

return $\frac{\partial \bm{e}^{\mathit{eom\mathchar`-rot}}}{\partial \bm{\theta}} \in \mathbb{R}^{3 \times N_{\mathit{var\mathchar`-joint}}}$
   "
   (reduce
    #'m+
    (remove
     nil
     (flatten
      (list
       (m* (outer-product-matrix (send self :mg-vec))
           (send _robot-env :robot-with-root-virtual :calc-cog-jacobian-from-joint-list
                 :union-joint-list _variant-joint-list
                 :translation-axis t
                 ))
       (mapcar
        #'(lambda (f c)
            (if (derivedp c cascaded-coords)
                (scale-matrix
                 -1
                 (m* (outer-product-matrix f)
                     (send _robot-env :calc-jacobian-from-joint-list
                           :union-joint-list _variant-joint-list
                           :move-target (list c)
                           :transform-coords (list (make-coords))
                           :translation-axis (list t)
                           :rotation-axis (list nil)
                           )
                     ))))
        (send self :force-list)
        _contact-target-coords-list
        )
       (mapcar
        #'(lambda (ex-f ex-c)
            (scale-matrix
             -1
             (m* (outer-product-matrix ex-f)
                 (send _robot-env :calc-jacobian-from-joint-list
                       :union-joint-list _variant-joint-list
                       :move-target (list ex-c)
                       :transform-coords (list (make-coords))
                       :translation-axis (list t)
                       :rotation-axis (list nil)
                       )
                 )))
        (send self :external-force-list)
        _external-coords-list
        )
       ))
     ))
   )
  (:eom-rot-task-jacobian-with-wrench
   ()
   "
\begin{eqnarray}
  \frac{\partial \bm{e}^{\mathit{eom\mathchar`-rot}}}{\partial \bm{\hat{w}}}
  &=& \begin{pmatrix}
    \frac{\partial \bm{e}^{\mathit{eom\mathchar`-rot}}}{\partial \bm{f}_1} & \frac{\partial \bm{e}^{\mathit{eom\mathchar`-rot}}}{\partial \bm{n}_1} & \cdots & \frac{\partial \bm{e}^{\mathit{eom\mathchar`-rot}}}{\partial \bm{f}_{N_{\mathit{cnt}}}} & \frac{\partial \bm{e}^{\mathit{eom\mathchar`-rot}}}{\partial \bm{n}_{N_{\mathit{cnt}}}}
  \end{pmatrix} \\
  \frac{\partial \bm{e}^{\mathit{eom\mathchar`-rot}}}{\partial \bm{f}_m}
  &=& \left[ \left(\bm{p}_m^{\mathit{cnt\mathchar`-trg}}(\bm{\theta}, \bm{\phi}) - \bm{p}_G(\bm{\theta}, \bm{\phi})\right) \times \right] \ \ (m = 1,2,\cdots,N_{\mathit{cnt}}) \\
  \frac{\partial \bm{e}^{\mathit{eom\mathchar`-rot}}}{\partial \bm{n}_m}
  &=& \bm{I}_3 \ \ (m = 1,2,\cdots,N_{\mathit{cnt}})
\end{eqnarray}

return $\frac{\partial \bm{e}^{\mathit{eom\mathchar`-rot}}}{\partial \bm{\hat{w}}} \in \mathbb{R}^{3 \times 6N_{\mathit{cnt}}}$
   "
   (apply #'concatenate-matrix-row
          (flatten
           (mapcar
            #'(lambda (p)
                (list (outer-product-matrix (v- p (send self :cog))) (unit-matrix 3)))
            (mapcar #'(lambda (c) (scale 1e-3 (send c :worldpos))) _contact-target-coords-list) ;; [m]
            )))
   )
  (:eom-rot-task-jacobian-with-phi
   ()
   "
\begin{eqnarray}
  \frac{\partial \bm{e}^{\mathit{eom\mathchar`-rot}}}{\partial \bm{\phi}}
  &=& \sum_{m=1}^{N_{\mathit{cnt}}} \left\{ - [\bm{f}_m \times] \left( \bm{J}_{\phi, m}^{\mathit{cnt\mathchar`-trg}}(\bm{\theta}, \bm{\phi}) - \bm{J}_{G\phi}(\bm{\theta}, \bm{\phi}) \right) \right\} \nonumber\\
  && + \sum_{m=1}^{N_{\mathit{ex}}} \left\{ - [\bm{f}_m^{\mathit{ex}} \times] \left( \bm{J}_{\phi, m}^{\mathit{ex}}(\bm{\theta}, \bm{\phi}) - \bm{J}_{G\phi}(\bm{\theta}, \bm{\phi}) \right) \right\} \\
  &=& \left[\left( \sum_{m=1}^{N_{\mathit{cnt}}} \bm{f}_m + \sum_{m=1}^{N_{\mathit{ex}}} \bm{f}_m^{\mathit{ex}} \right) \times \right] \bm{J}_{G\phi}(\bm{\theta}, \bm{\phi}) \nonumber\\
  && - \sum_{m=1}^{N_{\mathit{cnt}}}  [\bm{f}_m \times] \bm{J}_{\phi, m}^{\mathit{cnt\mathchar`-trg}}(\bm{\theta}, \bm{\phi}) - \sum_{m=1}^{N_{\mathit{ex}}}  [\bm{f}_m^{\mathit{ex}} \times] \bm{J}_{\phi, m}^{\mathit{ex}}(\bm{\theta}, \bm{\phi})
\end{eqnarray}
$\sum_{m=1}^{N_{\mathit{cnt}}} \bm{f}_m + \sum_{m=1}^{N_{\mathit{ex}}} \bm{f}_m^{\mathit{ex}} = m \bm{g}$つまり，eom-trans-taskが成立すると仮定すると次式が成り立つ．
\begin{eqnarray}
  \frac{\partial \bm{e}^{\mathit{eom\mathchar`-rot}}}{\partial \bm{\phi}}
  &=& \left[ m \bm{g} \times \right] \bm{J}_{G\phi}(\bm{\theta}, \bm{\phi}) - \sum_{m=1}^{N_{\mathit{cnt}}} [\bm{f}_m \times] \bm{J}_{\phi, m}^{\mathit{cnt\mathchar`-trg}}(\bm{\theta}, \bm{\phi}) - \sum_{m=1}^{N_{\mathit{ex}}} [\bm{f}_m^{\mathit{ex}} \times] \bm{J}_{\phi, m}^{\mathit{ex}}(\bm{\theta}, \bm{\phi})
\end{eqnarray}

return $\frac{\partial \bm{e}^{\mathit{eom\mathchar`-rot}}}{\partial \bm{\phi}} \in \mathbb{R}^{3 \times N_{\mathit{invar\mathchar`-joint}}}$
   "
   (reduce
    #'m+
    (remove
     nil
     (flatten
      (list
       (m* (outer-product-matrix (send self :mg-vec))
           (send _robot-env :robot-with-root-virtual :calc-cog-jacobian-from-joint-list
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
                  (send _robot-env :calc-jacobian-from-joint-list
                        :union-joint-list _invariant-joint-list
                        :move-target (list c)
                        :transform-coords (list (make-coords))
                        :translation-axis (list t)
                        :rotation-axis (list nil)
                        )
                  ))))
        (send self :force-list)
        _contact-target-coords-list
        )
       (mapcar
        #'(lambda (ex-f ex-c)
            (scale-matrix
             -1
             (m*
              (outer-product-matrix ex-f)
              (send _robot-env :calc-jacobian-from-joint-list
                    :union-joint-list _invariant-joint-list
                    :move-target (list ex-c)
                    :transform-coords (list (make-coords))
                    :translation-axis (list t)
                    :rotation-axis (list nil)
                    )
              )))
        (send self :external-force-list)
        _external-coords-list
        )
       ))
     ))
   )
  (:torque-task-jacobian-with-theta
   ()
   "
\begin{eqnarray}
  \frac{\partial \bm{e}^{\mathit{trq}}}{\partial \bm{\theta}}
  &=& \sum_{m=1}^{N_{\mathit{cnt}}} \frac{\partial \bm{\tau}^{\mathit{cnt}}_m}{\partial \bm{\theta}} - \frac{\partial \bm{\tau}^{\mathit{grav}}}{\partial \bm{\theta}} + \sum_{m=1}^{N_{\mathit{ex}}} \frac{\partial \bm{\tau}^{\mathit{ex}}_m}{\partial \bm{\theta}}
\end{eqnarray}

return $\frac{\partial \bm{e}^{\mathit{trq}}}{\partial \bm{\theta}} \in \mathbb{R}^{N_{\mathit{drive\mathchar`-joint}} \times N_{\mathit{var\mathchar`-joint}}}$
   "
   (reduce
    #'m+
    (list
     (get-contact-torque-jacobian
      :robot _robot-env
      :joint-list _variant-joint-list
      :drive-joint-list _drive-joint-list
      :wrench-list (send self :wrench-list)
      :contact-target-coords-list _contact-target-coords-list
      :contact-attention-coords-list _contact-attention-coords-list
      )
     (scale-matrix
      -1
      (get-gravity-torque-jacobian
       :robot _robot-env
       :joint-list _variant-joint-list
       :drive-joint-list _drive-joint-list
       :gravity-link-list (send _robot-env :robot :links)
       ))
     (get-contact-torque-jacobian
      :robot _robot-env
      :joint-list _variant-joint-list
      :drive-joint-list _drive-joint-list
      :wrench-list (send self :external-wrench-list)
      :contact-target-coords-list _external-coords-list
      :contact-attention-coords-list _external-coords-list
      )
     ))
   )
  (:torque-task-jacobian-with-wrench
   ()
   "
\begin{eqnarray}
  \frac{\partial \bm{e}^{\mathit{trq}}}{\partial \bm{\hat{w}}}
  &=& \begin{pmatrix}
    \frac{\partial \bm{e}^{\mathit{trq}}}{\partial \bm{w}_1} & \frac{\partial \bm{e}^{\mathit{trq}}}{\partial \bm{w}_2} & \cdots & \frac{\partial \bm{e}^{\mathit{trq}}}{\partial \bm{w}_{N_{\mathit{cnt}}}}
  \end{pmatrix} \\
  \frac{\partial \bm{e}^{\mathit{trq}}}{\partial \bm{w}_m}
  &=& \bm{J}^{\mathit{cnt\mathchar`-trg}}_{\mathit{drive\mathchar`-joint},m}(\bm{\theta}, \bm{\phi})^T \ \ (m = 1,2,\cdots,N_{\mathit{cnt}})
\end{eqnarray}

return $\frac{\partial \bm{e}^{\mathit{trq}}}{\partial \bm{\hat{w}}} \in \mathbb{R}^{N_{\mathit{drive\mathchar`-joint}} \times 6 N_{\mathit{cnt}}}$
   "
   (apply
    #'concatenate-matrix-row
    (mapcar
     #'(lambda (trg-coords att-coords)
         (transpose
          (get-link-jacobian-for-contact-torque
           :robot _robot-env
           :drive-joint-list _drive-joint-list
           :contact-coords trg-coords
           :contact-parent-link (send att-coords :parent)
           )))
     _contact-target-coords-list
     _contact-attention-coords-list
     ))
   )
  (:torque-task-jacobian-with-phi
   ()
   "
\begin{eqnarray}
  \frac{\partial \bm{e}^{\mathit{trq}}}{\partial \bm{\phi}}
  &=& \sum_{m=1}^{N_{\mathit{cnt}}} \frac{\partial \bm{\tau}^{\mathit{cnt}}_m}{\partial \bm{\phi}} - \frac{\partial \bm{\tau}^{\mathit{grav}}_m}{\partial \bm{\phi}} + \sum_{m=1}^{N_{\mathit{ex}}} \frac{\partial \bm{\tau}^{\mathit{ex}}_m}{\partial \bm{\phi}}
\end{eqnarray}

return $\frac{\partial \bm{e}^{\mathit{trq}}}{\partial \bm{\phi}} \in \mathbb{R}^{N_{\mathit{drive\mathchar`-joint}} \times N_{\mathit{invar\mathchar`-joint}}}$
   "
   (reduce
    #'m+
    (list
     (get-contact-torque-jacobian
      :robot _robot-env
      :joint-list _invariant-joint-list
      :drive-joint-list _drive-joint-list
      :wrench-list (send self :wrench-list)
      :contact-target-coords-list _contact-target-coords-list
      :contact-attention-coords-list _contact-attention-coords-list
      )
     (scale-matrix
      -1
      (get-gravity-torque-jacobian
       :robot _robot-env
       :joint-list _invariant-joint-list
       :drive-joint-list _drive-joint-list
       :gravity-link-list (send _robot-env :robot :links)
       ))
     (get-contact-torque-jacobian
      :robot _robot-env
      :joint-list _invariant-joint-list
      :drive-joint-list _drive-joint-list
      :wrench-list (send self :external-wrench-list)
      :contact-target-coords-list _external-coords-list
      :contact-attention-coords-list _external-coords-list
      )
     ))
   )
  (:torque-task-jacobian-with-torque
   ()
   "
\begin{eqnarray}
  \frac{\partial \bm{e}^{\mathit{trq}}}{\partial \bm{\tau}}
  &=& \bm{I}_{N_{\mathit{drive\mathchar`-joint}}}
\end{eqnarray}

return $\frac{\partial \bm{e}^{\mathit{trq}}}{\partial \bm{\tau}} \in \mathbb{R}^{N_{\mathit{drive\mathchar`-joint}} \times N_{\mathit{drive\mathchar`-joint}}}$
   "
   (unit-matrix _num-drive-joint)
   )
  (:posture-task-jacobian-with-theta
   (&key (update? nil))
   "
\begin{eqnarray}
  \left( \frac{\partial \bm{e}^{\mathit{posture}}}{\partial \bm{\theta}} \right)_{i,j} &=&
  \left\{
  \begin{array}{ll}
  - k_{\mathit{posture}} & (\mathcal{J}_{\mathit{posture,i}} = \mathcal{J}_{\mathit{var,j}}) \\
  0 & {\rm otherwise}
  \end{array}
  \right.
\end{eqnarray}

return $\frac{\partial \bm{e}^{\mathit{posture}}}{\partial \bm{\theta}} \in \mathbb{R}^{N_{\mathit{posture\mathchar`-joint}} \times N_{\mathit{var\mathchar`-joint}}}$
   "
   (when (or update? (not (send self :get :posture-task-jacobian-with-theta)))
     (send self :put :posture-task-jacobian-with-theta (make-matrix _num-posture-joint _num-variant-joint))
     (dotimes (i (length _posture-joint-list))
       (setf
        (aref (send self :get :posture-task-jacobian-with-theta)
              i (position (elt _posture-joint-list i) _variant-joint-list))
        (- (elt _target-posture-scale-list i))))
     )
   (send self :get :posture-task-jacobian-with-theta)
   )
  (:variant-task-jacobian
   ()
   "
\begin{eqnarray}
  \frac{\partial \bm{e}}{\partial \bm{q}_{\mathit{var}}}
  &=&
  \bordermatrix{
    & N_{\mathit{var\mathchar`-joint}} & 6 N_{\mathit{cnt}} & N_{\mathit{drive\mathchar`-joint}} \cr
    6 N_{\mathit{kin}} & \frac{\partial \bm{e}^{\mathit{kin}}}{\partial \bm{\theta}} & & \cr
    3 & & \frac{\partial \bm{e}^{\mathit{eom\mathchar`-trans}}}{\partial \bm{\hat{w}}} & & \cr
    3 & \frac{\partial \bm{e}^{\mathit{eom\mathchar`-rot}}}{\partial \bm{\theta}} & \frac{\partial \bm{e}^{\mathit{eom\mathchar`-rot}}}{\partial \bm{\hat{w}}} & & \cr
    N_{\mathit{drive\mathchar`-joint}} & \frac{\partial \bm{e}^{\mathit{trq}}}{\partial \bm{\theta}} & \frac{\partial \bm{e}^{\mathit{trq}}}{\partial \bm{\hat{w}}} & \frac{\partial \bm{e}^{\mathit{trq}}}{\partial \bm{\tau}} \cr
    N_{\mathit{posture\mathchar`-joint}} & \frac{\partial \bm{e}^{\mathit{posture}}}{\partial \bm{\theta}} & &
  }
\end{eqnarray}

return $\frac{\partial \bm{e}}{\partial \bm{q}_{\mathit{var}}} \in \mathbb{R}^{(6 N_{\mathit{kin}} + 3 + 3 + N_{\mathit{drive\mathchar`-joint}} + N_{\mathit{posture\mathchar`-joint}}) \times (N_{\mathit{var\mathchar`-joint}} + 6 N_{\mathit{cnt}} + N_{\mathit{drive\mathchar`-joint}})}$
   "
   (cond (_variant-task-jacobi
          (fill (array-entity _variant-task-jacobi) 0)
          )
         (t
          (setq _variant-task-jacobi (make-matrix _dim-task _dim-variant-config))
          ))
   (when (= _dim-task 0)
     (return-from :variant-task-jacobian _variant-task-jacobi))
   (copy-block-matrix _variant-task-jacobi
                      (send self :kinematics-task-jacobian-with-theta)
                      0 0)
   (unless _only-kinematics?
     (copy-block-matrix _variant-task-jacobi
                        (send self :eom-trans-task-jacobian-with-wrench)
                        (* 6 _num-kin) _num-variant-joint)
     (copy-block-matrix _variant-task-jacobi
                        (send self :eom-rot-task-jacobian-with-theta)
                        (+ (* 6 _num-kin) 3) 0)
     (copy-block-matrix _variant-task-jacobi
                        (send self :eom-rot-task-jacobian-with-wrench)
                        (+ (* 6 _num-kin) 3) _num-variant-joint)
     (when (> _num-drive-joint 0)
       (copy-block-matrix _variant-task-jacobi
                          (send self :torque-task-jacobian-with-theta)
                          (+ (* 6 _num-kin) 3 3) 0)
       (copy-block-matrix _variant-task-jacobi
                          (send self :torque-task-jacobian-with-wrench)
                          (+ (* 6 _num-kin) 3 3) _num-variant-joint)
       (copy-block-matrix _variant-task-jacobi
                          (send self :torque-task-jacobian-with-torque)
                          (+ (* 6 _num-kin) 3 3) (+ _num-variant-joint (* 6 _num-contact)))
       )
     )
   (when (> _num-posture-joint 0)
     (copy-block-matrix _variant-task-jacobi
                        (send self :posture-task-jacobian-with-theta)
                        (if _only-kinematics?
                            (* 6 _num-kin)
                          (+ (* 6 _num-kin) 3 3 _num-drive-joint))
                        0)
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
    N_{\mathit{drive\mathchar`-joint}} & \frac{\partial \bm{e}^{\mathit{trq}}}{\partial \bm{\phi}} \cr
    N_{\mathit{posture\mathchar`-joint}} &
  }
\end{eqnarray}

return $\frac{\partial \bm{e}}{\partial \bm{q}_{\mathit{invar}}} \in \mathbb{R}^{(6 N_{\mathit{kin}} + 3 + 3 + N_{\mathit{drive\mathchar`-joint}} + N_{\mathit{posture\mathchar`-joint}}) \times N_{\mathit{invar\mathchar`-joint}}}$
   "
   (cond (_invariant-task-jacobi
          (fill (array-entity _invariant-task-jacobi) 0)
          )
         (t
          (setq _invariant-task-jacobi (make-matrix _dim-task _dim-invariant-config))
          ))
   (when (= _dim-task 0)
     (return-from :invariant-task-jacobian _invariant-task-jacobi))
   (copy-block-matrix _invariant-task-jacobi
                      (send self :kinematics-task-jacobian-with-phi)
                      0 0)
   (unless _only-kinematics?
     (copy-block-matrix _invariant-task-jacobi
                        (send self :eom-rot-task-jacobian-with-phi)
                        (+ (* 6 _num-kin) 3) 0)
     (when (> _num-drive-joint 0)
       (copy-block-matrix _invariant-task-jacobi
                          (send self :torque-task-jacobian-with-phi)
                          (+ (* 6 _num-kin) 3 3) 0)
       )
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
    & N_{\mathit{var\mathchar`-joint}} & 6 N_{\mathit{cnt}} & N_{\mathit{drive\mathchar`-joint}} & N_{\mathit{invar\mathchar`-joint}} \cr
    6 N_{\mathit{kin}} & \frac{\partial \bm{e}^{\mathit{kin}}}{\partial \bm{\theta}} & & & \frac{\partial \bm{e}^{\mathit{kin}}}{\partial \bm{\phi}} \cr
    3 & & \frac{\partial \bm{e}^{\mathit{eom\mathchar`-trans}}}{\partial \bm{\hat{w}}} & & & \cr
    3 & \frac{\partial \bm{e}^{\mathit{eom\mathchar`-rot}}}{\partial \bm{\theta}} & \frac{\partial \bm{e}^{\mathit{eom\mathchar`-rot}}}{\partial \bm{\hat{w}}} & & \frac{\partial \bm{e}^{\mathit{eom\mathchar`-rot}}}{\partial \bm{\phi}} \cr
    N_{\mathit{drive\mathchar`-joint}} & \frac{\partial \bm{e}^{\mathit{trq}}}{\partial \bm{\theta}} & \frac{\partial \bm{e}^{\mathit{trq}}}{\partial \bm{\hat{w}}} & \frac{\partial \bm{e}^{\mathit{trq}}}{\partial \bm{\tau}} & \frac{\partial \bm{e}^{\mathit{trq}}}{\partial \bm{\phi}} \cr
    N_{\mathit{posture\mathchar`-joint}} & \frac{\partial \bm{e}^{\mathit{posture}}}{\partial \bm{\theta}} & & &
  }
\end{eqnarray}

return $\frac{\partial \bm{e}}{\partial \bm{q}} \in \mathbb{R}^{(6 N_{\mathit{kin}} + 3 + 3 + N_{\mathit{drive\mathchar`-joint}} + N_{\mathit{posture\mathchar`-joint}}) \times (N_{\mathit{var\mathchar`-joint}} + 6 N_{\mathit{cnt}} + N_{\mathit{drive\mathchar`-joint}} + N_{\mathit{invar\mathchar`-joint}})}$
   "
   (cond (_task-jacobi
          (fill (array-entity _task-jacobi) 0)
          )
         (t
          (setq _task-jacobi (make-matrix _dim-task _dim-config))
          ))
   (when (= _dim-task 0)
     (return-from :task-jacobian _task-jacobi))
   (copy-block-matrix _task-jacobi
                      (send self :variant-task-jacobian)
                      0 0)
   (when (> _num-invariant-joint 0)
     (copy-block-matrix _task-jacobi
                        (send self :invariant-task-jacobian)
                        0 _dim-variant-config)
     )
   _task-jacobi
   )
  )

(defmethod instant-configuration-task
  (:theta-max-vector
   (&key (update? nil))
   "
return $\bm{\theta}_{\mathit{max}} \in \mathbb{R}^{N_{\mathit{var\mathchar`-joint}}}$
   "
   (when (or update? (not (send self :get :theta-max-vector)))
     (send self :put :theta-max-vector
           (map float-vector
                #'(lambda (j)
                    (cond ((< (- (send j :max-angle) (send j :min-angle)) _variant-joint-angle-margin)
                           (send j :angle-to-speed (* 0.5 (+ (send j :max-angle) (send j :min-angle)))))
                          (t
                           (send j :angle-to-speed (- (send j :max-angle) _variant-joint-angle-margin))))
                    )
                _variant-joint-list
                ))
     )
   (send self :get :theta-max-vector)
   )
  (:theta-min-vector
   (&key (update? nil))
   "
return $\bm{\theta}_{\mathit{min}} \in \mathbb{R}^{N_{\mathit{var\mathchar`-joint}}}$
   "
   (when (or update? (not (send self :get :theta-min-vector)))
     (send self :put :theta-min-vector
           (map float-vector
                #'(lambda (j)
                    (cond ((< (- (send j :max-angle) (send j :min-angle)) _variant-joint-angle-margin)
                           (send j :angle-to-speed (* 0.5 (+ (send j :max-angle) (send j :min-angle)))))
                          (t
                           (send j :angle-to-speed (+ (send j :min-angle) _variant-joint-angle-margin))))
                    )
                _variant-joint-list
                ))
     )
   (send self :get :theta-min-vector)
   )
  (:delta-theta-limit-vector
   (&key (update? nil))
   "
get trust region of $\bm{\theta}$

return $\Delta \bm{\theta}_{\mathit{limit}}$
   "
   (unless (and _delta-linear-joint _delta-rotational-joint)
     (return-from :delta-theta-limit-vector nil))
   (when (or update? (not (send self :get :delta-theta-limit-vector)))
     (send self :put :delta-theta-limit-vector
           (map float-vector
                #'(lambda (j)
                    (send j :angle-to-speed
                          (if (derivedp j linear-joint) _delta-linear-joint _delta-rotational-joint)))
                _variant-joint-list))
     )
   (send self :get :delta-theta-limit-vector)
   )
  (:theta-inequality-constraint-matrix
   (&key (update? nil))
   "
\begin{eqnarray}
&& \left\{ \begin{array}{ll}
\bm{\theta}_{\mathit{min}} \leq \bm{\theta} + \Delta \bm{\theta} \leq \bm{\theta}_{\mathit{max}} & \\
- \Delta \bm{\theta}_{\mathit{limit}} \leq \Delta \bm{\theta} \leq \Delta \bm{\theta}_{\mathit{limit}} & ({\rm if} \ \ \Delta \bm{\theta}_{\mathit{limit}} \ \ {\rm is \ set})
\end{array} \right. \\
\Leftrightarrow&&\begin{pmatrix} \bm{I} \\ - \bm{I} \\ \bm{I} \\ - \bm{I}\end{pmatrix} \Delta \bm{\theta}
\geq \begin{pmatrix} \bm{\theta}_{\mathit{min}} - \bm{\theta} \\ - (\bm{\theta}_{\mathit{max}} - \bm{\theta}) \\ - \Delta \bm{\theta}_{\mathit{limit}} \\ - \Delta \bm{\theta}_{\mathit{limit}} \end{pmatrix} \\
\Leftrightarrow&&\bm{C_{\theta}} \Delta \bm{\theta} \geq \bm{d_{\theta}}
\end{eqnarray}

return $\bm{C_{\theta}} := \begin{pmatrix} \bm{I} \\ - \bm{I} \\ \bm{I} \\ - \bm{I}\end{pmatrix} \in \mathbb{R}^{4 N_{\mathit{var\mathchar`-joint}} \times N_{\mathit{var\mathchar`-joint}}}$
   "
   (when (or update? (not (send self :get :theta-inequality-constraint-matrix)))
     (send self :put :theta-inequality-constraint-matrix
           (apply
            #'concatenate-matrix-column
            (remove
             nil
             (list
              (unit-matrix _dim-theta)
              (scale-matrix -1 (unit-matrix _dim-theta))
              (if (send self :delta-theta-limit-vector)
                  (unit-matrix _dim-theta))
              (if (send self :delta-theta-limit-vector)
                  (scale-matrix -1 (unit-matrix _dim-theta)))
              )))
           )
     )
   (send self :get :theta-inequality-constraint-matrix)
   )
  (:theta-inequality-constraint-vector
   (&key (update? t))
   "
return $\bm{d_{\theta}} := \begin{pmatrix} \bm{\theta}_{\mathit{min}} - \bm{\theta} \\ - (\bm{\theta}_{\mathit{max}} - \bm{\theta}) \\ - \Delta \bm{\theta}_{\mathit{limit}} \\ - \Delta \bm{\theta}_{\mathit{limit}} \end{pmatrix} \in \mathbb{R}^{4 N_{\mathit{var\mathchar`-joint}}}$
   "
   (when (or update? (not (send self :get :theta-inequality-constraint-vector)))
     (send self :put :theta-inequality-constraint-vector
           (apply
            #'concatenate
            float-vector
            (remove
             nil
             (list
              (v- (send self :theta-min-vector) _theta-vector)
              (v- _theta-vector (send self :theta-max-vector))
              (if (send self :delta-theta-limit-vector)
                  (scale -1 (send self :delta-theta-limit-vector)))
              (if (send self :delta-theta-limit-vector)
                  (scale -1 (send self :delta-theta-limit-vector)))
              )))
           )
     )
   (send self :get :theta-inequality-constraint-vector)
   )
  (:_contact-constraint-matrix-list
   (&key (update? nil)) ;; this should be true
   "
return $\{\bm{C}_{w,1}, \bm{C}_{w,2}, \cdots, \bm{C}_{w,N_{\mathit{cnt}}}\}$
   "
   (when (or update? (not (send self :get :_contact-constraint-matrix-list)))
     (send self :put :_contact-constraint-matrix-list
           (mapcar
            #'(lambda (const coords) (send const :calc-constraint-matrix coords))
            _contact-constraint-list
            _contact-target-coords-list
            ))
     )
   (send self :get :_contact-constraint-matrix-list)
   )
  (:_contact-constraint-vector-list
   (&key (update? nil))
   "
return $\{\bm{d}_{w,1}, \bm{d}_{w,2}, \cdots, \bm{d}_{w,N_{\mathit{cnt}}}\}$
   "
   (when (or update? (not (send self :get :_contact-constraint-vector-list)))
     (send self :put :_contact-constraint-vector-list
           (mapcar
            #'(lambda (const) (send const :get-constraint-vector))
            _contact-constraint-list
            ))
     )
   (send self :get :_contact-constraint-vector-list)
   )
  (:wrench-inequality-constraint-matrix
   (&key (update? t))
   "
接触レンチ$\bm{w} \in \mathbb{R}^{6}$が満たすべき制約（非負制約，摩擦制約，圧力中心制約）が次式のように表されるとする．
\begin{eqnarray}
  \bm{C}_{w} \bm{w} \geq \bm{d}_{w}
\end{eqnarray}
$N_{\mathit{cnt}}$箇所の接触部位の接触レンチを並べたベクトル$\bm{\hat{w}}$の不等式制約は次式で表される．
\begin{eqnarray}
  &&\bm{C}_{w,m} (\bm{w}_m + \Delta \bm{w}_m) \geq \bm{d}_{w,m} \ \ (m=1,2,\cdots,N_{\mathit{cnt}}) \\
  \Leftrightarrow&&\bm{C}_{w,m} \Delta \bm{w}_m \geq \bm{d}_{w,m} - \bm{C}_{w,m} \bm{w}_m \ \ (m=1,2,\cdots,N_{\mathit{cnt}}) \\
  \Leftrightarrow&&
  \begin{pmatrix}\bm{C}_{w,1}&&&\\&\bm{C}_{w,2}&&\\&&\ddots&\\&&&\bm{C}_{w,N_{\mathit{cnt}}}\end{pmatrix}
  \begin{pmatrix}\Delta \bm{w}_1 \\ \Delta \bm{w}_2 \\ \vdots \\ \Delta \bm{w}_{N_{\mathit{cnt}}}\end{pmatrix}
  \geq
  \begin{pmatrix}\bm{d}_{w,1}-\bm{C}_{w,1}\bm{w}_1 \\ \bm{d}_{w,2}-\bm{C}_{w,2}\bm{w}_2 \\ \vdots \\ \bm{d}_{w,N_{\mathit{cnt}}}-\bm{C}_{w,N_{\mathit{cnt}}}\bm{w}_{N_{\mathit{cnt}}}\end{pmatrix} \\
  \Leftrightarrow&&\bm{C}_{\hat{w}} \Delta \bm{\hat{w}} \geq \bm{d}_{\hat{w}}
\end{eqnarray}

return $\bm{C}_{\hat{w}} := \begin{pmatrix}\bm{C}_{w,1}&&&\\&\bm{C}_{w,2}&&\\&&\ddots&\\&&&\bm{C}_{w,N_{\mathit{cnt}}}\end{pmatrix} \in \mathbb{R}^{N_{\mathit{wrench\mathchar`-ineq}} \times \mathit{dim}(\bm{\hat{w}})}$
   "
   (when (or update? (not (send self :get :wrench-inequality-constraint-matrix)))
     (send self :put :wrench-inequality-constraint-matrix
           (apply #'concatenate-matrix-diagonal (send self :_contact-constraint-matrix-list)))
     )
   (send self :get :wrench-inequality-constraint-matrix)
   )
  (:wrench-inequality-constraint-vector
   (&key (update? t))
   "
return $\bm{d}_{\hat{w}} := \begin{pmatrix}\bm{d}_{w,1}-\bm{C}_{w,1}\bm{w}_1 \\ \bm{d}_{w,2}-\bm{C}_{w,2}\bm{w}_2 \\ \vdots \\ \bm{d}_{w,N_{\mathit{cnt}}}-\bm{C}_{w,N_{\mathit{cnt}}}\bm{w}_{N_{\mathit{cnt}}}\end{pmatrix} \in \mathbb{R}^{N_{\mathit{wrench\mathchar`-ineq}}}$
   "
   (when (or update? (not (send self :get :wrench-inequality-constraint-vector)))
     (send self :put :wrench-inequality-constraint-vector
           (apply #'concatenate
                  float-vector
                  (mapcar
                   #'(lambda (v m w) (v- v (transform m w)))
                   (send self :_contact-constraint-vector-list)
                   (send self :_contact-constraint-matrix-list)
                   (send self :wrench-list)
                   )))
     )
   (send self :get :wrench-inequality-constraint-vector)
   )
  (:torque-max-vector
   (&key (update? nil))
   "
return $\bm{\tau}_{\mathit{max}} \in \mathbb{R}^{N_{\mathit{drive\mathchar`-joint}}}$
   "
   (when (or update? (not (send self :get :torque-max-vector)))
     (send self :put :torque-max-vector
           (map float-vector #'(lambda (j) (send j :max-joint-torque)) _drive-joint-list))
     )
   (send self :get :torque-max-vector)
   )
  (:torque-min-vector
   (&key (update? nil))
   "
return $\bm{\tau}_{\mathit{min}} \in \mathbb{R}^{N_{\mathit{drive\mathchar`-joint}}}$
   "
   (when (or update? (not (send self :get :torque-min-vector)))
     (send self :put :torque-min-vector
           (scale -1 (send self :torque-max-vector :update? update?)))
     )
   (send self :get :torque-min-vector)
   )
  (:torque-inequality-constraint-matrix
   (&key (update? nil))
   "
\begin{eqnarray}
&&\bm{\tau}_{\mathit{min}} \leq \bm{\tau} + \Delta \bm{\tau} \leq \bm{\tau}_{\mathit{max}} \\
\Leftrightarrow&&\begin{pmatrix} \bm{I} \\ - \bm{I}\end{pmatrix} \Delta \bm{\tau} \geq \begin{pmatrix} \bm{\tau}_{\mathit{min}} - \bm{\tau} \\ - (\bm{\tau}_{\mathit{max}} - \bm{\tau})\end{pmatrix} \\
\Leftrightarrow&&\bm{C_{\tau}} \Delta \bm{\tau} \geq \bm{d_{\tau}}
\end{eqnarray}

return $\bm{C_{\tau}} := \begin{pmatrix} \bm{I} \\ - \bm{I}\end{pmatrix} \in \mathbb{R}^{2 N_{\mathit{drive\mathchar`-joint}} \times N_{\mathit{drive\mathchar`-joint}}}$
   "
   (when (or update? (not (send self :get :torque-inequality-constraint-matrix)))
     (send self :put :torque-inequality-constraint-matrix
           (concatenate-matrix-column
            (unit-matrix _dim-torque)
            (scale-matrix -1 (unit-matrix _dim-torque)))
           )
     )
   (send self :get :torque-inequality-constraint-matrix)
   )
  (:torque-inequality-constraint-vector
   (&key (update? t))
   "
return $\bm{d_{\tau}} := \begin{pmatrix} \bm{\tau}_{\mathit{min}} - \bm{\tau} \\ - (\bm{\tau}_{\mathit{max}} - \bm{\tau})\end{pmatrix} \in \mathbb{R}^{2 N_{\mathit{drive\mathchar`-joint}}}$
   "
   (when (or update? (not (send self :get :torque-inequality-constraint-vector)))
     (send self :put :torque-inequality-constraint-vector
           (concatenate
            float-vector
            (v- (send self :torque-min-vector) _torque-vector)
            (v- _torque-vector (send self :torque-max-vector)))
           )
     )
   (send self :get :torque-inequality-constraint-vector)
   )
  (:phi-max-vector
   (&key (update? nil))
   "
return $\bm{\phi}_{\mathit{max}} \in \mathbb{R}^{N_{\mathit{invar\mathchar`-joint}}}$
   "
   (when (or update? (not (send self :get :phi-max-vector)))
     (send self :put :phi-max-vector
           (map float-vector
                #'(lambda (j)
                    (cond ((< (- (send j :max-angle) (send j :min-angle)) _invariant-joint-angle-margin)
                           (send j :angle-to-speed (* 0.5 (+ (send j :max-angle) (send j :min-angle)))))
                          (t
                           (send j :angle-to-speed (- (send j :max-angle) _invariant-joint-angle-margin))))
                    )
                _invariant-joint-list
                ))
     )
   (send self :get :phi-max-vector)
   )
  (:phi-min-vector
   (&key (update? nil))
   "
return $\bm{\phi}_{\mathit{min}} \in \mathbb{R}^{N_{\mathit{invar\mathchar`-joint}}}$
   "
   (when (or update? (not (send self :get :phi-min-vector)))
     (send self :put :phi-min-vector
           (map float-vector
                #'(lambda (j)
                    (cond ((< (- (send j :max-angle) (send j :min-angle)) _invariant-joint-angle-margin)
                           (send j :angle-to-speed (* 0.5 (+ (send j :max-angle) (send j :min-angle)))))
                          (t
                           (send j :angle-to-speed (+ (send j :min-angle) _invariant-joint-angle-margin))))
                    )
                _invariant-joint-list
                ))
     )
   (send self :get :phi-min-vector)
   )
  (:delta-phi-limit-vector
   (&key (update? nil))
   "
get trust region of $\bm{\phi}$

return $\Delta \bm{\phi}_{\mathit{limit}}$
   "
   (unless (and _delta-linear-joint _delta-rotational-joint)
     (return-from :delta-phi-limit-vector nil))
   (when (or update? (not (send self :get :delta-phi-limit-vector)))
     (send self :put :delta-phi-limit-vector
           (map float-vector
                #'(lambda (j)
                    (send j :angle-to-speed
                          (if (derivedp j linear-joint) _delta-linear-joint _delta-rotational-joint)))
                _invariant-joint-list))
     )
   (send self :get :delta-phi-limit-vector)
   )
  (:phi-inequality-constraint-matrix
   (&key (update? nil))
   "
\begin{eqnarray}
&& \left\{ \begin{array}{ll}
\bm{\phi}_{\mathit{min}} \leq \bm{\phi} + \Delta \bm{\phi} \leq \bm{\phi}_{\mathit{max}} & \\
- \Delta \bm{\phi}_{\mathit{limit}} \leq \Delta \bm{\phi} \leq \Delta \bm{\phi}_{\mathit{limit}} & ({\rm if} \ \ \Delta \bm{\phi}_{\mathit{limit}} \ \ {\rm is \ set})
\end{array} \right. \\
\Leftrightarrow&&\begin{pmatrix} \bm{I} \\ - \bm{I} \\ \bm{I} \\ - \bm{I}\end{pmatrix} \Delta \bm{\phi}
\geq \begin{pmatrix} \bm{\phi}_{\mathit{min}} - \bm{\phi} \\ - (\bm{\phi}_{\mathit{max}} - \bm{\phi}) \\ - \Delta \bm{\phi}_{\mathit{limit}} \\ - \Delta \bm{\phi}_{\mathit{limit}} \end{pmatrix} \\
\Leftrightarrow&&\bm{C_{\phi}} \Delta \bm{\phi} \geq \bm{d_{\phi}}
\end{eqnarray}

return $\bm{C_{\phi}} := \begin{pmatrix} \bm{I} \\ - \bm{I} \\ \bm{I} \\ - \bm{I}\end{pmatrix} \in \mathbb{R}^{4 N_{\mathit{invar\mathchar`-joint}} \times N_{\mathit{invar\mathchar`-joint}}}$
   "
   (when (or update? (not (send self :get :phi-inequality-constraint-matrix)))
     (send self :put :phi-inequality-constraint-matrix
           (apply
            #'concatenate-matrix-column
            (remove
             nil
             (list
              (unit-matrix _dim-phi)
              (scale-matrix -1 (unit-matrix _dim-phi))
              (if (send self :delta-phi-limit-vector)
                  (unit-matrix _dim-phi))
              (if (send self :delta-phi-limit-vector)
                  (scale-matrix -1 (unit-matrix _dim-phi)))
              )))
           )
     )
   (send self :get :phi-inequality-constraint-matrix)
   )
  (:phi-inequality-constraint-vector
   (&key (update? t))
   "
return $\bm{d_{\phi}} := \begin{pmatrix} \bm{\phi}_{\mathit{min}} - \bm{\phi} \\ - (\bm{\phi}_{\mathit{max}} - \bm{\phi}) \\ - \Delta \bm{\phi}_{\mathit{limit}} \\ - \Delta \bm{\phi}_{\mathit{limit}} \end{pmatrix} \in \mathbb{R}^{4 N_{\mathit{invar\mathchar`-joint}}}$
   "
   (when (or update? (not (send self :get :phi-inequality-constraint-vector)))
     (send self :put :phi-inequality-constraint-vector
           (apply
            #'concatenate
            float-vector
            (remove
             nil
             (list
              (v- (send self :phi-min-vector) _phi-vector)
              (v- _phi-vector (send self :phi-max-vector))
              (if (send self :delta-phi-limit-vector)
                  (scale -1 (send self :delta-phi-limit-vector)))
              (if (send self :delta-phi-limit-vector)
                  (scale -1 (send self :delta-phi-limit-vector)))
              )))
           )
     )
   (send self :get :phi-inequality-constraint-vector)
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
  \bm{C}_{\tau} \Delta \bm{\tau} \geq \bm{d}_{\tau}
  \end{array}
  \right. \\
  \Leftrightarrow&&
  \begin{pmatrix}\bm{C}_{\theta}&&\\&\bm{C}_{\hat{w}}&\\&&\bm{C}_{\tau}\end{pmatrix}
  \begin{pmatrix}\Delta \bm{\theta}\\\Delta \bm{\hat{w}}\\\Delta \bm{\tau}\end{pmatrix}
  \geq
  \begin{pmatrix}\bm{d}_{\theta}\\\bm{d}_{\hat{w}}\\\bm{d}_{\tau}\end{pmatrix} \\
  \Leftrightarrow&&
  \bm{C}_{\mathit{var}} \Delta \bm{q}_{\mathit{var}} \geq \bm{d}_{\mathit{var}}
\end{eqnarray}

return $\bm{C}_{\mathit{var}} := \begin{pmatrix}\bm{C}_{\theta}&&\\&\bm{C}_{\hat{w}}&\\&&\bm{C}_{\tau}\end{pmatrix} \in \mathbb{R}^{N_{\mathit{var\mathchar`-ineq}} \times \mathit{dim}(\bm{q}_{\mathit{var}})}$
   "
   (concatenate-matrix-diagonal
    (send self :theta-inequality-constraint-matrix :update? update?)
    (send self :wrench-inequality-constraint-matrix :update? update?)
    (send self :torque-inequality-constraint-matrix :update? update?)
    )
   )
  (:variant-config-inequality-constraint-vector
   (&key (update? t))
   "
return $\bm{d}_{\mathit{var}} := \begin{pmatrix}\bm{d}_{\theta}\\\bm{d}_{\hat{w}}\\\bm{d}_{\tau}\end{pmatrix} \in \mathbb{R}^{N_{\mathit{var\mathchar`-ineq}}}$
   "
   (concatenate
    float-vector
    (send self :theta-inequality-constraint-vector :update? update?)
    (send self :wrench-inequality-constraint-vector :update? update?)
    (send self :torque-inequality-constraint-vector :update? update?)
    )
   )
  (:invariant-config-inequality-constraint-matrix
   (&key (update? nil))
   "
\begin{eqnarray}
  &&
  \bm{C}_{\phi} \Delta \bm{\phi} \geq \bm{d}_{\phi} \\
  \Leftrightarrow&&
  \bm{C}_{\mathit{invar}} \Delta \bm{q}_{\mathit{invar}} \geq \bm{d}_{\mathit{invar}}
\end{eqnarray}

return $\bm{C}_{\mathit{invar}} := \bm{C}_{\phi} \in \mathbb{R}^{N_{\mathit{invar\mathchar`-ineq}} \times \mathit{dim}(\bm{q}_{\mathit{invar}})}$
   "
   (send self :phi-inequality-constraint-matrix :update? update?)
   )
  (:invariant-config-inequality-constraint-vector
   (&key (update? t))
   "
return $\bm{d}_{\mathit{invar}} := \bm{d}_{\phi} \in \mathbb{R}^{N_{\mathit{invar\mathchar`-ineq}}}$
   "
   (send self :phi-inequality-constraint-vector :update? update?)
   )
  (:config-inequality-constraint-matrix
   (&key (update? nil) (update-collision? nil))
   "
\begin{eqnarray}
  &&
  \left\{
  \begin{array}{l}
  \bm{C}_{\mathit{var}} \Delta \bm{q}_{\mathit{var}} \geq \bm{d}_{\mathit{var}} \\
  \bm{C}_{\mathit{invar}} \Delta \bm{q}_{\mathit{invar}} \geq \bm{d}_{\mathit{invar}} \\
  \bm{C}_{\mathit{col}} \begin{pmatrix}\Delta \bm{q}_{\mathit{var}}\\\Delta \bm{q}_{\mathit{invar}}\end{pmatrix} \geq \bm{d}_{\mathit{col}}
  \end{array}
  \right. \\
  \Leftrightarrow&&
  \begin{pmatrix}\bm{C}_{\mathit{var}}& \\ &\bm{C}_{\mathit{invar}} \\ \hdashline[2pt/2pt] \multicolumn{2}{c}{\bm{C}_{\mathit{col}}} \end{pmatrix}
  \begin{pmatrix}\Delta \bm{q}_{\mathit{var}}\\\Delta \bm{q}_{\mathit{invar}}\end{pmatrix}
  \geq
  \begin{pmatrix}\bm{d}_{\mathit{var}}\\\bm{d}_{\mathit{invar}}\\\bm{d}_{\mathit{col}}\end{pmatrix} \\
  \Leftrightarrow&&
  \bm{C} \Delta \bm{q} \geq \bm{d}
\end{eqnarray}

return $\bm{C} := \begin{pmatrix}\bm{C}_{\mathit{var}}& \\ &\bm{C}_{\mathit{invar}} \\ \hdashline[2pt/2pt] \multicolumn{2}{c}{\bm{C}_{\mathit{col}}} \end{pmatrix} \in \mathbb{R}^{N_{\mathit{ineq}} \times \mathit{dim}(\bm{q})}$
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
return $\bm{d} := \begin{pmatrix}\bm{d}_{\mathit{var}}\\\bm{d}_{\mathit{invar}}\\\bm{d}_{\mathit{col}}\end{pmatrix} \in \mathbb{R}^{N_{\mathit{ineq}}}$
   "
   (concatenate
    float-vector
    (send self :variant-config-inequality-constraint-vector :update? update?)
    (send self :invariant-config-inequality-constraint-vector :update? update?)
    (send self :collision-inequality-constraint-vector :update? update-collision?)
    )
   )
  )

(defmethod instant-configuration-task
  (:variant-config-equality-constraint-matrix
   (&key (update? nil))
   "
return $\bm{A}_{\mathit{var}} \in \mathbb{R}^{0 \times \mathit{dim}(\bm{q}_{\mathit{var}})}$ (no equality constraint)
   "
   (make-matrix 0 _dim-variant-config)
   )
  (:variant-config-equality-constraint-vector
   (&key (update? t))
   "
return $\bm{b}_{\mathit{var}} \in \mathbb{R}^{0}$ (no equality constraint)
   "
   (float-vector)
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
return $\bm{A} \in \mathbb{R}^{0 \times \mathit{dim}(\bm{q})}$ (no equality constraint)
   "
   (concatenate-matrix-diagonal
    (send self :variant-config-equality-constraint-matrix :update? update?)
    (send self :invariant-config-equality-constraint-matrix :update? update?))
   )
  (:config-equality-constraint-vector
   (&key (update? t))
   "
return $\bm{b} \in \mathbb{R}^{0}$ (no equality constraint)
   "
   (concatenate
    float-vector
    (send self :variant-config-equality-constraint-vector :update? update?)
    (send self :invariant-config-equality-constraint-vector :update? update?)
    ))
  )

(defmethod instant-configuration-task
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
    & \mathit{dim}(\bm{\theta}) & \mathit{dim}(\bm{\hat{w}}) & \mathit{dim}(\bm{\tau}) \cr
    \mathit{dim}(\bm{\theta}) &&& \cr
    \mathit{dim}(\bm{\hat{w}}) &&& \cr
    \mathit{dim}(\bm{\tau}) &&&\bm{\bar{W}}_{trq} \cr
  }
  \in \mathbb{R}^{\mathit{dim}(\bm{q}_{\mathit{var}}) \times \mathit{dim}(\bm{q}_{\mathit{var}})}
\end{eqnarray}

otherwise:
\begin{eqnarray}
  \bm{W}_{trq} :=
  \bordermatrix{
    & \mathit{dim}(\bm{\theta}) & \mathit{dim}(\bm{\hat{w}}) & \mathit{dim}(\bm{\tau}) & \mathit{dim}(\bm{\phi}) \cr
    \mathit{dim}(\bm{\theta}) &&&& \cr
    \mathit{dim}(\bm{\hat{w}}) &&&& \cr
    \mathit{dim}(\bm{\tau}) &&&\bm{\bar{W}}_{trq}& \cr
    \mathit{dim}(\bm{\phi}) &&&& \cr
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
          (setf (aref (send self :get :torque-regular-matrix) (+ _dim-theta _dim-wrench i) (+ _dim-theta _dim-wrench i))
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
          (setf (elt (send self :get :torque-regular-vector) (+ _dim-theta _dim-wrench i))
                (/ tq (* tq-max tq-max))))
      (range _dim-torque)
      (coerce _torque-vector cons)
      (coerce (send self :torque-max-vector) cons))
     )
   (send self :get :torque-regular-vector)
   )
  (:torque-ratio
   ()
   "
return $\frac{\bm{\tau}}{\bm{\tau}_{\mathit{max}}} := \begin{pmatrix} \frac{\tau_1}{\tau_{\mathit{max,1}}} \\ \frac{\tau_2}{\tau_{\mathit{max,2}}} \\ \vdots \\ \frac{\tau_{N_{\mathit{drive\mathchar`-joint}}}}{\tau_{\mathit{max,N_{\mathit{drive\mathchar`-joint}}}}} \end{pmatrix}$
   "
   (map float-vector #'/
        _torque-vector
        (send self :torque-max-vector))
   )
  (:wrench-maximize-regular-vector
   (&key
    (update? nil)
    (only-variant? nil)
    )
   "
\begin{eqnarray}
  \bm{\bar{v}}_{\mathit{w\mathchar`-max}} &:=&
  \begin{pmatrix} \bm{d}_1^{\mathit{w\mathchar`-max}} \\ \bm{d}_2^{\mathit{w\mathchar`-max}} \\ \vdots \\ \bm{d}_{N_{\mathit{cnt}}}^{\mathit{w\mathchar`-max}} \end{pmatrix}
  \in \mathbb{R}^{\mathit{dim}(\bm{\hat{w}})}
\end{eqnarray}

\it{only-variant?} is true:
\begin{eqnarray}
  \bm{v}_{\mathit{w\mathchar`-max}} :=
  \bordermatrix{
    & 1 \cr
    \mathit{dim}(\bm{\theta}) & \cr
    \mathit{dim}(\bm{\hat{w}}) & \bm{\bar{v}}_{\mathit{w\mathchar`-max}} \cr
    \mathit{dim}(\bm{\tau}) & \cr
  }
  \in \mathbb{R}^{\mathit{dim}(\bm{q}_{\mathit{var}})}
\end{eqnarray}

otherwise:
\begin{eqnarray}
  \bm{v}_{\mathit{w\mathchar`-max}} :=
  \bordermatrix{
    & 1 \cr
    \mathit{dim}(\bm{\theta}) & \cr
    \mathit{dim}(\bm{\hat{w}}) & \bm{\bar{v}}_{\mathit{w\mathchar`-max}} \cr
    \mathit{dim}(\bm{\tau}) & \cr
    \mathit{dim}(\bm{\phi}) & \cr
  }
  \in \mathbb{R}^{\mathit{dim}(\bm{q})}
\end{eqnarray}

return $\bm{v}_{\mathit{w\mathchar`-max}}$
   "
   (when (or update? (not (send self :get :wrench-maximize-regular-vector)))
     (send self :put :wrench-maximize-regular-vector
           (if only-variant?
               (make-array _dim-variant-config :element-type :float :initial-element 0)
             (make-array _dim-config :element-type :float :initial-element 0)))

     (copy-block-vec
      (send self :get :wrench-maximize-regular-vector)
      (scale -1 (apply #'concatenate float-vector _wrench-maximize-direction-list))
      _dim-theta)
     )
   (send self :get :wrench-maximize-regular-vector)
   )
  (:regular-matrix
   ()
   "
\begin{eqnarray}
  \bm{W}_{\mathit{reg}} := \min(k_{\mathit{max}}, k_{\mathit{coeff}} \|\bm{e}\|^2+k_{\mathit{off}}) \bm{I} + k_{\mathit{trq}} \bm{W}_{trq}
\end{eqnarray}

return $\bm{W}_{\mathit{reg}} \in \mathbb{R}^{\mathit{dim}(\bm{q}) \times \mathit{dim}(\bm{q})}$
   "
   (m+
    (scale-matrix
     (min _norm-regular-scale-max
          (+ (* _norm-regular-scale-coeff (norm2 (send self :task-value :update? nil)))
             _norm-regular-scale-offset))
     (send self :_dim-config-unit-matrix))
    (scale-matrix _torque-regular-scale (send self :torque-regular-matrix))
    (send self :_dim-config-buf-matrix)
    ))
  (:regular-vector
   ()
   "
\begin{eqnarray}
  \bm{v}_{\mathit{reg}} := k_{\mathit{trq}} \bm{v}_{trq} + k_{\mathit{w\mathchar`-max}} \bm{v}_{\mathit{w\mathchar`-max}}
\end{eqnarray}

return $\bm{v}_{\mathit{reg}} \in \mathbb{R}^{\mathit{dim}(\bm{q})}$
   "
   (reduce
    #'v+
    (remove
     nil
     (list
      (scale _torque-regular-scale (send self :torque-regular-vector))
      (if (> _wrench-maximize-scale 0) (scale _wrench-maximize-scale (send self :wrench-maximize-regular-vector)))
      )))
   )
  )

(defmethod instant-configuration-task
  (:update-collision-inequality-constraint
   ()
   "
リンク1とリンク2の最近点を$\bm{p}_1, \bm{p}_2$とする．
リンク1とリンク2が干渉しない条件を，
最近点$\bm{p}_1, \bm{p}_2$の距離が$d_{\mathit{margin}}$以上である条件に置き換えて考える．
これは次式で表される．
\begin{eqnarray}
  &&\bm{d}_{12}^T ( \bm{p}_1 - \bm{p}_2 ) \geq d_{\mathit{margin}} \\
  &&{\rm where} \ \ \bm{d}_{12} = \bm{p}_1 - \bm{p}_2
\end{eqnarray}
コンフィギュレーションが$\Delta \bm{q}$だけ更新されてもこれが成立するための条件は次式で表される．
\begin{eqnarray}
  &&\bm{d}_{12}^T \left\{ ( \bm{p}_1 + \Delta \bm{p}_1 ) - ( \bm{p}_2 + \Delta \bm{p}_2 ) \right\} \geq d_{\mathit{margin}} \\
  &&{\rm where} \ \
  \Delta \bm{p}_1 = \bm{J}_{\theta, 1} \Delta \bm{\theta} + \bm{J}_{\phi, 1} \Delta \bm{\phi} \\
  &&\phantom{\rm where} \ \
  \Delta \bm{p}_2 = \bm{J}_{\theta, 2} \Delta \bm{\theta} + \bm{J}_{\phi, 2} \Delta \bm{\phi} \\
  &&\phantom{\rm where} \ \
  \bm{J}_{\theta, i} = \frac{\partial \bm{p}_i}{\partial \bm{\theta}}, \ \ \bm{J}_{\phi, i} = \frac{\partial \bm{p}_i}{\partial \bm{\phi}} \ \ (i = 1,2)
\end{eqnarray}
これは以下のように変形される．
\begin{eqnarray}
  &&\bm{d}_{12}^T \left\{ ( \bm{p}_1 + \bm{J}_{\theta, 1} \Delta \bm{\theta} + \bm{J}_{\phi, 1} \Delta \bm{\phi} ) - ( \bm{p}_2 + \bm{J}_{\theta, 2} \Delta \bm{\theta} + \bm{J}_{\phi, 2} \Delta \bm{\phi} ) \right\} \geq d_{\mathit{margin}} \\
  \Leftrightarrow&& \bm{d}_{12}^T (\bm{J}_{\theta, 1} - \bm{J}_{\theta, 2}) \Delta \bm{\theta} + \bm{d}_{12}^T (\bm{J}_{\phi, 1} - \bm{J}_{\phi, 2}) \Delta \bm{\phi} \geq - ( \bm{d}_{12}^T (\bm{p}_{1} - \bm{p}_2) - d_{\mathit{margin}} ) \\
  \Leftrightarrow&& \bm{c}_{\mathit{col,var}}^T \Delta \bm{\theta} + \bm{c}_{\mathit{col,invar}}^T \Delta \bm{\phi} \geq d_{\mathit{col}} \\
  &&{\rm where} \ \
  \bm{c}_{\mathit{col,var}}^T = \bm{d}_{12}^T (\bm{J}_{\theta, 1} - \bm{J}_{\theta, 2}) \\
  &&\phantom{\rm where} \ \
  \bm{c}_{\mathit{col,invar}}^T = \bm{d}_{12}^T (\bm{J}_{\phi, 1} - \bm{J}_{\phi, 2}) \\
  &&\phantom{\rm where} \ \
  d_{\mathit{col}} = - ( \bm{d}_{12}^T (\bm{p}_{1} - \bm{p}_2) - d_{\mathit{margin}} )
\end{eqnarray}

$i$番目の干渉回避リンクペアに関する行列，ベクトルをそれぞれ$\bm{c}_{\mathit{col,var,i}}^T, \bm{c}_{\mathit{col,invar,i}}^T, d_{\mathit{col, i}}$とする．
$i = 1,2,\cdots,N_{\mathit{col}}$の全てのリンクペアにおいて干渉が生じないための条件は次式で表される．
\begin{eqnarray}
  &&\begin{pmatrix} \bm{C}_{\mathit{col},\theta} & \bm{C}_{\mathit{col},\phi} \end{pmatrix}
  \begin{pmatrix} \Delta \bm{\theta} \\ \Delta \bm{\phi} \end{pmatrix} \geq \bm{d}_{\mathit{col}} \\
  &&\bm{C}_{\mathit{col},\theta} := \begin{pmatrix} \bm{c}_{\mathit{col,var,1}}^T \\ \vdots \\ \bm{c}_{\mathit{col,var,N_{col}}}^T \end{pmatrix} \in \mathbb{R}^{N_{\mathit{col}} \times \mathit{dim}(\bm{\theta})} \\
  &&\bm{C}_{\mathit{col},\phi} := \begin{pmatrix} \bm{c}_{\mathit{col,invar,1}}^T \\ \vdots \\ \bm{c}_{\mathit{col,invar,N_{col}}}^T \end{pmatrix} \in \mathbb{R}^{N_{\mathit{col}} \times \mathit{dim}(\bm{\phi})}, \ \, \\
  &&\bm{d}_{\mathit{col}} := \begin{pmatrix} d_{\mathit{col,1}} \\ \vdots \\ d_{\mathit{col,N_{col}}} \end{pmatrix} \in \mathbb{R}^{N_{\mathit{col}}}
\end{eqnarray}

update inequality matrix $\bm{C}_{\mathit{col},\theta}, \bm{C}_{\mathit{col,\phi}}$ and inequality vector $\bm{d}_{\mathit{col}}$ for collision avoidance
   "
   (unless _collision-pair-list
     (setq _collision-theta-inequality-constraint-matrix
           (make-matrix 0 _dim-theta))
     (setq _collision-phi-inequality-constraint-matrix
           (make-matrix 0 _dim-phi))
     (setq _collision-inequality-constraint-vector
           (float-vector))
     (return-from :update-collision-inequality-constraint nil)
     )
   (let* ((variant-mat-vector-list)
          (invariant-mat-vector-list)
          (dist-list)
          )
     (send self :put :collision-line-list nil)
     (dotimes (i (length _collision-pair-list))
       (let* ((pair (elt _collision-pair-list i))
              (b1 (elt pair 0))
              (b2 (elt pair 1))
              (col-ret (pqp-collision-distance b1 b2 :qsize 2))
              (p1 (elt col-ret 1))
              (p2 (elt col-ret 2))
              (c1 (make-cascoords :pos p1 :parent b1))
              (c2 (make-cascoords :pos p2 :parent b2))
              (dir (normalize-vector (v- p1 p2)))
              (dist-margin (elt _collision-distance-margin-list i))
              (dist (- (* 1e-3 (elt col-ret 0)) dist-margin)) ;; [m]
              (variant-mat-vector
               (transform
                dir
                (m-
                 (send _robot-env :calc-jacobian-from-joint-list
                       :union-joint-list _variant-joint-list
                       :move-target (list c1)
                       :transform-coords (list (make-coords))
                       :translation-axis (list t)
                       :rotation-axis (list nil)
                       )
                 (send _robot-env :calc-jacobian-from-joint-list
                       :union-joint-list _variant-joint-list
                       :move-target (list c2)
                       :transform-coords (list (make-coords))
                       :translation-axis (list t)
                       :rotation-axis (list nil)
                       ))))
              (invariant-mat-vector
               (transform
                dir
                (m-
                 (send _robot-env :calc-jacobian-from-joint-list
                       :union-joint-list _invariant-joint-list
                       :move-target (list c1)
                       :transform-coords (list (make-coords))
                       :translation-axis (list t)
                       :rotation-axis (list nil)
                       )
                 (send _robot-env :calc-jacobian-from-joint-list
                       :union-joint-list _invariant-joint-list
                       :move-target (list c2)
                       :transform-coords (list (make-coords))
                       :translation-axis (list t)
                       :rotation-axis (list nil)
                       ))))
              )
         (send self :put :collision-line-list
               (append (list (make-line p1 p2)) (send self :get :collision-line-list)))
         (push variant-mat-vector variant-mat-vector-list)
         (push invariant-mat-vector invariant-mat-vector-list)
         (push dist dist-list)
         (send b1 :dissoc c1)
         (send b2 :dissoc c2)
         ))
     (setq _collision-theta-inequality-constraint-matrix
           (apply #'matrix variant-mat-vector-list))
     (setq _collision-phi-inequality-constraint-matrix
           (apply #'matrix invariant-mat-vector-list))
     (setq _collision-inequality-constraint-vector
           (scale -1 (coerce dist-list float-vector)))
     ))
  (:collision-theta-inequality-constraint-matrix
   ()
   "
return $\bm{C}_{\mathit{col},\theta} \in \mathbb{R}^{N_{\mathit{col}} \times \mathit{dim}(\bm{\theta})}$

   "
   _collision-theta-inequality-constraint-matrix
   )
  (:collision-phi-inequality-constraint-matrix
   ()
   "
return $\bm{C}_{\mathit{col,\phi}} \in \mathbb{R}^{N_{\mathit{col}} \times \mathit{dim}(\bm{\phi})}$
   "
   _collision-phi-inequality-constraint-matrix
   )
  (:collision-inequality-constraint-matrix
   (&key (update? nil))
   "
\begin{eqnarray}
  \bm{C}_{\mathit{col}} :=
  \bordermatrix{
    & \mathit{dim}(\bm{\theta}) & \mathit{dim}(\bm{\hat{w}}) & \mathit{dim}(\bm{\tau}) & \mathit{dim}(\bm{\phi}) \cr
    N_{\mathit{col}} & \bm{C}_{\mathit{col},\theta} & \bm{O} & \bm{O} & \bm{C}_{\mathit{col,\phi}}
  }
\end{eqnarray}

return $\bm{C}_{\mathit{col}} \in \mathbb{R}^{N_{\mathit{col}} \times \mathit{dim}(\bm{q})}$
   "
   (when update?
     (send self :update-collision-inequality-constraint))
   (concatenate-matrix-row
    (send self :collision-theta-inequality-constraint-matrix)
    (make-matrix _num-collision (+ _dim-wrench _dim-torque))
    (send self :collision-phi-inequality-constraint-matrix))
   )
  (:collision-inequality-constraint-vector
   (&key (update? nil))
   "
return $\bm{d}_{\mathit{col}} \in \mathbb{R}^{N_{\mathit{col}}}$
   "
   (when update?
     (send self :update-collision-inequality-constraint))
   _collision-inequality-constraint-vector
   )
  )

(defmethod instant-configuration-task
  (:update-viewer
   ()
   "
Update viewer.
   "
   (send-all (flatten _kin-target-coords-list) :draw-on :flush nil :color #f(0 1 0) :size 50 :width 10)
   (dolist (att-coords (flatten _kin-attention-coords-list))
     (cond ((equal att-coords :cog)
            (send (scale 1e3 (send self :cog)) :draw-on :flush nil :color #f(1 0 0) :size 100 :width 2)
            )
           (t
            (send att-coords :draw-on :flush nil :color #f(1 0 0) :size 100 :width 2)
            ))
     )
   (send-all _contact-constraint-list :draw-on :flush nil)
   (send-all (send self :get :collision-line-list) :draw-on :flush nil :color #f(1 0 0) :width 2)
   (unless _only-kinematics?
     (draw-force-value
      :force-list (send self :force-list)
      :moment-list (send self :moment-list)
      :coords-list _contact-target-coords-list
      )
     (draw-torque
      :torque-vector _torque-vector
      :torque-max-vector (send self :torque-max-vector)
      :joint-list _drive-joint-list
      :width 5 :size 200 :flush nil
      :color (float-vector 1 1 0) :warning-color (float-vector 1 0 0)
      :torque-threshold 1.01
      )
     (when (> _num-external 0)
       (mapcar
        #'(lambda (f c)
            (with-set-color-line-width
             *viewer* #f(0.3 0.8 0.8) 5
             (send *irtviewer* :viewer :draw-arrow
                   (send c :worldpos)
                   (v+ (send c :worldpos) (scale 1 f)))))
        (send self :external-force-list)
        _external-coords-list
        ))
     )
   )
  (:print-status
   ()
   "
Print status.
   "
   (when (= _dim-task 0)
     (return-from :print-status nil))
   (warning-message 9 "|task|=~a~%" (norm (send self :task-value :update? nil)))
   (warning-message 9 "|kin-task|=~a:~%" (norm (send self :kinematics-task-value :update? nil)))
   (mapcar
    #'(lambda (kin-task-i)
        (warning-message 9 "  |kin-task-i|=~a~%" (norm kin-task-i)))
    (divide-vector (send self :kinematics-task-value :update? nil) :segment-length 6)
    )
   (unless _only-kinematics?
     (warning-message 9 "|eom-trans-task|=~a~%" (norm (send self :eom-trans-task-value :update? nil)))
     (warning-message 9 "|eom-rot-task|=~a~%" (norm (send self :eom-rot-task-value :update? nil)))
     (warning-message 9 "|torque-task|=~a~%" (norm (send self :torque-task-value :update? nil)))
     )
   (warning-message 9 "|posture-task|=~a~%" (norm (send self :posture-task-value :update? nil)))
   )
  (:_get-contact-index-from-limb ;; used in trajectory-configuration-task
   (&key (limb))
   (position-if
    #'(lambda (att-coords)
        (some #'(lambda (limb-link)
                  (string= (send (send att-coords :parent) :name) (send limb-link :name)))
              (send _robot-env :robot limb))
        )
    _contact-attention-coords-list
    ))
  )
