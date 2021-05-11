(load "./robot-environment.l")

(unless (boundp 'robot-environment) ;; dummy class definition for document
  (defclass robot-environment :super object))


;;;;;;;;;;;;;;;;;;;;;;;;;
;; robot-object-environment
;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass robot-object-environment
  :super robot-environment
  :slots ((_obj "$\mathcal{O}$")
          (_obj-with-root-virtual "$\hat{\mathcal{O}}$")
          )
  :documentation
  "
ロボットと物体とロボット・環境間の接触のクラス．

以下を合わせた関節・リンク構造に関するメソッドが定義されている．
\begin{enumerate}
\setlength{\itemindent}{10mm}
  \item 浮遊ルートリンクのための仮想関節付きのロボットの関節
  \item 物体位置姿勢を表す仮想関節
  \item 接触位置を定める仮想関節
\end{enumerate}

関節・リンク構造を定めるために，初期化時に以下を与える
\begin{description}
\setlength{\itemindent}{10mm}
\setlength{\leftskip}{-10mm}
  \item[robot $\mathcal{R}$] ロボット(cascaded-linkクラスのインスタンス)．
  \item[object $\mathcal{O}$] 物体(cascaded-linkクラスのインスタンス)．関節をもたないことを前提とする．
  \item[contact-list $\{ \mathcal{C}_1, \mathcal{C}_2, \cdots, \mathcal{C}_{N_C} \}$] 接触(2d-planar-contactクラスなどのインスタンス)のリスト．
\end{description}

ロボット$R$に，浮遊ルートリンクの変位に対応する仮想関節を付加した仮想関節付きロボット$\hat{\mathcal{R}}$を内部で保持する．
同様に，物体$O$に，物体の変位に対応する仮想関節を付加した仮想関節付き物体$\hat{\mathcal{O}}$を内部で保持する．
  "
  )

(defmethod robot-object-environment
  (:init
   (&key
    (robot)
    (object)
    (contact-list)
    (root-virtual-mode :6dof) ;; :6dof, :planar, :fix are supported
    (root-virtual-joint-class-list)
    (root-virtual-joint-axis-list)
    )
   "
Initialize instance
   "
   (send-message self cascaded-link :init)
   (setq _robot robot)
   (setq _obj object)
   (setq _contact-list contact-list)
   (cond ((equal root-virtual-mode :fix)
          (setq _robot-with-root-virtual robot)
          )
         (t
          (setq _robot-with-root-virtual
                (send self :_generate-cascaded-link-with-root-virtual
                      :mode root-virtual-mode
                      :joint-class-list root-virtual-joint-class-list
                      :joint-axis-list root-virtual-joint-axis-list
                      ))
          ))
   (setq _obj-with-root-virtual
         (send self :_generate-cascaded-link-with-root-virtual :target _obj :mode :6dof))
   (setq links
         (flatten
          (list (send _robot-with-root-virtual :links) (send _obj-with-root-virtual :links) (send-all _contact-list :links))))
   (setq joint-list
         (flatten
          (list (send _robot-with-root-virtual :joint-list) (send _obj-with-root-virtual :joint-list) (send-all _contact-list :joint-list))))
   (setq bodies (append (send _robot :bodies) (send _obj :bodies)))
   (send self :update-descendants)
   (send self :_initialize-joint-list)
   self
   )
  (:object
   (&rest args)
   "
return $\mathcal{O}$
   "
   (forward-message-to _obj args)
   )
  (:object-with-root-virtual
   (&rest args)
   "
return $\hat{\mathcal{O}}$
   "
   (forward-message-to _obj-with-root-virtual args)
   )
  )
