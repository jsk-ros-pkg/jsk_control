(load "./util.l")
(load "./contact-kinematics.l")


;;;;;;;;;;;;;;;;;;;;;;;;;
;; robot-environment
;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass robot-environment
  :super cascaded-link
  :slots ((_robot "$\mathcal{R}$")
          (_robot-with-root-virtual "$\hat{\mathcal{R}}$")
          (_root-virtual-joint-list "list of root virtual joint")
          (_contact-list "$\{ \mathcal{C}_1, \mathcal{C}_2, \cdots, \mathcal{C}_{N_C} \}$")
          (_variant-joint-list "$\mathcal{J}_{\mathit{var}}$")
          (_invariant-joint-list "$\mathcal{J}_{\mathit{invar}}$")
          (_drive-joint-list "$\mathcal{J}_{\mathit{drive}}$")
          )
  :documentation
  "
ロボットとロボット・環境間の接触のクラス．

以下を合わせた関節・リンク構造に関するメソッドが定義されている．
\begin{enumerate}
\setlength{\itemindent}{10mm}
  \item 浮遊ルートリンクのための仮想関節付きのロボットの関節
  \item 接触位置を定める仮想関節
\end{enumerate}

関節・リンク構造を定めるために，初期化時に以下を与える
\begin{description}
\setlength{\itemindent}{10mm}
\setlength{\leftskip}{-10mm}
  \item[robot $\mathcal{R}$] ロボット(cascaded-linkクラスのインスタンス)．
  \item[contact-list $\{ \mathcal{C}_1, \mathcal{C}_2, \cdots, \mathcal{C}_{N_C} \}$] 接触(2d-planar-contactクラスなどのインスタンス)のリスト．
\end{description}

ロボット$R$に，浮遊ルートリンクの変位に対応する仮想関節を付加した仮想関節付きロボット$\hat{\mathcal{R}}$を内部で保持する．
  "
  )

(defmethod robot-environment
  (:init
   (&key
    (robot)
    (contact-list)
    (root-virtual-mode :6dof) ;; :6dof, :planar, :fix are supported
    (root-virtual-joint-class-list)
    (root-virtual-joint-axis-list)
    (root-virtual-joint-min-angle-list)
    (root-virtual-joint-max-angle-list)
    )
   "
Initialize instance
   "
   (send-super :init)
   (setq _robot robot)
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
                      :joint-min-angle-list root-virtual-joint-min-angle-list
                      :joint-max-angle-list root-virtual-joint-max-angle-list
                      ))
          ))
   (setq links (flatten (list (send _robot-with-root-virtual :links) (send-all _contact-list :links))))
   (setq joint-list (flatten (list (send _robot-with-root-virtual :joint-list) (send-all _contact-list :joint-list))))
   (setq bodies (send _robot :bodies))
   (send self :update-descendants)
   (send self :_initialize-joint-list)
   self
   )
  (:_generate-cascaded-link-with-root-virtual
   (&key
    (target _robot)
    (mode)
    (joint-class-list)
    (joint-axis-list)
    (joint-min-angle-list)
    (joint-max-angle-list)
    )
   (cond ((equal mode :6dof)
          (setq joint-class-list
                (list linear-joint linear-joint linear-joint
                      rotational-joint rotational-joint rotational-joint))
          (setq joint-axis-list
                (list :x :y :z :x :y :z))
          )
         ((equal mode :planar)
          (setq joint-class-list
                (list linear-joint linear-joint rotational-joint))
          (setq joint-axis-list
                (list :x :y :z))
          ))
   (unless joint-min-angle-list
     (setq joint-min-angle-list
           (make-list (length joint-class-list) :initial-element -1e10))
     )
   (unless joint-max-angle-list
     (setq joint-max-angle-list
           (make-list (length joint-class-list) :initial-element 1e10))
     )
   (let ((cl-with-rv (instance cascaded-link :init))
         rv-ll
         )
     ;; 1. setup link
     (setq rv-ll
           (mapcar
            #'(lambda (joint-class
                       joint-axis
                       )
                (instance bodyset-link :init
                          (make-cascoords)
                          :bodies (list (make-cube 1e-3 1e-3 1e-3))
                          :name
                          (read-from-string
                           (format nil ":~a-root-virtual-~a-~a" (send target :name) (send joint-class :name) (symbol-name joint-axis)))
                          :weight 0.0
                          ))
            joint-class-list
            joint-axis-list
            ))
     (dotimes (i (1- (length rv-ll)))
       (send (elt rv-ll i) :assoc (elt rv-ll (1+ i))))
     (send cl-with-rv :assoc (car rv-ll))
     (send (car (last rv-ll)) :assoc target)
     ;; 2. setup joint
     (setq _root-virtual-joint-list
           (mapcar #'(lambda (joint-class
                              joint-axis
                              parent-link
                              child-link
                              min-angle
                              max-angle
                              )
                       (instance joint-class :init
                                 :name
                                 (read-from-string
                                  (format nil ":~a-root-virtual-~a-~a" (send target :name) (send joint-class :name) (symbol-name joint-axis)))
                                 :parent-link parent-link :child-link child-link
                                 :axis joint-axis :min min-angle :max max-angle
                                 :max-joint-torque 0.0
                                 ))
                   joint-class-list
                   joint-axis-list
                   (butlast (append rv-ll (list target)))
                   (cdr (append rv-ll (list target)))
                   joint-min-angle-list
                   joint-max-angle-list
                   ))
     ;; 3. init-ending
     (mapcar #'(lambda (jnt
                        parent-link
                        child-link
                        )
                 (send child-link :add-joint jnt)
                 (send child-link :add-parent-link parent-link)
                 (send parent-link :add-child-links child-link)
                 )
             _root-virtual-joint-list
             (butlast (append rv-ll (list (car (send target :links)))))
             (cdr (append rv-ll (list (car (send target :links)))))
             )
     ;; set cascaded-link instance
     (setf (slot cl-with-rv cascaded-link 'links) (append rv-ll (send target :links)))
     (setf (slot cl-with-rv cascaded-link 'joint-list) (append _root-virtual-joint-list (send target :joint-list)))
     cl-with-rv
     ))
  (:_initialize-joint-list
   ()
   (setq _invariant-joint-list
         (flatten (send-all _contact-list :joint-list)))
   (setq _variant-joint-list
         (set-difference joint-list _invariant-joint-list))
   (setq _drive-joint-list
         (send _robot :joint-list))
   )
  (:dissoc-root-virtual
   ()
   "
dissoc root virtual parent/child structure.
   "
   (when _root-virtual-joint-list
     (send (send _robot :parent) :dissoc _robot)
     (send (car (send _robot :links)) :del-parent-link)
     (send (car (send _robot :links)) :del-joint)
     )
   )
  (:init-pose
   ()
   "
set zero joint angle.
   "
   (send-all joint-list :joint-angle 0)
   (send self :angle-vector)
   )
  (:robot
   (&rest args)
   "
return $\mathcal{R}$
   "
   (forward-message-to _robot args)
   )
  (:robot-with-root-virtual
   (&rest args)
   "
return $\hat{\mathcal{R}}$
   "
   (forward-message-to _robot-with-root-virtual args)
   )
  (:contact-list
   (&rest args)
   "
return $\{ \mathcal{C}_1, \mathcal{C}_2, \cdots, \mathcal{C}_{N_C} \}$
   "
   (forward-message-to _contact-list args)
   )
  (:contact
   (name &rest args)
   "
return $\mathcal{C}_i$
   "
   (forward-message-to (find-if #'(lambda (c) (equal (send c :name) name)) _contact-list) args)
   )
  (:variant-joint-list
   (&optional (jl :nil))
   "
return $\mathcal{J}_{\mathit{var}}$
   "
   (if (equal jl :nil) _variant-joint-list (setq _variant-joint-list jl))
   )
  (:invariant-joint-list
   (&optional (jl :nil))
   "
return $\mathcal{J}_{\mathit{invar}}$
   "
   (if (equal jl :nil) _invariant-joint-list (setq _invariant-joint-list jl))
   )
  (:drive-joint-list
   (&optional (jl :nil))
   "
return $\mathcal{J}_{\mathit{drive}}$
   "
   (if (equal jl :nil) _drive-joint-list (setq _drive-joint-list jl))
   )
  (:root-virtual-joint-list
   ()
   "
return list of root virtual joint
   "
   _root-virtual-joint-list
   )
  )
