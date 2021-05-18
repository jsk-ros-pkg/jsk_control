;;;;;;;;;;;;;;;;;;;;;;;;;
;; inverse-kinematics-variable
;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass inverse-kinematics-variable
  :super propertied-object
  :slots ((dim ", dimension of variable")
          (index ", indexからindex + dim - 1]番目の$\bm{x}$の要素が対応する")
          (initial-state ", prioritized-inverse-kinematicsの初期状態")
          )
  :documentation
  "
探索変数を表現するクラス
  "
  )

(defmethod inverse-kinematics-variable
  (:init
   ()
   "
Initialize instance
   "
   (setq dim 0)
   t
   )
  (:index (&optional idx) "update or return index" (if idx (setq index idx) index)
   )
  (:apply-x
   (x)
   "
計算された$\bm{x}$の対応する要素を実際に適用する
   "
   t
   )
  (:revert
   ()
   "
\it{prioritized-inverse-kinematics}の初期状態に戻す
   "
   t
   )
  (:init-form
   ()
   "
\it{prioritized-inverse-kinematics}の開始時に一回呼ばれる.\it{inverse-kinematics-task}の\it{:initialize}より前に呼ばれる
   "
   t)
  (:cleanup-form
   ()
   "
\it{prioritized-inverse-kinematics}の終了時に\it{unwind-protect}を用いて一回呼ばれる.
   "
   t)
  (:dim () "return dim" dim)
  )

;;;;;;;;;;;;;;;;;;;;;;;;;
;; value-variable
;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass value-variable
  :super inverse-kinematics-variable
  :slots ((value)
          (value-scale)
          )
  :documentation
  "
valueのクラス．
  "
  )

(defmethod value-variable
  (:init
   (
    &key
    (initial-value 0.0)
    ((:value-scale _value-scale) 1.0)
    )
   "
最大値等を表現するための探索変数．$\bm{x}$に\it{value-scale}を乗じたものが\it{value}の更新量に相当する．ユーザが直接使うことはない．
   "
   (setq value initial-value)
   (setq value-scale _value-scale)
   (setq dim 1)
   (setq initial-state value)
   t
   )
  (:apply-x
   (x)
   "
$\bm{x}$の値だけjoint-angleを相対的に更新する．
   "
   (incf value (* (elt x index) value-scale))
   t
   )
  (:revert
   ()
   "
prioritized-inverse-kinematicsの初期状態に戻す
   "
   (setq value initial-state)
   t
   )
  (:value (&optional _value) "return value" (if _value (setq value _value) value))
  (:value-scale () "return value-scale" value-scale)
  )


;;;;;;;;;;;;;;;;;;;;;;;;;
;; joint-variable
;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass joint-variable
  :super inverse-kinematics-variable
  :slots ((j ", joint")
          )
  :documentation
  "
jointのクラス．
  "
  )

(defmethod joint-variable
  (:init
   (_joint)
   "
\it{joint}のjoint-angleに相当する探索変数．\bm{x}は\it{joint}の変位に相当する．\bm{x}の単位はdegree, rad．
   "
   (setq j _joint)
   (setq dim (send j :joint-dof))
   (setq initial-state (copy-object (send j :joint-angle)))
   t
   )
  (:apply-x
   (x)
   "
$\bm{x}$の値だけjoint-angleを相対的に更新する．
   "
   (send j :joint-angle
         (send j :speed-to-angle
               (if (= dim 1)
                   (elt x index)
                   (subseq x index (+ index dim))))
         :relative t)
   t
   )
  (:revert
   ()
   "
prioritized-inverse-kinematicsの初期状態に戻す
   "
   (send j :joint-angle initial-state)
   t
   )
  (:joint () "return joint" j)
  )


;;;;;;;;;;;;;;;;;;;;;;;;;
;; virtual-joint
;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass virtual-joint-variable
  :super joint-variable
  :slots ((child ", virtual jointの子リンク")
          (parent ", virtual jointの親リンク")
          )
  :documentation
  "
仮想関節のjoint-angleに相当する探索変数．\bm{x}は仮想関節の変位に相当する．\bm{x}の単位はdegree, rad．
  "
  )

(defmethod virtual-joint-variable
  (:init
   (_child
    &key
    (joint-type 6dof-joint)
    ((:parent _parent) (instance bodyset-link
                                 :init (make-cascoords :pos (copy-object (send _child :worldpos)))
                                 :bodies (list (make-cube 10 10 10))
                                 :name (format nil "virtual-link-~A" (send _child :name))
                                 :weight 0 :centroid (float-vector 0 0 0)
                                 :inertia-tensor (make-matrix 3 3)))
    joint-args
    )
   "
\it{child}は仮想関節の子リンクとなる．\it{child}は\it{cascaded-link}クラスか\it{bodyset-link}であり，\it{cascaded-link}クラスのオブジェクトであるなら(car (send *robot* :links))を与えるのではなくそのまま*robot*を与えること．
   "
   (setq child (if (derivedp _child cascaded-link)
                   (car (send _child :links))
                   _child))
   (setq parent _parent)
   (send-super :init (instance joint-type :init :child-link _child :parent-link parent))
   nil
   )
  (:init-form
   ()
   "
仮想関節を取り付ける
   "
   (send-message child bodyset-link :add-joint j)
   (send child :add-parent-link parent)
   (send parent :add-child-links child)
   t)
  (:cleanup-form
   ()
   "
仮想関節を除去する
   "
   (send child :del-joint)
   (send child :del-parent-link)
   (send parent :del-child-link child)
   )
  )
