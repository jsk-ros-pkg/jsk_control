(require (format nil "~A/prioritized-inverse-kinematics/euslisp/util" (ros::resolve-ros-path "package://eus_qp")))
(require (format nil "~A/prioritized-inverse-kinematics/euslisp/inverse-kinematics-variable" (ros::resolve-ros-path "package://eus_qp")))

;;;;;;;;;;;;;;;;;;;;;;;;;
;; inverse-kinematics-task
;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass inverse-kinematics-task
  :super propertied-object
  :slots ((type ",type of task")
          (A "$\mathbf{A}$")
          (b "$\bm{b}$")
          (W "$\mathbf{W}$")
          (Asparce "$\mathbf{A}_{sparce}$, used for sparce matrix calculation")
          (rows "size of row of $\mathbf{A}$")
          (cols "size of column of $\mathbf{A}$ (constant)")
          )
  :documentation
  "
\ \ 各タスクを表すクラス．\\
\it{type}の値に応じて\it{prioritized-inverse-kinematics}中で次の問題に変換される．
\begin{itemize}
\item type = :equality-constraint\\
  \begin{eqnarray}
    \min_{\bm{x}} &\ & (\mathbf{A}\bm{x} - \bm{b})^\mathrm{T} \mathbf{W} (\mathbf{A}\bm{x} - \bm{b}) \nonumber
  \end{eqnarray}
\item type = :inequality-constraint\\
  \begin{eqnarray}
    \min_{\bm{x},\bm{\omega}} &\ & \bm{\omega}^\mathrm{T} \mathbf{W} \bm{\omega} \nonumber\\
    \rm{subject\ to} &\ & \mathbf{A}\bm{x} - \bm{b} \geqq \bm{\omega} \nonumber
  \end{eqnarray}
\end{itemize}
  "
  )

(defmethod inverse-kinematics-task
  (:init
   ()
   "
Initialize instance
   "
   (setq type :equality-constraint)
   (setq rows 0)
   t
   )
  (:initialize
   (variable-length variables)
   "
called at the start of prioritized-inverse-kinematics
   "
   (setq cols variable-length)
   (setq A (make-matrix rows cols))
   (setq Asparce (make-matrix rows cols))
   (setq b (instantiate float-vector rows))
   (setq W (make-matrix rows rows))
   t)
  (:update
   ()
   "
called at the start of each iteration.\\
update A, b , W, Asparce, rows
   "
   (setq rows 0)
   (if (and (equal (array-dimension A 0) rows) (equal (array-dimension Asparce 0) rows))
       (progn
         (fill A 0)
         (fill Asparce 0))
       (progn
         (setq A (make-matrix rows cols))
         (setq Asparce (make-matrix rows cols))))
   (if (equal (length b) rows)
       (fill b 0)
       (setq b (instantiate float-vector rows)))
   (if (equal (array-dimensions W) (list rows rows))
       (fill W 0)
       (setq W (make-matrix rows rows)))
   t)
  (:type () "return type" type)
  (:A () "return $\mathbf{A}$" A)
  (:b () "return $\bm{b}$" b)
  (:W () "return $\mathbf{W}$" W)
  (:Asparce () "return $\mathbf{A}_{sparce}$" Asparce)
  )


;;;;;;;;;;;;;;;;;;;;;;;;;
;; move-target-task
;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass move-target-task
  :super inverse-kinematics-task
  :slots ((target-coords)
          (move-target)
          (translation-axis ",represented in translation-coords")
          (translation-coords)
          (Wtrans "$\mathbf{W}_{trans}$, represented in translation-coords")
          (rows-trans "size of row of $\bm{e}_{trans}$")
          (rotation-axis ",represented in rotation-coords")
          (rotation-coords)
          (Wrot "$\mathbf{W}_{rot}$, represented in rotation-coords")
          (rows-rot "size of row of $\bm{e}_{rot}$")
          )
  :documentation
  "
\ \ 2つの\it{coordinates}を一致させるタスクを表すクラス．\it{move-target}と\it{target-coords}を一致させる．\it{move-target}と\it{target-coords}はどちらも動いてよい．\\
\it{move-target}と\it{target-coords}の並進誤差を\it{translation-coords}の座標系で表現し，\it{translation-axis}によって抽出された成分を$\bm{e}_{trans}$と表す．\it{move-target}と\it{target-coords}の回転誤差を\it{rotation-coords}の座標系で表現し，\it{rotation-axis}によって抽出された成分を$\bm{e}_{rot}$と表す．$\bm{e}_{trans}$，$\bm{e}_{rot}$のヤコビアンをそれぞれ$\mathbf{J}_{trans}$，$\mathbf{J}_{rot}$とすると，\it{prioritized-inverse-kinematics}中で次の問題に変換される．
\begin{eqnarray}
  \min_x &\ & (\mathbf{J}_{trans} \bm{x} - \bm{e}_{trans})^\mathrm{T} \mathbf{W}_{trans} (\mathbf{J}_{trans} \bm{x} - \bm{e}_{trans}) + (\mathbf{J}_{rot} \bm{x} - \bm{e}_{rot})^\mathrm{T} \mathbf{W}_{rot} (\mathbf{J}_{rot} \bm{x} - \bm{e}_{rot}) \nonumber
\end{eqnarray}
$\bm{e}_{trans}$の単位は\rm{m}，$\bm{e}_{rot}$の単位は\rm{rad}である．
"
  )

(defmethod move-target-task
  (:init
   (_target-coords
    _move-target
    &key
    ((:translation-axis _translation-axis) t)
    ((:rotation-axis _rotation-axis) t)
    ((:transform-coords _translation-coords) (make-coords))
    ((:rotation-coords _rotation-coords) (make-coords))
    ((:Wtrans _Wtrans) 1.0)
    ((:Wrot _Wrot) 1.0)
    )
   "
\begin{itemize}
  \item target-coords, move-target\\
\it{coordinate}クラスまたは\it{coordinate}クラスを返す\it{function}クラス.\it{cascaded-coords}を返す\it{function}クラスの場合，途中でそれまでと別のparent座標系をもつ\it{cascaded-coords}を返すように変わってはならない．
  \item translation-axis : t :x :y :z :xy :yx :yz :zy :zx :xz nil
  \item rotation-axis : t :x :y :z nil
  \item translation-coords\\
\it{translation-axis}及び\it{Wtrans}は\it{translation-coords}系で表現される
  \item rotation-coords\\
\it{rotation-axis}及び\it{Wrot}は\it{rotation-coords}系で表現される
  \item Wtrans\\
\it{float}クラスの場合$\mathbf{W}_{trans}$はその値を対角成分に並べた行列になる\\
\it{vector}クラスの場合$\mathbf{W}_{trans}$は各値を対角成分に並べた行列になる\\
\it{matrix}クラスの場合$\mathbf{W}_{trans}$としてそのまま使用される．
  \item Wrot\\
\it{float}クラスの場合$\mathbf{W}_{rot}$はその値を対角成分に並べた行列になる\\
\it{vector}クラスの場合$\mathbf{W}_{rot}$は各値を対角成分に並べた行列になる\\
\it{matrix}クラスの場合$\mathbf{W}_{rot}$としてそのまま使用される．
\end{itemize}
   "
   (setq target-coords _target-coords)
   (setq move-target _move-target)
   (setq translation-axis _translation-axis)
   (setq rotation-axis _rotation-axis)
   (setq translation-coords _translation-coords)
   (setq rotation-coords _rotation-coords)
   (setq rows-trans (case translation-axis
                      ((t) 3)
                      ((:x :y :z) 2)
                      ((:xy :yx :yz :zy :zx :xz) 1)
                      (nil 0)))
   (setq rows-rot (case rotation-axis
                      ((t) 3)
                      ((:x :y :z) 2)
                      (nil 0)))
   (cond
     ((numberp _Wtrans) (setq Wtrans (scale-matrix _Wtrans (unit-matrix rows-trans))))
     ((vectorp _Wtrans) (if (= (length _Wtrans) rows-trans)
                            (setq Wtrans (diagonal _Wtrans))
                            (error "length of Wtrans (~A) should be ~A~%" (length _Wtrans) rows-trans)))
     ((matrixp _Wtrans) (if (and (= (array-dimension _Wtrans 0) rows-trans) (= (array-dimension _Wtrans 1) rows-trans))
                            (setq Wtrans _Wtrans)
                            (error "size of Wtrans ~A should be ~A" (array-dimensions _Wtrans) rows-trans)))
     )
   (cond
     ((numberp _Wrot) (setq Wrot (scale-matrix _Wrot (unit-matrix rows-rot))))
     ((vectorp _Wrot) (if (= (length _Wrot) rows-rot)
                            (setq Wrot (diagonal _Wrot))
                            (error "length of Wrot (~A) should be ~A~%" (length _Wrot) rows-rot)))
     ((matrixp _Wrot) (if (and (= (array-dimension _Wrot 0) rows-rot) (= (array-dimension _Wrot 1) rows-rot))
                            (setq Wrot _Wrot)
                            (error "size of Wrot ~A should be ~A" (array-dimensions _Wrot) rows-rot)))
     )
   (setq type :equality-constraint)
   (setq rows (+ rows-trans rows-rot))
   t
   )
  (:initialize
   (variable-length variables)
   "
探索変数\it{variables}のうち，\it{target-coords},\it{move-target}に影響を与える成分を調べ，記録する．
   "
   (setq cols variable-length)
   ;;TODO initialize A, b , W, Asparce, 
   t
   )
  (:update
   ()
   "
現在の状態近傍で線形近似し$\mathbf{A}$, $\bm{b}$を更新する
   "
   ;;TODO
   t
   )
  (:type () "return type" type)
  (:A () "return A" A)
  (:b () "return b" b)
  (:W () "return W" W)
  (:Asparce () "return Asparce" Asparce)
  )
