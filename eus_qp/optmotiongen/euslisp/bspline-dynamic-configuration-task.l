(load "./bspline-configuration-task.l")


;;;;;;;;;;;;;;;;;;;;;;;;;
;; bspline-trajectory
;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass bspline-trajectory
  :super propertied-object
  :slots ((_start-time "$t_s$")
          (_finish-time "$t_f$")
          (_num-control-point "$N_{\mathit{ctrl}}$")
          (_bspline-order "B-spline order, $n$")
          (_dim-instant-config "$N_{\bar{q}}$")
          (_dim-control-vector "$\mathit{dim}(\bm{p}) := N_{\mathit{ctrl}} N_{\bar{q}}$")
          (_control-vector "$\bm{p}$")
          ;; scale
          (_zero-diff-stationery-start-finish-regular-scale "$k_{\mathit{stat,0}}$")
          (_first-diff-stationery-start-finish-regular-scale "$k_{\mathit{stat,1}}$")
          (_second-diff-stationery-start-finish-regular-scale "$k_{\mathit{stat,2}}$")
          (_diff-square-integration-regular-scale "$k_{\mathit{sqr}}$")
          ;; buffer
          (_diff-mat "buffer for $\bm{D}^k$")
          (_diff-mat-list "buffer for $\{ \bm{D}^1, \bm{D}^2, \cdots, \bm{D}^K \}$")
          (_extended-diff-mat-list "buffer for $\{ \bm{\hat{D}}_1, \bm{\hat{D}}_2, \cdots, \bm{\hat{D}}_K \}$")
          (_ineq-const-matrix "buffer for $\bm{C}_{p}$")
          (_ineq-const-vector "buffer for $\bm{d}_{p}$")
          )
  :documentation
  "
Bスプラインを利用した軌道のクラス．

Bスプラインベクトル・行列，制御点ベクトル・ベクトル，微分行列，瞬時コンフィギュレーションの取得や制御点ベクトルの更新のためのメソッドが定義されている．

Bスプライン軌道を定めるために，初期化時に以下を与える
\begin{description}
  \item[start-time $t_s$] 初期時刻
  \item[finish-time $t_f$] 終端時刻
  \item[num-control-point $N_{\mathit{ctrl}}$] 制御点の個数
  \item[bspline-order $n$] Bスプラインのオーダー
  \item[dim-instant-config $N_{\bar{q}}$] 瞬時コンフィギュレーションの次元
\end{description}

ある時刻の瞬時コンフィギュレーション$\bm{\bar{q}}(t) \in \mathbb{R}^{N_{\bar{q}}}$の$j$番目の要素$\bar{q}_j(t) \in \mathbb{R}$を次式で表す．
\begin{eqnarray}
  \bar{q}_j(t) = \sum_{i=0}^{N_{\mathit{ctrl}}-1} p_{j,i} b_{i,n}(t) = \bm{p}_j^T \bm{b}_n(t) \ \ (j=1,2,\cdots,N_{\bar{q}})
\end{eqnarray}
ただし，
\begin{eqnarray}
  \bm{p}_j &=& \begin{pmatrix} p_{j,0} \\ p_{j,1} \\ \vdots \\ p_{j,N_{\mathit{ctrl}}-1} \end{pmatrix} \in \mathbb{R}^{N_{\mathit{ctrl}}} \ \ (j=1,2,\cdots,N_{\bar{q}}) \\
  \bm{b}_n(t) &=& \begin{pmatrix} b_{0,n}(t) \\ b_{1,n}(t) \\ \vdots \\ b_{N_{\mathit{ctrl}}-1,n}(t) \end{pmatrix} \in \mathbb{R}^{N_{\mathit{ctrl}}}
\end{eqnarray}
$b_{i,n}(t)$はBスプライン基底関数である．
また，$p_{j,i}$をそれぞれ制御点と呼ぶ．

したがって，$\bm{\bar{q}}(t)$は次式で表される．
\begin{eqnarray}
  \bm{\bar{q}}(t)
  = \begin{pmatrix} \bar{q}_1(t) \\ \vdots \\ \bar{q}_{N_{\bar{q}}}(t) \end{pmatrix}
  = \begin{pmatrix}
    \bm{p}_1^T \bm{b}_n(t) \\ \bm{p}_2^T \bm{b}_n(t) \\ \vdots \\ \bm{p}_{N_{\bar{q}}}^T \bm{b}_n(t)
  \end{pmatrix}
  = \bm{P} \bm{b}_n(t)
\end{eqnarray}
ただし，
\begin{eqnarray}
  \bm{P} &=&
  \begin{pmatrix}
    \bm{p}_1^T \\ \bm{p}_2^T \\ \vdots \\ \bm{p}_{N_{\bar{q}}}^T
  \end{pmatrix} \in \mathbb{R}^{N_{\bar{q}} \times N_{\mathit{ctrl}}}
\end{eqnarray}

また，$\bm{\bar{q}}(t)$は，制御点を縦に並べたベクトルを分離して次式のようにも表される．
\begin{eqnarray}
  \bm{\bar{q}}(t)
  = \begin{pmatrix} \bm{b}_n^T(t) \bm{p}_1 \\  \bm{b}_n^T(t) \bm{p}_2 \\  \vdots \\ \bm{b}_n^T(t) \bm{p}_{N_{\bar{q}}} \end{pmatrix}
  = \begin{pmatrix} \bm{b}_n^T(t)&&&\bm{O}\\&\bm{b}_n^T(t)&&\\&&\ddots&\\\bm{O}&&&\bm{b}_n^T(t) \end{pmatrix}
  \begin{pmatrix} \bm{p}_1 \\  \bm{p}_2 \\  \vdots \\ \bm{p}_{N_{\bar{q}}} \end{pmatrix}
  = \bm{B}_n(t) \bm{p}
\end{eqnarray}
ただし，
\begin{eqnarray}
  \bm{B}_n(t) = \begin{pmatrix} \bm{b}_n^T(t)&&&\bm{O}\\&\bm{b}_n^T(t)&&\\&&\ddots&\\\bm{O}&&&\bm{b}_n^T(t) \end{pmatrix} \in \mathbb{R}^{N_{\bar{q}} \times N_{\mathit{ctrl}} N_{\bar{q}}}, \ \
  \bm{p} = \begin{pmatrix} \bm{p}_1 \\  \bm{p}_2 \\  \vdots \\ \bm{p}_{N_{\bar{q}}} \end{pmatrix} \in \mathbb{R}^{N_{\mathit{ctrl}} N_{\bar{q}}}
\end{eqnarray}


Bスプラインによる軌道表現の詳細については\secref{sec:theory-bspline}参照．
  "
  )

(defmethod bspline-trajectory
  (:init
   (&key
    (name)
    (start-time 0.0) ;; [sec]
    (finish-time 10.0) ;; [sec]
    (num-control-point 10)
    (bspline-order 3)
    (dim-instant-config 1)
    (stationery-start-finish-regular-scale 1.0)
    (zero-diff-stationery-start-finish-regular-scale 0.0)
    (first-diff-stationery-start-finish-regular-scale stationery-start-finish-regular-scale)
    (second-diff-stationery-start-finish-regular-scale stationery-start-finish-regular-scale)
    (diff-square-integration-regular-scale 1.0)
    )
   "
Initialize instance
   "
   (send self :name name)
   (setq _start-time start-time)
   (setq _finish-time finish-time)
   (setq _num-control-point num-control-point)
   (setq _bspline-order bspline-order)
   (setq _dim-instant-config dim-instant-config)
   (setq _dim-control-vector (* _num-control-point _dim-instant-config))
   (setq _control-vector (make-array _dim-control-vector :element-type :float))
   (setq _zero-diff-stationery-start-finish-regular-scale zero-diff-stationery-start-finish-regular-scale)
   (setq _first-diff-stationery-start-finish-regular-scale first-diff-stationery-start-finish-regular-scale)
   (setq _second-diff-stationery-start-finish-regular-scale second-diff-stationery-start-finish-regular-scale)
   (setq _diff-square-integration-regular-scale diff-square-integration-regular-scale)
   )
  (:start-time
   ()
   "
return $t_s$
   "
   _start-time
   )
  (:finish-time
   ()
   "
return $t_s$
   "
   _finish-time
   )
  (:num-control-point
   ()
   "
return $N_{\mathit{ctrl}}$
   "
   _num-control-point
   )
  (:dim-instant-confing
   ()
   "
return $N_{\bar{q}}$
   "
   _dim-instant-config
   )
  (:dim-control-vector
   ()
   "
return $\mathit{dim}(\bm{p}) := N_{\mathit{ctrl}} N_{\bar{q}}$
   "

   _dim-control-vector
   )
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
   (map
    float-vector
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
  \bm{B}_n(t) := \begin{pmatrix} \bm{b}_n^T(t)&&&\bm{O}\\&\bm{b}_n^T(t)&&\\&&\ddots&\\\bm{O}&&&\bm{b}_n^T(t) \end{pmatrix} \in \mathbb{R}^{N_{\bar{q}} \times N_{\mathit{ctrl}} N_{\bar{q}}}
\end{eqnarray}

return $\bm{B}_n(t)$
   "
   (let* ((bspline-matrix
           (make-matrix _dim-instant-config _dim-control-vector))
          (vec
           (send self :bspline-vector tm :order-offset order-offset))
          )
     (dotimes (i _dim-instant-config)
       (copy-block-matrix bspline-matrix
                          (matrix (copy-seq vec))
                          i (* i _num-control-point))
       )
     bspline-matrix
     ))
  (:control-vector
   ()
   "
\begin{eqnarray}
  \bm{p} := \begin{pmatrix} \bm{p}_1 \\ \vdots \\ \bm{p}_{N_{\bar{q}}} \end{pmatrix} \in \mathbb{R}^{N_{\mathit{ctrl}} N_{\bar{q}}}
\end{eqnarray}

return $\bm{p}$
   "
   _control-vector
   )
  (:control-matrix
   ()
   "
\begin{eqnarray}
  \bm{P} := \begin{pmatrix} \bm{p}_1^T \\ \vdots \\ \bm{p}_{N_{\bar{q}}}^T \end{pmatrix} \in \mathbb{R}^{N_{\bar{q}} \times N_{\mathit{ctrl}}}
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
  \bm{\hat{D}}_k &:=& \begin{pmatrix} (\bm{D}^k)^T&&\bm{O}\\&\ddots&\\\bm{O}&&(\bm{D}^k)^T \end{pmatrix} \in \mathbb{R}^{N_{\mathit{ctrl}} N_{\bar{q}} \times N_{\mathit{ctrl}} N_{\bar{q}}}
\end{eqnarray}

return $\bm{\hat{D}}_k$
   "
   (unless (assoc diff-order _extended-diff-mat-list)
     (let* ((extended-diff-mat
             (make-matrix _dim-control-vector _dim-control-vector))
            (diff-mat
             (transpose (send self :differential-matrix :diff-order diff-order)))
            )
       (dotimes (i _dim-instant-config)
         (copy-block-matrix extended-diff-mat
                            diff-mat
                            (* i _num-control-point) (* i _num-control-point))
         )
       (push (list diff-order extended-diff-mat) _extended-diff-mat-list)
       ))
   (cadr (assoc diff-order _extended-diff-mat-list))
   )
  (:instant-config
   (tm)
   "
return $\bm{\bar{q}}(t) = \begin{pmatrix} \bm{p}_1^T\bm{b}_n(t) \\ \vdots \\ \bm{p}_{N_{\bar{q}}}^T\bm{b}_n(t) \end{pmatrix} = \bm{P} \bm{b}_{n}(t) = \bm{B}_{n}(t) \bm{p} \in \mathbb{R}^{N_{\bar{q}}}$
   "
   (map
    float-vector
    #'(lambda (ctrl-vec)
        (v. ctrl-vec (send self :bspline-vector tm)))
    (divide-vector _control-vector :segment-length _num-control-point))
   )
  (:instant-config-dot
   (tm
    &key
    (diff-order 1)
    )
   "
return $\bm{\bar{q}}^{(k)}(t) = \dfrac{d^{(k)} \bm{\bar{q}}(t)}{d t^{(k)}} = \bm{P} \bm{D}^k \bm{b}_{n-k}(t)$
   "
   (transform
    (send self :control-matrix)
    (transform
     (send self :differential-matrix :diff-order diff-order)
     (send self :bspline-vector tm :order-offset (- diff-order))
     ))
   )
  (:set-control-vector
   (control-vector-new
    &key
    (relative? nil)
    )
   "
Set $\bm{p} \in \mathbb{R}^{N_{\mathit{ctrl}} N_{\bar{q}}}$.
   "
   (cond (relative?
          (v+ _control-vector control-vector-new _control-vector)
          )
         (t
          (setq _control-vector control-vector-new)
          ))
   )
  (:set-control-vector-from-instant-config
   (instant-config)
   "
Set $\bm{p} \in \mathbb{R}^{N_{\mathit{ctrl}} N_{\bar{q}}}$ from $\bm{\bar{q}} \in \mathbb{R}^{N_{\bar{q}}}$.
   "
   (send self :set-control-vector
         (apply
          #'concatenate
          float-vector
          (map
           cons
           #'(lambda (x)
               (make-array _num-control-point :element-type :float :initial-element x))
           instant-config
           )))
   )
  )

(defmethod bspline-trajectory
  (:convert-instant-inequality-constraint-matrix-for-control-vector
   (&key
    (instant-ineq-matrix)
    (update? nil)
    )
   "
\begin{eqnarray}
  \bm{\bar{q}}(t) = \begin{pmatrix} \bar{q}_1(t) \\ \vdots \\ \bar{q}_{N_{\bar{q}}}(t) \end{pmatrix}
  = \begin{pmatrix}
    \sum_{i=0}^{N_{\mathit{ctrl}}-1} p_{1,i} b_{i,n}(t) \\ \sum_{i=0}^{N_{\mathit{ctrl}}-1} p_{2,i} b_{i,n}(t) \\ \vdots \\ \sum_{i=0}^{N_{\mathit{ctrl}}-1} p_{N_{\bar{q}},i} b_{i,n}(t)
  \end{pmatrix}
  = \begin{pmatrix}
    \bm{p}_1^T \bm{b}_n(t) \\ \bm{p}_2^T \bm{b}_n(t) \\ \vdots \\ \bm{p}_{N_{\bar{q}}}^T \bm{b}_n(t)
  \end{pmatrix}
  = \bm{P} \bm{b}_n(t)
\end{eqnarray}
ただし，
\begin{eqnarray}
  \bm{P} &=&
  \begin{pmatrix}
    \bm{p}_1^T \\ \bm{p}_2^T \\ \vdots \\ \bm{p}_{N_{\bar{q}}}^T
  \end{pmatrix}
  = \begin{pmatrix}
    \bm{\tilde{p}}_0 & \bm{\tilde{p}}_1 & \cdots & \bm{\tilde{p}}_{N_{\mathit{ctrl}}-1}
  \end{pmatrix} \\
  \bm{\tilde{p}}_i &=& \begin{pmatrix} p_{1,i} \\ p_{2,i} \\ \vdots \\ p_{N_{\bar{q}},i} \end{pmatrix} \ \ (i = 0,1,\cdots,N_{\mathit{ctrl}}-1)
\end{eqnarray}

ここで制御点$\bm{p}$が次式を満たすとする．
\begin{eqnarray}
  \bm{c}^T \bm{\tilde{p}}_i \geq d \ \ (i = 0,1,\cdots,N_{\mathit{ctrl}}-1)
\end{eqnarray}
つまり，
\begin{eqnarray}
  \begin{pmatrix} c_1 & c_2 & \cdots & c_{N_{\bar{q}}} \end{pmatrix}
  \begin{pmatrix} p_{1,i} \\ p_{2,i} \\ \vdots \\ p_{N_{\bar{q}},i} \end{pmatrix}
  = \sum_{j=1}^{N_{\bar{q}}} c_j p_{j,i} \geq d \ \ (i = 0,1,\cdots,N_{\mathit{ctrl}}-1)
\end{eqnarray}
このとき，
\begin{eqnarray}
  \bm{c}^T \bm{\bar{q}}(t)
  &=& \begin{pmatrix} c_1 & c_2 & \cdots & c_{N_{\bar{q}}} \end{pmatrix}
  \begin{pmatrix}
    \sum_{i=0}^{N_{\mathit{ctrl}}-1} p_{1,i} b_{i,n}(t) \\ \sum_{i=0}^{N_{\mathit{ctrl}}-1} p_{2,i} b_{i,n}(t) \\ \vdots \\ \sum_{i=0}^{N_{\mathit{ctrl}}-1} p_{N_{\bar{q}},i} b_{i,n}(t)
  \end{pmatrix} \\
  &=& \sum_{j=1}^{N_{\bar{q}}} c_j \sum_{i=0}^{N_{\mathit{ctrl}}-1} p_{j,i} b_{i,n}(t) \\
  &=& \sum_{i=0}^{N_{\mathit{ctrl}}-1} \left( \sum_{j=1}^{N_{\bar{q}}} c_j p_{j,i} \right) b_{i,n}(t) \\
  &\geq& d \sum_{i=0}^{N_{\mathit{ctrl}}-1} b_{i,n}(t) \\
  &=& d
\end{eqnarray}
したがって，
\begin{eqnarray}
  &&\bm{C}_{\bar{q}} \bm{\bar{q}}(t) \geq \bm{d}_{\bar{q}} \\
  \Leftrightarrow&&
  \bm{C}_{\bar{q}} \bm{\tilde{p}}_i \geq \bm{d}_{\bar{q}} \ \ (i = 0,1,\cdots,N_{\mathit{ctrl}}-1) \\
  \Leftrightarrow&&
  \bordermatrix{
    & N_{\mathit{ctrl}} & N_{\mathit{ctrl}} & & N_{\mathit{ctrl}} \cr
    N_{\mathit{ineq}} & \bm{C}_{p,0,1} & \bm{C}_{p,0,2} & \cdots & \bm{C}_{p,0,N_{\bar{q}}} \cr
    N_{\mathit{ineq}} & \bm{C}_{p,1,1} & \bm{C}_{p,1,2} & \cdots & \bm{C}_{p,1,N_{\bar{q}}} \cr
    & \vdots & \vdots & \ddots & \vdots \cr
    N_{\mathit{ineq}} & \bm{C}_{p,N_{\mathit{ctrl}}-1,1} & \bm{C}_{p,N_{\mathit{ctrl}}-1,2} & \cdots & \bm{C}_{p,N_{\mathit{ctrl}}-1,N_{\bar{q}}} \cr
  }
  \begin{pmatrix} \bm{p}_1 \\ \bm{p}_2 \\ \vdots \\ \bm{p}_{N_{\bar{q}}} \end{pmatrix}
  \geq
  \begin{pmatrix} \bm{d}_{\bar{q}} \\ \bm{d}_{\bar{q}} \\ \vdots \\ \bm{d}_{\bar{q}} \end{pmatrix} \\
  \Leftrightarrow&&
  \bm{C}_{p} \bm{p} \geq \bm{d}_{p}
\end{eqnarray}
ただし，
\begin{eqnarray}
  \bm{C}_{\bar{q}} &=& \begin{pmatrix} \bm{c}_1 & \bm{c}_2 & \cdots & \bm{c}_{N_{\bar{q}}} \end{pmatrix}
  \in \mathbb{R}^{N_{\mathit{ineq}} \times N_{\bar{q}}} \\
  \bm{C}_{p,i,j} &=&
  \bordermatrix{
    & &&& i番目 &&& \cr
    & \bm{0}&\cdots&\bm{0}& \bm{c}_j & \bm{0}&\cdots&\bm{0} \cr
  } \in \mathbb{R}^{N_{\mathit{ineq}} \times N_{\mathit{ctrl}}} \\ &&(i = 0,1,\cdots,N_{\mathit{ctrl}}-1, \ j=1,2,\cdots,N_{\bar{q}})
\end{eqnarray}

このメソッドは$\bm{C}_{\bar{q}} \in \mathbb{R}^{N_{\mathit{ineq}} \times N_{\bar{q}}}$を受け取り，$\bm{C}_{p} \in \mathbb{R}^{N_{\mathit{ctrl}} N_{\mathit{ineq}} \times N_{\mathit{ctrl}} N_{\bar{q}}}$を返す．
   "
   (when (or update? (not (send self :get :convert-instant-inequality-constraint-matrix-for-control-vector)))
     (send self :put :convert-instant-inequality-constraint-matrix-for-control-vector
           (let* ((num-ineq
                   (elt (array-dimensions instant-ineq-matrix) 0))
                  (mat
                   (make-matrix (* _num-control-point num-ineq) (* _num-control-point _dim-instant-config)))
                  (bm (make-matrix num-ineq _num-control-point))
                  )
             (dotimes (i _num-control-point)
               (dotimes (j _dim-instant-config)
                 (let* ((c-vec (matrix-column instant-ineq-matrix j))
                        )
                   (fill (array-entity bm) 0)
                   (setf (matrix-column bm i) c-vec)
                   (copy-block-matrix mat
                                      bm
                                      (* i num-ineq)
                                      (* j _num-control-point))
                   )))
             mat
             ))
     )
   (send self :get :convert-instant-inequality-constraint-matrix-for-control-vector)
   )
  (:convert-instant-inequality-constraint-vector-for-control-vector
   (&key
    (instant-ineq-vector)
    (update? nil)
    )
   "
このメソッドは$\bm{d}_{\bar{q}} \in \mathbb{R}^{N_{\mathit{ineq}}}$を受け取り，$\bm{d}_{p} \in \mathbb{R}^{N_{\mathit{ctrl}} N_{\mathit{ineq}}}$を返す．
   "
   (when (or update? (not (send self :get :convert-instant-inequality-constraint-vector-for-control-vector)))
     (send self :put :convert-instant-inequality-constraint-vector-for-control-vector
           (apply #'concatenate float-vector (make-list _num-control-point :initial-element instant-ineq-vector)))
     )
   (send self :get :convert-instant-inequality-constraint-vector-for-control-vector)
   )
  (:stationery-start-finish-regular-matrix
   (&key
    (start-time _start-time)
    (finish-time _finish-time)
    (update? nil)
    )
   "
\begin{eqnarray}
  \bm{W}_{\mathit{stat}} &=&
  k_{\mathit{stat,0}} \bm{B}_{n}^T(t_s) \bm{B}_{n}(t_s) +
  k_{\mathit{stat,0}} \bm{B}_{n}^T(t_f) \bm{B}_{n}(t_f) \\ &+&
  k_{\mathit{stat,1}} (\bm{B}_{n-1}(t_s) \bm{\hat{D}}_1)^T (\bm{B}_{n-1}(t_s) \bm{\hat{D}}_1) +
  k_{\mathit{stat,1}} (\bm{B}_{n-1}(t_f) \bm{\hat{D}}_1)^T (\bm{B}_{n-1}(t_f) \bm{\hat{D}}_1) \\ &+&
  k_{\mathit{stat,2}} (\bm{B}_{n-2}(t_s) \bm{\hat{D}}_2)^T (\bm{B}_{n-2}(t_s) \bm{\hat{D}}_2) +
  k_{\mathit{stat,2}} (\bm{B}_{n-2}(t_f) \bm{\hat{D}}_2)^T (\bm{B}_{n-2}(t_f) \bm{\hat{D}}_2)
\end{eqnarray}

return $\bm{W}_{\mathit{stat}} \in \mathbb{R}^{N_{\mathit{ctrl}} N_{\bar{q}} \times N_{\mathit{ctrl}} N_{\bar{q}}}$
   "
   (when (= _dim-control-vector 0)
     (return-from :stationery-start-finish-regular-matrix (make-matrix 0 0)))
   (when (or update? (not (send self :get :stationery-start-finish-regular-matrix)))
     (send self :put :stationery-start-finish-regular-matrix
           (let* ((d1-mat
                   (send self :extended-differential-matrix :diff-order 1))
                  (d2-mat
                   (send self :extended-differential-matrix :diff-order 2))
                  (bd1-start-mat
                   (m* (send self :bspline-matrix start-time :order-offset -1) d1-mat))
                  (bd1-finish-mat
                   (m* (send self :bspline-matrix finish-time :order-offset -1) d1-mat))
                  (bd2-start-mat
                   (m* (send self :bspline-matrix start-time :order-offset -2) d2-mat))
                  (bd2-finish-mat
                   (m* (send self :bspline-matrix finish-time :order-offset -2) d2-mat))
                  (b0-start-mat
                   (send self :bspline-matrix start-time))
                  (b0-finish-mat
                   (send self :bspline-matrix finish-time))
                  )
             (reduce
              #'m+
              (list
               (scale-matrix
                _zero-diff-stationery-start-finish-regular-scale
                (m* (transpose b0-start-mat) b0-start-mat))
               (scale-matrix
                _zero-diff-stationery-start-finish-regular-scale
                (m* (transpose b0-finish-mat) b0-finish-mat))
               (scale-matrix
                _first-diff-stationery-start-finish-regular-scale
                (m* (transpose bd1-start-mat) bd1-start-mat))
               (scale-matrix
                _first-diff-stationery-start-finish-regular-scale
                (m* (transpose bd1-finish-mat) bd1-finish-mat))
               (scale-matrix
                _second-diff-stationery-start-finish-regular-scale
                (m* (transpose bd2-start-mat) bd2-start-mat))
               (scale-matrix
                _second-diff-stationery-start-finish-regular-scale
                (m* (transpose bd2-finish-mat) bd2-finish-mat))
               ))
             ))
     )
   (send self :get :stationery-start-finish-regular-matrix)
   )
  (:differential-square-integration-regular-matrix
   (&key
    (start-time _start-time)
    (finish-time _finish-time)
    (delta-time
     (/ (- finish-time start-time) 100.0)) ;; [sec]
    (diff-order 1)
    )
   "
\eqref{eq:bspline-square-integration}より，コンフィギュレーション微分の二乗積分は次式で得られる．
\begin{eqnarray}
  F_{sqr,k}(\bm{p})
  &=& \int_{t_s}^{t_f} \left\| \bm{\bar{q}}^{(k)}(t) \right\|^2 dt \\
  &=& \bm{p}^T \bm{W}_{\mathit{sqr,k}} \bm{p}
\end{eqnarray}
ただし，
\begin{eqnarray}
  \bm{W}_{\mathit{sqr,k}}
  &=& \int_{t_s}^{t_f} \left( \bm{B}_{n-k}(t) \bm{\hat{D}}_k \right)^T \bm{B}_{n-k}(t) \bm{\hat{D}}_k dt \\
  &=& \int_{t_s}^{t_f} \begin{pmatrix} \left( \bm{D}^k \bm{b}_{n-k}(t) \right) \left( \bm{D}^k \bm{b}_{n-k}(t) \right)^T && \bm{O} \\ &\ddots& \\ \bm{O} && \left( \bm{D}^k \bm{b}_{n-k}(t) \right) \left( \bm{D}^k \bm{b}_{n-k}(t) \right)^T \end{pmatrix} dt
\end{eqnarray}

return $k_{\mathit{sqr}} \bm{W}_{\mathit{sqr,k}} \in \mathbb{R}^{\mathit{dim}(\bm{p}) \times \mathit{dim}(\bm{p})}$
     "
   (when (= _dim-control-vector 0)
     (return-from :differential-square-integration-regular-matrix (make-matrix 0 0)))
   (let* ((reg-mat (make-matrix _num-control-point _num-control-point))
          (tm start-time)
          (tmp-vec)
          (finish? nil)
          )
     (while t
       (when (>= (+ tm delta-time) finish-time)
         (setq finish? t)
         (setq delta-time (- finish-time tm))
         )
       (setq tmp-vec
             (transform (send self :differential-matrix :diff-order diff-order)
                        (send self :bspline-vector (+ tm (* 0.5 delta-time)) :order-offset (- diff-order))))
       (m+ reg-mat (scale-matrix delta-time (m* (transpose (matrix tmp-vec)) (matrix tmp-vec))) reg-mat)
       (setq tm (+ tm delta-time))
       (when finish?
         (return-from nil nil))
       )
     (scale-matrix
      _diff-square-integration-regular-scale
      (apply #'concatenate-matrix-diagonal
             (make-list _dim-instant-config :initial-element reg-mat)))
     ))
  (:dump-config-data
   (&key
    (start-time _start-time)
    (finish-time _finish-time)
    (delta-time
     (/ (- finish-time start-time) 100.0)) ;; [sec]
    (data-filename
     (format nil "/tmp/~a.dat" (send self :name)))
    (diff-order 0)
    )
   (unix:system
    (format nil "DIRNAME=`dirname ~a`; if [ ! -d $DIRNAME ]; then mkdir -p $DIRNAME; fi" data-filename))
   (with-open-file
    (f data-filename :direction :output :if-exists :new-version :if-does-not-exist :create)
    (let* ((tm start-time)
           (finish? nil)
           (instant-config)
           (data-name-list
            (or (send self :get :data-name-list) (range _dim-instant-config)))
           )
      ;; dump header
      (format f "time ")
      (dolist (data-name data-name-list)
        (format f "~a " data-name))
      (format f "~%")
      (while t
        (setq instant-config
              (cond ((= diff-order 0)
                     (send self :instant-config tm)
                     )
                    ((> diff-order 0)
                     (send self :instant-config-dot tm :diff-order diff-order)
                     )))
        (format f "~a " tm)
        (dotimes (j _dim-instant-config)
          (format f "~a " (elt instant-config j)))
        (format f "~%")
        ;; update time
        (when finish?
          (return-from nil nil))
        (setq tm (+ tm delta-time))
        (when (>= tm finish-time)
          (setq finish? t)
          (setq tm finish-time)
          )
        )
      ))
   )
  )


;;;;;;;;;;;;;;;;;;;;;;;;;
;; bspline-dynamic-configuration-task
;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass bspline-dynamic-configuration-task
  :super propertied-object
  :slots ((_robot-env "robot-environment instance")
          ;; bspline trajectory instance of configuration
          (_theta-bst "bspline trajectory instance for $\bm{\theta}$")
          (_cog-bst "bspline trajectory instance for $\bm{c}$")
          (_ang-moment-bst "bspline trajectory instance for $\bm{L}$")
          (_wrench-bst "bspline trajectory instance for $\bm{\hat{w}}$")
          (_torque-bst "bspline trajectory instance for $\bm{\tau}$")
          (_phi-vector "$\bm{\phi}$")
          ;; number
          (_num-kin "$N_{\mathit{kin}} := |\mathcal{T}^{\mathit{kin\mathchar`-trg}}| = |\mathcal{T}^{\mathit{kin\mathchar`-att}}|$")
          (_num-contact "$N_{\mathit{cnt}} := |\mathcal{T}^{\mathit{cnt\mathchar`-trg}}| = |\mathcal{T}^{\mathit{cnt\mathchar`-att}}|$")
          (_num-variant-joint "$N_\mathit{var\mathchar`-joint} := |\mathcal{J}_{\mathit{var}}|$")
          (_num-invariant-joint "$N_\mathit{invar\mathchar`-joint} := |\mathcal{J}_{\mathit{invar}}|$")
          (_num-drive-joint "$N_\mathit{drive\mathchar`-joint} := |\mathcal{J}_{\mathit{drive}}|$")
          (_num-posture-joint "$N_{\mathit{posture\mathchar`-joint}} := |\mathcal{J}_{\mathit{posture}}|$")
          (_num-collision "$N_{\mathit{col}} :=$ number of collision check pairs")
          ;; dimensions
          (_dim-theta-control-vector "$\mathit{dim}(\bm{p}_{\theta}) := N_{\mathit{var\mathchar`-joint}} N_{\mathit{\theta\mathchar`-ctrl}}$")
          (_dim-cog-control-vector "$\mathit{dim}(\bm{p}_{c}) := 3 N_{\mathit{c\mathchar`-ctrl}}$")
          (_dim-ang-moment-control-vector "$\mathit{dim}(\bm{p}_{L}) := 3 N_{\mathit{L\mathchar`-ctrl}}$")
          (_dim-wrench-control-vector "$\mathit{dim}(\bm{p}_{\hat{w}}) := 6 N_{\mathit{cnt}} N_{\mathit{\hat{w}\mathchar`-ctrl}}$")
          (_dim-torque-control-vector "$\mathit{dim}(\bm{p}_{\tau}) := N_{\mathit{drive\mathchar`-joint}} N_{\mathit{\tau\mathchar`-ctrl}}$")
          (_dim-phi "$\mathit{dim}(\bm{\phi}) := N_{\mathit{invar\mathchar`-joint}}$")
          (_dim-config "$\mathit{dim}(\bm{q}) := \mathit{dim}(\bm{p}_{\theta})\!+\!\mathit{dim}(\bm{p}_{c})\!+\!\mathit{dim}(\bm{p}_{L})\!+\!\mathit{dim}(\bm{p}_{\hat{w}})\!+\!\mathit{dim}(\bm{p}_{\tau})\!+\!\mathit{dim}(\bm{\phi})$")
          (_dim-kin-task "$\mathit{dim}(\bm{e}^{\mathit{kin}})$")
          (_dim-eom-trans-task "$\mathit{dim}(\bm{e}^{\mathit{eom\mathchar`-trans}})$")
          (_dim-eom-rot-task "$\mathit{dim}(\bm{e}^{\mathit{eom\mathchar`-rot}})$")
          (_dim-cog-task "$\mathit{dim}(\bm{e}^{\mathit{cog}})$")
          (_dim-ang-moment-task "$\mathit{dim}(\bm{e}^{\mathit{ang\mathchar`-moment}})$")
          (_dim-torque-task "$\mathit{dim}(\bm{e}^{\mathit{trq}})$")
          (_dim-posture-task "$\mathit{dim}(\bm{e}^{\mathit{posture}})$")
          (_dim-task "$\mathit{dim}(\bm{e})$")
          ;; scale
          (_kin-task-scale "$k_{\mathit{kin}}$")
          (_kin-task-scale-mat-list-func "function returning list of $K_{\mathit{kin}}$")
          (_eom-trans-task-scale "$k_{\mathit{eom\mathchar`-trans}}$")
          (_eom-rot-task-scale "$k_{\mathit{eom\mathchar`-rot}}$")
          (_cog-task-scale "$k_{\mathit{cog}}$")
          (_ang-moment-task-scale "$k_{\mathit{ang\mathchar`-moment}}$")
          (_torque-task-scale "$k_{\mathit{trq}}$")
          (_posture-task-scale "$k_{\mathit{posture}}$")
          (_torque-regular-scale "$k_{\mathit{trq}}$")
          (_stationery-start-finish-regular-scale "$k_{\mathit{stat}}$")
          (_first-diff-square-integration-regular-scale "$k_{\mathit{sqr,1}}$")
          (_second-diff-square-integration-regular-scale "$k_{\mathit{sqr,2}}$")
          (_third-diff-square-integration-regular-scale "$k_{\mathit{sqr,3}}$")
          (_norm-regular-scale-max "$k_{\mathit{max}}$")
          (_norm-regular-scale-offset "$k_{\mathit{off}}$")
          ;; variables
          (_variant-joint-list "$\mathcal{J}_{\mathit{var}}$")
          (_invariant-joint-list "$\mathcal{J}_{\mathit{invar}}$")
          (_drive-joint-list "$\mathcal{J}_{\mathit{drive}}$")
          (_posture-joint-list "$\mathcal{J}_{\mathit{posture}}$")
          (_kin-task-time-list "time list for kinematics task")
          (_eom-task-time-list "time list for eom task")
          (_centroid-task-time-list "time list for centroid task")
          (_posture-task-time-list "time list for posture task")
          (_kin-target-coords-list-func "function returning $\mathcal{T}^{\mathit{kin\mathchar`-trg}}$")
          (_kin-attention-coords-list-func "function returning $\mathcal{T}^{\mathit{kin\mathchar`-att}}$")
          (_contact-target-coords-list-func "function returning $\mathcal{T}^{\mathit{cnt\mathchar`-trg}}$")
          (_contact-attention-coords-list-func "function returning $\mathcal{T}^{\mathit{cnt\mathchar`-att}}$")
          (_contact-constraint-list-func "function returning list of contact-constraint")
          (_posture-joint-angle-list "$\bm{\bar{\theta}}^{\mathit{trg}}$")
          (_variant-joint-angle-margin "margin of $\bm{\theta}$ [deg] [mm]")
          (_invariant-joint-angle-margin "margin of $\bm{\phi}$ [deg] [mm]")
          (_collision-pair-list "list of bodyset-link or body pair")
          (_collision-distance-margin-list "list of collision distance margin")
          ;; buffer
          (_task-jacobi "buffer for $\frac{\partial \bm{e}}{\partial \bm{q}}$")
          (_collision-theta-inequality-constraint-matrix "buffer for $\bm{C}_{\mathit{col},\theta}$")
          (_collision-phi-inequality-constraint-matrix "buffer for $\bm{C}_{\mathit{col,\phi}}$")
          (_collision-inequality-constraint-vector "buffer for $\bm{C}_{\mathit{col}}$")
          )
  :documentation
  "
Bスプラインを利用した動的動作生成のためのコンフィギュレーション$\bm{q}$とタスク関数$\bm{e}(\bm{q})$のクラス．

コンフィギュレーション$\bm{q}$の取得・更新，タスク関数$\bm{e}(\bm{q})$の取得，タスク関数のヤコビ行列$\frac{\partial \bm{e}(\bm{q})}{\partial \bm{q}}$の取得，コンフィギュレーションの等式・不等式制約$\bm{A}, \bm{b}, \bm{C}, \bm{d}$の取得のためのメソッドが定義されている．

\subsubsection*{初期化}

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
\item Bスプライン軌道
\begin{description}
  \item[theta-bst] 時変関節位置$\bm{\theta}$のBスプライン軌道のインスタンス
  \item[cog-bst] 重心位置$\bm{c}$のBスプライン軌道のインスタンス
  \item[ang-moment-bst] 角運動量$\bm{L}$のBスプライン軌道のインスタンス
  \item[wrench-bst] 接触レンチ$\bm{\hat{w}}$のBスプライン軌道のインスタンス
  \item[torque-bst] 関節トルク$\bm{\tau}$のBスプライン軌道のインスタンス
\end{description}
\item タスク関数のサンプリング時刻
\begin{description}
  \item[kin-task-time-list] 幾何到達拘束$\bm{e}^{\mathit{kin}}$の時刻のリスト
  \item[eom-task-time-list] 並進運動方程式$\bm{e}^{\mathit{eom\mathchar`-trans}}$，回転運動方程式$\bm{e}^{\mathit{eom\mathchar`-rot}}$の時刻リスト
  \item[centroid-task-time-list] 重心位置$\bm{e}^{\mathit{cog}}$，角運動量$\bm{e}^{\mathit{ang\mathchar`-moment}}$の時刻リスト
  \item[posture-task-time-list] 関節角目標$\bm{e}^{\mathit{posture}}$の時刻リスト
\end{description}
\item 幾何拘束
\begin{description}
  \item[kin-target-coords-list-func] 幾何到達目標位置姿勢リスト$\mathcal{T}^{\mathit{kin\mathchar`-trg}}$を返す関数
  \item[kin-attention-coords-list-func] 幾何到達着目位置姿勢リスト$\mathcal{T}^{\mathit{kin\mathchar`-att}}$を返す関数
\end{description}
\item 接触拘束
\begin{description}
  \item[contact-target-coords-list-func] 接触目標位置姿勢リスト$\mathcal{T}^{\mathit{cnt\mathchar`-trg}}$を返す関数
  \item[contact-attention-coords-list-func] 接触着目位置姿勢リスト$\mathcal{T}^{\mathit{cnt\mathchar`-att}}$を返す関数
  \item[contact-constraint-list-func] 接触レンチ制約リストを返す関数
\end{description}
\item コンフィギュレーション拘束(必要な場合のみ)
\begin{description}
  \item[posture-joint-list $\mathcal{J}_{\mathit{posture}}$] 着目関節リスト
  \item[posture-joint-angle-list $\bm{\bar{\theta}}^{\mathit{trg}}$] 着目関節の目標関節角
\end{description}
\item 干渉回避拘束(必要な場合のみ)
\begin{description}
  \item[collision-pair-list] 干渉回避するbodyset-linkもしくはbodyのペアのリスト
  \item[collision-distance-margin] 干渉回避の距離マージン(全てのペアで同じ値の場合)
  \item[collision-distance-margin-list] 干渉回避の距離マージンのリスト(ペアごとに異なる値の場合)
\end{description}
\item 目的関数の重み
\begin{description}
  \item[kin-task-scale $k_{\mathit{kin}}$] 幾何到達拘束タスクの重み
  \item[kin-task-scale-mat-list-func] 幾何到達拘束タスクの重み行列$K_{\mathit{kin}}$を返す関数
  \item[eom-trans-task-scale $k_{\mathit{eom\mathchar`-trans}}$] 並進運動方程式タスクの重み
  \item[eom-rot-task-scale $k_{\mathit{eom\mathchar`-rot}}$] 回転運動方程式タスクの重み
  \item[cog-task-scale $k_{\mathit{cog}}$] 重心位置タスクの重み
  \item[ang-moment-task-scale $k_{\mathit{ang\mathchar`-moment}}$] 角運動量タスクの重み
  \item[torque-task-scale $k_{\mathit{trq}}$] 関節トルクの釣り合いタスクの重み
  \item[posture-task-scale $k_{\mathit{posture}}$] 目標関節角タスクの重み
  \item[torque-regular-scale $k_{\mathit{trq}}$] トルク正則化の重み
  \item[stationery-start-finish-regular-scale $k_{\mathit{stat}}$] 初期・終端静止正則化の重み
  \item[first-diff-square-integration-regular-scale $k_{\mathit{sqr,1}}$] 速度正則化の重み
  \item[second-diff-square-integration-regular-scale $k_{\mathit{sqr,2}}$] 加速度正則化の重み
  \item[third-diff-square-integration-regular-scale $k_{\mathit{sqr,3}}$] 躍度正則化の重み
  \item[norm-regular-scale-max $k_{\mathit{max}}$] コンフィギュレーション更新量正則化の重み最大値
  \item[norm-regular-scale-offset $k_{\mathit{off}}$] コンフィギュレーション更新量正則化の重みオフセット
\end{description}
\end{itemize}

\subsubsection*{コンフィギュレーション}

動的動作は各瞬間において以下の瞬時状態$\bm{\bar{q}}(t)$を定めることで表現される．
\begin{eqnarray}
  \bm{\bar{q}}(t) := \begin{pmatrix} \bm{\theta}(t) \\ \bm{c}(t) \\ \bm{L}(t) \\ \bm{\hat{w}}(t) \\ \bm{\tau}(t) \\ \bm{\phi} \end{pmatrix}
\end{eqnarray}
\begin{description}
  \item[$\bm{\theta} \in \mathbb{R}^{N_{\mathit{var\mathchar`-joint}}}$] 時変関節位置 [rad] [m]
  \item[$\bm{c} \in \mathbb{R}^{3}$] 重心位置 [m]
  \item[$\bm{L} \in \mathbb{R}^{3}$] 角運動量 ${\rm [kg m^2 / s]}$
  \item[$\bm{\hat{w}} \in \mathbb{R}^{6 N_{\mathit{cnt}}}$] 接触レンチ [N] [Nm]
  \item[$\bm{\tau} \in \mathbb{R}^{N_{\mathit{drive\mathchar`-joint}}}$] 関節トルク [Nm] [N]
  \item[$\bm{\phi} \in \mathbb{R}^{N_{\mathit{invar\mathchar`-joint}}}$] 時不変関節位置 [rad] [m]
\end{description}
$\bm{\hat{w}}$は次式のように，全接触部位でのワールド座標系での力・モーメントを並べたベクトルである．
\begin{eqnarray}
  \bm{\hat{w}} &=& \begin{pmatrix} \bm{w}_1^T & \bm{w}_2^T & \cdots & \bm{w}_{N_{\mathit{cnt}}}^T \end{pmatrix}^T \\
  &=& \begin{pmatrix} \bm{f}_1^T &  \bm{n}_1^T &  \bm{f}_2^T &  \bm{n}_2^T & \cdots & \bm{f}_{N_{\mathit{cnt}}}^T &  \bm{n}_{N_{\mathit{cnt}}}^T \end{pmatrix}^T
\end{eqnarray}

本クラスでは，瞬時状態$\bm{\bar{q}}(t)$の軌道をBスプラインで表現する．
設計対称のコンフィギュレーション$\bm{q}$は以下から構成される．
\begin{eqnarray}
  \bm{q} := \begin{pmatrix} \bm{p}_{\theta} \\ \bm{p}_{c} \\ \bm{p}_{L} \\ \bm{p}_{\hat{w}} \\ \bm{p}_{\tau} \\ \bm{\phi} \end{pmatrix}
\end{eqnarray}
\begin{description}
  \item[$\bm{p}_{\theta} \in \mathbb{R}^{N_{\mathit{var\mathchar`-joint}} N_{\mathit{\theta\mathchar`-ctrl}}}$] 時変関節位置の制御点 [rad] [m]
  \item[$\bm{p}_{c} \in \mathbb{R}^{3 N_{\mathit{c\mathchar`-ctrl}}}$] 重心位置の制御点 [m]
  \item[$\bm{p}_{L} \in \mathbb{R}^{3 N_{\mathit{L\mathchar`-ctrl}}}$] 角運動量の制御点 ${\rm [kg m^2 / s]}$
  \item[$\bm{p}_{\hat{w}} \in \mathbb{R}^{6 N_{\mathit{cnt}} N_{\mathit{\hat{w}\mathchar`-ctrl}}}$] 接触レンチの制御点 [N] [Nm]
  \item[$\bm{p}_{\tau} \in \mathbb{R}^{N_{\mathit{drive\mathchar`-joint}} N_{\mathit{\tau\mathchar`-ctrl}}}$] 関節トルクの制御点 [Nm] [N]
  \item[$\bm{\phi} \in \mathbb{R}^{N_{\mathit{invar\mathchar`-joint}}}$] 時不変関節位置 [rad] [m]
\end{description}
制御点とは，Bスプライン基底関数の重み係数を意味する．

\subsubsection*{タスク関数}

瞬時状態$\bm{\bar{q}}(t)$に関するタスク関数$\bm{\bar{e}}(\bm{\bar{q}}(t))$は以下から構成される．
\begin{eqnarray}
  \bm{\bar{e}}(\bm{\bar{q}})
  =
  \begin{pmatrix}
    \bm{\bar{e}}^{\mathit{kin}}(\bm{\theta}, \bm{\phi}) \\
    \bm{\bar{e}}^{\mathit{eom\mathchar`-trans}}(\bm{c}, \bm{\hat{w}}) \\
    \bm{\bar{e}}^{\mathit{eom\mathchar`-rot}}(\bm{\theta}, \bm{c}, \bm{L}, \bm{\hat{w}}, \bm{\phi}) \\
    \bm{\bar{e}}^{\mathit{cog}}(\bm{\theta}, \bm{c}, \bm{\phi}) \\
    \bm{\bar{e}}^{\mathit{ang\mathchar`-moment}}(\bm{\theta}, \bm{L}, \bm{\phi}) \\
    \bm{\bar{e}}^{\mathit{trq}}(\bm{\theta}, \bm{\hat{w}}, \bm{\tau}, \bm{\phi}) \\
    \bm{\bar{e}}^{\mathit{posture}}(\bm{\theta})
  \end{pmatrix}
\end{eqnarray}
\begin{description}
\setlength{\itemindent}{-6mm}
\item[$\bm{\bar{e}}^{\mathit{kin}}(\bm{\theta}, \bm{\phi}) \in \mathbb{R}^{6 \bar{N}_{\mathit{kin}}(t)}$] 幾何到達拘束 [rad] [m]
  \begin{eqnarray}
    \bm{\bar{e}}^{\mathit{kin}}(\bm{\theta}, \bm{\phi})
    &=& \begin{pmatrix}
      \bm{\bar{e}}_{1}^{\mathit{kin}}(\bm{\theta}, \bm{\phi}) \\ \bm{\bar{e}}_{2}^{\mathit{kin}}(\bm{\theta}, \bm{\phi}) \\ \vdots \\ \bm{\bar{e}}_{\bar{N}_{\mathit{kin}}(t)}^{\mathit{kin}}(\bm{\theta}, \bm{\phi})
    \end{pmatrix} \\
    \bm{\bar{e}}_{m}^{\mathit{kin}}(\bm{\theta}, \bm{\phi}) &=&
    \begin{pmatrix}
      \bm{p}_{m}^{\mathit{kin\mathchar`-trg}}(\bm{\theta}, \bm{\phi}) - \bm{p}_{m}^{\mathit{kin\mathchar`-att}}(\bm{\theta}, \bm{\phi}) \\
      \bm{a}\left(\bm{R}_{m}^{\mathit{kin\mathchar`-trg}}(\bm{\theta}, \bm{\phi}) \bm{R}_{m}^{\mathit{kin\mathchar`-att}}(\bm{\theta}, \bm{\phi})^T\right)
    \end{pmatrix} \in \mathbb{R}^6 \ \ (m = 1,2,\cdots,\bar{N}_{\mathit{kin}}(t))
  \end{eqnarray}
  $\bm{a}(\bm{R})$は姿勢行列$\bm{R}$の等価角軸ベクトルを表す．
\item[$\bm{\bar{e}}^{\mathit{eom\mathchar`-trans}}(\bm{c}, \bm{\hat{w}}) \in \mathbb{R}^3$] 並進運動方程式 $\rm [kg \, m / s^2]$
  \begin{eqnarray}
    \bm{\bar{e}}^{\mathit{eom\mathchar`-trans}}(\bm{c}, \bm{\hat{w}})
    &=& m \bm{\ddot{c}} - \left\{ \sum_{m=1}^{N_{\mathit{cnt}}} \bm{f}_m - m \bm{g} + \sum_{m=1}^{N_{\mathit{ex}}} \bm{f}_{m}^{\mathit{ex}} \right\}
  \end{eqnarray}
\item[$\bm{\bar{e}}^{\mathit{eom\mathchar`-rot}}(\bm{\theta}, \bm{c}, \bm{L}, \bm{\hat{w}}, \bm{\phi}) \in \mathbb{R}^3$] 回転運動方程式 $\rm [kg \, m^2 / s^2]$
  \begin{eqnarray}
    \bm{\bar{e}}^{\mathit{eom\mathchar`-rot}}(\bm{\theta}, \bm{c}, \bm{L}, \bm{\hat{w}}, \bm{\phi})
    &=& \bm{\dot{L}} - \bigg( \sum_{m=1}^{N_{\mathit{cnt}}} \left\{ \left(\bm{p}_m^{\mathit{cnt\mathchar`-trg}}(\bm{\theta}, \bm{\phi}) - \bm{c}\right) \times \bm{f}_m + \bm{n}_m \right\} \nonumber\\
    && \hspace{10mm} + \sum_{m=1}^{N_{\mathit{ex}}} \left\{ \left(\bm{p}_m^{\mathit{ex}}(\bm{\theta}, \bm{\phi}) - \bm{c}\right) \times \bm{f}_m^{\mathit{ex}} + \bm{n}_m^{\mathit{ex}} \right\} \bigg)
  \end{eqnarray}
\item[$\bm{\bar{e}}^{\mathit{cog}}(\bm{\theta}, \bm{c}, \bm{\phi}) \in \mathbb{R}^3$] 重心位置 [m]
  \begin{eqnarray}
    \bm{\bar{e}}^{\mathit{cog}}(\bm{\theta}, \bm{c}, \bm{\phi})
    &=& \bm{p}_G(\bm{\theta}, \bm{\phi}) - \bm{c}
  \end{eqnarray}
\item[$\bm{\bar{e}}^{\mathit{ang\mathchar`-moment}}(\bm{\theta}, \bm{L}, \bm{\phi}) \in \mathbb{R}^3$] 角運動量 $\rm [kg \, m^2/s]$
  \begin{eqnarray}
    \bm{\bar{e}}^{\mathit{ang\mathchar`-moment}}(\bm{\theta}, \bm{L}, \bm{\phi})
    &=& \bm{L} - \left\{ \bm{A}_{\theta}(\bm{\theta}, \bm{\phi}) \bm{\dot{\theta}} + \bm{A}_{\phi}(\bm{\theta}, \bm{\phi}) \bm{\dot{\phi}} \right\}
  \end{eqnarray}
\item[$\bm{\bar{e}}^{\mathit{trq}}(\bm{\theta}, \bm{\hat{w}}, \bm{\tau}, \bm{\phi}) \in \mathbb{R}^{N_{\mathit{drive\mathchar`-joint}}}$] 関節トルクの釣り合い [rad] [m]
  \begin{eqnarray}
    \bm{\bar{e}}^{\mathit{trq}}(\bm{\theta}, \bm{\hat{w}}, \bm{\tau}, \bm{\phi})
    &=& \sum_{m=1}^{N_{\mathit{cnt}}} \left\{ \left(\bm{p}_m^{\mathit{cnt\mathchar`-trg}}(\bm{\theta}, \bm{\phi}) - \bm{c}\right) \times \bm{f}_m + \bm{n}_m \right\} \nonumber\\
    && + \sum_{m=1}^{N_{\mathit{ex}}} \left\{ \left(\bm{p}_m^{\mathit{ex}}(\bm{\theta}, \bm{\phi}) - \bm{c}\right) \times \bm{f}_m^{\mathit{ex}} + \bm{n}_m^{\mathit{ex}} \right\}
  \end{eqnarray}
\item[$\bm{\bar{e}}^{\mathit{posture}}(\bm{\theta}) \in \mathbb{R}^{N_{\mathit{posture\mathchar`-joint}}}$] 関節角目標 [rad] [m]
\begin{eqnarray}
  \bm{\bar{e}}^{\mathit{posture}}(\bm{\theta})
  &=& \left( \bm{\bar{\theta}}^{\mathit{trg}} - \bm{\bar{\theta}} \right)
\end{eqnarray}
\end{description}

瞬時状態$\bm{\bar{q}}(t)$の軌道をBスプラインで表現することで，
設計対称のコンフィギュレーション$\bm{q}$に関するタスク関数$\bm{e}(\bm{q})$は以下から構成される．
\begin{eqnarray}
  \bm{e}(\bm{q})
  =
  \begin{pmatrix}
    \bm{e}^{\mathit{kin}}(\bm{p}_{\theta}, \bm{\phi}) \\
    \bm{e}^{\mathit{eom\mathchar`-trans}}(\bm{p}_{c}, \bm{p}_{\hat{w}}) \\
    \bm{e}^{\mathit{eom\mathchar`-rot}}(\bm{p}_{\theta}, \bm{p}_{c}, \bm{p}_{L}, \bm{p}_{\hat{w}}, \bm{\phi}) \\
    \bm{e}^{\mathit{cog}}(\bm{p}_{\theta}, \bm{p}_{c}, \bm{\phi}) \\
    \bm{e}^{\mathit{ang\mathchar`-moment}}(\bm{p}_{\theta}, \bm{p}_{L}, \bm{\phi}) \\
    \bm{e}^{\mathit{trq}}(\bm{p}_{\theta}, \bm{p}_{\hat{w}}, \bm{p}_{\tau}, \bm{\phi}) \\
    \bm{e}^{\mathit{posture}}(\bm{p}_{\theta})
  \end{pmatrix}
\end{eqnarray}
ただし，
\begin{eqnarray}
  \bm{e}^{\ast}(\bm{q})
  =
  \begin{pmatrix}
    \bm{\bar{e}}^{\ast}(\bm{\bar{q}}(t_1)) \\
    \vdots \\
    \bm{\bar{e}}^{\ast}(\bm{\bar{q}}(t_{N_{\mathit{tm}}}))
  \end{pmatrix}
  \in \mathbb{R}^{N_{\mathit{tm}} \mathit{dim}(\bm{\bar{e}}^{\ast}(\bm{\bar{q}})(t))}
\end{eqnarray}
  "
  )

(defmethod bspline-dynamic-configuration-task
  (:init
   (&key
    (name)
    (robot-env)
    (variant-joint-list (send robot-env :variant-joint-list))
    (invariant-joint-list (send robot-env :invariant-joint-list))
    (drive-joint-list (send robot-env :drive-joint-list))
    (posture-joint-list)
    (kin-task-time-list)
    (eom-task-time-list)
    (centroid-task-time-list)
    (posture-task-time-list)
    (theta-bst)
    (cog-bst)
    (ang-moment-bst)
    (wrench-bst)
    (torque-bst)
    (kin-target-coords-list-func)
    (kin-attention-coords-list-func)
    (contact-target-coords-list-func)
    (contact-attention-coords-list-func)
    (contact-constraint-list-func)
    (posture-joint-angle-list) ;; [deg] [mm]
    (variant-joint-angle-margin 3.0) ;; [deg] [mm]
    (invariant-joint-angle-margin 3.0) ;; [deg] [mm]
    (collision-pair-list)
    (collision-distance-margin 0.01) ;; [m]
    (collision-distance-margin-list) ;; [m]
    (kin-task-scale 1.0)
    (kin-task-scale-mat-list-func)
    (eom-trans-task-scale 1.0)
    (eom-rot-task-scale 1.0)
    (cog-task-scale 1.0)
    (ang-moment-task-scale 1.0)
    (torque-task-scale 1.0)
    (posture-task-scale 1e-3)
    (torque-regular-scale 1e-4)
    (stationery-start-finish-regular-scale 1e-4)
    (first-diff-square-integration-regular-scale 1e-7)
    (second-diff-square-integration-regular-scale 1e-7)
    (third-diff-square-integration-regular-scale 1e-7)
    (norm-regular-scale-max 1e-5)
    (norm-regular-scale-offset 1e-7)
    )
   "
Initialize instance
   "
   (send self :name name)
   (setq _robot-env robot-env)
   (setq _variant-joint-list variant-joint-list)
   (setq _invariant-joint-list invariant-joint-list)
   (setq _drive-joint-list drive-joint-list)
   (setq _posture-joint-list posture-joint-list)
   (setq _kin-task-time-list kin-task-time-list)
   (setq _eom-task-time-list eom-task-time-list)
   (setq _centroid-task-time-list centroid-task-time-list)
   (setq _posture-task-time-list posture-task-time-list)
   (setq _theta-bst theta-bst)
   (setq _cog-bst cog-bst)
   (setq _ang-moment-bst ang-moment-bst)
   (setq _wrench-bst wrench-bst)
   (setq _torque-bst torque-bst)
   (setq _kin-target-coords-list-func kin-target-coords-list-func)
   (setq _kin-attention-coords-list-func kin-attention-coords-list-func)
   (setq _contact-target-coords-list-func contact-target-coords-list-func)
   (setq _contact-attention-coords-list-func contact-attention-coords-list-func)
   (setq _contact-constraint-list-func contact-constraint-list-func)
   (setq _posture-joint-angle-list posture-joint-angle-list)
   (setq _variant-joint-angle-margin variant-joint-angle-margin)
   (setq _invariant-joint-angle-margin invariant-joint-angle-margin)
   (setq _collision-pair-list collision-pair-list)
   (setq _collision-distance-margin-list
         (or collision-distance-margin-list
             (make-list (length _collision-pair-list) :initial-element collision-distance-margin)))
   (setq _kin-task-scale kin-task-scale)
   (setq _kin-task-scale-mat-list-func kin-task-scale-mat-list-func)
   (setq _eom-trans-task-scale eom-trans-task-scale)
   (setq _eom-rot-task-scale eom-rot-task-scale)
   (setq _cog-task-scale cog-task-scale)
   (setq _ang-moment-task-scale ang-moment-task-scale)
   (setq _torque-task-scale torque-task-scale)
   (setq _posture-task-scale posture-task-scale)
   (setq _torque-regular-scale torque-regular-scale)
   (setq _stationery-start-finish-regular-scale stationery-start-finish-regular-scale)
   (setq _first-diff-square-integration-regular-scale first-diff-square-integration-regular-scale)
   (setq _second-diff-square-integration-regular-scale second-diff-square-integration-regular-scale)
   (setq _third-diff-square-integration-regular-scale third-diff-square-integration-regular-scale)
   (setq _norm-regular-scale-max norm-regular-scale-max)
   (setq _norm-regular-scale-offset norm-regular-scale-offset)
   ;; sub methods
   (send self :_initialize-number)
   (send self :_initialize-dimension)
   (send self :_initialize-configuration-vector)
   (send self :_contact-constraint-matrix-list)
   )
  (:_initialize-number
   ()
   (setq _num-kin
         (apply #'+ (mapcar #'(lambda (tm) (length (send self :kin-target-coords-list tm))) _kin-task-time-list)))
   (setq _num-contact (length (send self :contact-target-coords-list 0)))
   (setq _num-variant-joint (length _variant-joint-list))
   (setq _num-invariant-joint (length _invariant-joint-list))
   (setq _num-drive-joint (length _drive-joint-list))
   (setq _num-posture-joint (length _posture-joint-list))
   (setq _num-collision (length _collision-pair-list))
   )
  (:_initialize-dimension
   ()
   (setq _dim-theta-control-vector (send _theta-bst :dim-control-vector))
   (setq _dim-cog-control-vector (send _cog-bst :dim-control-vector))
   (setq _dim-ang-moment-control-vector (send _ang-moment-bst :dim-control-vector))
   (setq _dim-wrench-control-vector (send _wrench-bst :dim-control-vector))
   (setq _dim-torque-control-vector (send _torque-bst :dim-control-vector))
   (setq _dim-phi _num-invariant-joint)
   (setq _dim-config
         (+ _dim-theta-control-vector
            _dim-cog-control-vector
            _dim-ang-moment-control-vector
            _dim-wrench-control-vector
            _dim-torque-control-vector
            _dim-phi
            ))
   (setq _dim-kin-task (* 6 _num-kin))
   (setq _dim-eom-trans-task (* 3 (length _eom-task-time-list)))
   (setq _dim-eom-rot-task (* 3 (length _eom-task-time-list)))
   (setq _dim-cog-task (* 3 (length _centroid-task-time-list)))
   (setq _dim-ang-moment-task (* 3 (length _centroid-task-time-list)))
   (setq _dim-torque-task 0)
   (setq _dim-posture-task (* _num-posture-joint (length _posture-task-time-list)))
   (setq _dim-task
         (+ _dim-kin-task
            _dim-eom-trans-task _dim-eom-rot-task
            _dim-cog-task _dim-ang-moment-task
            _dim-torque-task
            _dim-posture-task
            ))
   )
  (:_initialize-configuration-vector
   ()
   (send _theta-bst :name "theta")
   (send _cog-bst :name "cog")
   (send _ang-moment-bst :name "angular_momentum")
   (send _wrench-bst :name "wrench")
   (send _torque-bst :name "torque")
   (unless (send _theta-bst :get :data-name-list)
     (send _theta-bst :put :data-name-list (send-all _variant-joint-list :name)))
   (unless (send _cog-bst :get :data-name-list)
     (send _cog-bst :put :data-name-list (list "x" "y" "z")))
   (unless (send _ang-moment-bst :get :data-name-list)
     (send _ang-moment-bst :put :data-name-list (list "x" "y" "z")))
   (unless (send _wrench-bst :get :data-name-list)
     (send _wrench-bst :put :data-name-list
           (mapcar
            #'(lambda (contact-name)
                (mapcar
                 #'(lambda (elem) (format nil "contact~a_~a" contact-name elem))
                 (list "fx" "fy" "fz" "nx" "ny" "nz")))
            (range _num-contact)
            )))
   (unless (send _torque-bst :get :data-name-list)
     (send _torque-bst :put :data-name-list (list)))
   (send _theta-bst :set-control-vector-from-instant-config
         (map float-vector #'(lambda (j) (send j :angle-to-speed (send j :joint-angle))) _variant-joint-list))
   (send _cog-bst :set-control-vector-from-instant-config
         (send self :cog-from-model))
   (send _ang-moment-bst :set-control-vector
         (make-array _dim-ang-moment-control-vector :element-type :float))
   (send _wrench-bst :set-control-vector
         (make-array _dim-wrench-control-vector :element-type :float))
   (send _torque-bst :set-control-vector
         (make-array _dim-torque-control-vector :element-type :float))
   (setq _phi-vector
         (map float-vector #'(lambda (j) (send j :angle-to-speed (send j :joint-angle))) _invariant-joint-list))
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
return $\mathit{dim}(\bm{q})$
   "
   _dim-config
   )
  (:dim-task
   ()
   "
return $\mathit{dim}(\bm{e})$
   "
   _dim-task
   )
  (:theta-control-vector
   ()
   "
return $\bm{p}_{\theta}$
   "
   (send _theta-bst :control-vector)
   )
  (:cog-control-vector
   ()
   "
return $\bm{p}_{c}$
   "
   (send _cog-bst :control-vector)
   )
  (:ang-moment-control-vector
   ()
   "
return $\bm{p}_{L}$
   "
   (send _ang-moment-bst :control-vector)
   )
  (:wrench-control-vector
   ()
   "
return $\bm{p}_{\hat{w}}$
   "
   (send _wrench-bst :control-vector)
   )
  (:torque-control-vector
   ()
   "
return $\bm{p}_{\tau}$
   "
   (send _torque-bst :control-vector)
   )
  (:phi
   ()
   "
return $\bm{\phi}$
   "
   _phi-vector
   )
  (:config-vector
   ()
   "
return $\bm{q}$
   "
   (concatenate
    float-vector
    (send self :theta-control-vector)
    (send self :cog-control-vector)
    (send self :ang-moment-control-vector)
    (send self :wrench-control-vector)
    (send self :torque-control-vector)
    (send self :phi)
    )
   )
  (:_set-control-vector
   (control-vector-new
    bst
    &key
    (relative? nil)
    )
   (send bst :set-control-vector
         (if relative? (v+ (send bst :control-vector) control-vector-new) control-vector-new))
   )
  (:set-theta-control-vector
   (control-vector-new
    &key
    (relative? nil)
    )
   "
Set $\bm{p}_{\theta}$.
   "
   (send self :_set-control-vector control-vector-new _theta-bst :relative? relative?)
   )
  (:set-cog-control-vector
   (control-vector-new
    &key
    (relative? nil)
    )
   "
Set $\bm{p}_{c}$.
   "
   (send self :_set-control-vector control-vector-new _cog-bst :relative? relative?)
   )
  (:set-ang-moment-control-vector
   (control-vector-new
    &key
    (relative? nil)
    )
   "
Set $\bm{p}_{L}$.
   "
   (send self :_set-control-vector control-vector-new _ang-moment-bst :relative? relative?)
   )
  (:set-wrench-control-vector
   (control-vector-new
    &key
    (relative? nil)
    )
   "
Set $\bm{p}_{\hat{w}}$.
   "
   (send self :_set-control-vector control-vector-new _wrench-bst :relative? relative?)
   )
  (:set-torque-control-vector
   (control-vector-new
    &key
    (relative? nil)
    )
   "
Set $\bm{p}_{\tau}$.
   "
   (send self :_set-control-vector control-vector-new _torque-bst :relative? relative?)
   )
  (:set-phi
   (phi-new
    &key
    (relative? nil)
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
   )
  (:set-config
   (config-new
    &key
    (relative? nil)
    )
   "
Set $\bm{q}$.
   "
   (let* ((idx 0)
          (next-idx _dim-theta-control-vector)
          )
     (send self :set-theta-control-vector
           (subseq config-new idx next-idx) :relative? relative?)

     (setq idx next-idx)
     (incf next-idx _dim-cog-control-vector)
     (send self :set-cog-control-vector
           (subseq config-new idx next-idx) :relative? relative?)

     (setq idx next-idx)
     (incf next-idx _dim-ang-moment-control-vector)
     (send self :set-ang-moment-control-vector
           (subseq config-new idx next-idx) :relative? relative?)

     (setq idx next-idx)
     (incf next-idx _dim-wrench-control-vector)
     (send self :set-wrench-control-vector
           (subseq config-new idx next-idx) :relative? relative?)

     (setq idx next-idx)
     (incf next-idx _dim-torque-control-vector)
     (send self :set-torque-control-vector
           (subseq config-new idx next-idx) :relative? relative?)

     (setq idx next-idx)
     (incf next-idx _dim-phi)
     (send self :set-phi
           (subseq config-new idx next-idx) :relative? relative?)
     ))
  (:theta
   (tm &key (diff-order 0))
   "
return $\bm{\theta}(t)$ \ [rad] [m]
   "
   (cond ((= diff-order 0)
          (send _theta-bst :instant-config tm)
          )
         ((> diff-order 0)
          (send _theta-bst :instant-config-dot tm :diff-order diff-order)
          ))
   )
  (:cog
   (tm &key (diff-order 0))
   "
return $\bm{c}(t)$ \ [m]
   "
   (cond ((= diff-order 0)
          (send _cog-bst :instant-config tm)
          )
         ((> diff-order 0)
          (send _cog-bst :instant-config-dot tm :diff-order diff-order)
          ))
   )
  (:ang-moment
   (tm &key (diff-order 0))
   "
return $\bm{L}(t)$ \ ${\rm [kg m^2 / s]}$
   "
   (cond ((= diff-order 0)
          (send _ang-moment-bst :instant-config tm)
          )
         ((> diff-order 0)
          (send _ang-moment-bst :instant-config-dot tm :diff-order diff-order)
          ))
   )
  (:wrench
   (tm &key (diff-order 0))
   "
return $\bm{\hat{w}}(t)$ \  [N] [Nm]
   "
   (cond ((= diff-order 0)
          (send _wrench-bst :instant-config tm)
          )
         ((> diff-order 0)
          (send _wrench-bst :instant-config-dot tm :diff-order diff-order)
          ))
   )
  (:torque
   (tm &key (diff-order 0))
   "
return $\bm{\tau}(t)$ \ [Nm] [N]
   "
   (cond ((= diff-order 0)
          (send _torque-bst :instant-config tm)
          )
         ((> diff-order 0)
          (send _torque-bst :instant-config-dot tm :diff-order diff-order)
          ))
   )
  (:_apply-theta-to-robot
   (tm)
   "
apply \bm{\theta}(t) to robot.
   "
   (mapcar
    #'(lambda (j v)
        (send j :joint-angle (send j :speed-to-angle v))) ;; [mm] [deg]
    _variant-joint-list
    (coerce (send self :theta tm) cons))
   )
  (:_apply-phi-to-robot
   ()
   "
apply \bm{\phi} to robot.
   "
   (mapcar
    #'(lambda (j v)
        (send j :joint-angle (send j :speed-to-angle v))) ;; [mm] [deg]
    _invariant-joint-list
    (coerce (send self :phi) cons))
   )
  (:apply-config-to-robot
   (tm)
   "
apply \bm{q}(t) to robot.
   "
   (send self :_apply-theta-to-robot tm)
   (send self :_apply-phi-to-robot)
   )
  )

(defmethod bspline-dynamic-configuration-task
  (:kin-target-coords-list
   (tm)
   "
\begin{eqnarray}
  T_m^{\mathit{kin\mathchar`-trg}} = \{\bm{p}_{m}^{\mathit{kin\mathchar`-trg}}, \bm{R}_{m}^{\mathit{kin\mathchar`-trg}}\} \ \ (m = 1,2,\cdots,N_{\mathit{kin}})
\end{eqnarray}

return $\mathcal{T}^{\mathit{kin\mathchar`-trg}} := \{T_1^{\mathit{kin\mathchar`-trg}}, T_2^{\mathit{kin\mathchar`-trg}}, \cdots, T_{N_\mathit{kin}}^{\mathit{kin\mathchar`-trg}} \}$
   "
   (funcall _kin-target-coords-list-func _robot-env tm)
   )
  (:kin-attention-coords-list
   (tm)
   "
\begin{eqnarray}
  T_m^{\mathit{kin\mathchar`-att}} = \{\bm{p}_{m}^{\mathit{kin\mathchar`-att}}, \bm{R}_{m}^{\mathit{kin\mathchar`-att}}\} \ \ (m = 1,2,\cdots,N_{\mathit{kin}})
\end{eqnarray}

return $\mathcal{T}^{\mathit{kin\mathchar`-att}} := \{T_1^{\mathit{kin\mathchar`-att}}, T_2^{\mathit{kin\mathchar`-att}}, \cdots, T_{N_\mathit{kin}}^{\mathit{kin\mathchar`-att}} \}$
   "
   (funcall _kin-attention-coords-list-func _robot-env tm)
   )
  (:kin-scale-mat-list
   (tm)
   "
return list of $K_{\mathit{kin}}$
   "
   (cond (_kin-task-scale-mat-list-func
          (funcall _kin-task-scale-mat-list-func _robot-env tm)
          )
         (t
          (make-list (length (send self :kin-target-coords-list tm))
                     :initial-element (diagonal (make-list 6 :initial-element _kin-task-scale)))
          ))
   )
  (:contact-target-coords-list
   (tm)
   "
\begin{eqnarray}
  T_m^{\mathit{cnt\mathchar`-trg}} = \{\bm{p}_{m}^{\mathit{cnt\mathchar`-trg}}, \bm{R}_{m}^{\mathit{cnt\mathchar`-trg}}\} \ \ (m = 1,2,\cdots,N_{\mathit{cnt}})
\end{eqnarray}

return $\mathcal{T}^{\mathit{cnt\mathchar`-trg}} := \{T_1^{\mathit{cnt\mathchar`-trg}}, T_2^{\mathit{cnt\mathchar`-trg}}, \cdots, T_{N_\mathit{cnt}}^{\mathit{cnt\mathchar`-trg}} \}$
   "
   (funcall _contact-target-coords-list-func _robot-env tm)
   )
  (:contact-attention-coords-list
   (tm)
   "
\begin{eqnarray}
  T_m^{\mathit{cnt\mathchar`-att}} = \{\bm{p}_{m}^{\mathit{cnt\mathchar`-att}}, \bm{R}_{m}^{\mathit{cnt\mathchar`-att}}\} \ \ (m = 1,2,\cdots,N_{\mathit{cnt}})
\end{eqnarray}

return $\mathcal{T}^{\mathit{cnt\mathchar`-att}} := \{T_1^{\mathit{cnt\mathchar`-att}}, T_2^{\mathit{cnt\mathchar`-att}}, \cdots, T_{N_\mathit{cnt}}^{\mathit{cnt\mathchar`-att}} \}$
   "
   (funcall _contact-attention-coords-list-func _robot-env tm)
   )
  (:contact-constraint-list
   (tm)
   "
return list of contact-constraint
   "
   (funcall _contact-constraint-list-func _robot-env tm)
   )
  (:wrench-list
   (tm)
   "
return $\{ \bm{w}_1, \bm{w}_2, \cdots, \bm{w}_{N_{\mathit{cnt}}} \}$
   "
   (divide-vector (send self :wrench tm) :segment-length 6)
   )
  (:force-list
   (tm)
   "
return $\{ \bm{f}_1, \bm{f}_2, \cdots, \bm{f}_{N_{\mathit{cnt}}} \}$
   "
   (divide-vector (send self :wrench tm) :segment-length 3 :interval-length 3)
   )
  (:moment-list
   (tm)
   "
return $\{ \bm{n}_1, \bm{n}_2, \cdots, \bm{n}_{N_{\mathit{cnt}}} \}$
   "
   (divide-vector (subseq (send self :wrench tm) 3) :segment-length 3 :interval-length 3)
   )
  (:mass
   ()
   "
return $m$ [kg]
   "
   (* 1e-3 (send _robot-env :robot :weight))
   )
  (:mg-vec
   ()
   "
return $m \bm{g} \ {\rm [kg \, m / s^2]}$
   "
   (or (send self :get :mg-vec)
       (send self :put :mg-vec
             (scale (send self :mass) (scale 1e-3 *g-vec*)))) ;; [N]
   )
  (:cog-from-model
   (&key (update? t))
   "
return $\bm{p}_G(\bm{q})$ [m]
   "
   (scale 1e-3 (send _robot-env :robot :centroid update?))
   )
  )

(defmethod bspline-dynamic-configuration-task
  (:kinematics-instant-task-value
   (tm)
   "
\begin{eqnarray}
  \bm{\bar{e}}^{\mathit{kin}}(\bm{\theta}(t), \bm{\phi})
  &=& \begin{pmatrix}
    \bm{\bar{e}}_{1}^{\mathit{kin}}(\bm{\theta}(t), \bm{\phi}) \\ \bm{\bar{e}}_{2}^{\mathit{kin}}(\bm{\theta}(t), \bm{\phi}) \\ \vdots \\ \bm{\bar{e}}_{\bar{N}_{\mathit{kin}}(t)}^{\mathit{kin}}(\bm{\theta}(t), \bm{\phi})
  \end{pmatrix} \in \mathbb{R}^{6 \bar{N}_{\mathit{kin}}(t)} \\
  \bm{\bar{e}}_{m}^{\mathit{kin}}(\bm{\theta}, \bm{\phi}) &=&
  \begin{pmatrix}
    \bm{p}_{m}^{\mathit{kin\mathchar`-trg}}(\bm{\theta}, \bm{\phi}) - \bm{p}_{m}^{\mathit{kin\mathchar`-att}}(\bm{\theta}, \bm{\phi}) \\
    \bm{a}\left(\bm{R}_{m}^{\mathit{kin\mathchar`-trg}}(\bm{\theta}, \bm{\phi}) \bm{R}_{m}^{\mathit{kin\mathchar`-att}}(\bm{\theta}, \bm{\phi})^T\right)
  \end{pmatrix} \in \mathbb{R}^6 \ \ (m = 1,2,\cdots,\bar{N}_{\mathit{kin}}(t))
\end{eqnarray}
$\bm{a}(\bm{R})$は姿勢行列$\bm{R}$の等価角軸ベクトルを表す．

return $\bm{\bar{e}}^{\mathit{kin}}(\bm{\theta}(t), \bm{\phi}) \in \mathbb{R}^{6 \bar{N}_{\mathit{kin}}(t)}$
   "
   (when (> (length (send self :kin-target-coords-list tm)) 0)
     (send self :apply-config-to-robot tm)
     (apply
      #'concatenate
      float-vector
      (mapcar
       #'(lambda (trg-coords att-coords sc-mat)
           (let* ((pos-diff ;; [m]
                   (scale 1e-3 (v- (send trg-coords :worldpos) (send att-coords :worldpos))))
                  (rot-diff ;; [rad]
                   (matrix-log (m* (send trg-coords :worldrot) (transpose (send att-coords :worldrot)))))
                  )
             (cond (sc-mat
                    (transform sc-mat (concatenate float-vector pos-diff rot-diff)))
                   (t
                    (concatenate float-vector pos-diff rot-diff)
                    ))
             ))
       (send self :kin-target-coords-list tm)
       (send self :kin-attention-coords-list tm)
       (send self :kin-scale-mat-list tm)
       ))
     ))
  (:kinematics-task-value
   (&key (update? t))
   "
\begin{eqnarray}
  \bm{e}^{\mathit{kin}}(\bm{p}_{\theta}, \bm{\phi})
  &=& \begin{pmatrix}
    \bm{\bar{e}}^{\mathit{kin}}(\bm{\theta}(t_1), \bm{\phi}) \\ \vdots \\ \bm{\bar{e}}^{\mathit{kin}}(\bm{\theta}(t_{N_{\mathit{tm\mathchar`-kin}}}), \bm{\phi})
  \end{pmatrix}
  \in \mathbb{R}^{6 N_{\mathit{kin}}} \ \ \ \ \left( N_{\mathit{kin}}:=\sum_{t=1}^{t_{N_{\mathit{tm\mathchar`-kin}}}} \bar{N}_{\mathit{kin}}(t) \right)
\end{eqnarray}

return $\bm{e}^{\mathit{kin}}(\bm{p}_{\theta}, \bm{\phi})$
   "
   (when (or update? (not (send self :get :kinematics-task-value)))
     (send self :put :kinematics-task-value
           (apply
            #'concatenate
            float-vector
            (remove
             nil
             (mapcar
              #'(lambda (tm) (send self :kinematics-instant-task-value tm))
              _kin-task-time-list
              ))))
     )
   (send self :get :kinematics-task-value)
   )
  (:eom-trans-instant-task-value
   (tm)
   "
\begin{eqnarray}
  \bm{\bar{e}}^{\mathit{eom\mathchar`-trans}}(\bm{c}(t), \bm{\hat{w}}(t))
  &=& m \bm{\ddot{c}} - \left\{ \sum_{\substack{m=1\\m \in \mathit{contact}}}^{N_{\mathit{cnt}}} \bm{f}_m - m \bm{g} \right\} \\
  &=& m \bm{\ddot{c}} - \sum_{\substack{m=1\\m \in \mathit{contact}}}^{N_{\mathit{cnt}}} \bm{f}_m + m \bm{g} \in \mathbb{R}^{3}
\end{eqnarray}

return $\bm{\bar{e}}^{\mathit{eom\mathchar`-trans}}(\bm{c}(t), \bm{\hat{w}}(t))$
   "
   (scale
    _eom-trans-task-scale
    (apply
     #'v++
     (append
      (list (scale (send self :mass) (send self :cog tm :diff-order 2)))
      (remove
       nil
       (mapcar
        #'(lambda (f c)
            (if c (scale -1 f)))
        (send self :force-list tm)
        (send self :contact-target-coords-list tm)
        ))
      (list (send self :mg-vec))
      )))
   )
  (:eom-trans-task-value
   (&key (update? t))
   "
\begin{eqnarray}
  \bm{e}^{\mathit{eom\mathchar`-trans}}(\bm{p}_{c}, \bm{p}_{\hat{w}})
  &=& \begin{pmatrix}
    \bm{\bar{e}}^{\mathit{eom\mathchar`-trans}}(\bm{c}(t_1), \bm{\hat{w}}(t_1)) \\ \vdots \\ \bm{\bar{e}}^{\mathit{eom\mathchar`-trans}}(\bm{c}(t_{N_{\mathit{tm\mathchar`-eom}}}), \bm{\hat{w}}(t_{N_{\mathit{tm\mathchar`-eom}}}))
  \end{pmatrix}
  \in \mathbb{R}^{3 N_{\mathit{tm\mathchar`-eom}}}
\end{eqnarray}

return $\bm{e}^{\mathit{eom\mathchar`-trans}}(\bm{p}_{c}, \bm{p}_{\hat{w}})$
   "
   (when (or update? (not (send self :get :eom-trans-task-value)))
     (send self :put :eom-trans-task-value
           (apply
            #'concatenate
            float-vector
            (mapcar
             #'(lambda (tm) (send self :eom-trans-instant-task-value tm))
             _eom-task-time-list
             )))
     )
   (send self :get :eom-trans-task-value)
   )
  (:eom-rot-instant-task-value
   (tm)
   "
\begin{eqnarray}
  \bm{\bar{e}}^{\mathit{eom\mathchar`-rot}}(\bm{\theta}(t), \bm{c}(t), \bm{L}(t), \bm{\hat{w}}(t), \bm{\phi})
  &=& \bm{\dot{L}} - \sum_{\substack{m=1\\m \in \mathit{contact}}}^{N_{\mathit{cnt}}} \left\{ \left(\bm{p}_m^{\mathit{cnt\mathchar`-trg}}(\bm{\theta}, \bm{\phi}) - \bm{c}\right) \times \bm{f}_m + \bm{n}_m \right\} \in \mathbb{R}^3
\end{eqnarray}

return $\bm{\bar{e}}^{\mathit{eom\mathchar`-rot}}(\bm{\theta}(t), \bm{c}(t), \bm{L}(t), \bm{\hat{w}}(t), \bm{\phi})$
   "
   (send self :apply-config-to-robot tm)
   (let* ((cog (send self :cog tm))
          )
     (scale
      _eom-rot-task-scale
      (apply
       #'v++
       (append
        (list (send self :ang-moment tm :diff-order 1))
        (remove
         nil
         (mapcar
          #'(lambda (f n c)
              (if c (scale -1 (v+ (v* (v- (scale 1e-3 (send c :worldpos)) cog) f) n))))
          (send self :force-list tm)
          (send self :moment-list tm)
          (send self :contact-target-coords-list tm)
          ))
        )))
     ))
  (:eom-rot-task-value
   (&key (update? t))
   "
\begin{eqnarray}
  \bm{e}^{\mathit{eom\mathchar`-rot}}(\bm{p}_{\theta}, \bm{p}_{c}, \bm{p}_{L}, \bm{p}_{\hat{w}}, \bm{\phi})
  &=& \begin{pmatrix}
    \bm{\bar{e}}^{\mathit{eom\mathchar`-rot}}(\bm{\theta}(t_1), \bm{c}(t_1), \bm{L}(t_1), \bm{\hat{w}}(t_1), \bm{\phi}) \\ \vdots \\ \bm{\bar{e}}^{\mathit{eom\mathchar`-rot}}(\bm{\theta}(t_{N_{\mathit{tm\mathchar`-eom}}}), \bm{c}(t_{N_{\mathit{tm\mathchar`-eom}}}), \bm{L}(t_{N_{\mathit{tm\mathchar`-eom}}}), \bm{\hat{w}}(t_{N_{\mathit{tm\mathchar`-eom}}}), \bm{\phi})
  \end{pmatrix}
  \in \mathbb{R}^{3 N_{\mathit{tm\mathchar`-eom}}}
\end{eqnarray}

return $\bm{e}^{\mathit{eom\mathchar`-rot}}(\bm{p}_{\theta}, \bm{p}_{c}, \bm{p}_{L}, \bm{p}_{\hat{w}}, \bm{\phi})$
   "
   (when (or update? (not (send self :get :eom-rot-task-value)))
     (send self :put :eom-rot-task-value
           (apply
            #'concatenate
            float-vector
            (mapcar
             #'(lambda (tm) (send self :eom-rot-instant-task-value tm))
             _eom-task-time-list
             )))
     )
   (send self :get :eom-rot-task-value)
   )
  (:cog-instant-task-value
   (tm)
   "
\begin{eqnarray}
  \bm{\bar{e}}^{\mathit{cog}}(\bm{\theta}(t), \bm{c}(t), \bm{\phi}) &=& \bm{p}_G(\bm{\theta}, \bm{\phi}) - \bm{c} \in \mathbb{R}^{3}
\end{eqnarray}

return $\bm{\bar{e}}^{\mathit{cog}}(\bm{\theta}(t), \bm{c}(t), \bm{\phi})$
   "
   (send self :apply-config-to-robot tm)
   (scale
    _cog-task-scale
    (v- (send self :cog-from-model) (send self :cog tm)))
   )
  (:cog-task-value
   (&key (update? t))
   "
\begin{eqnarray}
  \bm{e}^{\mathit{cog}}(\bm{p}_{\theta}, \bm{p}_{c}, \bm{\phi})
  &=& \begin{pmatrix}
    \bm{\bar{e}}^{\mathit{cog}}(\bm{\theta}(t_1), \bm{c}(t_1), \bm{\phi}) \\ \vdots \\ \bm{\bar{e}}^{\mathit{cog}}(\bm{\theta}(t_{N_{\mathit{tm\mathchar`-eom}}}), \bm{c}(t_{N_{\mathit{tm\mathchar`-eom}}}), \bm{\phi})
  \end{pmatrix}
  \in \mathbb{R}^{3 N_{\mathit{tm\mathchar`-eom}}}
\end{eqnarray}

return $\bm{e}^{\mathit{cog}}(\bm{p}_{\theta}, \bm{p}_{c}, \bm{\phi})$
   "
   (when (or update? (not (send self :get :cog-task-value)))
     (send self :put :cog-task-value
           (apply
            #'concatenate
            float-vector
            (mapcar
             #'(lambda (tm) (send self :cog-instant-task-value tm))
             _centroid-task-time-list
             )))
     )
   (send self :get :cog-task-value)
   )
  (:ang-moment-instant-task-value
   (tm)
   "
\begin{eqnarray}
  \bm{\bar{e}}^{\mathit{ang\mathchar`-moment}}(\bm{\theta}(t), \bm{L}(t), \bm{\phi})
  &=& \bm{L}(t) - \left\{ \bm{A}_{\theta}(\bm{\theta}(t), \bm{\phi}(t)) \bm{\dot{\theta}}(t) + \bm{A}_{\phi}(\bm{\theta}(t), \bm{\phi}(t)) \bm{\dot{\phi}}(t) \right\} \in \mathbb{R}^3
\end{eqnarray}
本実装では，$\bm{A}_{\theta} = \bm{A}_{\phi} = \bm{O}$という仮定を置く．このとき，タスク関数は次式となる．
\begin{eqnarray}
  \bm{\bar{e}}^{\mathit{ang\mathchar`-moment}}(\bm{L}(t))
  &=& \bm{L}(t) \in \mathbb{R}^3
\end{eqnarray}

return $\bm{\bar{e}}^{\mathit{ang\mathchar`-moment}}(\bm{L}(t))$
   "
   (scale
    _ang-moment-task-scale
    (send self :ang-moment tm))
   )
  (:ang-moment-task-value
   (&key (update? t))
   "
\begin{eqnarray}
  \bm{e}^{\mathit{ang\mathchar`-moment}}(\bm{p}_{L})
  &=& \begin{pmatrix}
    \bm{\bar{e}}^{\mathit{ang\mathchar`-moment}}(\bm{L}(t_1)) \\ \vdots \\ \bm{\bar{e}}^{\mathit{ang\mathchar`-moment}}(\bm{L}(t_{N_{\mathit{tm\mathchar`-eom}}}))
  \end{pmatrix}
  \in \mathbb{R}^{3 N_{\mathit{tm\mathchar`-eom}}}
\end{eqnarray}

return $\bm{e}^{\mathit{ang\mathchar`-moment}}(\bm{p}_{L})$
   "
   (when (or update? (not (send self :get :ang-moment-task-value)))
     (send self :put :ang-moment-task-value
           (apply
            #'concatenate
            float-vector
            (mapcar
             #'(lambda (tm) (send self :ang-moment-instant-task-value tm))
             _centroid-task-time-list
             )))
     )
   (send self :get :ang-moment-task-value)
   )
  (:posture-instant-task-value
   (tm)
   "
\begin{eqnarray}
  \bm{\bar{e}}^{\mathit{posture}}(\bm{\theta}(t))
  &=& k_{\mathit{posture}} \left( \bm{\theta}_{\mathit{posture}}^{\mathit{trg}} - \bm{\theta}_{\mathit{posture}} \right) \in \mathbb{R}^{N_{\mathit{posture\mathchar`-joint}}}
\end{eqnarray}
$\bm{\theta}_{\mathit{posture}}^{\mathit{trg}}, \bm{\theta}_{\mathit{posture}}$は着目関節リスト$\mathcal{J}_{\mathit{posture}}$の目標関節角と現在の関節角．

return $\bm{\bar{e}}^{\mathit{posture}}(\bm{\theta}(t))$
   "
   (send self :apply-config-to-robot tm)
   (map float-vector
        #'(lambda (j a)
            (* _posture-task-scale (send j :angle-to-speed (- a (send j :joint-angle)))))
        _posture-joint-list
        _posture-joint-angle-list)
   )
  (:posture-task-value
   (&key (update? t))
   "
\begin{eqnarray}
  \bm{e}^{\mathit{posture}}(\bm{p}_{\theta})
  &=& \begin{pmatrix}
    \bm{\bar{e}}^{\mathit{posture}}(\bm{\theta}(t_1)) \\ \vdots \\ \bm{\bar{e}}^{\mathit{posture}}(\bm{\theta}(t_{N_{\mathit{tm\mathchar`-kin}}}))
  \end{pmatrix}
  \in \mathbb{R}^{N_{\mathit{posture\mathchar`-joint}} N_{\mathit{tm\mathchar`-kin}}}
\end{eqnarray}

return $\bm{e}^{\mathit{posture}}(\bm{p}_{\theta})$
   "
   (when (or update? (not (send self :get :posture-task-value)))
     (send self :put :posture-task-value
           (apply
            #'concatenate
            float-vector
            (mapcar
             #'(lambda (tm) (send self :posture-instant-task-value tm))
             _posture-task-time-list
             )))
     )
   (send self :get :posture-task-value)
   )
  (:task-value
   (&key (update? t))
   "
\begin{eqnarray}
  \bm{e}(\bm{q}) =
  \begin{pmatrix}
    \bm{e}^{\mathit{kin}}(\bm{p}_{\theta}, \bm{\phi}) \\
    \bm{e}^{\mathit{eom\mathchar`-trans}}(\bm{p}_{c}, \bm{p}_{\hat{w}}) \\
    \bm{e}^{\mathit{eom\mathchar`-rot}}(\bm{p}_{\theta}, \bm{p}_{c}, \bm{p}_{L}, \bm{p}_{\hat{w}}, \bm{\phi}) \\
    \bm{e}^{\mathit{cog}}(\bm{p}_{\theta}, \bm{p}_{c}, \bm{\phi}) \\
    \bm{e}^{\mathit{ang\mathchar`-moment}}(\bm{p}_{\theta}, \bm{p}_{L}, \bm{\phi}) \\
    \bm{e}^{\mathit{trq}}(\bm{p}_{\theta}, \bm{p}_{\hat{w}}, \bm{p}_{\tau}, \bm{\phi}) \\
    \bm{e}^{\mathit{posture}}(\bm{p}_{\theta})
  \end{pmatrix}
\end{eqnarray}

return $\bm{e}(\bm{q}) \in \mathbb{R}^{\mathit{dim}(\bm{e})}$
   "
   (concatenate
    float-vector
    (send self :kinematics-task-value :update? update?)
    (send self :eom-trans-task-value :update? update?)
    (send self :eom-rot-task-value :update? update?)
    (send self :cog-task-value :update? update?)
    (send self :ang-moment-task-value :update? update?)
    (send self :posture-task-value :update? update?)
    ))
  )

(defmethod bspline-dynamic-configuration-task
  (:kinematics-instant-task-jacobian-with-theta-control-vector
   (tm)
   "
\begin{eqnarray}
  \frac{\partial \bm{\bar{e}}^{\mathit{kin}}(\bm{\theta}(t), \bm{\phi})}{\partial \bm{p}_{\theta}}
  &=&
  \frac{\partial \bm{\bar{e}}^{\mathit{kin}}(\bm{\theta}(t), \bm{\phi})}{\partial \bm{\theta}}
  \frac{\partial \bm{\theta}(t)}{\partial \bm{p}_{\theta}} \\
  \frac{\partial \bm{\bar{e}}^{\mathit{kin}}(\bm{\theta}(t), \bm{\phi})}{\partial \bm{\theta}}
  &=&
  \begin{pmatrix}
    \frac{\partial \bm{\bar{e}}_{1}^{\mathit{kin}}(\bm{\theta}(t), \bm{\phi})}{\partial \bm{\theta}} \\ \vdots \\ \frac{\partial \bm{\bar{e}}_{\bar{N}_{\mathit{kin}}(t)}^{\mathit{kin}}(\bm{\theta}(t), \bm{\phi})}{\partial \bm{\theta}}
  \end{pmatrix}
  =
  \begin{pmatrix}
    \bm{J}_{1,\theta}(t) \\ \vdots \\ \bm{J}_{\bar{N}_{\mathit{kin}}(t),\theta}(t)
  \end{pmatrix} \\
  \frac{\partial \bm{\theta}(t)}{\partial \bm{p}_{\theta}}
  &=&
  \frac{\partial}{\partial \bm{p}_{\theta}} \bm{B}_{\theta,n}(t) \bm{p}_{\theta}
  =
  \bm{B}_{\theta,n}(t)
\end{eqnarray}

return $\dfrac{\partial \bm{e}^{\mathit{kin}}}{\partial \bm{p}_{\theta}} \in \mathbb{R}^{6 N_{\mathit{kin}} \times \mathit{dim}(\bm{p}_{\theta})}$
   "
   (when (> (length (send self :kin-target-coords-list tm)) 0)
     (send self :apply-config-to-robot tm)
     (m*
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
                        (send _robot-env :calc-jacobian-from-joint-list
                              :union-joint-list _variant-joint-list
                              :move-target (list att-coords)
                              :transform-coords (list (make-coords))
                              :translation-axis (list t)
                              :rotation-axis (list t)
                              )
                        )
                       )
                      )))
                   )
              (cond (sc-mat (m* sc-mat mat))
                    (t mat))
              ))
        (send self :kin-target-coords-list tm)
        (send self :kin-attention-coords-list tm)
        (send self :kin-scale-mat-list tm)
        ))
      (send _theta-bst :bspline-matrix tm)
      )
     ))
  (:kinematics-task-jacobian-with-theta-control-vector
   ()
   "
\begin{eqnarray}
  \frac{\partial \bm{e}^{\mathit{kin}}}{\partial \bm{p}_{\theta}}
  &=&
  \begin{pmatrix}
    \frac{\partial \bm{\bar{e}}^{\mathit{kin}}(\bm{\theta}(t_1), \bm{\phi}(t_1))}{\partial \bm{p}_{\theta}} \\ \vdots \\ \frac{\partial \bm{\bar{e}}^{\mathit{kin}}(\bm{\theta}(t_{N_{\mathit{tm\mathchar`-kin}}}), \bm{\phi}(t_{N_{\mathit{tm\mathchar`-kin}}}))}{\partial \bm{p}_{\theta}}
  \end{pmatrix}
\end{eqnarray}

return $\dfrac{\partial \bm{e}^{\mathit{kin}}}{\partial \bm{p}_{\theta}} \in \mathbb{R}^{6 N_{\mathit{kin}} \times \mathit{dim}(\bm{p}_{\theta})}$
   "
   (apply
    #'concatenate-matrix-column
    (remove
     nil
     (mapcar
      #'(lambda (tm)
          (send self :kinematics-instant-task-jacobian-with-theta-control-vector tm))
      _kin-task-time-list
      )))
   )
  (:kinematics-instant-task-jacobian-with-phi
   (tm)
   "
\begin{eqnarray}
  \frac{\partial \bm{\bar{e}}^{\mathit{kin}}(\bm{\theta}(t), \bm{\phi})}{\partial \bm{\phi}}
  &=&
  \frac{\partial \bm{\bar{e}}^{\mathit{kin}}(\bm{\theta}(t), \bm{\phi})}{\partial \bm{\phi}} \\
  \frac{\partial \bm{\bar{e}}^{\mathit{kin}}(\bm{\theta}(t), \bm{\phi})}{\partial \bm{\phi}}
  &=&
  \begin{pmatrix}
    \frac{\partial \bm{\bar{e}}_{1}^{\mathit{kin}}(\bm{\theta}(t), \bm{\phi})}{\partial \bm{\phi}} \\ \vdots \\ \frac{\partial \bm{\bar{e}}_{\bar{N}_{\mathit{kin}}(t)}^{\mathit{kin}}(\bm{\theta}(t), \bm{\phi})}{\partial \bm{\phi}}
  \end{pmatrix}
  =
  \begin{pmatrix}
    \bm{J}_{1,\phi}(t) \\ \vdots \\ \bm{J}_{\bar{N}_{\mathit{kin}}(t),\phi}(t)
  \end{pmatrix}
\end{eqnarray}

return $\dfrac{\partial \bm{e}^{\mathit{kin}}}{\partial \bm{\phi}} \in \mathbb{R}^{6 N_{\mathit{kin}} \times \mathit{dim}(\bm{\phi})}$
   "
   (when (> (length (send self :kin-target-coords-list tm)) 0)
     (send self :apply-config-to-robot tm)
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
                       (send _robot-env :calc-jacobian-from-joint-list
                             :union-joint-list _invariant-joint-list
                             :move-target (list att-coords)
                             :transform-coords (list (make-coords))
                             :translation-axis (list t)
                             :rotation-axis (list t)
                             )
                       )
                      )
                     )))
                  )
             (cond (sc-mat (m* sc-mat mat))
                   (t mat))
             ))
       (send self :kin-target-coords-list tm)
       (send self :kin-attention-coords-list tm)
       (send self :kin-scale-mat-list tm)
       ))
     ))
  (:kinematics-task-jacobian-with-phi
   ()
   "
\begin{eqnarray}
  \frac{\partial \bm{e}^{\mathit{kin}}}{\partial \bm{\phi}}
  &=&
  \begin{pmatrix}
    \frac{\partial \bm{\bar{e}}^{\mathit{kin}}(\bm{\theta}(t_1), \bm{\phi}(t_1))}{\partial \bm{\phi}} \\ \vdots \\ \frac{\partial \bm{\bar{e}}^{\mathit{kin}}(\bm{\theta}(t_{N_{\mathit{tm\mathchar`-kin}}}), \bm{\phi}(t_{N_{\mathit{tm\mathchar`-kin}}}))}{\partial \bm{\phi}}
  \end{pmatrix}
\end{eqnarray}

return $\dfrac{\partial \bm{e}^{\mathit{kin}}}{\partial \bm{\phi}} \in \mathbb{R}^{6 N_{\mathit{kin}} \times \mathit{dim}(\bm{\phi})}$
   "
   (apply
    #'concatenate-matrix-column
    (remove
     nil
     (mapcar
      #'(lambda (tm)
          (send self :kinematics-instant-task-jacobian-with-phi tm))
      _kin-task-time-list
      )))
   )
  (:eom-trans-instant-task-jacobian-with-cog-control-vector
   (tm)
   "
\begin{eqnarray}
  \frac{\partial\bm{\bar{e}}^{\mathit{eom\mathchar`-trans}}(\bm{c}(t), \bm{\hat{w}}(t))}{\partial \bm{p}_{c}}
  &=&
  m \frac{\partial \bm{\ddot{c}}(t)}{\partial \bm{p}_{c}} \\
  &=&
  m \frac{\partial}{\partial \bm{p}_{c}} \bm{B}_{c,n-2}(t) \bm{\hat{D}}_2 \bm{p}_{c} \\
  &=&
  m \bm{B}_{c,n-2}(t) \bm{\hat{D}}_2
\end{eqnarray}

return $\dfrac{\partial\bm{\bar{e}}^{\mathit{eom\mathchar`-trans}}(\bm{c}(t), \bm{\hat{w}}(t))}{\partial \bm{p}_{c}} \in \mathbb{R}^{3 \times \mathit{dim}(\bm{p}_{c})}$
   "
   (scale-matrix
    (* _eom-trans-task-scale (send self :mass))
    (m* (send _cog-bst :bspline-matrix tm :order-offset -2)
        (send _cog-bst :extended-differential-matrix :diff-order 2)))
   )
  (:eom-trans-task-jacobian-with-cog-control-vector
   ()
   "
\begin{eqnarray}
  \frac{\partial \bm{e}^{\mathit{eom\mathchar`-trans}}}{\partial \bm{p}_{c}}
  &=&
  \begin{pmatrix}
    \frac{\partial \bm{\bar{e}}^{\mathit{eom\mathchar`-trans}}(\bm{c}(t_1), \bm{\hat{w}}(t_1))}{\partial \bm{p}_{c}} \\ \vdots \\ \frac{\partial \bm{\bar{e}}^{\mathit{eom\mathchar`-trans}}(\bm{c}(t_{N_{\mathit{tm\mathchar`-eom}}}), \bm{\hat{w}}(t_{N_{\mathit{tm\mathchar`-eom}}}))}{\partial \bm{p}_{c}}
  \end{pmatrix}
\end{eqnarray}

return $\dfrac{\partial \bm{e}^{\mathit{eom\mathchar`-trans}}}{\partial \bm{p}_{c}} \in \mathbb{R}^{3 N_{\mathit{tm\mathchar`-eom}} \times \mathit{dim}(\bm{p}_{c})}$
   "
   (apply
    #'concatenate-matrix-column
    (mapcar
     #'(lambda (tm)
         (send self :eom-trans-instant-task-jacobian-with-cog-control-vector tm))
     _eom-task-time-list
     ))
   )
  (:eom-trans-instant-task-jacobian-with-wrench-control-vector
   (tm)
   "
\begin{eqnarray}
  \frac{\partial\bm{\bar{e}}^{\mathit{eom\mathchar`-trans}}(\bm{c}(t), \bm{\hat{w}}(t))}{\partial \bm{p}_{\hat{w}}}
  &=&
  \frac{\partial\bm{\bar{e}}^{\mathit{eom\mathchar`-trans}}(\bm{c}(t), \bm{\hat{w}}(t))}{\partial \bm{\hat{w}}} \frac{\partial \bm{\hat{w}}(t)}{\partial \bm{p}_{\hat{w}}} \\
  \frac{\partial\bm{\bar{e}}^{\mathit{eom\mathchar`-trans}}(\bm{c}(t), \bm{\hat{w}}(t))}{\partial \bm{\hat{w}}}
  &=&
  \begin{pmatrix} - \bm{I}_3 & \bm{O}_3 & \cdots & - \bm{I}_3 & \bm{O}_3  \end{pmatrix} \\
  && \hspace{-30mm} (ただし，\bm{p}_m^{\mathit{cnt\mathchar`-trg}}が\mathit{nil}の接触については，\bm{O}_3とする) \nonumber \\
  \frac{\partial \bm{\hat{w}}(t)}{\partial \bm{p}_{\hat{w}}}
  &=&
  \frac{\partial}{\partial \bm{p}_{\hat{w}}} \bm{B}_{\hat{w},n}(t) \bm{p}_{\hat{w}}
  =
  \bm{B}_{\hat{w},n}(t)
\end{eqnarray}

return $\dfrac{\partial\bm{\bar{e}}^{\mathit{eom\mathchar`-trans}}(\bm{c}(t), \bm{\hat{w}}(t))}{\partial \bm{p}_{\hat{w}}} \in \mathbb{R}^{3 \times \mathit{dim}(\bm{p}_{\hat{w}})}$
   "
   (scale-matrix
    _eom-trans-task-scale
    (m*
     (apply
      #'concatenate-matrix-row
      (flatten
       (mapcar
        #'(lambda (c)
            (list (if c (scale-matrix -1 (unit-matrix 3)) (make-matrix 3 3))
                  (make-matrix 3 3)))
        (send self :contact-target-coords-list tm)
        )))
     (send _wrench-bst :bspline-matrix tm)))
   )
  (:eom-trans-task-jacobian-with-wrench-control-vector
   ()
   "
\begin{eqnarray}
  \frac{\partial \bm{e}^{\mathit{eom\mathchar`-trans}}}{\partial \bm{p}_{\hat{w}}}
  &=&
  \begin{pmatrix}
    \frac{\partial \bm{\bar{e}}^{\mathit{eom\mathchar`-trans}}(\bm{c}(t_1), \bm{\hat{w}}(t_1))}{\partial \bm{p}_{\hat{w}}} \\ \vdots \\ \frac{\partial \bm{\bar{e}}^{\mathit{eom\mathchar`-trans}}(\bm{c}(t_{N_{\mathit{tm\mathchar`-eom}}}), \bm{\hat{w}}(t_{N_{\mathit{tm\mathchar`-eom}}}))}{\partial \bm{p}_{\hat{w}}}
  \end{pmatrix}
\end{eqnarray}

return $\dfrac{\partial \bm{e}^{\mathit{eom\mathchar`-trans}}}{\partial \bm{p}_{\hat{w}}} \in \mathbb{R}^{3 N_{\mathit{tm\mathchar`-eom}} \times \mathit{dim}(\bm{p}_{\hat{w}})}$
   "
   (apply
    #'concatenate-matrix-column
    (mapcar
     #'(lambda (tm)
         (send self :eom-trans-instant-task-jacobian-with-wrench-control-vector tm))
     _eom-task-time-list
     ))
   )
  (:eom-rot-instant-task-jacobian-with-theta-control-vector
   (tm)
   "
\begin{eqnarray}
  \frac{\partial\bm{\bar{e}}^{\mathit{eom\mathchar`-rot}}(\bm{\theta}(t), \bm{c}(t), \bm{L}(t), \bm{\hat{w}}(t), \bm{\phi})}{\partial \bm{p}_{\theta}}
  &=&
  \frac{\partial\bm{\bar{e}}^{\mathit{eom\mathchar`-rot}}(\bm{\theta}(t), \bm{c}(t), \bm{L}(t), \bm{\hat{w}}(t), \bm{\phi})}{\partial \bm{\theta}} \frac{\partial \bm{\theta}(t)}{\partial \bm{p}_{\theta}} \\
  \frac{\partial\bm{\bar{e}}^{\mathit{eom\mathchar`-rot}}(\bm{\theta}(t), \bm{c}(t), \bm{L}(t), \bm{\hat{w}}(t), \bm{\phi})}{\partial \bm{\theta}}
  &=& \sum_{\substack{m=1\\m \in \mathit{contact}}}^{N_{\mathit{cnt}}} \left\{ [ \bm{f}_m(t) \times ] \bm{J}^{\mathit{cnt\mathchar`-trg}}_{m,\theta}(t) \right\} \\
  \frac{\partial \bm{\theta}(t)}{\partial \bm{p}_{\theta}}
  &=&
  \frac{\partial}{\partial \bm{p}_{\theta}} \bm{B}_{\theta,n}(t) \bm{p}_{\theta}
  =
  \bm{B}_{\theta,n}(t)
\end{eqnarray}

return $\dfrac{\partial\bm{\bar{e}}^{\mathit{eom\mathchar`-rot}}(\bm{\theta}(t), \bm{c}(t), \bm{L}(t), \bm{\hat{w}}(t), \bm{\phi})}{\partial \bm{p}_{\theta}} \in \mathbb{R}^{3 \times \mathit{dim}(\bm{p}_{\theta})}$
   "
   (send self :apply-config-to-robot tm)
   (scale-matrix
    _eom-rot-task-scale
    (m*
     (reduce
      #'m+
      (append
       (list (make-matrix 3 (length _variant-joint-list)))
       (remove
        nil
        (mapcar
         #'(lambda (f c)
             (if (derivedp c cascaded-coords)
                 (m*
                  (outer-product-matrix f)
                  (send _robot-env :calc-jacobian-from-joint-list
                        :union-joint-list _variant-joint-list
                        :move-target (list c)
                        :transform-coords (list (make-coords))
                        :translation-axis (list t)
                        :rotation-axis (list nil)
                        ))))
         (send self :force-list tm)
         (send self :contact-target-coords-list tm)
         ))))
     (send _theta-bst :bspline-matrix tm)))
   )
  (:eom-rot-task-jacobian-with-theta-control-vector
   ()
   "
\begin{eqnarray}
  \frac{\partial \bm{e}^{\mathit{eom\mathchar`-rot}}}{\partial \bm{p}_{\theta}}
  &=&
  \begin{pmatrix}
    \frac{\partial \bm{\bar{e}}^{\mathit{eom\mathchar`-rot}}(\bm{\theta}(t_1), \bm{c}(t_1), \bm{L}(t_1), \bm{\hat{w}}(t_1), \bm{\phi})}{\partial \bm{p}_{\theta}} \\ \vdots \\ \frac{\partial \bm{\bar{e}}^{\mathit{eom\mathchar`-rot}}(\bm{\theta}(t_{N_{\mathit{tm\mathchar`-eom}}}), \bm{c}(t_{N_{\mathit{tm\mathchar`-eom}}}), \bm{L}(t_{N_{\mathit{tm\mathchar`-eom}}}), \bm{\hat{w}}(t_{N_{\mathit{tm\mathchar`-eom}}}), \bm{\phi})}{\partial \bm{p}_{\theta}}
  \end{pmatrix}
\end{eqnarray}

return $\dfrac{\partial \bm{e}^{\mathit{eom\mathchar`-rot}}}{\partial \bm{p}_{\theta}} \in \mathbb{R}^{3 N_{\mathit{tm\mathchar`-eom}} \times \mathit{dim}(\bm{p}_{\theta})}$
   "
   (apply
    #'concatenate-matrix-column
    (mapcar
     #'(lambda (tm)
         (send self :eom-rot-instant-task-jacobian-with-theta-control-vector tm))
     _eom-task-time-list
     ))
   )
  (:eom-rot-instant-task-jacobian-with-cog-control-vector
   (tm)
   "
\begin{eqnarray}
  \frac{\partial\bm{\bar{e}}^{\mathit{eom\mathchar`-rot}}(\bm{\theta}(t), \bm{c}(t), \bm{L}(t), \bm{\hat{w}}(t), \bm{\phi})}{\partial \bm{p}_{c}}
  &=&
  \frac{\partial\bm{\bar{e}}^{\mathit{eom\mathchar`-rot}}(\bm{\theta}(t), \bm{c}(t), \bm{L}(t), \bm{\hat{w}}(t), \bm{\phi})}{\partial \bm{c}} \frac{\partial \bm{c}(t)}{\partial \bm{p}_{c}} \\
  \frac{\partial\bm{\bar{e}}^{\mathit{eom\mathchar`-rot}}(\bm{\theta}(t), \bm{c}(t), \bm{L}(t), \bm{\hat{w}}(t), \bm{\phi})}{\partial \bm{c}}
  &=&
  - \sum_{\substack{m=1\\m \in \mathit{contact}}}^{N_{\mathit{cnt}}} \left[ \bm{f}_m \times \right] = \left[ \left( - \sum_{\substack{m=1\\m \in \mathit{contact}}}^{N_{\mathit{cnt}}} \bm{f}_m \right) \times \right] \\
  \frac{\partial \bm{c}(t)}{\partial \bm{p}_{c}}
  &=&
  \frac{\partial}{\partial \bm{p}_{c}} \bm{B}_{c,n}(t) \bm{p}_{c}
  =
  \bm{B}_{c,n}(t)
\end{eqnarray}

return $\dfrac{\partial\bm{\bar{e}}^{\mathit{eom\mathchar`-rot}}(\bm{\theta}(t), \bm{c}(t), \bm{L}(t), \bm{\hat{w}}(t), \bm{\phi})}{\partial \bm{p}_{c}} \in \mathbb{R}^{3 \times \mathit{dim}(\bm{p}_{c})}$
   "
   (scale-matrix
    _eom-rot-task-scale
    (m*
     (outer-product-matrix
      (scale
       -1
       (apply
        #'v++
        (remove
         nil
         (append
          (list (float-vector 0 0 0))
          (mapcar
           #'(lambda (f c)
               (if c f))
           (send self :force-list tm)
           (send self :contact-target-coords-list tm)
           ))))))
     (send _cog-bst :bspline-matrix tm)))
   )
  (:eom-rot-task-jacobian-with-cog-control-vector
   ()
   "
\begin{eqnarray}
  \frac{\partial \bm{e}^{\mathit{eom\mathchar`-rot}}}{\partial \bm{p}_{c}}
  &=&
  \begin{pmatrix}
    \frac{\partial \bm{\bar{e}}^{\mathit{eom\mathchar`-rot}}(\bm{\theta}(t_1), \bm{c}(t_1), \bm{L}(t_1), \bm{\hat{w}}(t_1), \bm{\phi})}{\partial \bm{p}_{c}} \\ \vdots \\ \frac{\partial \bm{\bar{e}}^{\mathit{eom\mathchar`-rot}}(\bm{\theta}(t_{N_{\mathit{tm\mathchar`-eom}}}), \bm{c}(t_{N_{\mathit{tm\mathchar`-eom}}}), \bm{L}(t_{N_{\mathit{tm\mathchar`-eom}}}), \bm{\hat{w}}(t_{N_{\mathit{tm\mathchar`-eom}}}), \bm{\phi})}{\partial \bm{p}_{c}}
  \end{pmatrix}
\end{eqnarray}

return $\dfrac{\partial \bm{e}^{\mathit{eom\mathchar`-rot}}}{\partial \bm{p}_{c}} \in \mathbb{R}^{3 N_{\mathit{tm\mathchar`-eom}} \times \mathit{dim}(\bm{p}_{c})}$
   "
   (apply
    #'concatenate-matrix-column
    (mapcar
     #'(lambda (tm)
         (send self :eom-rot-instant-task-jacobian-with-cog-control-vector tm))
     _eom-task-time-list
     ))
   )
  (:eom-rot-instant-task-jacobian-with-ang-moment-control-vector
   (tm)
   "
\begin{eqnarray}
  \frac{\partial\bm{\bar{e}}^{\mathit{eom\mathchar`-rot}}(\bm{\theta}(t), \bm{c}(t), \bm{L}(t), \bm{\hat{w}}(t), \bm{\phi})}{\partial \bm{p}_{L}}
  &=& \frac{\partial \bm{\dot{L}}(t)}{\partial \bm{p}_{L}} \\
  &=& \frac{\partial}{\partial \bm{p}_{L}} \bm{B}_{L,n-1}(t) \bm{\hat{D}}_1 \bm{p}_{L} \\
  &=& \bm{B}_{L,n-1}(t) \bm{\hat{D}}_1
\end{eqnarray}

return $\dfrac{\partial\bm{\bar{e}}^{\mathit{eom\mathchar`-rot}}(\bm{\theta}(t), \bm{c}(t), \bm{L}(t), \bm{\hat{w}}(t), \bm{\phi})}{\partial \bm{p}_{L}} \in \mathbb{R}^{3 \times \mathit{dim}(\bm{p}_{L})}$
   "
   (scale-matrix
    _eom-rot-task-scale
    (m* (send _ang-moment-bst :bspline-matrix tm :order-offset -1)
        (send _ang-moment-bst :extended-differential-matrix :diff-order 1)))
   )
  (:eom-rot-task-jacobian-with-ang-moment-control-vector
   ()
   "
\begin{eqnarray}
  \frac{\partial \bm{e}^{\mathit{eom\mathchar`-rot}}}{\partial \bm{p}_{L}}
  &=&
  \begin{pmatrix}
    \frac{\partial \bm{\bar{e}}^{\mathit{eom\mathchar`-rot}}(\bm{\theta}(t_1), \bm{c}(t_1), \bm{L}(t_1), \bm{\hat{w}}(t_1), \bm{\phi})}{\partial \bm{p}_{L}} \\ \vdots \\ \frac{\partial \bm{\bar{e}}^{\mathit{eom\mathchar`-rot}}(\bm{\theta}(t_{N_{\mathit{tm\mathchar`-eom}}}), \bm{c}(t_{N_{\mathit{tm\mathchar`-eom}}}), \bm{L}(t_{N_{\mathit{tm\mathchar`-eom}}}), \bm{\hat{w}}(t_{N_{\mathit{tm\mathchar`-eom}}}), \bm{\phi})}{\partial \bm{p}_{L}}
  \end{pmatrix}
\end{eqnarray}

return $\dfrac{\partial \bm{e}^{\mathit{eom\mathchar`-rot}}}{\partial \bm{p}_{L}} \in \mathbb{R}^{3 N_{\mathit{tm\mathchar`-eom}} \times \mathit{dim}(\bm{p}_{L})}$
   "
   (apply
    #'concatenate-matrix-column
    (mapcar
     #'(lambda (tm)
         (send self :eom-rot-instant-task-jacobian-with-ang-moment-control-vector tm))
     _eom-task-time-list
     ))
   )
  (:eom-rot-instant-task-jacobian-with-wrench-control-vector
   (tm)
   "
\begin{eqnarray}
  \frac{\partial\bm{\bar{e}}^{\mathit{eom\mathchar`-rot}}(\bm{\theta}(t), \bm{c}(t), \bm{L}(t), \bm{\hat{w}}(t), \bm{\phi})}{\partial \bm{p}_{\hat{w}}}
  &=&
  \frac{\partial\bm{\bar{e}}^{\mathit{eom\mathchar`-rot}}(\bm{\theta}(t), \bm{c}(t), \bm{L}(t), \bm{\hat{w}}(t), \bm{\phi})}{\partial \bm{\hat{w}}} \frac{\partial \bm{\hat{w}}(t)}{\partial \bm{p}_{\hat{w}}} \\
  \frac{\partial\bm{\bar{e}}^{\mathit{eom\mathchar`-rot}}(\bm{\theta}(t), \bm{c}(t), \bm{L}(t), \bm{\hat{w}}(t), \bm{\phi})}{\partial \bm{\hat{w}}}
  &=&
  \begin{pmatrix}
    \left[ - \left(\bm{p}_1^{\mathit{cnt\mathchar`-trg}}(\bm{\theta}, \bm{\phi}) - \bm{c}\right) \times \right] & - \bm{I}_3 & \cdots &
    \left[ - \left(\bm{p}_{N_{\mathit{cnt}}}^{\mathit{cnt\mathchar`-trg}}(\bm{\theta}, \bm{\phi}) - \bm{c}\right) \times \right] & - \bm{I}_3
  \end{pmatrix} \\
  && \hspace{-30mm} (ただし，\bm{p}_m^{\mathit{cnt\mathchar`-trg}}が\mathit{nil}の接触については，\bm{O}_3とする) \nonumber \\
  \frac{\partial \bm{\hat{w}}(t)}{\partial \bm{p}_{\hat{w}}}
  &=&
  \frac{\partial}{\partial \bm{p}_{\hat{w}}} \bm{B}_{\hat{w},n}(t) \bm{p}_{\hat{w}}
  =
  \bm{B}_{\hat{w},n}(t)
\end{eqnarray}

return $\dfrac{\partial\bm{\bar{e}}^{\mathit{eom\mathchar`-rot}}(\bm{\theta}(t), \bm{c}(t), \bm{L}(t), \bm{\hat{w}}(t), \bm{\phi})}{\partial \bm{p}_{\hat{w}}} \in \mathbb{R}^{3 \times \mathit{dim}(\bm{p}_{\hat{w}})}$
   "
   (send self :apply-config-to-robot tm)
   (let* ((cog (send self :cog tm))
          )
     (scale-matrix
      _eom-rot-task-scale
      (m*
       (apply
        #'concatenate-matrix-row
        (flatten
         (mapcar
          #'(lambda (f n c)
              (cond (c
                     (list (outer-product-matrix
                            (scale -1 (v- (scale 1e-3 (send c :worldpos)) cog)))
                           (scale-matrix -1 (unit-matrix 3)))
                     )
                    (t
                     (make-matrix 3 6)
                     )))
          (send self :force-list tm)
          (send self :moment-list tm)
          (send self :contact-target-coords-list tm)
          )))
       (send _wrench-bst :bspline-matrix tm)))
     ))
  (:eom-rot-task-jacobian-with-wrench-control-vector
   ()
   "
\begin{eqnarray}
  \frac{\partial \bm{e}^{\mathit{eom\mathchar`-rot}}}{\partial \bm{p}_{\hat{w}}}
  &=&
  \begin{pmatrix}
    \frac{\partial \bm{\bar{e}}^{\mathit{eom\mathchar`-rot}}(\bm{\theta}(t_1), \bm{c}(t_1), \bm{L}(t_1), \bm{\hat{w}}(t_1), \bm{\phi})}{\partial \bm{p}_{\hat{w}}} \\ \vdots \\ \frac{\partial \bm{\bar{e}}^{\mathit{eom\mathchar`-rot}}(\bm{\theta}(t_{N_{\mathit{tm\mathchar`-eom}}}), \bm{c}(t_{N_{\mathit{tm\mathchar`-eom}}}), \bm{L}(t_{N_{\mathit{tm\mathchar`-eom}}}), \bm{\hat{w}}(t_{N_{\mathit{tm\mathchar`-eom}}}), \bm{\phi})}{\partial \bm{p}_{\hat{w}}}
  \end{pmatrix}
\end{eqnarray}

return $\dfrac{\partial \bm{e}^{\mathit{eom\mathchar`-rot}}}{\partial \bm{p}_{\hat{w}}} \in \mathbb{R}^{3 N_{\mathit{tm\mathchar`-eom}} \times \mathit{dim}(\bm{p}_{\hat{w}})}$
   "
   (apply
    #'concatenate-matrix-column
    (mapcar
     #'(lambda (tm)
         (send self :eom-rot-instant-task-jacobian-with-wrench-control-vector tm))
     _eom-task-time-list
     ))
   )
  (:eom-rot-instant-task-jacobian-with-phi
   (tm)
   "
\begin{eqnarray}
  \frac{\partial\bm{\bar{e}}^{\mathit{eom\mathchar`-rot}}(\bm{\theta}(t), \bm{c}(t), \bm{L}(t), \bm{\hat{w}}(t), \bm{\phi})}{\partial \bm{\phi}}
  &=&
  \frac{\partial\bm{\bar{e}}^{\mathit{eom\mathchar`-rot}}(\bm{\theta}(t), \bm{c}(t), \bm{L}(t), \bm{\hat{w}}(t), \bm{\phi})}{\partial \bm{\phi}} \\
  &=&
  \sum_{\substack{m=1\\m \in \mathit{contact}}}^{N_{\mathit{cnt}}} \left\{ [ \bm{f}_m(t) \times ] \bm{J}^{\mathit{cnt\mathchar`-trg}}_{m,\phi}(t) \right\}
\end{eqnarray}

return $\dfrac{\partial\bm{\bar{e}}^{\mathit{eom\mathchar`-rot}}(\bm{\theta}(t), \bm{c}(t), \bm{L}(t), \bm{\hat{w}}(t), \bm{\phi})}{\partial \bm{\phi}} \in \mathbb{R}^{3 \times \mathit{dim}(\bm{\phi})}$
   "
   (send self :apply-config-to-robot tm)
   (scale-matrix
    _eom-rot-task-scale
    (reduce
     #'m+
     (append
      (list (make-matrix 3 (length _invariant-joint-list)))
      (remove
       nil
       (mapcar
        #'(lambda (f c)
            (if (derivedp c cascaded-coords)
                (m*
                 (outer-product-matrix f)
                 (send _robot-env :calc-jacobian-from-joint-list
                       :union-joint-list _invariant-joint-list
                       :move-target (list c)
                       :transform-coords (list (make-coords))
                       :translation-axis (list t)
                       :rotation-axis (list nil)
                       ))))
        (send self :force-list tm)
        (send self :contact-target-coords-list tm)
        )))))
   )
  (:eom-rot-task-jacobian-with-phi
   ()
   "
\begin{eqnarray}
  \frac{\partial \bm{e}^{\mathit{eom\mathchar`-rot}}}{\partial \bm{\phi}}
  &=&
  \begin{pmatrix}
    \frac{\partial \bm{\bar{e}}^{\mathit{eom\mathchar`-rot}}(\bm{\theta}(t_1), \bm{c}(t_1), \bm{L}(t_1), \bm{\hat{w}}(t_1), \bm{\phi})}{\partial \bm{\phi}} \\ \vdots \\ \frac{\partial \bm{\bar{e}}^{\mathit{eom\mathchar`-rot}}(\bm{\theta}(t_{N_{\mathit{tm\mathchar`-eom}}}), \bm{c}(t_{N_{\mathit{tm\mathchar`-eom}}}), \bm{L}(t_{N_{\mathit{tm\mathchar`-eom}}}), \bm{\hat{w}}(t_{N_{\mathit{tm\mathchar`-eom}}}), \bm{\phi})}{\partial \bm{\phi}}
  \end{pmatrix}
\end{eqnarray}

return $\dfrac{\partial \bm{e}^{\mathit{eom\mathchar`-rot}}}{\partial \bm{\phi}} \in \mathbb{R}^{3 N_{\mathit{tm\mathchar`-eom}} \times \mathit{dim}(\bm{\phi})}$
   "
   (apply
    #'concatenate-matrix-column
    (mapcar
     #'(lambda (tm)
         (send self :eom-rot-instant-task-jacobian-with-phi tm))
     _eom-task-time-list
     ))
   )
  (:cog-instant-task-jacobian-with-theta-control-vector
   (tm)
   "
\begin{eqnarray}
  \frac{\partial \bm{\bar{e}}^{\mathit{cog}}(\bm{\theta}(t), \bm{c}(t), \bm{\phi})}{\partial \bm{p}_{\theta}}
  &=&
  \frac{\partial \bm{\bar{e}}^{\mathit{cog}}(\bm{\theta}(t), \bm{c}(t), \bm{\phi})}{\partial \bm{\theta}} \frac{\partial \bm{\theta}(t)}{\partial \bm{p}_{\theta}} \\
  \frac{\partial \bm{\bar{e}}^{\mathit{cog}}(\bm{\theta}(t), \bm{c}(t), \bm{\phi})}{\partial \bm{\theta}}
  &=&
  \bm{J}_{G,\theta}(t) \\
  \frac{\partial \bm{\theta}(t)}{\partial \bm{p}_{\theta}}
  &=&
  \frac{\partial}{\partial \bm{p}_{\theta}} \bm{B}_{\theta,n}(t) \bm{p}_{\theta}
  =
  \bm{B}_{\theta,n}(t)
\end{eqnarray}

return $\dfrac{\partial \bm{\bar{e}}^{\mathit{cog}}(\bm{\theta}(t), \bm{c}(t), \bm{\phi})}{\partial \bm{p}_{\theta}} \in \mathbb{R}^{3 \times \mathit{dim}(\bm{p}_{\theta})}$
   "
   (send self :apply-config-to-robot tm)
   (scale-matrix
    _cog-task-scale
    (m*
     (send _robot-env :robot-with-root-virtual :calc-cog-jacobian-from-joint-list
           :union-joint-list _variant-joint-list
           :translation-axis t
           )
     (send _theta-bst :bspline-matrix tm)))
   )
  (:cog-task-jacobian-with-theta-control-vector
   ()
   "
\begin{eqnarray}
  \frac{\partial \bm{e}^{\mathit{cog}}}{\partial \bm{p}_{\theta}}
  &=&
  \begin{pmatrix}
    \frac{\partial \bm{\bar{e}}^{\mathit{cog}}(\bm{\theta}(t_1), \bm{c}(t_1), \bm{\phi})}{\partial \bm{p}_{\theta}} \\ \vdots \\ \frac{\partial \bm{\bar{e}}^{\mathit{cog}}(\bm{\theta}(t_{N_{\mathit{tm\mathchar`-eom}}}), \bm{c}(t_{N_{\mathit{tm\mathchar`-eom}}}), \bm{\phi})}{\partial \bm{p}_{\theta}}
  \end{pmatrix}
\end{eqnarray}

return $\dfrac{\partial \bm{e}^{\mathit{cog}}}{\partial \bm{p}_{\theta}} \in \mathbb{R}^{3 N_{\mathit{tm\mathchar`-eom}} \times \mathit{dim}(\bm{p}_{\theta})}$
   "
   (apply
    #'concatenate-matrix-column
    (mapcar
     #'(lambda (tm)
         (send self :cog-instant-task-jacobian-with-theta-control-vector tm))
     _centroid-task-time-list
     ))
   )
  (:cog-instant-task-jacobian-with-cog-control-vector
   (tm)
   "
\begin{eqnarray}
  \frac{\partial \bm{\bar{e}}^{\mathit{cog}}(\bm{\theta}(t), \bm{c}(t), \bm{\phi})}{\partial \bm{p}_{c}}
  &=&
  \frac{\partial \bm{\bar{e}}^{\mathit{cog}}(\bm{\theta}(t), \bm{c}(t), \bm{\phi})}{\partial \bm{c}} \frac{\partial \bm{c}(t)}{\partial \bm{p}_{c}} \\
  \frac{\partial \bm{\bar{e}}^{\mathit{cog}}(\bm{\theta}(t), \bm{c}(t), \bm{\phi})}{\partial \bm{c}}
  &=&
  - \bm{I}_3 \\
  \frac{\partial \bm{c}(t)}{\partial \bm{p}_{c}}
  &=&
  \frac{\partial}{\partial \bm{p}_{c}} \bm{B}_{c,n}(t) \bm{p}_{c}
  =
  \bm{B}_{c,n}(t)
\end{eqnarray}

return $\dfrac{\partial \bm{\bar{e}}^{\mathit{cog}}(\bm{\theta}(t), \bm{c}(t), \bm{\phi})}{\partial \bm{p}_{c}} \in \mathbb{R}^{3 \times \mathit{dim}(\bm{p}_{c})}$
   "
   (scale-matrix
    (- _cog-task-scale)
    (send _cog-bst :bspline-matrix tm))
   )
  (:cog-task-jacobian-with-cog-control-vector
   ()
   "
\begin{eqnarray}
  \frac{\partial \bm{e}^{\mathit{cog}}}{\partial \bm{p}_{c}}
  &=&
  \begin{pmatrix}
    \frac{\partial \bm{\bar{e}}^{\mathit{cog}}(\bm{\theta}(t_1), \bm{c}(t_1), \bm{\phi})}{\partial \bm{p}_{c}} \\ \vdots \\ \frac{\partial \bm{\bar{e}}^{\mathit{cog}}(\bm{\theta}(t_{N_{\mathit{tm\mathchar`-eom}}}), \bm{c}(t_{N_{\mathit{tm\mathchar`-eom}}}), \bm{\phi})}{\partial \bm{p}_{c}}
  \end{pmatrix}
\end{eqnarray}

return $\dfrac{\partial \bm{e}^{\mathit{cog}}}{\partial \bm{p}_{c}} \in \mathbb{R}^{3 N_{\mathit{tm\mathchar`-eom}} \times \mathit{dim}(\bm{p}_{c})}$
   "
   (apply
    #'concatenate-matrix-column
    (mapcar
     #'(lambda (tm)
         (send self :cog-instant-task-jacobian-with-cog-control-vector tm))
     _centroid-task-time-list
     ))
   )
  (:cog-instant-task-jacobian-with-phi
   (tm)
   "
\begin{eqnarray}
  \frac{\partial \bm{\bar{e}}^{\mathit{cog}}(\bm{\theta}(t), \bm{c}(t), \bm{\phi})}{\partial \bm{\phi}}
  &=&
  \frac{\partial \bm{\bar{e}}^{\mathit{cog}}(\bm{\theta}(t), \bm{c}(t), \bm{\phi})}{\partial \bm{\phi}} = \bm{J}_{G,\phi}(t)
\end{eqnarray}

return $\dfrac{\partial \bm{\bar{e}}^{\mathit{cog}}(\bm{\theta}(t), \bm{c}(t), \bm{\phi})}{\partial \bm{\phi}} \in \mathbb{R}^{3 \times \mathit{dim}(\bm{\phi})}$
   "
   (send self :apply-config-to-robot tm)
   (scale-matrix
    _cog-task-scale
    (send _robot-env :robot-with-root-virtual :calc-cog-jacobian-from-joint-list
          :union-joint-list _invariant-joint-list
          :translation-axis t
          ))
   )
  (:cog-task-jacobian-with-phi
   ()
   "
\begin{eqnarray}
  \frac{\partial \bm{e}^{\mathit{cog}}}{\partial \bm{\phi}}
  &=&
  \begin{pmatrix}
    \frac{\partial \bm{\bar{e}}^{\mathit{cog}}(\bm{\theta}(t_1), \bm{c}(t_1), \bm{\phi})}{\partial \bm{\phi}} \\ \vdots \\ \frac{\partial \bm{\bar{e}}^{\mathit{cog}}(\bm{\theta}(t_{N_{\mathit{tm\mathchar`-eom}}}), \bm{c}(t_{N_{\mathit{tm\mathchar`-eom}}}), \bm{\phi})}{\partial \bm{\phi}}
  \end{pmatrix}
\end{eqnarray}

return $\dfrac{\partial \bm{e}^{\mathit{cog}}}{\partial \bm{\phi}} \in \mathbb{R}^{3 N_{\mathit{tm\mathchar`-eom}} \times \mathit{dim}(\bm{\phi})}$
   "
   (apply
    #'concatenate-matrix-column
    (mapcar
     #'(lambda (tm)
         (send self :cog-instant-task-jacobian-with-phi tm))
     _centroid-task-time-list
     ))
   )
  (:ang-moment-instant-task-jacobian-with-ang-moment-control-vector
   (tm)
   "
\begin{eqnarray}
  \frac{\partial \bm{\bar{e}}^{\mathit{ang\mathchar`-moment}}(\bm{L}(t))}{\partial \bm{p}_{L}}
  &=&
  \frac{\partial \bm{\bar{e}}^{\mathit{ang\mathchar`-moment}}(\bm{L}(t))}{\partial \bm{L}} \frac{\partial \bm{L}(t)}{\partial \bm{p}_{L}} \\
  \frac{\partial \bm{\bar{e}}^{\mathit{ang\mathchar`-moment}}(\bm{L}(t))}{\partial \bm{L}}
  &=& \bm{I}_3 \\
  \frac{\partial \bm{L}(t)}{\partial \bm{p}_{L}}
  &=&
  \frac{\partial}{\partial \bm{p}_{L}} \bm{B}_{L,n}(t) \bm{p}_{L}
  =
  \bm{B}_{L,n}(t)
\end{eqnarray}

return $\dfrac{\partial \bm{\bar{e}}^{\mathit{ang\mathchar`-moment}}(\bm{L}(t))}{\partial \bm{p}_{L}} \in \mathbb{R}^{3 \times \mathit{dim}(\bm{p}_{L})}$
   "
   (scale-matrix
    _ang-moment-task-scale
    (send _ang-moment-bst :bspline-matrix tm))
   )
  (:ang-moment-task-jacobian-with-ang-moment-control-vector
   ()
   "
\begin{eqnarray}
  \frac{\partial \bm{e}^{\mathit{ang\mathchar`-moment}}}{\partial \bm{p}_{L}}
  &=&
  \begin{pmatrix}
    \frac{\partial \bm{\bar{e}}^{\mathit{ang\mathchar`-moment}}(\bm{L}(t_1))}{\partial \bm{p}_{L}} \\ \vdots \\ \frac{\partial \bm{\bar{e}}^{\mathit{ang\mathchar`-moment}}(\bm{L}(t_{N_{\mathit{tm\mathchar`-eom}}}))}{\partial \bm{p}_{L}}
  \end{pmatrix}
\end{eqnarray}

return $\dfrac{\partial \bm{e}^{\mathit{ang\mathchar`-moment}}}{\partial \bm{p}_{L}} \in \mathbb{R}^{3 N_{\mathit{tm\mathchar`-eom}} \times \mathit{dim}(\bm{p}_{L})}$
   "
   (apply
    #'concatenate-matrix-column
    (mapcar
     #'(lambda (tm)
         (send self :ang-moment-instant-task-jacobian-with-ang-moment-control-vector tm))
     _centroid-task-time-list
     ))
   )
  (:posture-instant-task-jacobian-with-theta-control-vector
   (tm)
   "
\begin{eqnarray}
  \frac{\partial \bm{\bar{e}}^{\mathit{posture}}(\bm{\theta}(t))}{\partial \bm{p}_{\theta}}
  &=&
  \frac{\partial \bm{\bar{e}}^{\mathit{posture}}(\bm{\theta}(t))}{\partial \bm{\theta}} \frac{\partial \bm{\theta}(t)}{\partial \bm{p}_{\theta}} \\
  \left( \frac{\partial \bm{\bar{e}}^{\mathit{posture}}(\bm{\theta}(t))}{\partial \bm{\theta}} \right)_{i,j} &=&
  \left\{
  \begin{array}{ll}
  - k_{\mathit{posture}} & (\mathcal{J}_{\mathit{posture,i}} = \mathcal{J}_{\mathit{var,j}}) \\
  0 & {\rm otherwise}
  \end{array}
  \right.\\
  \frac{\partial \bm{\theta}(t)}{\partial \bm{p}_{\theta}}
  &=&
  \frac{\partial}{\partial \bm{p}_{\theta}} \bm{B}_{\theta,n}(t) \bm{p}_{\theta}
  =
  \bm{B}_{\theta,n}(t)
\end{eqnarray}

return $\dfrac{\partial \bm{\bar{e}}^{\mathit{posture}}(\bm{\theta}(t))}{\partial \bm{p}_{\theta}} \in \mathbb{R}^{N_{\mathit{posture\mathchar`-joint}} \times \mathit{dim}(\bm{p}_{\theta})}$
   "
   (send self :apply-config-to-robot tm)
   (let* ((mat
           (make-matrix _num-posture-joint _num-variant-joint))
          )
     (dotimes (i (length _posture-joint-list))
       (setf
        (aref mat
              i (position (elt _posture-joint-list i) _variant-joint-list))
        (- _posture-task-scale)))
     (m* mat (send _theta-bst :bspline-matrix tm))
     ))
  (:posture-task-jacobian-with-theta-control-vector
   ()
   "
\begin{eqnarray}
  \frac{\partial \bm{e}^{\mathit{posture}}}{\partial \bm{p}_{\theta}}
  &=&
  \begin{pmatrix}
    \frac{\partial \bm{\bar{e}}^{\mathit{posture}}(\bm{\theta}(t_1))}{\partial \bm{p}_{\theta}} \\ \vdots \\ \frac{\partial \bm{\bar{e}}^{\mathit{posture}}(\bm{\theta}(t_{N_{\mathit{tm\mathchar`-kin}}}))}{\partial \bm{p}_{\theta}}
  \end{pmatrix}
\end{eqnarray}

return $\dfrac{\partial \bm{e}^{\mathit{posture}}}{\partial \bm{p}_{\theta}} \in \mathbb{R}^{N_{\mathit{posture\mathchar`-joint}} N_{\mathit{tm\mathchar`-kin}} \times \mathit{dim}(\bm{p}_{\theta})}$
   "
   (apply
    #'concatenate-matrix-column
    (mapcar
     #'(lambda (tm)
         (send self :posture-instant-task-jacobian-with-theta-control-vector tm))
     _posture-task-time-list
     ))
   )
  (:task-jacobian
   ()
   "
\begin{eqnarray}
  \frac{\partial \bm{e}}{\partial \bm{q}}
  &=&
  \bordermatrix{
    & \mathit{dim}(\bm{p}_{\theta}) & \mathit{dim}(\bm{p}_{c}) & \mathit{dim}(\bm{p}_{L}) & \mathit{dim}(\bm{p}_{\hat{w}}) & \mathit{dim}(\bm{p}_{\tau}) & \mathit{dim}(\bm{\phi}) \cr
    \mathit{dim}(\bm{e}^{\mathit{kin}}(\bm{p}_{\theta}, \bm{\phi})) & \frac{\partial \bm{e}^{\mathit{kin}}}{\partial \bm{p}_{\theta}} &&&&& \frac{\partial \bm{e}^{\mathit{kin}}}{\partial \bm{\phi}} \cr
    \mathit{dim}(\bm{e}^{\mathit{eom\mathchar`-trans}}(\bm{p}_{c}, \bm{p}_{\hat{w}})) && \frac{\partial \bm{e}^{\mathit{eom\mathchar`-trans}}}{\partial \bm{p}_c} && \frac{\partial \bm{e}^{\mathit{eom\mathchar`-trans}}}{\partial \bm{p}_{\hat{w}}} \cr
    \mathit{dim}(\bm{e}^{\mathit{eom\mathchar`-rot}}(\bm{p}_{\theta}, \bm{p}_{c}, \bm{p}_{L}, \bm{p}_{\hat{w}}, \bm{\phi})) & \frac{\partial \bm{e}^{\mathit{eom\mathchar`-rot}}}{\partial \bm{p}_{\theta}} & \frac{\partial \bm{e}^{\mathit{eom\mathchar`-rot}}}{\partial \bm{p}_{c}} & \frac{\partial \bm{e}^{\mathit{eom\mathchar`-rot}}}{\partial \bm{p}_{L}} & \frac{\partial \bm{e}^{\mathit{eom\mathchar`-rot}}}{\partial \bm{p}_{\hat{w}}} && \frac{\partial \bm{e}^{\mathit{eom\mathchar`-rot}}}{\partial \bm{\phi}} \cr
    \mathit{dim}(\bm{e}^{\mathit{cog}}(\bm{p}_{\theta}, \bm{p}_{c}, \bm{\phi})) & \frac{\partial \bm{e}^{\mathit{cog}}}{\partial \bm{p}_{\theta}} & \frac{\partial \bm{e}^{\mathit{cog}}}{\partial \bm{p}_c} &&&& \frac{\partial \bm{e}^{\mathit{cog}}}{\partial \bm{\phi}}\cr
    \mathit{dim}(\bm{e}^{\mathit{ang\mathchar`-moment}}(\bm{p}_{\theta}, \bm{p}_{L}, \bm{\phi})) & && \frac{\partial \bm{e}^{\mathit{ang\mathchar`-moment}}}{\partial \bm{p}_L}\cr
    \mathit{dim}(\bm{e}^{\mathit{trq}}(\bm{p}_{\theta}, \bm{p}_{\hat{w}}, \bm{p}_{\tau}, \bm{\phi})) & \cr
    \mathit{dim}(\bm{e}^{\mathit{posture}}(\bm{p}_{\theta})) & \frac{\partial \bm{e}^{\mathit{posture}}}{\partial \bm{p}_{\theta}} \cr
  }
\end{eqnarray}

return $\frac{\partial \bm{e}}{\partial \bm{q}} = \mathbb{R}^{\mathit{dim}(\bm{e}) \times \mathit{dim}(\bm{q})}$
   "
   (cond (_task-jacobi
          (fill (array-entity _task-jacobi) 0)
          )
         (t
          (setq _task-jacobi (make-matrix _dim-task _dim-config))
          ))
   ;; kin-task
   (copy-block-matrix _task-jacobi
                      (send self :kinematics-task-jacobian-with-theta-control-vector)
                      0
                      0)
   (when (> _num-invariant-joint 0)
     (copy-block-matrix _task-jacobi
                        (send self :kinematics-task-jacobian-with-phi)
                        0
                        (+ _dim-theta-control-vector _dim-cog-control-vector _dim-ang-moment-control-vector _dim-wrench-control-vector _dim-torque-control-vector))
     )
   ;; eom-trans-task
   (copy-block-matrix _task-jacobi
                      (send self :eom-trans-task-jacobian-with-cog-control-vector)
                      _dim-kin-task
                      _dim-theta-control-vector)
   (copy-block-matrix _task-jacobi
                      (send self :eom-trans-task-jacobian-with-wrench-control-vector)
                      _dim-kin-task
                      (+ _dim-theta-control-vector _dim-cog-control-vector _dim-ang-moment-control-vector))
   ;; eom-rot-task
   (copy-block-matrix _task-jacobi
                      (send self :eom-rot-task-jacobian-with-theta-control-vector)
                      (+ _dim-kin-task _dim-eom-trans-task)
                      0)
   (copy-block-matrix _task-jacobi
                      (send self :eom-rot-task-jacobian-with-cog-control-vector)
                      (+ _dim-kin-task _dim-eom-trans-task)
                      _dim-theta-control-vector)
   (copy-block-matrix _task-jacobi
                      (send self :eom-rot-task-jacobian-with-ang-moment-control-vector)
                      (+ _dim-kin-task _dim-eom-trans-task)
                      (+ _dim-theta-control-vector _dim-cog-control-vector))
   (copy-block-matrix _task-jacobi
                      (send self :eom-rot-task-jacobian-with-wrench-control-vector)
                      (+ _dim-kin-task _dim-eom-trans-task)
                      (+ _dim-theta-control-vector _dim-cog-control-vector _dim-ang-moment-control-vector))
   (when (> _num-invariant-joint 0)
     (copy-block-matrix _task-jacobi
                        (send self :eom-rot-task-jacobian-with-phi)
                        (+ _dim-kin-task _dim-eom-trans-task)
                        (+ _dim-theta-control-vector _dim-cog-control-vector _dim-ang-moment-control-vector _dim-wrench-control-vector _dim-torque-control-vector))
     )
   ;; cog-task
   (copy-block-matrix _task-jacobi
                      (send self :cog-task-jacobian-with-theta-control-vector)
                      (+ _dim-kin-task _dim-eom-trans-task _dim-eom-rot-task)
                      0)
   (copy-block-matrix _task-jacobi
                      (send self :cog-task-jacobian-with-cog-control-vector)
                      (+ _dim-kin-task _dim-eom-trans-task _dim-eom-rot-task)
                      _dim-theta-control-vector)
   (when (> _num-invariant-joint 0)
     (copy-block-matrix _task-jacobi
                        (send self :cog-task-jacobian-with-phi)
                        (+ _dim-kin-task _dim-eom-trans-task _dim-eom-rot-task)
                        (+ _dim-theta-control-vector _dim-cog-control-vector _dim-ang-moment-control-vector _dim-wrench-control-vector _dim-torque-control-vector))
     )
   ;; ang-moment
   (copy-block-matrix _task-jacobi
                      (send self :ang-moment-task-jacobian-with-ang-moment-control-vector)
                      (+ _dim-kin-task _dim-eom-trans-task _dim-eom-rot-task _dim-cog-task)
                      (+ _dim-theta-control-vector _dim-cog-control-vector))
   ;; posture
   (copy-block-matrix _task-jacobi
                      (send self :posture-task-jacobian-with-theta-control-vector)
                      (+ _dim-kin-task _dim-eom-trans-task _dim-eom-rot-task _dim-cog-task _dim-ang-moment-task _dim-torque-task)
                      0)
   _task-jacobi
   )
  )

(defmethod bspline-dynamic-configuration-task
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
  (:theta-instant-inequality-constraint-matrix
   (&key (update? nil))
   "
\begin{eqnarray}
&& \bm{\theta}_{\mathit{min}} \leq \bm{\theta} \leq \bm{\theta}_{\mathit{max}} \\
\Leftrightarrow&&\begin{pmatrix} \bm{I} \\ - \bm{I} \end{pmatrix} \bm{\theta}
\geq \begin{pmatrix} \bm{\theta}_{\mathit{min}} \\ - \bm{\theta}_{\mathit{max}} \end{pmatrix} \\
\Leftrightarrow&&\bm{C_{\theta}} \bm{\theta} \geq \bm{\bar{d}_{\theta}}
\end{eqnarray}

return $\bm{C_{\theta}} := \begin{pmatrix} \bm{I} \\ - \bm{I}\end{pmatrix} \in \mathbb{R}^{2 N_{\mathit{var\mathchar`-joint}} \times N_{\mathit{var\mathchar`-joint}}}$
   "
   (when (or update? (not (send self :get :theta-instant-inequality-constraint-matrix)))
     (send self :put :theta-instant-inequality-constraint-matrix
           (concatenate-matrix-column
            (unit-matrix _num-variant-joint)
            (scale-matrix -1 (unit-matrix _num-variant-joint))
            ))
     )
   (send self :get :theta-instant-inequality-constraint-matrix)
   )
  (:theta-instant-inequality-constraint-vector
   (&key (update? nil))
   "
return $\bm{\bar{d}_{\theta}} := \begin{pmatrix} \bm{\theta}_{\mathit{min}} \\ - \bm{\theta}_{\mathit{max}} \end{pmatrix} \in \mathbb{R}^{2 N_{\mathit{var\mathchar`-joint}}}$
   "
   (when (or update? (not (send self :get :theta-instant-inequality-constraint-vector)))
     (send self :put :theta-instant-inequality-constraint-vector
           (concatenate
            float-vector
            (send self :theta-min-vector)
            (scale -1 (send self :theta-max-vector))
            ))
     )
   (send self :get :theta-instant-inequality-constraint-vector)
   )
  (:theta-control-vector-inequality-constraint-matrix
   (&key (update? nil))
   "
\begin{eqnarray}
  &&\bm{C_{\theta}} \bm{\theta} \geq \bm{\bar{d}_{\theta}} \\
  \Leftrightarrow&&\bm{C_{p_{\theta}}} \bm{p}_{\theta} \geq \bm{\bar{d}_{p_{\theta}}}
\end{eqnarray}
差分形式で表すと次式となる．
\begin{eqnarray}
  &&\bm{C_{p_{\theta}}} (\bm{p}_{\theta} + \Delta \bm{p}_{\theta}) \geq \bm{\bar{d}_{p_{\theta}}} \\
  \Leftrightarrow&&\bm{C_{p_{\theta}}} \Delta \bm{p}_{\theta} \geq \bm{\bar{d}_{p_{\theta}}} - \bm{C_{p_{\theta}}} \bm{p}_{\theta} \\
  \Leftrightarrow&&\bm{C_{p_{\theta}}} \Delta \bm{p}_{\theta} \geq \bm{d_{p_{\theta}}}
\end{eqnarray}

return $\bm{C}_{p_{\theta}}$
   "
   (when (or update? (not (send self :get :theta-control-vector-inequality-constraint-matrix)))
     (send self :put :theta-control-vector-inequality-constraint-matrix
           (send _theta-bst :convert-instant-inequality-constraint-matrix-for-control-vector
                 :instant-ineq-matrix
                 (send self :theta-instant-inequality-constraint-matrix)
                 ))
     )
   (send self :get :theta-control-vector-inequality-constraint-matrix)
   )
  (:theta-control-vector-inequality-constraint-vector
   (&key (update? t))
   "
return $\bm{d}_{p_{\theta}} := \bm{\bar{d}_{p_{\theta}}} - \bm{C_{p_{\theta}}} \bm{p}_{\theta}$
   "
   (when (or update? (not (send self :get :theta-control-vector-inequality-constraint-vector)))
     (send self :put :theta-control-vector-inequality-constraint-vector
           (v- (send _theta-bst :convert-instant-inequality-constraint-vector-for-control-vector
                     :instant-ineq-vector (send self :theta-instant-inequality-constraint-vector))
               (transform (send self :theta-control-vector-inequality-constraint-matrix)
                          (send _theta-bst :control-vector))
               ))
     )
   (send self :get :theta-control-vector-inequality-constraint-vector)
   )
  (:cog-max-vector
   (&key (update? nil))
   "
return $\bm{c}_{\mathit{max}} \in \mathbb{R}^{3}$ [m]
   "
   (when (or update? (not (send self :get :cog-max-vector)))
     (send self :put :cog-max-vector
           (or (send _robot-env :get :max-cog) (float-vector 10 10 10)))
     )
   (send self :get :cog-max-vector)
   )
  (:cog-instant-inequality-constraint-matrix
   (&key (update? nil))
   "
\begin{eqnarray}
&& - \bm{c}_{\mathit{max}} \leq \bm{c} \leq \bm{c}_{\mathit{max}} \\
\Leftrightarrow&&\begin{pmatrix} \bm{I} \\ - \bm{I}\end{pmatrix} \bm{c}
\geq \begin{pmatrix} - \bm{c}_{\mathit{max}} \\ - \bm{c}_{\mathit{max}} \end{pmatrix} \\
\Leftrightarrow&&\bm{C_{c}} \bm{c} \geq \bm{\bar{d}_{c}}
\end{eqnarray}

return $\bm{C_{c}} := \begin{pmatrix} \bm{I} \\ - \bm{I} \end{pmatrix} \in \mathbb{R}^{6 \times 3}$
   "
   (when (or update? (not (send self :get :cog-instant-inequality-constraint-matrix)))
     (send self :put :cog-instant-inequality-constraint-matrix
           (concatenate-matrix-column
            (unit-matrix 3)
            (scale-matrix -1 (unit-matrix 3))
            ))
     )
   (send self :get :cog-instant-inequality-constraint-matrix)
   )
  (:cog-instant-inequality-constraint-vector
   (&key (update? nil))
   "
return $\bm{\bar{d}_{c}} := \begin{pmatrix} - \bm{c}_{\mathit{max}} \\ - \bm{c}_{\mathit{max}} \end{pmatrix} \in \mathbb{R}^{6}$
   "
   (when (or update? (not (send self :get :cog-instant-inequality-constraint-vector)))
     (send self :put :cog-instant-inequality-constraint-vector
           (concatenate
            float-vector
            (scale -1 (send self :cog-max-vector))
            (scale -1 (send self :cog-max-vector))
            ))
     )
   (send self :get :cog-instant-inequality-constraint-vector)
   )
  (:cog-control-vector-inequality-constraint-matrix
   (&key (update? nil))
   "
\begin{eqnarray}
  &&\bm{C_{c}} \bm{c} \geq \bm{\bar{d}_{c}} \\
  \Leftrightarrow&&\bm{C_{p_{c}}} \bm{p}_{c} \geq \bm{\bar{d}_{p_{c}}}
\end{eqnarray}
差分形式で表すと次式となる．
\begin{eqnarray}
  &&\bm{C_{p_{c}}} (\bm{p}_{c} + \Delta \bm{p}_{c}) \geq \bm{\bar{d}_{p_{c}}} \\
  \Leftrightarrow&&\bm{C_{p_{c}}} \Delta \bm{p}_{c} \geq \bm{\bar{d}_{p_{c}}} - \bm{C_{p_{c}}} \bm{p}_{c} \\
  \Leftrightarrow&&\bm{C_{p_{c}}} \Delta \bm{p}_{c} \geq \bm{d_{p_{c}}}
\end{eqnarray}

return $\bm{C}_{p_{c}}$
   "
   (when (or update? (not (send self :get :cog-control-vector-inequality-constraint-matrix)))
     (send self :put :cog-control-vector-inequality-constraint-matrix
           (send _cog-bst :convert-instant-inequality-constraint-matrix-for-control-vector
                 :instant-ineq-matrix
                 (send self :cog-instant-inequality-constraint-matrix)
                 ))
     )
   (send self :get :cog-control-vector-inequality-constraint-matrix)
   )
  (:cog-control-vector-inequality-constraint-vector
   (&key (update? t))
   "
return $\bm{d}_{p_{c}} := \bm{\bar{d}_{p_{c}}} - \bm{C_{p_{c}}} \bm{p}_{c}$
   "
   (when (or update? (not (send self :get :cog-control-vector-inequality-constraint-vector)))
     (send self :put :cog-control-vector-inequality-constraint-vector
           (v- (send _cog-bst :convert-instant-inequality-constraint-vector-for-control-vector
                     :instant-ineq-vector
                     (send self :cog-instant-inequality-constraint-vector))
               (transform (send self :cog-control-vector-inequality-constraint-matrix)
                          (send _cog-bst :control-vector))
               ))
     )
   (send self :get :cog-control-vector-inequality-constraint-vector)
   )
  (:ang-moment-max-vector
   (&key (update? nil))
   "
return $\bm{L}_{\mathit{max}} \in \mathbb{R}^{3} \ {\rm [kg m^2 / s]}$
   "
   (when (or update? (not (send self :get :ang-moment-max-vector)))
     (send self :put :ang-moment-max-vector
           (or (send _robot-env :get :max-angular-moment) (float-vector 0 0 0)))
     )
   (send self :get :ang-moment-max-vector)
   )
  (:ang-moment-instant-inequality-constraint-matrix
   (&key (update? nil))
   "
\begin{eqnarray}
&& - \bm{L}_{\mathit{max}} \leq \bm{L} \leq \bm{L}_{\mathit{max}} \\
\Leftrightarrow&&\begin{pmatrix} \bm{I} \\ - \bm{I}\end{pmatrix} \bm{L}
\geq \begin{pmatrix} - \bm{L}_{\mathit{max}} \\ - \bm{L}_{\mathit{max}} \end{pmatrix} \\
\Leftrightarrow&&\bm{C_{L}} \bm{L} \geq \bm{\bar{d}_{L}}
\end{eqnarray}

return $\bm{C_{L}} := \begin{pmatrix} \bm{I} \\ - \bm{I} \end{pmatrix} \in \mathbb{R}^{6 \times 3}$
   "
   (when (or update? (not (send self :get :ang-moment-instant-inequality-constraint-matrix)))
     (send self :put :ang-moment-instant-inequality-constraint-matrix
           (concatenate-matrix-column
            (unit-matrix 3)
            (scale-matrix -1 (unit-matrix 3))
            ))
     )
   (send self :get :ang-moment-instant-inequality-constraint-matrix)
   )
  (:ang-moment-instant-inequality-constraint-vector
   (&key (update? nil))
   "
return $\bm{\bar{d}_{L}} := \begin{pmatrix} - \bm{L}_{\mathit{max}} \\ - \bm{L}_{\mathit{max}} \end{pmatrix} \in \mathbb{R}^{6}$
   "
   (when (or update? (not (send self :get :ang-moment-instant-inequality-constraint-vector)))
     (send self :put :ang-moment-instant-inequality-constraint-vector
           (concatenate
            float-vector
            (scale -1 (send self :ang-moment-max-vector))
            (scale -1 (send self :ang-moment-max-vector))
            ))
     )
   (send self :get :ang-moment-instant-inequality-constraint-vector)
   )
  (:ang-moment-control-vector-inequality-constraint-matrix
   (&key (update? nil))
   "
\begin{eqnarray}
  &&\bm{C_{L}} \bm{L} \geq \bm{\bar{d}_{L}} \\
  \Leftrightarrow&&\bm{C_{p_{L}}} \bm{p}_{L} \geq \bm{\bar{d}_{p_{L}}}
\end{eqnarray}
差分形式で表すと次式となる．
\begin{eqnarray}
  &&\bm{C_{p_{L}}} (\bm{p}_{L} + \Delta \bm{p}_{L}) \geq \bm{\bar{d}_{p_{L}}} \\
  \Leftrightarrow&&\bm{C_{p_{L}}} \Delta \bm{p}_{L} \geq \bm{\bar{d}_{p_{L}}} - \bm{C_{p_{L}}} \bm{p}_{L} \\
  \Leftrightarrow&&\bm{C_{p_{L}}} \Delta \bm{p}_{L} \geq \bm{d_{p_{L}}}
\end{eqnarray}

return $\bm{C}_{p_{L}}$
   "
   (when (or update? (not (send self :get :ang-moment-control-vector-inequality-constraint-matrix)))
     (send self :put :ang-moment-control-vector-inequality-constraint-matrix
           (send _ang-moment-bst :convert-instant-inequality-constraint-matrix-for-control-vector
                 :instant-ineq-matrix
                 (send self :ang-moment-instant-inequality-constraint-matrix)
                 ))
     )
   (send self :get :ang-moment-control-vector-inequality-constraint-matrix)
   )
  (:ang-moment-control-vector-inequality-constraint-vector
   (&key (update? t))
   "
return $\bm{d}_{p_{L}} := \bm{\bar{d}_{p_{L}}} - \bm{C_{p_{L}}} \bm{p}_{L}$
   "
   (when (or update? (not (send self :get :ang-moment-control-vector-inequality-constraint-vector)))
     (send self :put :ang-moment-control-vector-inequality-constraint-vector
           (v- (send _ang-moment-bst :convert-instant-inequality-constraint-vector-for-control-vector
                     :instant-ineq-vector
                     (send self :ang-moment-instant-inequality-constraint-vector))
               (transform (send self :ang-moment-control-vector-inequality-constraint-matrix)
                          (send _ang-moment-bst :control-vector))
               ))
     )
   (send self :get :ang-moment-control-vector-inequality-constraint-vector)
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
            (send self :contact-constraint-list 0) ;; this should be set correctly
            (send self :contact-target-coords-list 0) ;; this should be set correctly
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
            (send self :contact-constraint-list 0) ;; this should be set correctly
            ))
     )
   (send self :get :_contact-constraint-vector-list)
   )
  (:wrench-instant-inequality-constraint-matrix
   (&key (update? t))
   "
接触レンチ$\bm{w} \in \mathbb{R}^{6}$が満たすべき制約（非負制約，摩擦制約，圧力中心制約）が次式のように表されるとする．
\begin{eqnarray}
  \bm{C}_{w} \bm{w} \geq \bm{d}_{w}
\end{eqnarray}
$N_{\mathit{cnt}}$箇所の接触部位の接触レンチを並べたベクトル$\bm{\hat{w}}$の不等式制約は次式で表される．
\begin{eqnarray}
  &&\bm{C}_{w,m} \bm{w}_m \geq \bm{d}_{w,m} \ \ (m=1,2,\cdots,N_{\mathit{cnt}}) \\
  \Leftrightarrow&&
  \begin{pmatrix}\bm{C}_{w,1}&&&\\&\bm{C}_{w,2}&&\\&&\ddots&\\&&&\bm{C}_{w,N_{\mathit{cnt}}}\end{pmatrix}
  \begin{pmatrix}\bm{w}_1 \\ \bm{w}_2 \\ \vdots \\ \bm{w}_{N_{\mathit{cnt}}}\end{pmatrix}
  \geq
  \begin{pmatrix}\bm{d}_{w,1} \\ \bm{d}_{w,2} \\ \vdots \\ \bm{d}_{w,N_{\mathit{cnt}}}\end{pmatrix} \\
  \Leftrightarrow&&\bm{C}_{\hat{w}} \bm{\hat{w}} \geq \bm{d}_{\hat{w}}
\end{eqnarray}

return $\bm{C}_{\hat{w}} := \begin{pmatrix}\bm{C}_{w,1}&&&\\&\bm{C}_{w,2}&&\\&&\ddots&\\&&&\bm{C}_{w,N_{\mathit{cnt}}}\end{pmatrix} \in \mathbb{R}^{N_{\mathit{\hat{w}\mathchar`-ineq}} \times \mathit{dim}(\bm{\hat{w}})}$
   "
   (when (or update? (not (send self :get :wrench-inequality-constraint-matrix)))
     (send self :put :wrench-inequality-constraint-matrix
           (apply #'concatenate-matrix-diagonal (send self :_contact-constraint-matrix-list)))
     )
   (send self :get :wrench-inequality-constraint-matrix)
   )
  (:wrench-instant-inequality-constraint-vector
   (&key (update? nil))
   "
return $\bm{d}_{\hat{w}} := \begin{pmatrix}\bm{d}_{w,1} \\ \bm{d}_{w,2} \\ \vdots \\ \bm{d}_{w,N_{\mathit{cnt}}}\end{pmatrix} \in \mathbb{R}^{N_{\mathit{\hat{w}\mathchar`-ineq}}}$
   "
   (when (or update? (not (send self :get :wrench-inequality-constraint-vector)))
     (send self :put :wrench-inequality-constraint-vector
           (apply #'concatenate
                  float-vector
                  (send self :_contact-constraint-vector-list)))
     )
   (send self :get :wrench-inequality-constraint-vector)
   )
  (:wrench-control-vector-inequality-constraint-matrix
   (&key (update? nil))
   "
\begin{eqnarray}
  &&\bm{C_{\hat{w}}} \bm{\hat{w}} \geq \bm{\bar{d}_{\hat{w}}} \\
  \Leftrightarrow&&\bm{C_{p_{\hat{w}}}} \bm{p}_{\hat{w}} \geq \bm{\bar{d}_{p_{\hat{w}}}}
\end{eqnarray}
差分形式で表すと次式となる．
\begin{eqnarray}
  &&\bm{C_{p_{\hat{w}}}} (\bm{p}_{\hat{w}} + \Delta \bm{p}_{\hat{w}}) \geq \bm{\bar{d}_{p_{\hat{w}}}} \\
  \Leftrightarrow&&\bm{C_{p_{\hat{w}}}} \Delta \bm{p}_{\hat{w}} \geq \bm{\bar{d}_{p_{\hat{w}}}} - \bm{C_{p_{\hat{w}}}} \bm{p}_{\hat{w}} \\
  \Leftrightarrow&&\bm{C_{p_{\hat{w}}}} \Delta \bm{p}_{\hat{w}} \geq \bm{d_{p_{\hat{w}}}}
\end{eqnarray}

return $\bm{C}_{p_{\hat{w}}}$
   "
   (when (or update? (not (send self :get :wrench-control-vector-inequality-constraint-matrix)))
     (send self :put :wrench-control-vector-inequality-constraint-matrix
           (send _wrench-bst :convert-instant-inequality-constraint-matrix-for-control-vector
                 :instant-ineq-matrix
                 (send self :wrench-instant-inequality-constraint-matrix)
                 ))
     )
   (send self :get :wrench-control-vector-inequality-constraint-matrix)
   )
  (:wrench-control-vector-inequality-constraint-vector
   (&key (update? t))
   "
return $\bm{d}_{p_{\hat{w}}} := \bm{\bar{d}_{p_{\hat{w}}}} - \bm{C_{p_{\hat{w}}}} \bm{p}_{\hat{w}}$
   "
   (when (or update? (not (send self :get :wrench-control-vector-inequality-constraint-vector)))
     (send self :put :wrench-control-vector-inequality-constraint-vector
           (v- (send _wrench-bst :convert-instant-inequality-constraint-vector-for-control-vector
                     :instant-ineq-vector
                     (send self :wrench-instant-inequality-constraint-vector))
               (transform (send self :wrench-control-vector-inequality-constraint-matrix)
                          (send _wrench-bst :control-vector))
               ))
     )
   (send self :get :wrench-control-vector-inequality-constraint-vector)
   )
  (:torque-control-vector-inequality-constraint-matrix
   ()
   "
todo
   "
   (make-matrix 0 _dim-torque-control-vector)
   )
  (:torque-control-vector-inequality-constraint-vector
   ()
   "
todo
   "
   (float-vector)
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
  (:phi-inequality-constraint-matrix
   (&key (update? nil))
   "
\begin{eqnarray}
&&\bm{\phi}_{\mathit{min}} \leq \bm{\phi} + \Delta \bm{\phi} \leq \bm{\phi}_{\mathit{max}} \\
\Leftrightarrow&&\begin{pmatrix} \bm{I} \\ - \bm{I}\end{pmatrix} \Delta \bm{\phi}
\geq \begin{pmatrix} \bm{\phi}_{\mathit{min}} - \bm{\phi} \\ - (\bm{\phi}_{\mathit{max}} - \bm{\phi})\end{pmatrix} \\
\Leftrightarrow&&\bm{C_{\phi}} \Delta \bm{\phi} \geq \bm{d_{\phi}}
\end{eqnarray}

return $\bm{C_{\phi}} := \begin{pmatrix} \bm{I} \\ - \bm{I}\end{pmatrix} \in \mathbb{R}^{2 N_{\mathit{invar\mathchar`-joint}} \times N_{\mathit{invar\mathchar`-joint}}}$
   "
   (when (or update? (not (send self :get :phi-inequality-constraint-matrix)))
     (send self :put :phi-inequality-constraint-matrix
           (concatenate-matrix-column
            (unit-matrix _dim-phi)
            (scale-matrix -1 (unit-matrix _dim-phi)))
           )
     )
   (send self :get :phi-inequality-constraint-matrix)
   )
  (:phi-inequality-constraint-vector
   (&key (update? t))
   "
return $\bm{d_{\phi}} := \begin{pmatrix} \bm{\phi}_{\mathit{min}} - \bm{\phi} \\ - (\bm{\phi}_{\mathit{max}} - \bm{\phi})\end{pmatrix} \in \mathbb{R}^{2 N_{\mathit{invar\mathchar`-joint}}}$
   "
   (when (or update? (not (send self :get :phi-inequality-constraint-vector)))
     (send self :put :phi-inequality-constraint-vector
           (concatenate
            float-vector
            (v- (send self :phi-min-vector) _phi-vector)
            (v- _phi-vector (send self :phi-max-vector)))
           )
     )
   (send self :get :phi-inequality-constraint-vector)
   )
  (:config-inequality-constraint-matrix
   ()
   "
\begin{eqnarray}
  &&
  \left\{
  \begin{array}{l}
  \bm{C}_{p_{\theta}} \Delta \bm{p}_{\theta} \geq \bm{d}_{p_{\theta}} \\
  \bm{C}_{p_{c}} \Delta \bm{p}_{c} \geq \bm{d}_{p_{c}} \\
  \bm{C}_{p_{L}} \Delta \bm{p}_{L} \geq \bm{d}_{p_{L}} \\
  \bm{C}_{p_{\hat{w}}} \Delta \bm{p}_{\hat{w}} \geq \bm{d}_{p_{\hat{w}}} \\
  \bm{C}_{p_{\tau}} \Delta \bm{p}_{\tau} \geq \bm{d}_{p_{\tau}} \\
  \bm{C}_{\phi} \Delta \bm{\phi} \geq \bm{d}_{\phi}
  \end{array}
  \right. \\
  \Leftrightarrow&&
  \begin{pmatrix}
    \bm{C}_{p_{\theta}} &&&&&\\
    & \bm{C}_{p_{c}}  &&&&\\
    && \bm{C}_{p_{L}}  &&&\\
    &&& \bm{C}_{p_{\hat{w}}} &&\\
    &&&& \bm{C}_{p_{\tau}} &\\
    &&&&&\bm{C}_{\phi}
  \end{pmatrix}
  \begin{pmatrix}
    \Delta \bm{p}_{\theta} \\ \Delta \bm{p}_{c} \\ \Delta \bm{p}_{L} \\ \Delta \bm{p}_{\hat{w}} \\ \Delta \bm{p}_{\tau} \\ \Delta \bm{\phi}
  \end{pmatrix}
  \geq
  \begin{pmatrix}
    \bm{d}_{p_{\theta}} \\ \bm{d}_{p_{c}} \\ \bm{d}_{p_{L}} \\ \bm{d}_{p_{\hat{w}}} \\ \bm{d}_{p_{\tau}} \\ \bm{d}_{\phi}
  \end{pmatrix} \\
  \Leftrightarrow&&
  \bm{C} \Delta \bm{q} \geq \bm{d}
\end{eqnarray}

return $\bm{C}$
   "
   (concatenate-matrix-diagonal
    (send self :theta-control-vector-inequality-constraint-matrix)
    (send self :cog-control-vector-inequality-constraint-matrix)
    (send self :ang-moment-control-vector-inequality-constraint-matrix)
    (send self :wrench-control-vector-inequality-constraint-matrix)
    (send self :torque-control-vector-inequality-constraint-matrix)
    (send self :phi-inequality-constraint-matrix)
    )
   )
  (:config-inequality-constraint-vector
   ()
   "
return $\bm{d}$
   "
   (concatenate
    float-vector
    (send self :theta-control-vector-inequality-constraint-vector)
    (send self :cog-control-vector-inequality-constraint-vector)
    (send self :ang-moment-control-vector-inequality-constraint-vector)
    (send self :wrench-control-vector-inequality-constraint-vector)
    (send self :torque-control-vector-inequality-constraint-vector)
    (send self :phi-inequality-constraint-vector)
    )
   )
  )

(defmethod bspline-dynamic-configuration-task
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

(defmethod bspline-dynamic-configuration-task
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
  (:stationery-start-finish-regular-matrix
   (&key
    (update? nil)
    )
   "
return $\bm{W}_{\mathit{stat}} \in \mathbb{R}^{\mathit{dim}(\bm{q}) \times \mathit{dim}(\bm{q})}$
   "
   (when (or update? (not (send self :get :stationery-start-finish-regular-matrix)))
     (send self :put :stationery-start-finish-regular-matrix
           (concatenate-matrix-diagonal
            (send _theta-bst :stationery-start-finish-regular-matrix)
            (send _cog-bst :stationery-start-finish-regular-matrix)
            (send _ang-moment-bst :stationery-start-finish-regular-matrix)
            (send _wrench-bst :stationery-start-finish-regular-matrix)
            (send _torque-bst :stationery-start-finish-regular-matrix)
            (make-matrix _dim-phi _dim-phi)
            ))
     )
   (send self :get :stationery-start-finish-regular-matrix)
   )
  (:differential-square-integration-regular-matrix
   (&key
    (diff-order 1)
    )
   "
  return $\bm{W}_{\mathit{sqr,d}} \in \mathbb{R}^{\mathit{dim}(\bm{q}) \times \mathit{dim}(\bm{q})}$
     "
   (concatenate-matrix-diagonal
    (send _theta-bst :differential-square-integration-regular-matrix :diff-order diff-order)
    (send _cog-bst :differential-square-integration-regular-matrix :diff-order diff-order)
    (send _ang-moment-bst :differential-square-integration-regular-matrix :diff-order diff-order)
    (send _wrench-bst :differential-square-integration-regular-matrix :diff-order diff-order)
    (send _torque-bst :differential-square-integration-regular-matrix :diff-order diff-order)
    (make-matrix _dim-phi _dim-phi)
    )
   )
  (:first-differential-square-integration-regular-matrix
   (&key
    (update? nil)
    )
   "
  return $\bm{W}_{\mathit{sqr,1}} \in \mathbb{R}^{\mathit{dim}(\bm{q}) \times \mathit{dim}(\bm{q})}$
     "
   (when (or update? (not (send self :get :first-differential-square-integration-regular-matrix)))
     (send self :put :first-differential-square-integration-regular-matrix
           (send self :differential-square-integration-regular-matrix :diff-order 1))
     )
   (send self :get :first-differential-square-integration-regular-matrix)
   )
  (:second-differential-square-integration-regular-matrix
   (&key
    (update? nil)
    )
   "
  return $\bm{W}_{\mathit{sqr,2}} \in \mathbb{R}^{\mathit{dim}(\bm{q}) \times \mathit{dim}(\bm{q})}$
     "
   (when (or update? (not (send self :get :second-differential-square-integration-regular-matrix)))
     (send self :put :second-differential-square-integration-regular-matrix
           (send self :differential-square-integration-regular-matrix :diff-order 2))
     )
   (send self :get :second-differential-square-integration-regular-matrix)
   )
  (:third-differential-square-integration-regular-matrix
   (&key
    (update? nil)
    )
   "
  return $\bm{W}_{\mathit{sqr,3}} \in \mathbb{R}^{\mathit{dim}(\bm{q}) \times \mathit{dim}(\bm{q})}$
     "
   (when (or update? (not (send self :get :third-differential-square-integration-regular-matrix)))
     (send self :put :third-differential-square-integration-regular-matrix
           (send self :differential-square-integration-regular-matrix :diff-order 3))
     )
   (send self :get :third-differential-square-integration-regular-matrix)
   )
  (:regular-matrix
   ()
   "
\begin{eqnarray}
  \bm{W}_{\mathit{reg}} := \min(k_{\mathit{max}}, \|\bm{e}\|^2+k_{\mathit{off}}) \bm{I}
  + k_{\mathit{stat}} \bm{W}_{\mathit{stat}} + \sum_{\mathit{d}=1}^{3} k_{\mathit{sqr,d}} \bm{W}_{\mathit{sqr,d}}
\end{eqnarray}

return $\bm{W}_{\mathit{reg}} \in \mathbb{R}^{\mathit{dim}(\bm{q}) \times \mathit{dim}(\bm{q})}$
   "
   (reduce
    #'(lambda (m1 m2) (m+ m1 m2 (send self :_dim-config-buf-matrix)))
    (list
     (scale-matrix (min _norm-regular-scale-max (+ (norm2 (send self :task-value :update? nil)) _norm-regular-scale-offset))
                   (send self :_dim-config-unit-matrix))
     (scale-matrix _stationery-start-finish-regular-scale
                   (send self :stationery-start-finish-regular-matrix))
     (scale-matrix _first-diff-square-integration-regular-scale
                   (send self :first-differential-square-integration-regular-matrix))
     (scale-matrix _second-diff-square-integration-regular-scale
                   (send self :second-differential-square-integration-regular-matrix))
     (scale-matrix _third-diff-square-integration-regular-scale
                   (send self :third-differential-square-integration-regular-matrix))
     ))
   )
  (:regular-vector
   ()
   "
\begin{eqnarray}
  \bm{v}_{\mathit{reg}} := k_{\mathit{stat}} \bm{W}_{\mathit{stat}} \bm{q} + \sum_{\mathit{d}=1}^{3} k_{\mathit{sqr,d}} \bm{W}_{\mathit{sqr,d}} \bm{q}
\end{eqnarray}

return $\bm{v}_{\mathit{reg}} \in \mathbb{R}^{\mathit{dim}(\bm{q})}$
   "
   (v++
    (scale _stationery-start-finish-regular-scale
           (transform (send self :stationery-start-finish-regular-matrix) (send self :config-vector)))
    (scale _first-diff-square-integration-regular-scale
           (transform (send self :first-differential-square-integration-regular-matrix) (send self :config-vector)))
    (scale _second-diff-square-integration-regular-scale
           (transform (send self :second-differential-square-integration-regular-matrix) (send self :config-vector)))
    (scale _third-diff-square-integration-regular-scale
           (transform (send self :third-differential-square-integration-regular-matrix) (send self :config-vector)))
    ))
  )

(defmethod bspline-dynamic-configuration-task
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

(defmethod bspline-dynamic-configuration-task
  (:update-viewer
   (&key
    (start-time
     (send _theta-bst :start-time))
    (finish-time
     (send _theta-bst :finish-time))
    (delta-time
     (/ (- finish-time start-time) 100.0)) ;; [sec]
    )
   "
  Update viewer.
   "
   ;; visualize attention coords trajectory
   (send *irtviewer* :draw-objects :flush nil)
   (when (> delta-time 0)
     (let* ((tm start-time)
            (finish? nil)
            )
       (while t
         (send self :apply-config-to-robot tm)
         (send (scale 1e3 (send self :cog tm)) :draw-on :flush nil :size 50 :width 10 :color (float-vector 0.2 1 0.5))
         (send (scale 1e3 (send self :cog-from-model)) :draw-on :flush nil :size 100 :width 2 :color (float-vector 1 0.5 0.2))
         (send-all (send self :kin-target-coords-list tm) :draw-on :flush nil :size 50 :width 10 :color #f(0 1 0))
         (send-all (send self :kin-attention-coords-list tm) :draw-on :flush nil :size 100 :width 2 :color #f(1 0 0))
         (draw-force-value
          :force-list (send self :force-list tm)
          :moment-list (send self :moment-list tm)
          :coords-list (send self :contact-target-coords-list tm)
          :robot (send _robot-env :robot)
          :draw-cop? t
          )
         (when finish?
           (return-from nil nil))
         (setq tm (+ tm delta-time))
         (when (> tm finish-time)
           (setq finish? t)
           (setq tm (min tm finish-time))
           )
         )))
   (send-all (send self :contact-constraint-list 0) :draw-on :flush nil)
   ;; (send-all (send self :get :collision-line-list) :draw-on :flush nil :color #f(1 0 0) :width 2)
   (send *irtviewer* :viewer :flush)
   )
  (:print-setting-information
   ()
   "
Print setting information.
   "
   (warning-message 9 "=== ~a ===~%" (send (class self) :name))
   (warning-message 9 "dim-config: ~a~%" _dim-config)
   (warning-message 9 "  dim-theta-control-vector: ~a~%" _dim-theta-control-vector)
   (warning-message 9 "  dim-cog-control-vector: ~a~%" _dim-cog-control-vector)
   (warning-message 9 "  dim-ang-moment-control-vector: ~a~%" _dim-ang-moment-control-vector)
   (warning-message 9 "  dim-wrench-control-vector: ~a~%" _dim-wrench-control-vector)
   (warning-message 9 "  dim-torque-control-vector: ~a~%" _dim-torque-control-vector)
   (warning-message 9 "  dim-phi: ~a~%" _dim-phi)
   (warning-message 9 "dim-task: ~a~%" _dim-task)
   (warning-message 9 "  dim-kin-task: ~a~%" _dim-kin-task)
   (warning-message 9 "  dim-eom-trans-task: ~a~%" _dim-eom-trans-task)
   (warning-message 9 "  dim-eom-rot-task: ~a~%" _dim-eom-rot-task)
   (warning-message 9 "  dim-cog-task: ~a~%" _dim-cog-task)
   (warning-message 9 "  dim-ang-moment-task: ~a~%" _dim-ang-moment-task)
   (warning-message 9 "  dim-torque-task: ~a~%" _dim-torque-task)
   (warning-message 9 "  dim-posture-task: ~a~%" _dim-posture-task)
   )
  (:print-status
   ()
   "
Print status.
   "
   (when (= _dim-task 0)
     (return-from :print-status nil))
   (warning-message 9 "|task|=~a~%" (norm (send self :task-value :update? nil)))
   (warning-message 9 "|kin-task|=~a  (ave: ~a)~%"
                    (norm (send self :kinematics-task-value :update? nil))
                    (/ (norm (send self :kinematics-task-value :update? nil)) (length _kin-task-time-list)))
   (warning-message 9 "|eom-trans-task|=~a  (ave: ~a)~%"
                    (norm (send self :eom-trans-task-value :update? nil))
                    (/ (norm (send self :eom-trans-task-value :update? nil)) (length _eom-task-time-list)))
   (warning-message 9 "|eom-rot-task|=~a  (ave: ~a)~%"
                    (norm (send self :eom-rot-task-value :update? nil))
                    (/ (norm (send self :eom-rot-task-value :update? nil)) (length _eom-task-time-list)))
   (warning-message 9 "|cog-task|=~a  (ave: ~a)~%"
                    (norm (send self :cog-task-value :update? nil))
                    (/ (norm (send self :cog-task-value :update? nil)) (length _centroid-task-time-list)))
   (warning-message 9 "|ang-moment-task|=~a  (ave: ~a)~%"
                    (norm (send self :ang-moment-task-value :update? nil))
                    (/ (norm (send self :ang-moment-task-value :update? nil)) (length _centroid-task-time-list)))
   ;; (warning-message 9 "|torque-task|=~a~%" (norm (send self :torque-task-value :update? nil)))
   (warning-message 9 "|posture-task|=~a  (ave: ~a)~%"
                    (norm (send self :posture-task-value :update? nil))
                    (/ (norm (send self :posture-task-value :update? nil)) (length _posture-task-time-list)))
   (unless (= _stationery-start-finish-regular-scale 0.0)
     (warning-message
      9 "|stationery-start-finish-reg|=~a~%"
      (* _stationery-start-finish-regular-scale
         (v. (send self :config-vector)
             (transform (send self :stationery-start-finish-regular-matrix)
                        (send self :config-vector))))
      ))
   (unless (= _first-diff-square-integration-regular-scale 0.0)
     (warning-message
      9 "|first-diff-square-integration-reg|=~a~%"
      (* _first-diff-square-integration-regular-scale
         (v. (send self :config-vector)
             (transform (send self :first-differential-square-integration-regular-matrix)
                        (send self :config-vector))))
      ))
   (unless (= _second-diff-square-integration-regular-scale 0.0)
     (warning-message
      9 "|second-diff-square-integration-reg|=~a~%"
      (* _second-diff-square-integration-regular-scale
         (v. (send self :config-vector)
             (transform (send self :second-differential-square-integration-regular-matrix)
                        (send self :config-vector))))
      ))
   (unless (= _third-diff-square-integration-regular-scale 0.0)
     (warning-message
      9 "|third-diff-square-integration-reg|=~a~%"
      (* _third-diff-square-integration-regular-scale
         (v. (send self :config-vector)
             (transform (send self :third-differential-square-integration-regular-matrix)
                        (send self :config-vector))))
      ))
   )
  (:play-animation
   (&key
    (robot-env)
    (start-time
     (send _theta-bst :start-time))
    (finish-time
     (send _theta-bst :finish-time))
    (delta-time
     (/ (- finish-time start-time) 100.0)) ;; [sec]
    (loop? t)
    (visualize-callback-func)
    )
   "
  Play motion animation.
   "
   (do-until-key
    (let* ((tm start-time)
           (finish? nil)
           )
      (while t
        (send self :apply-config-to-robot tm)
        (send robot-env :angle-vector (send _robot-env :angle-vector))
        ;; draw
        (x::window-main-one)
        (send *irtviewer* :draw-objects :flush nil)
        (send (scale 1e3 (send self :cog tm)) :draw-on :flush nil :size 50 :width 10 :color (float-vector 0.2 1 0.5))
        (send (scale 1e3 (send self :cog-from-model)) :draw-on :flush nil :size 100 :width 2 :color (float-vector 1 0.5 0.2))
        (send-all (send self :kin-target-coords-list tm) :draw-on :flush nil :size 50 :width 10 :color #f(0 1 0))
        (send-all (send self :kin-attention-coords-list tm) :draw-on :flush nil :size 100 :width 2 :color #f(1 0 0))
        (draw-force-value
         :force-list (send self :force-list tm)
         :moment-list (send self :moment-list tm)
         :coords-list (send self :contact-target-coords-list tm)
         :robot (send robot-env :robot)
         :draw-cop? t
         )
        (map
         nil
         #'(lambda (coords const)
             (when coords (send const :draw-on :flush nil)))
         (send self :contact-target-coords-list tm)
         (send self :contact-constraint-list 0)
         )
        (send *irtviewer* :viewer :flush)
        (unix::usleep (round (* delta-time 1000 1000)))
        (when visualize-callback-func
          (funcall visualize-callback-func))
        ;; update time
        (when finish?
          (return-from nil nil))
        (setq tm (+ tm delta-time))
        (when (> tm finish-time)
          (setq finish? t)
          (setq tm (min tm finish-time))
          )
        ))
    (unless loop?
      (return-from nil nil))
    ))
  (:_dump-task-data
   (&key
    (task-method)
    (start-time)
    (finish-time)
    (delta-time)
    (data-filename)
    (dim-task)
    (data-name-list (range dim-task))
    )
   (unix:system
    (format nil "DIRNAME=`dirname ~a`; if [ ! -d $DIRNAME ]; then mkdir -p $DIRNAME; fi" data-filename))
   (with-open-file
    (f data-filename :direction :output :if-exists :new-version :if-does-not-exist :create)
    (let* ((tm start-time)
           (finish? nil)
           (task-value)
           )
      ;; dump header
      (format f "time ")
      (dolist (data-name data-name-list)
        (format f "~a " data-name))
      (format f "~%")
      (while t
        (setq task-value
              (send self task-method tm))
        (when (< (length task-value) dim-task)
          (setq task-value
                (concatenate
                 float-vector
                 task-value
                 (make-array (- dim-task (length task-value)) :element-type :float)))
          )
        (format f "~a " tm)
        (dotimes (j dim-task)
          (format f "~a " (elt task-value j)))
        (format f "~%")
        ;; update time
        (when finish?
          (return-from nil nil))
        (setq tm (+ tm delta-time))
        (when (>= tm finish-time)
          (setq finish? t)
          (setq tm finish-time)
          )
        )
      ))
   )
  (:_max-dim-kin-instant-task
   ()
   (or (send self :get :_max-dim-kin-instant-task)
       (send self :put :_max-dim-kin-instant-task
             (apply
              #'max
              (mapcar
               #'(lambda (tm) (length (send self :kinematics-instant-task-value tm)))
               _kin-task-time-list))
             ))
   )
  (:generate-graph
   (&key
    (start-time
     (send _theta-bst :start-time))
    (finish-time
     (send _theta-bst :finish-time))
    (delta-time
     (/ (- finish-time start-time) 100.0)) ;; [sec]
    (data-dirname
     "/tmp/bspline-dynamic-config-task")
    (graph-filename
     "/tmp/bspline-dynamic-config-task/graph.pdf")
    )
   "
Generate graph from configuration and task trajectory.
   "
   ;; dump config data
   (dolist (bst (list _theta-bst _cog-bst _ang-moment-bst _wrench-bst))
     (send bst :dump-config-data
           :start-time start-time
           :finish-time finish-time
           :delta-time delta-time
           :data-filename (format nil "~a/~a.dat" data-dirname (send bst :name))
           )
     )
   ;; dump task data
   (map
    nil
    #'(lambda
        (task-method
         task-name
         dim-task
         data-name-list
         )
        (send self :_dump-task-data
              :task-method task-method
              :start-time start-time
              :finish-time finish-time
              :delta-time delta-time
              :data-filename (format nil "~a/~a.dat" data-dirname task-name)
              :dim-task dim-task
              :data-name-list data-name-list
              ))
    (list :kinematics-instant-task-value
          :eom-trans-instant-task-value :eom-rot-instant-task-value
          :cog-instant-task-value :ang-moment-instant-task-value
          :posture-instant-task-value)
    (list "kinematics_task"
          "translational_eom_task" "rotational_eom_task"
          "cog_task" "angular_momentum_task"
          "posture_task")
    (list (send self :_max-dim-kin-instant-task)
          3 3
          3 3
          _num-posture-joint)
    (list (range (send self :_max-dim-kin-instant-task))
          (list "x" "y" "z") (list "x" "y" "z")
          (list "x" "y" "z") (list "x" "y" "z")
          (send-all _posture-joint-list :name))
    )
   ;; plot
   (unix:system
    (format nil "DIRNAME=`dirname ~a`; if [ ! -d $DIRNAME ]; then mkdir -p $DIRNAME; fi" graph-filename))
   (unix:system
    (format nil (ros::resolve-ros-path "package://eus_qp/optmotiongen/scripts/generate_graph_from_config_task.py \\
--data-dirname ~a --output-filename ~a \\
--kin-task-time-list ~a --eom-task-time-list ~a --centroid-task-time-list ~a --posture-task-time-list ~a")
            data-dirname graph-filename
            (reduce #'(lambda (x y) (format nil "~a ~a " x y)) _kin-task-time-list)
            (reduce #'(lambda (x y) (format nil "~a ~a " x y)) _eom-task-time-list)
            (reduce #'(lambda (x y) (format nil "~a ~a " x y)) _centroid-task-time-list)
            (reduce #'(lambda (x y) (format nil "~a ~a " x y)) _posture-task-time-list)
            ))
   )
  (:generate-robot-state-list
   (&key
    (robot-env _robot-env)
    (start-time
     (send _theta-bst :start-time))
    (finish-time
     (send _theta-bst :finish-time))
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
   (let* ((tm start-time)
          (finish? nil)
          (rs-list)
          (contact-map)
          (av)
          (rc)
          (zmp)
          (wl)
          (cs)
          )
     (while t
       (send self :apply-config-to-robot tm)
       (send robot-env :angle-vector (send _robot-env :angle-vector))
       (setq av
             (map float-vector
                  #'(lambda (joint-name) (send (send robot-env :robot :joint joint-name) :joint-angle))
                  joint-name-list))
       (setq rc
             (send (send robot-env :robot :link root-link-name) :copy-worldcoords))
       ;; (= (elt contact-map i) j) means that (elt limb-list i) corresponds to (elt contact-attention-coords-list j)
       (setq contact-map
             (mapcar
              #'(lambda (limb)
                  (send self :_get-contact-index-from-limb tm :limb limb))
              limb-list
              ))
       (setq cs
             (mapcar
              #'(lambda (cm) (if (numberp cm) :support :swing))
              contact-map
              ))
       (setq wl
             (mapcar
              #'(lambda (cm)
                  (if (numberp cm)
                      (elt (send self :wrench-list tm) cm)
                    (float-vector 0 0 0 0 0 0)
                    ))
              contact-map
              ))
       (cond ((some #'eval contact-map)
              (setq zmp
                    (scale
                     (/ 1.0 ;; denominator of zmp
                        (apply
                         #'+
                         (remove
                          nil
                          (mapcar
                           #'(lambda (cm w)
                               (if (numberp cm) (elt w 2)))
                           contact-map
                           wl
                           ))))
                     (reduce ;; numerator of zmp
                      #'v+
                      (remove
                       nil
                       (mapcar
                        #'(lambda (cm w)
                            (if (numberp cm)
                                (let* ((fx (elt w 0))
                                       (fy (elt w 1))
                                       (fz (elt w 2))
                                       (nx (elt w 3))
                                       (ny (elt w 4))
                                       (nz (elt w 5))
                                       (trg-coords
                                        (elt (send self :contact-target-coords-list tm) cm))
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
                        wl
                        ))))
                    ))
             (t
              (setq zmp (float-vector 0 0 0))
              ))
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
       ;; update time
       (when finish?
         (return-from nil nil))
       (setq tm (+ tm step-time))
       (when (> tm finish-time)
         (setq finish? t)
         (setq tm (min tm finish-time))
         )
       )
     rs-list
     ))
  (:_get-contact-index-from-limb
   (tm
    &key
    (limb)
    )
   (position-if
    #'(lambda (att-trg-coords-list)
        (and (elt att-trg-coords-list 0)
             (elt att-trg-coords-list 1)
             (some #'(lambda (limb-link)
                       (string= (send (send (elt att-trg-coords-list 0) :parent) :name) (send limb-link :name)))
                   (send _robot-env :robot limb)))
        )
    (mapcar
     #'(lambda (x y) (list x y))
     (send self :contact-attention-coords-list tm)
     (send self :contact-target-coords-list tm))
    ))
  )
