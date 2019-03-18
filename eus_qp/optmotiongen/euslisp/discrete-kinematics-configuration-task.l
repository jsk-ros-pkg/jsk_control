(load "./instant-configuration-task.l")

(unless (boundp 'instant-configuration-task) ;; dummy class definition for document
  (defclass instant-configuration-task :super object))


;;;;;;;;;;;;;;;;;;;;;;;;;
;; discrete-kinematics-configuration-task
;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass discrete-kinematics-configuration-task
  :super instant-configuration-task
  :slots ((_smooth-alpha "$\alpha$")
          )
  :documentation
  "
離散的な幾何目標を扱えるように拡張された
瞬時コンフィギュレーション$\bm{q}^{(l)}$と瞬時タスク関数$\bm{e}^{(l)}(\bm{q}^{(l)})$のクラス．

離散的な幾何目標とは，
kin-target-coords-listやkin-attention-coords-listとして
目標位置姿勢や着目位置姿勢が複数ペア与えられ，
それらのいずれかが成り立てば良いという制約のことを指す．
離散的な幾何目標のタスク関数に含まれるmin関数を微分可能関数で近似することで，タスク関数のヤコビ行列を求める．
  "
  )

(defmethod discrete-kinematics-configuration-task
  (:init
   (&rest
    args
    &key
    (smooth-alpha 20.0)
    &allow-other-keys
    )
   "
Initialize instance
   "
   (send-super* :init args)
   (setq _smooth-alpha smooth-alpha)
   )
  (:_calc-smooth-min-for-kinematics-task-value
   (trg-coords-dsc-list att-coords-dsc-list sc-mat)
   (when (atom trg-coords-dsc-list)
     (setq trg-coords-dsc-list (list trg-coords-dsc-list)))
   (when (atom att-coords-dsc-list)
     (setq att-coords-dsc-list (list att-coords-dsc-list)))
   (let* ((diff-list
           (mapcar
            #'(lambda (trg-coords att-coords)
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
            trg-coords-dsc-list
            att-coords-dsc-list
            ))
          (diff-exp-list
           (mapcar #'(lambda (diff) (exp (* (- _smooth-alpha) (norm2 diff)))) diff-list))
          )
     (scale (/ 1.0 (reduce #'+ diff-exp-list))
            (apply #'v++ (mapcar #'(lambda (diff diff-exp) (scale diff-exp diff)) diff-list diff-exp-list)))
     ))
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
  \argmin_{\bm{e}_{m,i}^{\mathit{kin}} \in \mathcal{E}_{m}^{\mathit{kin}}} \| \bm{e}_{m,i}^{\mathit{kin}}(\bm{\theta}, \bm{\phi}) \|^2
  \in \mathbb{R}^6 \ \ (m = 1,2,\cdots,N_{\mathit{kin}})
  \\
  &&{\rm where} \ \ \mathcal{E}_{m}^{\mathit{kin}} = \{ \bm{e}_{m,i}^{\mathit{kin}} \ | \ i = 1,2,\cdots,N_{\mathit{kin\mathchar`-dis,m}} \}
  \\
  \bm{e}_{m,i}^{\mathit{kin}}(\bm{\theta}, \bm{\phi}) &=&
  K_{\mathit{kin}}
  \begin{pmatrix}
    \bm{p}_{m,i}^{\mathit{kin\mathchar`-trg}}(\bm{\theta}, \bm{\phi}) - \bm{p}_{m,i}^{\mathit{kin\mathchar`-att}}(\bm{\theta}, \bm{\phi}) \\
    \bm{a}\left(\bm{R}_{m,i}^{\mathit{kin\mathchar`-trg}}(\bm{\theta}, \bm{\phi}) \bm{R}_{m,i}^{\mathit{kin\mathchar`-att}}(\bm{\theta}, \bm{\phi})^T\right)
  \end{pmatrix}
  \in \mathbb{R}^6 \ \ (i = 1,2,\cdots,N_{\mathit{kin\mathchar`-dis,m}})
\end{eqnarray}
$\bm{a}(\bm{R})$は姿勢行列$\bm{R}$の等価角軸ベクトルを表す．
$\bm{e}_{m}^{\mathit{kin}}(\bm{\theta}, \bm{\phi})$を次式で近似する．
\begin{eqnarray}
  \bm{e}_{m}^{\mathit{kin}}(\bm{\theta}, \bm{\phi}) &=&
  \dfrac{1}{\sum_{\bm{e}_{m,i}^{\mathit{kin}} \in \mathcal{E}_{m}^{\mathit{kin}}} \exp(- \alpha \| \bm{e}_{m,i}^{\mathit{kin}}(\bm{\theta}, \bm{\phi}) \|^2)}
  \sum_{\bm{e}_{m,i}^{\mathit{kin}} \in \mathcal{E}_{m}^{\mathit{kin}}} \exp(- \alpha \| \bm{e}_{m,i}^{\mathit{kin}}(\bm{\theta}, \bm{\phi}) \|^2) \bm{e}_{m,i}^{\mathit{kin}}(\bm{\theta}, \bm{\phi}) \nonumber \\
  &\in& \mathbb{R}^6 \ \ (m = 1,2,\cdots,N_{\mathit{kin}})
\end{eqnarray}
$\alpha$は正の定数で大きいほど近似精度が増す．

return $\bm{e}^{\mathit{kin}}(\bm{q}) \in \mathbb{R}^{6 N_{\mathit{kin}}}$
   "
   (when (or update? (not (send self :get :kinematics-task-value)))
     (send self :put :kinematics-task-value
           (apply #'concatenate
                  float-vector
                  (mapcar
                   #'(lambda (trg-coords att-coords sc-mat)
                       (send self :_calc-smooth-min-for-kinematics-task-value trg-coords att-coords sc-mat))
                   _kin-target-coords-list
                   _kin-attention-coords-list
                   _kin-scale-mat-list
                   ))
           ))
   (send self :get :kinematics-task-value)
   )
  (:_calc-smooth-min-for-kinematics-task-jacobian
   (trg-coords-dsc-list att-coords-dsc-list sc-mat)
   (let* ((jacobi-list
           (mapcar
            #'(lambda (trg-coords att-coords)
                (m*
                 sc-mat
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
                     ))
                   ))))
            trg-coords-dsc-list att-coords-dsc-list
            ))
          (diff-list ;; [ToDo] reuse the calculation result in :_calc-smooth-min-for-kinematics-task-value
           (mapcar
            #'(lambda (trg-coords att-coords)
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
            trg-coords-dsc-list
            att-coords-dsc-list
            ))
          (diff-exp-list ;; [ToDo] reuse the calculation result in :_calc-smooth-min-for-kinematics-task-value
           (mapcar #'(lambda (diff) (exp (* (- _smooth-alpha) (norm2 diff)))) diff-list))
          (u
           (/ 1.0 (reduce #'+ diff-exp-list)))
          (v
           (apply #'v++ (mapcar #'(lambda (diff diff-exp) (scale diff-exp diff)) diff-list diff-exp-list)))
          (u-jacobi
           (scale (/ 1.0 (* (reduce #'+ diff-exp-list) (reduce #'+ diff-exp-list)))
                  (apply
                   #'v++
                   (mapcar
                    #'(lambda (diff diff-exp jacobi)
                        (scale (* 2 _smooth-alpha diff-exp) (transform (transpose jacobi) diff)))
                    diff-list
                    diff-exp-list
                    jacobi-list
                    ))))
          (v-jacobi
           (reduce
            #'m+
            (mapcar
             #'(lambda (diff diff-exp jacobi)
                 (scale-matrix
                  diff-exp
                  (m+
                   (scale-matrix (* -2 _smooth-alpha)
                                 (m* (m* (transpose (matrix diff)) (matrix diff)) jacobi))
                   jacobi
                   )))
             diff-list
             diff-exp-list
             jacobi-list
             )))
          )
     (m+ (m* (transpose (matrix v)) (matrix u-jacobi))
         (scale-matrix u v-jacobi))
     ))
  (:kinematics-task-jacobian-with-theta
   ()
   "
\begin{eqnarray}
  \frac{\partial \bm{e}^{\mathit{kin}}}{\partial \bm{\theta}}
  &=& \begin{pmatrix}
    \frac{\partial \bm{e}_1^{\mathit{kin}}}{\partial \bm{\theta}} \\ \frac{\partial \bm{e}_2^{\mathit{kin}}}{\partial \bm{\theta}} \\ \vdots \\ \frac{\partial \bm{e}_{N_{\mathit{kin}}}^{\mathit{kin}}}{\partial \bm{\theta}}
  \end{pmatrix}
\end{eqnarray}
ここで，
\begin{eqnarray}
  \bm{e}_{m}^{\mathit{kin}}(\bm{\theta}, \bm{\phi}) &=&
  \dfrac{1}{\sum_{\bm{e}_{m,i}^{\mathit{kin}} \in \mathcal{E}_{m}^{\mathit{kin}}} \exp(- \alpha \| \bm{e}_{m,i}^{\mathit{kin}}(\bm{\theta}, \bm{\phi}) \|^2)}
  \sum_{\bm{e}_{m,i}^{\mathit{kin}} \in \mathcal{E}_{m}^{\mathit{kin}}} \exp(- \alpha \| \bm{e}_{m,i}^{\mathit{kin}}(\bm{\theta}, \bm{\phi}) \|^2) \bm{e}_{m,i}^{\mathit{kin}}(\bm{\theta}, \bm{\phi}) \nonumber \\
  &=& u(\bm{\theta}, \bm{\phi}) \bm{v}(\bm{\theta}, \bm{\phi}) \\
  {\rm where} \ \ && u(\bm{\theta}, \bm{\phi})
  = \dfrac{1}{\sum_{\bm{e}_{m,i}^{\mathit{kin}} \in \mathcal{E}_{m}^{\mathit{kin}}} \exp(- \alpha \| \bm{e}_{m,i}^{\mathit{kin}}(\bm{\theta}, \bm{\phi}) \|^2)}
  \in \mathbb{R} \\
  \phantom{\rm where} \ \ && \bm{v}(\bm{\theta}, \bm{\phi})
  = \sum_{\bm{e}_{m,i}^{\mathit{kin}} \in \mathcal{E}_{m}^{\mathit{kin}}} \exp(- \alpha \| \bm{e}_{m,i}^{\mathit{kin}}(\bm{\theta}, \bm{\phi}) \|^2) \bm{e}_{m,i}^{\mathit{kin}}(\bm{\theta}, \bm{\phi})
  \in \mathbb{R}^6
\end{eqnarray}
であるから，
\begin{eqnarray}
  \frac{\partial \bm{e}_{m}^{\mathit{kin}}(\bm{\theta}, \bm{\phi})}{\partial \bm{\theta}} &=&
  \bm{v}(\bm{\theta}, \bm{\phi}) \left( \frac{\partial u(\bm{\theta}, \bm{\phi})}{\partial \bm{\theta}} \right)^T +
  u(\bm{\theta}, \bm{\phi}) \left( \frac{\partial \bm{v}(\bm{\theta}, \bm{\phi})}{\partial \bm{\theta}} \right) \\
  \frac{\partial u(\bm{\theta}, \bm{\phi})}{\partial \bm{\theta}} &=&
  \dfrac{\sum_{\bm{e}_{m,i}^{\mathit{kin}} \in \mathcal{E}_{m}^{\mathit{kin}}} 2 \alpha A \left( \frac{\partial \bm{e}_{m,i}^{\mathit{kin}}(\bm{\theta}, \bm{\phi})}{\partial \bm{\theta}} \right)^T \bm{e}_{m,i}^{\mathit{kin}}(\bm{\theta}, \bm{\phi}) }
        {\left\{ \sum_{\bm{e}_{m,i}^{\mathit{kin}} \in \mathcal{E}_{m}^{\mathit{kin}}} A \right\}^2}
  \in \mathbb{R}^{N_\mathit{var\mathchar`-joint}} \\
  \frac{\partial \bm{v}(\bm{\theta}, \bm{\phi})}{\partial \bm{\theta}} &=&
  \sum_{\bm{e}_{m,i}^{\mathit{kin}} \in \mathcal{E}_{m}^{\mathit{kin}}} A \left\{ -2 \alpha \bm{e}_{m,i}^{\mathit{kin}}(\bm{\theta}, \bm{\phi}) \bm{e}_{m,i}^{\mathit{kin}}(\bm{\theta}, \bm{\phi})^T \left( \frac{\partial \bm{e}_{m,i}^{\mathit{kin}}(\bm{\theta}, \bm{\phi})}{\partial \bm{\theta}} \right) + \left( \frac{\partial \bm{e}_{m,i}^{\mathit{kin}}(\bm{\theta}, \bm{\phi})}{\partial \bm{\theta}} \right) \right\}
  \in \mathbb{R}^{6 \times N_\mathit{var\mathchar`-joint}} \\
  \frac{\partial \bm{e}_{m,i}^{\mathit{kin}}}{\partial \bm{\theta}}
  &=& K_{\mathit{kin}} \left\{ \bm{J}_{\theta,m,i}^{\mathit{kin\mathchar`-trg}}(\bm{\theta}, \bm{\phi}) - \bm{J}_{\theta,m,i}^{\mathit{kin\mathchar`-att}}(\bm{\theta}, \bm{\phi}) \right\} \\
  && (m = 1,2,\cdots,N_{\mathit{kin}}, \ \ i = 1,2,\cdots,N_{\mathit{kin\mathchar`-dis,m}}) \nonumber
\end{eqnarray}
ただし，
\begin{eqnarray}
  A = \exp(- \alpha \| \bm{e}_{m,i}^{\mathit{kin}}(\bm{\theta}, \bm{\phi}) \|^2)
\end{eqnarray}
とした．

return $\dfrac{\partial \bm{e}^{\mathit{kin}}}{\partial \bm{\theta}} \in \mathbb{R}^{6 N_{\mathit{kin}} \times N_\mathit{var\mathchar`-joint}}$
   "
   (apply
    #'concatenate-matrix-column
    (mapcar
     #'(lambda (trg-coords att-coords sc-mat)
         (send self :_calc-smooth-min-for-kinematics-task-jacobian trg-coords att-coords sc-mat))
     _kin-target-coords-list
     _kin-attention-coords-list
     _kin-scale-mat-list
     ))
   )
  )
