(load "./util.l")


;;;;;;;;;;;;;;;;;;;;;;;;;
;; torque
;;;;;;;;;;;;;;;;;;;;;;;;;

(defun get-link-jacobian-for-contact-torque
    (&key
     (robot)
     (drive-joint-list)
     (contact-coords)
     (contact-parent-link)
     )
  "
\it{contact-coords}に対応する接触部位の番号を$m$とする．
\it{contact-coords}の位置姿勢を$\bm{r}_{m} \in \mathbb{R}^6$，
\it{drive-joint-list}の関節角度ベクトルを$\bm{\psi} \in \mathbb{R}^{N_{\mathit{drive\mathchar`-joint}}}$として，
次式を満たすヤコビ行列$\bm{J}_m$を返す．
\begin{eqnarray}
  \bm{\dot{r}}_m &=& \bm{J}_m \bm{\dot{\psi}} \\ &=& \sum_{i=1}^{N_{\mathit{drive\mathchar`-joint}}} \bm{j}_m^{(i)} \dot{\psi}_i \\
  \bm{j}_m^{(i)} &=& \begin{pmatrix} \bm{a}_{\psi_i} \times (\bm{p}_m - \bm{p}_{\psi_i}) \\ \bm{a}_{\psi_i} \end{pmatrix} \\
  \bm{J}_m &=& \begin{pmatrix}\bm{j}_m^{(1)} & \bm{j}_m^{(2)} & \cdots & \bm{j}_m^{N_{\mathit{drive\mathchar`-joint}}}\end{pmatrix} \label{eq:jacobian-for-contact-torque}
\end{eqnarray}
$\bm{a}_{\psi_i}, \bm{p}_{\psi_i} \in \mathbb{R}^3$は$i$番目の関節の回転軸ベクトルと位置である．

return $\bm{J}_m \in \mathbb{R}^{6 \times N_{\mathit{drive\mathchar`-joint}}}$
  "
  (let* ((jacobi
          (make-matrix 6 (length drive-joint-list)))
         (drive-jnt)
         )
    (dotimes (i (length drive-joint-list))
      (setq drive-jnt (elt drive-joint-list i))
      (when (member (send drive-jnt :child-link) (send robot :link-list contact-parent-link))
        (let* ((end-pos ;; [m]
                (scale 1e-3 (send contact-coords :worldpos)))
               (jnt-axis
                (send drive-jnt :axis-vector))
               (jnt-pos ;; [m]
                (scale 1e-3 (send drive-jnt :pos)))
               (jacobi-vec
                (cond ((derivedp drive-jnt linear-joint)
                       (concatenate float-vector jnt-axis (float-vector 0 0 0))
                       )
                      ((derivedp drive-jnt rotational-joint)
                       (concatenate float-vector (v* jnt-axis (v- end-pos jnt-pos)) jnt-axis)
                       )
                      (t
                       (error "unsupported joint type: ~a~%" drive-jnt)
                       ))
                )
               )
          (setf (matrix-column jacobi i) jacobi-vec)
          )))
    jacobi
    ))

(defun get-contact-torque
    (&key
     (robot)
     (drive-joint-list)
     (wrench-list)
     (contact-target-coords-list)
     (contact-attention-coords-list)
     )
  "
ロボットの接触部位に加わる接触レンチによって生じる関節トルク$\bm{\tau}^{\mathit{cnt}}$は，以下で得られる．
\begin{eqnarray}
  \bm{\tau}^{\mathit{cnt}} = \sum_{m=1}^{N_{\mathit{cnt}}} \bm{J}_m^T \bm{w}_m \label{eq:contact-torque} \label{eq:contact-torque}
\end{eqnarray}
$\bm{w}_m$は$m$番目の接触部位で受ける接触レンチである．

return $\bm{\tau}^{\mathit{cnt}} \in \mathbb{R}^{N_{\mathit{drive\mathchar`-joint}}}$
  "
  (apply
   #'v++
   (mapcar
    #'(lambda (w trg-coords att-coords)
        (transform
         (transpose
          (get-link-jacobian-for-contact-torque
           :robot robot
           :drive-joint-list drive-joint-list
           :contact-coords trg-coords
           :contact-parent-link (send att-coords :parent)
           ))
         w
         ))
    wrench-list
    contact-target-coords-list
    contact-attention-coords-list
    ))
  )

(defun get-contact-torque-jacobian
    (&key
     (robot)
     (joint-list)
     (drive-joint-list)
     (wrench-list)
     (contact-target-coords-list)
     (contact-attention-coords-list)
     )
  "
\eqref{eq:jacobian-for-contact-torque}の$\bm{J}_{m}$を以下のように分解して利用する．
\begin{eqnarray}
  \bm{J}_{m}
  = \begin{pmatrix} \bm{j}_{\mathit{x,m}}^T \\ \bm{j}_{\mathit{y,m}}^T \\ \bm{j}_{\mathit{z,m}}^T \\ \bm{j}_{\mathit{R,m}}^T \\ \bm{j}_{\mathit{P,m}}^T \\ \bm{j}_{\mathit{Y,m}}^T \end{pmatrix}
  = \begin{pmatrix} \left(\frac{\partial r_{\mathit{x,m}}}{\partial \bm{\psi}}\right)^T \\ \left(\frac{\partial r_{\mathit{y,m}}}{\partial \bm{\psi}}\right)^T \\ \left(\frac{\partial r_{\mathit{z,m}}}{\partial \bm{\psi}}\right)^T \\ \left(\frac{\partial r_{\mathit{R,m}}}{\partial \bm{\psi}}\right)^T \\ \left(\frac{\partial r_{\mathit{P,m}}}{\partial \bm{\psi}}\right)^T \\ \left(\frac{\partial r_{\mathit{Y,m}}}{\partial \bm{\psi}}\right)^T \end{pmatrix}
\end{eqnarray}
これを\eqref{eq:contact-torque}に代入すると，
\begin{eqnarray}
\bm{\tau}^{\mathit{cnt}} &=& \sum_{m=1}^{N_{\mathit{cnt}}} \bm{J}_m^T \bm{w}_m \\
  &=&
  \sum_{m=1}^{N_{\mathit{cnt}}}
  \begin{pmatrix}
    \bm{j}_{x,m} & \bm{j}_{y,m} & \bm{j}_{z,m} & \bm{j}_{R,m} & \bm{j}_{P,m} & \bm{j}_{Y,m}
  \end{pmatrix}
  \begin{pmatrix}
    f_{x,m} \\ f_{y,m} \\ f_{z,m} \\ n_{x,m} \\ n_{y,m} \\ n_{z,m}
  \end{pmatrix} \\
  &=&
  \sum_{m=1}^{N_{\mathit{cnt}}}
  \left(
  \bm{j}_{x,m} f_{x,m} + \bm{j}_{y,m} f_{y,m} + \bm{j}_{z,m} f_{z,m} + \bm{j}_{R,m} n_{x,m} + \bm{j}_{P,m} n_{y,m} + \bm{j}_{Y,m} n_{z,m}
  \right) \\
  &=&
  \sum_{m=1}^{N_{\mathit{cnt}}}
  \left(
  \frac{\partial r_{x,m}}{\partial \bm{\psi}} f_{x,m} + \frac{\partial r_{y,m}}{\partial \bm{\psi}} f_{y,m} + \frac{\partial r_{z,m}}{\partial \bm{\psi}} f_{z,m} +
  \frac{\partial r_{R,m}}{\partial \bm{\psi}} n_{x,m} + \frac{\partial r_{P,m}}{\partial \bm{\psi}} n_{y,m} + \frac{\partial r_{Y,m}}{\partial \bm{\psi}} n_{z,m}
  \right)
\end{eqnarray}

\it{joint-list}の関節角度ベクトルを$\bm{\theta} \in \mathbb{R}^{N_{\mathit{joint}}}$，
\it{drive-joint-list}の関節角度ベクトルを$\bm{\psi} \in \mathbb{R}^{N_{\mathit{drive\mathchar`-joint}}}$とする．
トルク勾配行列$\frac{\partial \bm{\tau}^{\mathit{cnt}}}{\partial \bm{\theta}}$は次式で得られる．
\begin{eqnarray}
  \frac{\partial \bm{\tau}^{\mathit{cnt}}}{\partial \bm{\theta}}
  &=&
  \sum_{m=1}^{N_{\mathit{cnt}}} \left( \frac{\partial \bm{J}_m}{\partial \bm{\theta}} \right)^T \bm{w}_m \\
  &=&
  \sum_{m=1}^{N_{\mathit{cnt}}}
  \left(
  f_{x,m} \frac{\partial^2 r_{x,m}}{\partial \bm{\psi} \partial \bm{\theta}} +
  f_{y,m} \frac{\partial^2 r_{y,m}}{\partial \bm{\psi} \partial \bm{\theta}} +
  f_{z,m} \frac{\partial^2 r_{z,m}}{\partial \bm{\psi} \partial \bm{\theta}} +
  n_{x,m} \frac{\partial^2 r_{R,m}}{\partial \bm{\psi} \partial \bm{\theta}} +
  n_{y,m} \frac{\partial^2 r_{P,m}}{\partial \bm{\psi} \partial \bm{\theta}} +
  n_{z,m} \frac{\partial^2 r_{Y,m}}{\partial \bm{\psi} \partial \bm{\theta}}
  \right) \\
  &=&
  \left[
    \sum_{m=1}^{M}
    \begin{pmatrix}
      f_{x,m} & f_{y,m} & f_{z,m} & n_{x,m} & n_{y,m} & n_{z,m}
    \end{pmatrix}
    \begin{pmatrix}
      \frac{\partial^2 r_{x,m}}{\partial \psi_i \partial \theta_j} \\
      \frac{\partial^2 r_{y,m}}{\partial \psi_i \partial \theta_j} \\
      \frac{\partial^2 r_{z,m}}{\partial \psi_i \partial \theta_j} \\
      \frac{\partial^2 r_{R,m}}{\partial \psi_i \partial \theta_j} \\
      \frac{\partial^2 r_{P,m}}{\partial \psi_i \partial \theta_j} \\
      \frac{\partial^2 r_{Y,m}}{\partial \psi_i \partial \theta_j}
    \end{pmatrix}
    \right]_{i=1,\cdots,N_{\mathit{drive\mathchar`-joint}},j=1,\cdots,N_{\mathit{joint}}}
\end{eqnarray}

return $\frac{\partial \bm{\tau}^{\mathit{cnt}}}{\partial \bm{\theta}} \in \mathbb{R}^{N_{\mathit{drive\mathchar`-joint}} \times N_{\mathit{joint}}}$
  "
  (let* ((jacobi
          (make-matrix (length drive-joint-list) (length joint-list)))
         (drive-jnt)
         (jnt)
         (dependency-mode)
         (contact-coords)
         (contact-parent-link)
         (wrench)
         (link-list-of-contact)
         )
    (dotimes (i (length drive-joint-list))
      (setq drive-jnt (elt drive-joint-list i))
      (dotimes (j (length joint-list))
        (setq jnt (elt joint-list j))
        ;; drive-jntとjntのどちらがルート側にあるか調べる
        (cond ((member (send drive-jnt :child-link) (send robot :link-list (send jnt :child-link)))
               (setq dependency-mode :drive-jnt-is-root)
               )
              ((member (send jnt :child-link) (send robot :link-list (send drive-jnt :child-link)))
               (setq dependency-mode :jnt-is-root)
               )
              (t
               (setq dependency-mode nil)
               ))
        (dotimes (k (length contact-attention-coords-list))
          (setq contact-coords (elt contact-target-coords-list k))
          (setq contact-parent-link (send (elt contact-attention-coords-list k) :parent))
          (setq wrench (elt wrench-list k))
          (setq link-list-of-contact (send robot :link-list contact-parent-link))
          ;; [root, contact]間にdrive-jntがある and [root, contact]間にjntがある ときだけ処理する
          (when (and (member (send drive-jnt :child-link) link-list-of-contact)
                     (member (send jnt :child-link) link-list-of-contact))
            (let* ((drive-jnt-axis
                    (send drive-jnt :axis-vector))
                   (jnt-axis
                    (send jnt :axis-vector))
                   (hessian-vec
                    (cond ((and (derivedp drive-jnt rotational-joint)
                                (derivedp jnt rotational-joint))
                           (cond ((equal dependency-mode :jnt-is-root)
                                  (concatenate float-vector
                                               (v* jnt-axis (v* drive-jnt-axis
                                                                (scale 1e-3 (v- (send contact-coords :worldpos) (send drive-jnt :pos)))))
                                               (v* jnt-axis drive-jnt-axis)
                                               )
                                  )
                                 (t
                                  (concatenate float-vector
                                               (v* drive-jnt-axis (v* jnt-axis
                                                                      (scale 1e-3 (v- (send contact-coords :worldpos) (send jnt :pos)))))
                                               (v* drive-jnt-axis jnt-axis)
                                               )
                                  ))
                           )
                          ((and (derivedp drive-jnt linear-joint)
                                (derivedp jnt linear-joint))
                           (float-vector 0 0 0 0 0 0)
                           )
                          ((and (derivedp drive-jnt linear-joint)
                                (derivedp jnt rotational-joint))
                           (cond ((equal dependency-mode :drive-jnt-is-root)
                                  (float-vector 0 0 0 0 0 0) ;; linear-jointがルート側いあったら零ベクトル
                                  )
                                 (t
                                  (concatenate float-vector
                                               (v* jnt-axis drive-jnt-axis)
                                               (float-vector 0 0 0)
                                               )
                                  ))
                           )
                          ((and (derivedp drive-jnt rotational-joint)
                                (derivedp jnt linear-joint))
                           (cond ((equal dependency-mode :jnt-is-root)
                                  (float-vector 0 0 0 0 0 0) ;; linear-jointがルート側いあったら零ベクトル
                                  )
                                 (t
                                  (concatenate float-vector
                                               (v* drive-jnt-axis jnt-axis)
                                               (float-vector 0 0 0)
                                               )
                                  ))
                           )
                          (t
                           (error "unsupported joint type pair: ~a ~a~%" drive-jnt jnt)
                           ))
                    )
                   )
              (cond ((equal dependency-mode :jnt-is-root)
                     (setf (aref jacobi i j)
                           (+ (aref jacobi i j) (v. wrench hessian-vec)))
                     )
                    (t
                     (setf (aref jacobi i j)
                           (+ (aref jacobi i j) (v. (subseq wrench 0 3) (subseq hessian-vec 0 3))))
                     ))
              )))
        ))
    jacobi
    ))

(defun get-link-jacobian-for-gravity-torque
    (&key
     (robot)
     (drive-joint-list)
     (gravity-link)
     )
  "
\it{gravity-link}のリンク番号を$k$とする．
\it{gravity-link}の重心位置を$\bm{p}_{\mathit{cog,k}} \in \mathbb{R}^3$，
\it{drive-joint-list}の関節角度ベクトルを$\bm{\psi} \in \mathbb{R}^{N_{\mathit{drive\mathchar`-joint}}}$として，
次式を満たすヤコビ行列$\bm{J}_{\mathit{cog,k}}$を返す．
\begin{eqnarray}
  \bm{\dot{p}}_{\mathit{cog,k}} &=& \bm{J}_{\mathit{cog,k}} \bm{\dot{\psi}} \\ &=& \sum_{i=1}^{N_k} \bm{j}_{\mathit{cog,k}}^{(i)} \dot{\psi}_i \\
  \bm{j}_{\mathit{cog,k}}^{(i)} &=& \left\{\begin{array}{ll} \bm{a}_{\psi_i} \times (\bm{p}_{\mathit{cog,k}} - \bm{p}_{\psi_i}) & (1 \leq i \leq N_k) \\ \bm{0}_3 & (N_k+1 \leq i \leq N_{\mathit{drive\mathchar`-joint}}) \end{array}\right.
\end{eqnarray}
$\bm{a}_{\psi_i}, \bm{p}_{\psi_i} \in \mathbb{R}^3$は$i$番目の関節の回転軸ベクトルと位置である．
\it{gravity-link}よりもルート側にある関節の番号を$1,\cdots,N_k$，
\it{gravity-link}よりも末端側にある関節の番号を$N_k+1,\cdots,N_{\mathit{drive\mathchar`-joint}}$とする．
リンクの重心位置と関節角度の依存関係から，ヤコビ行列の右には次式のように零ベクトルが並ぶ．
\begin{eqnarray}
  \bm{J}_{\mathit{cog,k}}
  &=& \begin{pmatrix}\bm{j}_{\mathit{cog,k}}^{(1)} & \cdots & \bm{j}_{\mathit{cog,k}}^{(N_k)} & \bm{j}_{\mathit{cog,k}}^{(N_k+1)} & \cdots & \bm{j}_{\mathit{cog,k}}^{N_{\mathit{drive\mathchar`-joint}}}\end{pmatrix} \\
  &=& \begin{pmatrix}\bm{j}_{\mathit{cog,k}}^{(1)} & \cdots & \bm{j}_{\mathit{cog,k}}^{(N_k)} & \bm{0} & \cdots & \bm{0}\end{pmatrix} \label{eq:jacobian-for-gravity-torque} \label{eq:jacobian-for-gravity-torque}
\end{eqnarray}

return $\bm{J}_{\mathit{cog,k}} \in \mathbb{R}^{3 \times N_{\mathit{drive\mathchar`-joint}}}$
  "
  (let* ((jacobi
          (make-matrix 3 (length drive-joint-list)))
         (gravity-link-cog-world ;; [m]
          (scale 1e-3 (send gravity-link :centroid)))
         (drive-jnt)
         )
    (dotimes (i (length drive-joint-list))
      (setq drive-jnt (elt drive-joint-list i))
      (when (member (send drive-jnt :child-link) (send robot :link-list gravity-link))
        (let* ((jnt-axis
                (send drive-jnt :axis-vector))
               (jnt-pos ;; [m]
                (scale 1e-3 (send drive-jnt :pos)))
               (gravity-link-cog-local
                (v- gravity-link-cog-world jnt-pos))
               (jacobi-vec
                (cond ((derivedp drive-jnt linear-joint)
                       jnt-axis
                       )
                      ((derivedp drive-jnt rotational-joint)
                       (v* jnt-axis gravity-link-cog-local)
                       )
                      (t
                       (error "unsupported joint type: ~a~%" drive-jnt)
                       ))
                )
               )
          (setf (matrix-column jacobi i) jacobi-vec)
          )))
    jacobi
    ))

(defun get-gravity-torque
    (&key
     (robot)
     (drive-joint-list)
     (gravity-link-list)
     )
  "
ロボットのリンク自重によって生じる関節トルク$\bm{\tau}^{\mathit{grav}}$は，ロボットモーションP111式(3.3.22)より以下で得られる．
\begin{eqnarray}
  \bm{\tau}^{\mathit{grav}} = \left( \sum_{k=1}^{N_{\mathit{gravity\mathchar`-link}}} m_k \bm{J}_{\mathit{cog,k}}^T \right) \bm{g} \label{eq:gravity-torque}
\end{eqnarray}
$m_k$は$k$番目のリンクの質量である．

return $\bm{\tau}^{\mathit{grav}} \in \mathbb{R}^{N_{\mathit{drive\mathchar`-joint}}}$
  "
  (let* ((torque
          (make-array (length drive-joint-list) :element-type :float :initial-element 0))
         )
    (dolist (gravity-link gravity-link-list)
      (v+ torque
          (transform
           (transpose
            (get-link-jacobian-for-gravity-torque
             :robot robot
             :gravity-link gravity-link
             :drive-joint-list drive-joint-list
             ))
           (send gravity-link :mg-vec))
          torque)
      )
    torque
    ))

(defun get-gravity-torque-jacobian
    (&key
     (robot)
     (joint-list)
     (drive-joint-list)
     (gravity-link-list)
     )
  "
\eqref{eq:jacobian-for-gravity-torque}の$\bm{J}_{\mathit{cog,k}}$を以下のように分解して利用する．
\begin{eqnarray}
  \bm{J}_{\mathit{cog,k}}
  = \begin{pmatrix} \bm{j}_{\mathit{cog,x,k}}^T \\ \bm{j}_{\mathit{cog,y,k}}^T \\ \bm{j}_{\mathit{cog,z,k}}^T \end{pmatrix}
  = \begin{pmatrix} \left(\frac{\partial p_{\mathit{cog,x,k}}}{\partial \bm{\psi}}\right)^T \\ \left(\frac{\partial p_{\mathit{cog,y,k}}}{\partial \bm{\psi}}\right)^T \\ \left(\frac{\partial p_{\mathit{cog,z,k}}}{\partial \bm{\psi}}\right)^T \end{pmatrix}
\end{eqnarray}
これを\eqref{eq:gravity-torque}に代入すると，
\begin{eqnarray}
  \bm{\tau}^{\mathit{grav}} &=& \left( \sum_{k=1}^{N_{\mathit{gravity\mathchar`-link}}} m_k \bm{J}_{\mathit{cog,k}}^T \right) \bm{g} \\
  &=& \sum_{k=1}^{N_{\mathit{gravity\mathchar`-link}}} m_k \begin{pmatrix} \bm{j}_{\mathit{cog,x,k}} & \bm{j}_{\mathit{cog,y,k}} & \bm{j}_{\mathit{cog,z,k}} \end{pmatrix} \begin{pmatrix} 0 \\ 0 \\ g \end{pmatrix} \\
  &=& \sum_{k=1}^{N_{\mathit{gravity\mathchar`-link}}} m_k g \bm{j}_{\mathit{cog,z,k}} \\
  &=& \sum_{k=1}^{N_{\mathit{gravity\mathchar`-link}}} m_k g \frac{\partial p_{\mathit{cog,z,k}}}{\partial \bm{\psi}}
\end{eqnarray}

\it{joint-list}の関節角度ベクトルを$\bm{\theta} \in \mathbb{R}^{N_{\mathit{joint}}}$，
\it{drive-joint-list}の関節角度ベクトルを$\bm{\psi} \in \mathbb{R}^{N_{\mathit{drive\mathchar`-joint}}}$とする．
トルク勾配行列$\frac{\partial \bm{\tau}^{\mathit{grav}}}{\partial \bm{\theta}}$は次式で得られる．これは対称行列である．
\begin{eqnarray}
  \frac{\partial \bm{\tau}^{\mathit{grav}}}{\partial \bm{\theta}} &=& \sum_{k=1}^{N_{\mathit{gravity\mathchar`-link}}} m_k g \frac{\partial^2 p_{\mathit{cog,z,k}}}{\partial \bm{\psi} \partial \bm{\theta}} \\
  \frac{\partial^2 p_{\mathit{cog,z,k}}}{\partial \bm{\psi} \partial \bm{\theta}} &=& \left[ \frac{\partial^2 p_{\mathit{cog,z,k}}}{\partial \psi_i \partial \theta_j} \right]_{i=1,\cdots,N_{\mathit{drive\mathchar`-joint}},j=1,\cdots,N_{\mathit{joint}}}
\end{eqnarray}
つまり
\begin{eqnarray}
  \frac{\partial \bm{\tau}^{\mathit{grav}}}{\partial \bm{\theta}}
  &=&
  \left[ \sum_{k=1}^{N_{\mathit{gravity\mathchar`-link}}} m_k g \frac{\partial^2 p_{\mathit{cog,z,k}}}{\partial \psi_i \partial \theta_j} \right]_{i=1,\cdots,N_{\mathit{drive\mathchar`-joint}},j=1,\cdots,N_{\mathit{joint}}}
\end{eqnarray}

return $\frac{\partial \bm{\tau}^{\mathit{grav}}}{\partial \bm{\theta}} \in \mathbb{R}^{N_{\mathit{drive\mathchar`-joint}} \times N_{\mathit{joint}}}$
  "
  (let* ((jacobi
          (make-matrix (length drive-joint-list) (length joint-list)))
         (link-list-of-gravity-link)
         (drive-jnt)
         (jnt)
         (dependency-mode)
         )
    (dotimes (i (length drive-joint-list))
      (setq drive-jnt (elt drive-joint-list i))
      (dotimes (j (length joint-list))
        (setq jnt (elt joint-list j))
        ;; drive-jntとjntのどちらがルート側にあるか調べる
        (cond ((member (send drive-jnt :child-link) (send robot :link-list (send jnt :child-link)))
               (setq dependency-mode :drive-jnt-is-root)
               )
              ((member (send jnt :child-link) (send robot :link-list (send drive-jnt :child-link)))
               (setq dependency-mode :jnt-is-root)
               )
              (t
               (setq dependency-mode nil)
               ))
        ;; [root, jnt]間にdrive-jntがある or [root, drive-jnt]間にjntがある ときだけ処理する
        (when dependency-mode
          (dolist (gravity-link gravity-link-list)
            ;; gravity-linkがdrive-jntよりも末端側にある and gravity-linkがjntよりも末端側にある ときだけ処理する
            (setq link-list-of-gravity-link (send robot :link-list gravity-link))
            (when (and (member (send drive-jnt :child-link) link-list-of-gravity-link)
                       (member (send jnt :child-link) link-list-of-gravity-link))
              (let* ((drive-jnt-axis
                      (send drive-jnt :axis-vector))
                     (jnt-axis
                      (send jnt :axis-vector))
                     (hessian-vec
                      (cond ((and (derivedp drive-jnt rotational-joint)
                                  (derivedp jnt rotational-joint))
                             (cond ((equal dependency-mode :jnt-is-root)
                                    (v* jnt-axis (v* drive-jnt-axis (scale 1e-3 (v- (send gravity-link :centroid) (send drive-jnt :pos)))))
                                    )
                                   (t
                                    (v* drive-jnt-axis (v* jnt-axis (scale 1e-3 (v- (send gravity-link :centroid) (send jnt :pos)))))
                                    ))
                             )
                            ((and (derivedp drive-jnt linear-joint)
                                  (derivedp jnt linear-joint))
                             (float-vector 0 0 0)
                             )
                            ((and (derivedp drive-jnt linear-joint)
                                  (derivedp jnt rotational-joint))
                             (cond ((equal dependency-mode :drive-jnt-is-root)
                                    (float-vector 0 0 0) ;; linear-jointがルート側いあったら零ベクトル
                                    )
                                   (t
                                    (v* jnt-axis drive-jnt-axis)
                                    ))
                             )
                            ((and (derivedp drive-jnt rotational-joint)
                                  (derivedp jnt linear-joint))
                             (cond ((equal dependency-mode :jnt-is-root)
                                    (float-vector 0 0 0) ;; linear-jointがルート側いあったら零ベクトル
                                    )
                                   (t
                                    (v* drive-jnt-axis jnt-axis)
                                    ))
                             )
                            (t
                             (error "unsupported joint type pair: ~a ~a~%" drive-jnt jnt)
                             ))
                      )
                     )
                (setf (aref jacobi i j)
                      (+ (aref jacobi i j) (* (send gravity-link :mg) (elt hessian-vec 2))))
                ))))
        ))
    jacobi
    ))
