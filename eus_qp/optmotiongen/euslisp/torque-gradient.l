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
\it{contact-coords}の位置を$\bm{p}_{m} \in \mathbb{R}^3$，
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
 以下では，$\bm{p}^A_B$は$A$から$B$へ向かう位置ベクトルをワールド座標系で表記したものとする．
 $A$,$B$は，\it{drive-joint-list}の関節位置$\psi_i$，\it{joint-list}の関節位置$\theta_i$，\it{contact-coords}の位置$m$のいずれかを指す．
 \begin{eqnarray}
   \frac{\partial \bm{\tau}^{\mathit{cnt}}}{\partial \bm{\theta}}
   &=& \frac{\partial}{\partial \bm{\theta}} \sum_{m=1}^{N_{\mathit{cnt}}} \bm{J}_m^T \bm{w}_m \\
   &=&
   \frac{\partial}{\partial \bm{\theta}} \sum_{m=1}^{N_{\mathit{cnt}}}
   \begin{pmatrix}
     \bm{a}_{\psi_1} \times \bm{p}^{\psi_1}_m & \bm{a}_{\psi_2} \times \bm{p}^{\psi_2}_m & \cdots & \bm{a}_{\psi_{N_{drive\mathchar`-joint}}} \times \bm{p}^{\psi_{N_{drive\mathchar`-joint}}}_m \\
     \bm{a}_{\psi_1} & \bm{a}_{\psi_2} & \cdots & \bm{a}_{\psi_{N_{drive\mathchar`-joint}}}
   \end{pmatrix}^T
   \begin{pmatrix}
     \bm{f}_m \\ \bm{n}_m
   \end{pmatrix} \\
   &=&
   \sum_{m=1}^{N_{\mathit{cnt}}}
   \frac{\partial}{\partial \bm{\theta}}
   \begin{pmatrix}
     \bm{f}_m^T \left( \bm{a}_{\psi_1} \times \bm{p}^{\psi_1}_m \right) + \bm{n}_m^T \bm{a}_{\psi_1} \\
     \bm{f}_m^T \left( \bm{a}_{\psi_2} \times \bm{p}^{\psi_2}_m \right) + \bm{n}_m^T \bm{a}_{\psi_2} \\
     \vdots \\
     \bm{f}_m^T \left( \bm{a}_{\psi_{drive\mathchar`-joint}} \times \bm{p}^{\psi_{drive\mathchar`-joint}}_m \right) +
     \bm{n}_m^T \bm{a}_{\psi_{drive\mathchar`-joint}}
   \end{pmatrix} \\
   &=&
   \sum_{m=1}^{N_{\mathit{cnt}}}
   \left[
     \bm{f}_m^T \frac{\partial}{\partial \theta_j} \left( \bm{a}_{\psi_i} \times \bm{p}^{\psi_i}_m \right) + \bm{n}_m^T
     \frac{\partial}{\partial \theta_j} \bm{a}_{\psi_i} \right]_{i,j} \\
   &=&
   \sum_{m=1}^{N_{\mathit{cnt}}}
   \left[
     \bm{f}_m^T \left\{ \left( \frac{\partial}{\partial \theta_j} \bm{a}_{\psi_i} \right) \times \bm{p}^{\psi_i}_m +
     \bm{a}_{\psi_i} \times \left( \frac{\partial}{\partial \theta_j} \bm{p}_m - \frac{\partial}{\partial \theta_j}
     \bm{p}_{\psi_i} \right) \right\} + \bm{n}_m^T \frac{\partial}{\partial \theta_j} \bm{a}_{\psi_i}
     \right]_{i,j} \nonumber \\
   && \hspace{40mm} (i=1,2,\cdots,N_{drive\mathchar`-joint}, \ \ j=1,2,\cdots,N_{joint})
 \end{eqnarray}

 $\frac{\partial}{\partial \theta_j} \bm{a}_{\psi_i}, \frac{\partial}{\partial \theta_j} \bm{p}_{\psi_i}$は以下のように計算される．
 \begin{enumerate}
 \renewcommand{\labelenumi}{(\Alph{enumi})}
 \item 関節$\theta_j$が関節$\psi_i$よりもルートリンクに近いとき，もしくは関節$\theta_j$と関節$\psi_i$が同一のとき，
   \begin{enumerate}
   \renewcommand{\labelenumii}{(\Roman{enumii})}
     \item 関節$\theta_j$が回転関節のとき，回転系での基礎方程式から，
     \begin{eqnarray}
       \frac{\partial}{\partial \theta_j} \bm{a}_{\psi_i} &=& \bm{a}_{\theta_j} \times \bm{a}_{\psi_i} \\
       \frac{\partial}{\partial \theta_j} \bm{p}_{\psi_i} &=& \bm{a}_{\theta_j} \times \bm{p}^{\theta_j}_{\psi_i}
     \end{eqnarray}
     \item 関節$\theta_j$が直動関節のとき
     \begin{eqnarray}
       \frac{\partial}{\partial \theta_j} \bm{a}_{\psi_i} &=& \bm{0} \\
       \frac{\partial}{\partial \theta_j}  \bm{p}_{\psi_i} &=& \bm{a}_{\theta_j}
     \end{eqnarray}
   \end{enumerate}
 \item (A)でないとき，つまり\\ 関節$\psi_i$が関節$\theta_j$よりもルートリンクに近いとき，もしくは，ルートリンクから関節$\theta_j$までの間とルートリンクから関節$\psi_i$までの間に共通の関節が存在しないとき，関節$\theta_j$の変化は関節$\psi_i$の位置，回転軸のベクトルに影響を与えないため，
 \begin{eqnarray}
   \frac{\partial}{\partial \theta_j} \bm{a}_{\psi_i} &=& \bm{0} \\
   \frac{\partial}{\partial \theta_j}  \bm{p}_{\psi_i} &=& \bm{0}
 \end{eqnarray}
 \end{enumerate}

 $\frac{\partial}{\partial \theta_j} \bm{p}_m$は以下のように計算される．
 \begin{enumerate}
 \renewcommand{\labelenumi}{(\alph{enumi})}
 \item 関節$\theta_j$の変位が$\bm{p}_m$に影響を与えるとき(このパターンは\it{contact-target-coords}が仮想関節の先にが設置されている場合などに発生する)
   \begin{enumerate}
   \renewcommand{\labelenumii}{(\roman{enumii})}
     \item 関節$\theta_j$が回転関節のとき
     \begin{eqnarray}
       \frac{\partial}{\partial \theta_j} \bm{p}_m &=& \bm{a}_{\theta_j} \times \bm{p}^{\theta_j}_m
     \end{eqnarray}
     \item 関節$\theta_j$が直動関節のとき
     \begin{eqnarray}
       \frac{\partial}{\partial \theta_j} \bm{p}_m &=& \bm{a}_{\theta_j}
     \end{eqnarray}
   \end{enumerate}
 \item  (a)でないとき，つまり\\ 関節$\theta_j$の変位が$\bm{p}_m$に影響を与えないとき
 \begin{eqnarray}
   \frac{\partial}{\partial \theta_j} \bm{p}_m &=& \bm{0}
 \end{eqnarray}
 \end{enumerate}
 "
  (let* ((jacobi
          (make-matrix (length drive-joint-list) (length joint-list)))
         (drive-jnt)
         (jnt)
         (dependency-mode)
         (contact-coords)
         (wrench)
         (drive-jnt-axis-derivative)
         (drive-jnt-pos-derivative)
         (contact-pos-derivative)
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
               (setq dependency-mode :no-dependency)
               ))
        (dotimes (k (length contact-attention-coords-list))
          (setq contact-coords (elt contact-target-coords-list k))
          (setq wrench (elt wrench-list k))
          (when (member (send drive-jnt :child-link)
                        (send robot :link-list (send (elt contact-attention-coords-list k) :parent)))
            ;; calculate drive-jnt-axis-derivative and drive-jnt-pos-derivative
            (cond (;; case (A) jntがdrive-jntよりもルートリンクに近いとき
                   (equal dependency-mode :jnt-is-root)
                   (cond (;; case (A).(I)
                          (derivedp jnt rotational-joint)
                          (setq drive-jnt-axis-derivative (v* (send jnt :axis-vector) (send drive-jnt :axis-vector)))
                          (setq drive-jnt-pos-derivative (v* (send jnt :axis-vector) (scale 1e-3 (v- (send drive-jnt :pos) (send jnt :pos)))))
                          )
                         (;; case (A).(II)
                          t ;; (derivedp jnt linear-joint)
                          (setq drive-jnt-axis-derivative (float-vector 0 0 0))
                          (setq drive-jnt-pos-derivative (send jnt :axis-vector))
                          ))
                   )
                  (;; case (B) drive-jntがjntよりもルートリンクに近いとき
                   t
                   (setq drive-jnt-axis-derivative (float-vector 0 0 0))
                   (setq drive-jnt-pos-derivative (float-vector 0 0 0))
                   ))
            ;; calculate contact-pos-derivative
            (cond (;; case (a) jntの変位がcontact-target-coordsに影響を与えるとき
                   (and (derivedp contact-coords cascaded-coords)
                        (member (send jnt :child-link) (send robot :link-list (send contact-coords :parent))))
                   (cond (;; case (a).(i)
                          (derivedp jnt rotational-joint)
                          (setq contact-pos-derivative (v* (send jnt :axis-vector) (scale 1e-3 (v- (send contact-coords :worldpos) (send jnt :pos)))))
                          )
                         (;; case (b).(ii)
                          t ;; (derivedp jnt linear-joint)
                          (setq contact-pos-derivative (send jnt :axis-vector))
                          ))
                   )
                  (;; case (a) jntの変位がcontact-target-coordsに影響を与えないとき
                   t
                   (setq contact-pos-derivative (float-vector 0 0 0))
                   ))
            ;; set jacobian element
            (setf (aref jacobi i j)
                  (+ (aref jacobi i j)
                     (v. (subseq wrench 0 3)
                         (v+ (v* drive-jnt-axis-derivative (scale 1e-3 (v- (send contact-coords :worldpos) (send drive-jnt :pos))))
                             (v* (send drive-jnt :axis-vector) (v- contact-pos-derivative drive-jnt-pos-derivative))
                             ))
                     (v. (subseq wrench 3 6)
                         drive-jnt-axis-derivative
                         )))
            )
          )))
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
