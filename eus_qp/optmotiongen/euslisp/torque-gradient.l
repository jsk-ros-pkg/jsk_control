(load "./util.l")
(load "./extended-joint-link.l")


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
\it{drive-joint-list}の関節角度ベクトルを$\bm{\psi} \in \mathbb{R}^{(N_{\mathit{drive\mathchar`-joint}})}$として，
次式を満たすヤコビ行列$\bm{J}_m$を返す．
\begin{eqnarray}
  \bm{J}_m &=& \begin{pmatrix}\bm{j}_m^{(1)} & \bm{j}_m^{(2)} & \cdots & \bm{j}_m^{(N_{\mathit{drive\mathchar`-joint}})}\end{pmatrix} \label{eq:jacobian-for-contact-torque} \\
  \bm{j}_m^{(i)} &=& \left\{ \begin{array}{ll} \bm{\bar{j}}_m^{(i)} & \ \ 接触リンクがi番目の駆動関節変位\psi_iに依存している場合 \\ \bm{0} & \ \ {\rm otherwise} \ \ \end{array} \right.
\end{eqnarray}
$\bm{\bar{j}}_m^{(i)}$は基礎ヤコビ行列の列ベクトルで次式で表される．\\
$\psi_i$が回転関節の場合
\begin{eqnarray}
  \bm{\bar{j}}_m^{(i)} &=& \begin{pmatrix} \bm{a}_{\psi_i} \times (\bm{p}_m - \bm{p}_{\psi_i}) \\ \bm{a}_{\psi_i} \end{pmatrix}
\end{eqnarray}
$\psi_i$が直動関節の場合
\begin{eqnarray}
  \bm{\bar{j}}_m^{(i)} &=& \begin{pmatrix} \bm{a}_{\psi_i} \\ \bm{0} \end{pmatrix}
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
   \begin{pmatrix}\bm{j}_m^{(1)} & \bm{j}_m^{(2)} & \cdots & \bm{j}_m^{(N_{\mathit{drive\mathchar`-joint}})}\end{pmatrix}^T
   \bm{w}_m \\
   &=&
   \sum_{m=1}^{N_{\mathit{cnt}}}
   \frac{\partial}{\partial \bm{\theta}}
   \begin{pmatrix}
     \bm{w}_m^T \bm{j}_m^{(1)} \\
     \bm{w}_m^T \bm{j}_m^{(2)} \\
     \vdots \\
     \bm{w}_m^T \bm{j}_m^{(N_{\mathit{drive\mathchar`-joint}})}
   \end{pmatrix} \\
   &=&
   \sum_{m=1}^{N_{\mathit{cnt}}}
   \left[ \bm{w}_m^T \frac{\partial}{\partial \theta_j} \bm{j}_m^{(i)} \right]_{i,j} \ \ (i=1,2,\cdots,N_{drive\mathchar`-joint}, \ \ j=1,2,\cdots,N_{joint})
 \end{eqnarray}
 したがって，各接触力によるトルクのヤコビ行列の各要素は次式で得られる．\\
 $\psi_i$が回転関節の場合
 \begin{eqnarray}
   \bm{w}_m^T \frac{\partial}{\partial \theta_j} \bm{j}_m^{(i)}
   &=&
   \begin{pmatrix} \bm{f}_m \\ \bm{n}_m \end{pmatrix}^T \frac{\partial}{\partial \theta_j} \begin{pmatrix} \bm{a}_{\psi_i} \times \bm{p}^{\psi_i}_m \\ \bm{a}_{\psi_i} \end{pmatrix}
   \\
   &=&
   \bm{f}_m^T \frac{\partial}{\partial \theta_j} \left( \bm{a}_{\psi_i} \times \bm{p}^{\psi_i}_m \right)
   + \bm{n}_m^T \frac{\partial}{\partial \theta_j} \bm{a}_{\psi_i} \\
   &=&
   \bm{f}_m^T \left\{ \left( \frac{\partial}{\partial \theta_j} \bm{a}_{\psi_i} \right) \times \bm{p}^{\psi_i}_m +
   \bm{a}_{\psi_i} \times \left( \frac{\partial}{\partial \theta_j} \bm{p}_m - \frac{\partial}{\partial \theta_j}
   \bm{p}_{\psi_i} \right) \right\} + \bm{n}_m^T \frac{\partial}{\partial \theta_j} \bm{a}_{\psi_i}
 \end{eqnarray}
 $\psi_i$が直動関節の場合
 \begin{eqnarray}
   \bm{w}_m^T \frac{\partial}{\partial \theta_j} \bm{j}_m^{(i)}
   &=&
   \begin{pmatrix} \bm{f}_m \\ \bm{n}_m \end{pmatrix}^T \frac{\partial}{\partial \theta_j} \begin{pmatrix} \bm{a}_{\psi_i} \\ \bm{0} \end{pmatrix}
   \\
   &=&
   \bm{f}_m^T \frac{\partial}{\partial \theta_j} \bm{a}_{\psi_i}
 \end{eqnarray}

 $\frac{\partial}{\partial \theta_j} \bm{a}_{\psi_i}$(\it{drive-jnt-axis-derivative}), $\frac{\partial}{\partial \theta_j} \bm{p}_{\psi_i}$(\it{drive-jnt-pos-derivative})は以下のように計算される．
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

 $\frac{\partial}{\partial \theta_j} \bm{p}_m$(\it{contact-pos-derivative})は以下のように計算される．
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

return $\frac{\partial \bm{\tau}^{\mathit{cnt}}}{\partial \bm{\theta}} \in \mathbb{R}^{N_{\mathit{drive\mathchar`-joint}} \times N_{\mathit{joint}}}$
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
         (jacobi-element)
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
                         (;; case (a).(ii)
                          t ;; (derivedp jnt linear-joint)
                          (setq contact-pos-derivative (send jnt :axis-vector))
                          ))
                   )
                  (;; case (b) jntの変位がcontact-target-coordsに影響を与えないとき
                   t
                   (setq contact-pos-derivative (float-vector 0 0 0))
                   ))
            ;; set jacobian element
            (cond ((derivedp drive-jnt rotational-joint)
                   (setq jacobi-element
                         (+
                          (v. (subseq wrench 0 3)
                              (v+ (v* drive-jnt-axis-derivative (scale 1e-3 (v- (send contact-coords :worldpos) (send drive-jnt :pos))))
                                  (v* (send drive-jnt :axis-vector) (v- contact-pos-derivative drive-jnt-pos-derivative))
                                  ))
                          (v. (subseq wrench 3 6) drive-jnt-axis-derivative)
                          ))
                   )
                  (t ;; (derivedp drive-jnt linear-joint)
                   (setq jacobi-element
                         (v. (subseq wrench 0 3) drive-jnt-axis-derivative))
                   ))
            (setf (aref jacobi i j) (+ (aref jacobi i j) jacobi-element))
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
  \bm{\dot{p}}_{\mathit{cog,k}} &=& \bm{J}_{\mathit{cog,k}} \bm{\dot{\psi}} = \sum_{i=1}^{N_k} \bm{j}_{\mathit{cog,k}}^{(i)} \dot{\psi}_i
\end{eqnarray}
\begin{eqnarray}
  \bm{J}_{\mathit{cog,k}} &=& \begin{pmatrix}\bm{j}_{\mathit{cog,k}}^{(1)} & \bm{j}_{\mathit{cog,k}}^{(2)} & \cdots & \bm{j}_{\mathit{cog,k}}^{(N_{\mathit{drive\mathchar`-joint}})}\end{pmatrix} \label{eq:jacobian-for-gravity-torque} \\
  \bm{j}_{\mathit{cog,k}}^{(i)} &=& \left\{ \begin{array}{ll} \bm{\bar{j}}_{\mathit{cog,k}}^{(i)} & \ \ \mathit{gravity\mathchar`-link}がi番目の駆動関節変位\psi_iに依存している場合 \\
  \bm{0} & \ \ {\rm otherwise} \ \ \end{array} \right.
\end{eqnarray}
$\bm{\bar{j}}_{\mathit{cog,k}}^{(i)}$は基礎ヤコビ行列の列ベクトルで次式で表される．\\
$\psi_i$が回転関節の場合
\begin{eqnarray}
  \bm{\bar{j}}_{\mathit{cog,k}}^{(i)} &=& \bm{a}_{\psi_i} \times (\bm{p}_{\mathit{cog,k}} - \bm{p}_{\psi_i})
\end{eqnarray}
$\psi_i$が直動関節の場合
\begin{eqnarray}
  \bm{\bar{j}}_{\mathit{cog,k}}^{(i)} &=& \bm{a}_{\psi_i}
\end{eqnarray}
$\bm{a}_{\psi_i}, \bm{p}_{\psi_i} \in \mathbb{R}^3$は$i$番目の関節の回転軸ベクトルと位置である．

return $\bm{J}_{\mathit{cog,k}} \in \mathbb{R}^{3 \times N_{\mathit{drive\mathchar`-joint}}}$
  "
  (let* ((jacobi
          (make-matrix 3 (length drive-joint-list)))
         (gravity-link-cog-world ;; [m]
          (scale 1e-3 (send gravity-link :centroid-with-fixed-child-links)))
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
                      (t ;; (derivedp drive-jnt rotational-joint)
                       (v* jnt-axis gravity-link-cog-local)
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
  (set-mass-property-with-fixed-child-links robot)
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
\begin{eqnarray}
  \frac{\partial \bm{\tau}^{\mathit{grav}}}{\partial \bm{\theta}}
  &=& \frac{\partial}{\partial \theta_j} \sum_{k=1}^{N_{\mathit{gravity\mathchar`-link}}} \bm{J}_{\mathit{cog,k}}^T m_k \bm{g} \\
  &=& \frac{\partial}{\partial \theta_j} \sum_{k=1}^{N_{\mathit{gravity\mathchar`-link}}} \begin{pmatrix}\bm{j}_{\mathit{cog,k}}^{(1)} & \bm{j}_{\mathit{cog,k}}^{(2)} & \cdots & \bm{j}_{\mathit{cog,k}}^{(N_{\mathit{drive\mathchar`-joint}})}\end{pmatrix}^T m_k \bm{g} \\
  &=& \sum_{k=1}^{N_{\mathit{gravity\mathchar`-link}}} \frac{\partial}{\partial \theta_j} \begin{pmatrix}m_k \bm{g}^T \bm{j}_{\mathit{cog,k}}^{(1)} \\ m_k \bm{g}^T \bm{j}_{\mathit{cog,k}}^{(2)} \\ \vdots \\ m_k \bm{g}^T\bm{j}_{\mathit{cog,k}}^{(N_{\mathit{drive\mathchar`-joint}})}\end{pmatrix} \\
   &=&
   \sum_{k=1}^{N_{\mathit{gravity\mathchar`-link}}}
   \left[ m_k \bm{g}^T \frac{\partial}{\partial \theta_j} \bm{j}_{\mathit{cog,k}}^{(i)} \right]_{i,j} \ \ (i=1,2,\cdots,N_{drive\mathchar`-joint}, \ \ j=1,2,\cdots,N_{joint})
\end{eqnarray}
 したがって，各リンクの重力によるトルクのヤコビ行列の各要素は次式で得られる．\\
 $k$番目の\it{gravity-link-list}が$i$番目の駆動関節変位$\psi_i$に依存していない場合
 \begin{eqnarray}
   m_k \bm{g}^T \frac{\partial}{\partial \theta_j} \bm{j}_{\mathit{cog,k}}^{(i)} &=& 0
 \end{eqnarray}
 $k$番目の\it{gravity-link-list}が$i$番目の駆動関節変位$\psi_i$に依存していて，$\psi_i$が回転関節の場合
 \begin{eqnarray}
   m_k \bm{g}^T \frac{\partial}{\partial \theta_j} \bm{j}_{\mathit{cog,k}}^{(i)}
   &=&
   m_k \bm{g}^T \frac{\partial}{\partial \theta_j} \left( \bm{a}_{\psi_i} \times \bm{p}^{\psi_i}_{\mathit{cog,k}} \right) \\
   &=&
   m_k \bm{g}^T \left\{ \left( \frac{\partial}{\partial \theta_j} \bm{a}_{\psi_i} \right) \times \bm{p}^{\psi_i}_{\mathit{cog,k}} +
   \bm{a}_{\psi_i} \times \left( \frac{\partial}{\partial \theta_j} \bm{p}_{\mathit{cog,k}} - \frac{\partial}{\partial \theta_j}
   \bm{p}_{\psi_i} \right) \right\}
 \end{eqnarray}
 $k$番目の\it{gravity-link-list}が$i$番目の駆動関節変位$\psi_i$に依存していて，$\psi_i$が直動関節の場合
 \begin{eqnarray}
   m_k \bm{g}^T \frac{\partial}{\partial \theta_j} \bm{j}_{\mathit{cog,k}}^{(i)}
   &=&
   m_k \bm{g}^T \frac{\partial}{\partial \theta_j} \bm{a}_{\psi_i}
 \end{eqnarray}

 $\frac{\partial}{\partial \theta_j} \bm{a}_{\psi_i}$(\it{drive-jnt-axis-derivative}), $\frac{\partial}{\partial \theta_j} \bm{p}_{\psi_i}$(\it{drive-jnt-pos-derivative})は以下のように計算される．
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

 $\frac{\partial}{\partial \theta_j} \bm{p}_{\mathit{cog,k}}$(\it{centroid-derivative})は以下のように計算される．
 \begin{enumerate}
 \renewcommand{\labelenumi}{(\alph{enumi})}
 \item $k$番目の\it{gravity-link-list}が$j$番目の関節変位$\theta_j$に依存しているとき
   \begin{enumerate}
   \renewcommand{\labelenumii}{(\roman{enumii})}
     \item 関節$\theta_j$が回転関節のとき
     \begin{eqnarray}
       \frac{\partial}{\partial \theta_j} \bm{p}_{\mathit{cog,k}} &=& \bm{a}_{\theta_j} \times \bm{p}^{\theta_j}_{\mathit{cog,k}}
     \end{eqnarray}
     \item 関節$\theta_j$が直動関節のとき
     \begin{eqnarray}
       \frac{\partial}{\partial \theta_j} \bm{p}_{\mathit{cog,k}} &=& \bm{a}_{\theta_j}
     \end{eqnarray}
   \end{enumerate}
 \item  (a)でないとき，つまり\\ $k$番目の\it{gravity-link-list}が$j$番目の関節変位$\theta_j$に依存していないとき
 \begin{eqnarray}
   \frac{\partial}{\partial \theta_j} \bm{p}_{\mathit{cog,k}} &=& \bm{0}
 \end{eqnarray}
 \end{enumerate}

return $\frac{\partial \bm{\tau}^{\mathit{grav}}}{\partial \bm{\theta}} \in \mathbb{R}^{N_{\mathit{drive\mathchar`-joint}} \times N_{\mathit{joint}}}$
  "
  (set-mass-property-with-fixed-child-links robot)
  (let* ((jacobi
          (make-matrix (length drive-joint-list) (length joint-list)))
         (link-list-of-gravity-link)
         (drive-jnt)
         (jnt)
         (dependency-mode)
         (drive-jnt-axis-derivative)
         (drive-jnt-pos-derivative)
         (centroid-derivative)
         (jacobi-element)
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
        (dolist (gravity-link gravity-link-list)
          (setq link-list-of-gravity-link (send robot :link-list gravity-link))
          ;; k番目のgravity-link-listがi番目の駆動関節に依存しているとき
          (when (member (send drive-jnt :child-link) link-list-of-gravity-link)
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
            ;; calculate centroid-derivative
            (cond (;; case (a) gravity-linkがjntに依存しているとき
                   (member (send jnt :child-link) link-list-of-gravity-link)
                   (cond (;; case (a).(i)
                          (derivedp jnt rotational-joint)
                          (setq centroid-derivative
                                (v* (send jnt :axis-vector)
                                    (scale 1e-3 (v- (send gravity-link :centroid-with-fixed-child-links) (send jnt :pos)))))
                          )
                         (;; case (a).(ii)
                          t ;; (derivedp jnt linear-joint)
                          (setq centroid-derivative (send jnt :axis-vector))
                          ))
                   )
                  (;; case (b) gravity-linkがjntに依存していないとき
                   t
                   (setq centroid-derivative (float-vector 0 0 0))
                   ))
            ;; set jacobian element
            (cond ((derivedp drive-jnt rotational-joint)
                   (setq jacobi-element
                         (v. (send gravity-link :mg-vec)
                             (v+ (v* drive-jnt-axis-derivative
                                     (scale 1e-3 (v- (send gravity-link :centroid-with-fixed-child-links) (send drive-jnt :pos))))
                                 (v* (send drive-jnt :axis-vector) (v- centroid-derivative drive-jnt-pos-derivative))
                                 )))
                   )
                  (t ;; (derivedp drive-jnt linear-joint)
                   (setq jacobi-element
                         (v. (send gravity-link :mg-vec) drive-jnt-axis-derivative))
                   ))
            (setf (aref jacobi i j) (+ (aref jacobi i j) jacobi-element))
            ))
        ))
    jacobi
    ))
