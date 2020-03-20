(require (format nil "~A/prioritized-inverse-kinematics/euslisp/util" (ros::resolve-ros-path "package://eus_qp")))
(require (format nil "~A/prioritized-inverse-kinematics/euslisp/inverse-kinematics-variable" (ros::resolve-ros-path "package://eus_qp")))
(require (format nil "~A/euslisp/contact-optimization" (ros::resolve-ros-path "package://eus_qp")))
(require (format nil "~A/euslisp/eus-cddlib-compiled.l" (ros::resolve-ros-path "package://eus_cddlib")))

;;;;;;;;;;;;;;;;;;;;;;;;;
;; inverse-kinematics-task
;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass inverse-kinematics-task
  :super propertied-object
  :slots ((A "$\mathbf{A}$")
          (b "$\bm{b}$")
          (C "$\mathbf{C}$")
          (dl "$\mathbf{d}_l$")
          (du "$\mathbf{d}_u$")
          (Wa "$\mathbf{W}_A$")
          (Wc "$\mathbf{W}_C$")
          (Asparce "$\mathbf{A}_{sparce}$, used for sparce matrix calculation")
          (Csparce "$\mathbf{C}_{sparce}$, used for sparce matrix calculation")
          (equality-rows "size of row of $\mathbf{A}$")
          (inequality-rows "size of row of $\mathbf{C}$")
          (cols "size of column of $\mathbf{A}$ (constant)")
          )
  :documentation
  "
\ \ 各タスクを表すクラス．\\
\it{prioritized-inverse-kinematics}中で次の問題に変換される．
\begin{eqnarray}
  \min_{\bm{x},\bm{\omega}} &\ & (\mathbf{A}\bm{x} - \bm{b})^\mathrm{T} \mathbf{W}_A (\mathbf{A}\bm{x} - \bm{b}) + \bm{\omega}^\mathrm{T} \mathbf{W}_C \bm{\omega} \nonumber\\
  \rm{subject\ to} &\ & \bm{d}_l \leqq \mathbf{C}\bm{x} + \bm{\omega} \leqq \bm{d}_u \nonumber
\end{eqnarray}
  "
  )

(defmethod inverse-kinematics-task
  (:init
   ()
   "
Initialize instance
   "
   (setq equality-rows 0)
   (setq inequality-rows 0)
   (setq Wa (make-matrix equality-rows equality-rows))
   (setq Wc (make-matrix inequality-rows inequality-rows))
   t
   )
  (:initialize
   (variable-length variables)
   "
called at the start of prioritized-inverse-kinematics
   "
   (setq cols variable-length)
   (setq A (make-matrix equality-rows cols))
   (setq Asparce (make-matrix equality-rows cols))
   (setq b (instantiate float-vector equality-rows))
   (setq C (make-matrix inequality-rows cols))
   (setq Csparce (make-matrix inequality-rows cols))
   (setq du (instantiate float-vector inequality-rows))
   (setq dl (instantiate float-vector inequality-rows))
   t)
  (:update
   ()
   "
 called at the start of each iteration.\\
update A, b , Wa, Asparce, equality-rows, C, dl, du , Wc, Csparce, inequality-rows
   "
   t)
  (:is-satisfied
   (&optional force-check)
   "
終了判定に用いる．
   "
   t)
  (:draw
   ()
   "
debug view
   "
   t)
  (:debug
   ()
   "
debug message
   "
   t)
  (:A () "return $\mathbf{A}$" A)
  (:b () "return $\bm{b}$" b)
  (:Wa () "return $\mathbf{W}_A$" Wa)
  (:Asparce () "return $\mathbf{A}_{sparce}$" Asparce)
  (:C () "return $\mathbf{C}$" C)
  (:dl () "return $\bm{d}_l$" dl)
  (:du () "return $\bm{d}_u$" du)
  (:Wc () "return $\mathbf{W}_C$" Wc)
  (:Csparce () "return $\mathbf{C}_{sparce}$" Csparce)
  (:additional-variables () "return list of additional variable" nil)
  (:additional-task0s () "return list of additional task0" nil)
  )


;;;;;;;;;;;;;;;;;;;;;;;;;
;; minmax-angle-task
;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass minmax-angle-task
  :super inverse-kinematics-task
  :slots ((j ", target joint")
          (max-angle)
          (min-angle)
          (target-variable ", inverse-kinematics-variable corresponding to target joint")
          (check "終了判定を行うか否か")
          (check-margin "終了判定時のマージン")
          )
  :documentation
  "
\ \ \it{joint}の角度上下限制約を表現するクラス．\\
\it{joint}の角度上下限を$\bm{\theta}_{max}$，$\bm{\theta}_{min}$，現在の角度を$\bm{\theta}$とすると，\it{prioritized-inverse-kinematics}中で次の問題に変換される．
\begin{eqnarray}
  \min_{\bm{x},\bm{\omega}} &\ & \bm{\omega}^\mathrm{T} W \bm{\omega} \nonumber\\
  \rm{subject\ to} &\ & \bm{\theta}_{min} \leqq \bm{\theta} + \bm{x} + \bm{\omega} \leqq \bm{\theta}_{max}
\end{eqnarray}
$\bm{\theta}$の単位は\rm{m}，\rm{rad}である．
"
  )

(defmethod minmax-angle-task
  (:init
   (_joint
    &key
    ((:W _W) 1.0)
    ((:max-angle _max-angle))
    ((:min-angle _min-angle))
    ((:check _check) t)
    ((:check-margin _check-margin))
    )
   "
\begin{itemize}
  \item joint\\
\it{joint}クラス.上下限を考える対象の関節である．仮想関節の場合は，子の\it{cascaded-link}クラス，それがなければ\it{bodyset-link}クラスを与える．
  \item W\\
\it{float}クラスの場合$\mathbf{W}$はその値を対角成分に並べた行列になる\\
\it{vector}クラスの場合$\mathbf{W}$は各値を対角成分に並べた行列になる\\
\it{matrix}クラスの場合$\mathbf{W}$としてそのまま使用される．
  \item max-angle, min-angle\\
\it{float}または\it{float-vector}クラス．関節上下限を表す．単位は\rm{mm}，\rm{deg}．デフォルトは(send joint :max-angle) (send joint :min-angle)．
  \item check : t or nil\\
終了判定を行うか否か
  \item check-margin\\
\it{float}クラス．終了判定時にこの値以下の侵犯を許容する．デフォルトは1e-5だが，sphere関節は1e-3．単位は\rm{m},\rm{rad}
\end{itemize}
   "
   ;;for virtual joint
   (when (or (derivedp _joint cascaded-link)
             (derivedp _joint bodyset-link))
     (setq j _joint)
     (setq Wc _W)
     (setq max-angle _max-angle)
     (setq min-angle _min-angle)
     (setq check _check)
     (setq check-margin _check-margin)
     (return-from :init t))
   (setq j _joint)
   (setq equality-rows 0)
   (setq inequality-rows (send j :joint-dof))
   (setq max-angle _max-angle)
   (setq min-angle _min-angle)
   (unless max-angle
     (setq max-angle (send j :max-angle)))
   (unless min-angle
     (setq min-angle (send j :min-angle)))
   (setq check _check)
   (setq check-margin (cond
                        ((numberp _check-margin)
                         (fill (instantiate float-vector inequality-rows) _check-margin))
                        ((vectorp _check-margin)
                         (unless (= (length _check-margin) inequality-rows)
                           (error "length of check-margin (~A) should be ~A" (length _check-margin) inequality-rows))
                         _check-margin)
                        (t
                         (cond ;;sphere jointは線形近似誤差の影響を受けやすい
                           ((derivedp j 6dof-joint) (float-vector 1e-5 1e-5 1e-5 1e-3 1e-3 1e-3))
                           ((derivedp j sphere-joint) (float-vector 1e-3 1e-3 1e-3))
                           (t (fill (instantiate float-vector inequality-rows) 1e-5))))))
   (unless (or (and (numberp max-angle) (= 1 inequality-rows)) (= (length max-angle) inequality-rows))
     (error "length of max-angle (~A) should be ~A" (if (numberp max-angle) 1 (length max-angle)) inequality-rows))
   (unless (or (and (numberp min-angle) (= 1 inequality-rows)) (= (length min-angle) inequality-rows))
     (error "length of min-angle (~A) should be ~A" (if (numberp min-angle) 1 (length min-angle)) inequality-rows))
   (cond
     ((numberp _W) (setq Wc (scale-matrix _W (unit-matrix inequality-rows))))
     ((vectorp _W) (if (= (length _W) inequality-rows)
                       (setq Wc (diagonal _W))
                       (error "length of W (~A) should be ~A~%" (length _W) inequality-rows)))
     ((matrixp _W) (if (and (= (array-dimension _W 0) inequality-rows) (= (array-dimension _W 1) inequality-rows))
                            (setq Wc _W)
                            (error "size of W ~A should be ~A" (array-dimensions _W) inequality-rows)))
     )
   (setq Wa (make-matrix equality-rows equality-rows))
   t
   )
  (:initialize
   (variable-length variables)
   "
探索変数\it{variables}のうち，該当\it{joint}に影響を与える成分を調べ，記録する．0または1個の\it{variables}が発見されなければならない．
   "
   ;;for virtual joint
   (when (or (derivedp j cascaded-link)
             (derivedp j bodyset-link))
     (send self :init
                 (cond
                   ((derivedp j cascaded-link) (send (car (send j :links)) :joint))
                   ((derivedp j bodyset-link) (send j :joint)))
                 :W Wc
                 :max-angle max-angle
                 :min-angle min-angle
                 :check check
                 :check-margin check-margin))

   (send-super :initialize variable-length variables)
   (dolist (variable variables)
     (when (and (derivedp variable joint-variable)
                (eq (send variable :joint) j))
       (setq target-variable variable)
       )
     )
   (when target-variable
     (copy-block-matrix C (unit-matrix (send target-variable :dim)) 0 (send target-variable :index))
     (copy-block-matrix Csparce (unit-matrix (send target-variable :dim)) 0 (send target-variable :index))
     )
   t
   )
  (:update
   ()
   "
現在の関節角度に応じて$\bm{d}_l$，$\bm{d}_u$を更新する
   "
   (when target-variable
     (cond
       ((numberp (send j :joint-angle))
        (setelt dl 0 (send j :angle-to-speed (- min-angle (send j :joint-angle))))
        (setelt du 0 (send j :angle-to-speed (- max-angle (send j :joint-angle)))))
       (t
        (copy-block-vec dl (send j :angle-to-speed (v- min-angle (send j :joint-angle))))
        (copy-block-vec du (send j :angle-to-speed (v- max-angle (send j :joint-angle)))))
       )
     )
   t
   )
  (:is-satisfied
   (&optional force-check)
   "
関節角度上下限を満足しているかどうかを判定
   "
   (if (or check force-check)
       (and
        (every #'(lambda (v) (> v 0)) (v+ du check-margin))
        (every #'(lambda (v) (< v 0)) (v- dl check-margin)))
       t)
   )
  (:debug
   ()
   "
現在の関節角度と上下限を表示
   "
   (warning-message (cond
                      ((send self :is-satisfied t) 7);;white
                      (check 1) ;;red
                      (t 3)) ;;yellow
                    "    minmax-angle-task: ~A ~A < ~A < ~A [mm, deg]~%" (send j :name) min-angle (send j :joint-angle) max-angle)
   t)
  )



;;;;;;;;;;;;;;;;;;;;;;;;;
;; joint-velocity-task
;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass joint-velocity-task
  :super inverse-kinematics-task
  :slots ((j ", target joint")
          (max-vel "$\dot{\bm{\theta}}_{max}$[m/s][rad/s]")
          (min-vel "$\dot{\bm{\theta}}_{min}$[m/s][rad/s]")
          (periodic-time "$\Delta t$[s], 一回の反復計算の周期")
          (target-variable ", inverse-kinematics-variable corresponding to target joint")
          )
  :documentation
  "
\ \ \it{joint}の角速度上下限制約を表現するクラス．\\
\it{joint}の角速度上下限を$\dot{\bm{\theta}}_{max}$，$\dot{\bm{\theta}}_{min}$，一回の反復計算の周期を$\Delta t$とすると，\it{prioritized-inverse-kinematics}中で次の問題に変換される．
\begin{eqnarray}
  \min_{\bm{x},\bm{\omega}} &\ & \bm{\omega}^\mathrm{T} W \bm{\omega} \nonumber\\
  \rm{subject\ to} &\ & \dot{\bm{\theta}}_{min} * \Delta t \leqq \bm{x} + \bm{\omega} \leqq \dot{\bm{\theta}}_{max} * \Delta t
\end{eqnarray}
$\dot{\bm{\theta}}$の単位は\rm{m/s}，\rm{rad/s}である．
"
  )

(defmethod joint-velocity-task
  (:init
   (_joint
    &key
    ((:W _W) 1.0)
    ((:max-vel _max-vel))
    ((:min-vel _min-vel))
    ((:periodic-time _periodic-time) 0.05)
    )
   "
\begin{itemize}
  \item joint\\
\it{joint}クラス.上下限を考える対象の関節である．仮想関節の場合は，子の\it{cascaded-link}クラス，それがなければ\it{bodyset-link}クラスを与える．
  \item W\\
\it{float}クラスの場合$\mathbf{W}$はその値を対角成分に並べた行列になる\\
\it{vector}クラスの場合$\mathbf{W}$は各値を対角成分に並べた行列になる\\
\it{matrix}クラスの場合$\mathbf{W}$としてそのまま使用される．
  \item max-vel, min-vel\\
\it{float}または\it{float-vector}クラス．関節速度上下限を表す．単位は\rm{m/s}，\rm{rad/s}．デフォルトは(send joint :max-joint-velocity)．
  \item periodic-time\\
\it{float}クラス一回の反復計算の周期．
\end{itemize}
   "
   ;;for virtual joint
   (when (or (derivedp _joint cascaded-link)
             (derivedp _joint bodyset-link))
     (setq j _joint)
     (setq Wc _W)
     (setq max-vel _max-vel)
     (setq min-vel _min-vel)
     (setq periodic-time _periodic-time)
     (return-from :init t))
   (setq j _joint)
   (setq periodic-time _periodic-time)
   (setq equality-rows 0)
   (setq inequality-rows (send j :joint-dof))
   (setq max-vel _max-vel)
   (setq min-vel _min-vel)
   (unless max-vel
     (setq max-vel (send j :max-joint-velocity)))
   (unless min-vel
     (setq min-vel (send j :max-joint-velocity))
     (cond ((numberp min-vel) (setq min-vel (- min-vel)))
           (t (setq min-vel (v- min-vel)))))
   (unless (or (and (numberp max-vel) (= 1 inequality-rows)) (= (length max-vel) inequality-rows))
     (error "length of max-vel (~A) should be ~A" (if (numberp max-vel) 1 (length max-vel)) inequality-rows))
   (unless (or (and (numberp min-vel) (= 1 inequality-rows)) (= (length min-vel) inequality-rows))
     (error "length of min-vel (~A) should be ~A" (if (numberp min-vel) 1 (length min-vel)) inequality-rows))
   (cond
     ((numberp _W) (setq Wc (scale-matrix _W (unit-matrix inequality-rows))))
     ((vectorp _W) (if (= (length _W) inequality-rows)
                       (setq Wc (diagonal _W))
                       (error "length of W (~A) should be ~A~%" (length _W) inequality-rows)))
     ((matrixp _W) (if (and (= (array-dimension _W 0) inequality-rows) (= (array-dimension _W 1) inequality-rows))
                            (setq Wc _W)
                            (error "size of W ~A should be ~A" (array-dimensions _W) inequality-rows)))
     )
   (setq Wa (make-matrix equality-rows equality-rows))
   t
   )
  (:initialize
   (variable-length variables)
   "
探索変数\it{variables}のうち，該当\it{joint}に影響を与える成分を調べ，記録する．0または1個の\it{variables}が発見されなければならない．
   "
   ;;for virtual joint
   (when (or (derivedp j cascaded-link)
             (derivedp j bodyset-link))
     (send self :init
                 (cond
                   ((derivedp j cascaded-link) (send (car (send j :links)) :joint))
                   ((derivedp j bodyset-link) (send j :joint)))
                 :W Wc
                 :max-vel max-vel
                 :min-vel min-vel
                 :periodic-time periodic-time))

   (send-super :initialize variable-length variables)
   (dolist (variable variables)
     (when (and (derivedp variable joint-variable)
                (eq (send variable :joint) j))
       (setq target-variable variable)
       )
     )
   (when target-variable
     (copy-block-matrix C (unit-matrix (send target-variable :dim)) 0 (send target-variable :index))
     (copy-block-matrix Csparce (unit-matrix (send target-variable :dim)) 0 (send target-variable :index))
     )
   t
   )
  (:update
   ()
   "
現在の関節角度に応じて$\bm{d}_l$，$\bm{d}_u$を更新する
   "
   (when target-variable
     (cond
       ((numberp (send j :joint-angle))
        (setelt dl 0 (* min-vel periodic-time))
        (setelt du 0 (* max-vel periodic-time)))
       (t
        (copy-block-vec dl (scale periodic-time min-vel))
        (copy-block-vec du (scale periodic-time max-vel)))
       )
     )
   t
   )
  (:debug
   ()
   "
現在の速度上下限を表示
   "
   (warning-message
    7
    "    joint-velocity-task: ~A [~A ~A] [m, rad]~%"
    (send j :name)
    (cond
      ((numberp (send j :joint-angle)) (* min-vel periodic-time))
      (t (scale periodic-time min-vel)))
    (cond
      ((numberp (send j :joint-angle)) (* max-vel periodic-time))
      (t (scale periodic-time max-vel))))
   t)
  )



;;;;;;;;;;;;;;;;;;;;;;;;;
;; joint-angle-task
;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass joint-angle-task
  :super inverse-kinematics-task
  :slots ((j1 ", target joint 1")
          (j2 ", target joint 2")
          (target-variable1 ", inverse-kinematics-variable corresponding to target joint 1")
          (target-variable2 ", inverse-kinematics-variable corresponding to target joint 2")
          (check "終了判定を行うか否か")
          (check-margin "終了判定時のマージン")
          )
  :documentation
  "
\ \ 2つの関節角度を一致させる制約を表現するクラス．\\
2つの関節角度の誤差(joint2 - joint1)を$\bm{e}$とおき，そのヤコビアンを$\mathbf{J}$とすると，\it{prioritized-inverse-kinematics}中で次の問題に変換される．
\begin{eqnarray}
  \min_{\bm{x}} &\ & (\mathbf{J} \bm{x} + \bm{e})^\mathrm{T} \mathbf{W} (\mathbf{J} \bm{x} + \bm{e}) \nonumber
\end{eqnarray}
$\bm{e}$の単位は\rm{m}，\rm{rad}である．
"
  )

(defmethod joint-angle-task
  (:init
   (_joint1
    _joint2
    &key
    ((:W _W) 1.0)
    ((:check _check) t)
    ((:check-margin _check-margin))
    )
   "
\begin{itemize}
  \item joint1, joint2\\
\it{joint}クラスまたは\it{float},\it{float-vector}クラス(\rm{mm}，\rm{deg}).一致させる2つの関節角度を表す．仮想関節の場合は，子の\it{cascaded-link}クラス，それがなければ\it{bodyset-link}クラスを与える．
  \item W\\
\it{float}クラスの場合$\mathbf{W}$はその値を対角成分に並べた行列になる\\
\it{vector}クラスの場合$\mathbf{W}$は各値を対角成分に並べた行列になる\\
\it{matrix}クラスの場合$\mathbf{W}$としてそのまま使用される．
  \item check : t or nil\\
終了判定を行うか否か
  \item check-margin\\
\it{float}クラス．終了判定時にこの値以下の侵犯を許容する．デフォルトは1e-5だが，sphere関節は1e-3．単位は\rm{m},\rm{rad}
\end{itemize}
   "
   ;;for virtual joint
   (when (or (derivedp _joint1 cascaded-link)
             (derivedp _joint1 bodyset-link)
             (derivedp _joint2 cascaded-link)
             (derivedp _joint2 bodyset-link))
     (setq j1 _joint1)
     (setq j2 _joint2)
     (setq Wa _W)
     (return-from :init t))
   (setq j1 _joint1)
   (setq j2 _joint2)
   (unless (= (cond ((numberp j1) 1)
                    ((vectorp j1) (length j1))
                    (t (send j1 :joint-dof)))
              (cond ((numberp j2) 1)
                    ((vectorp j2) (length j1))
                    (t (send j2 :joint-dof))))
     (error "joint dof should be same ~A ~A" j1 j2))
   (setq equality-rows (cond ((numberp j1) 1)
                             ((vectorp j1) (length j1))
                             (t (send j1 :joint-dof))))
   (setq inequality-rows 0)
   (cond
     ((numberp _W) (setq Wa (scale-matrix _W (unit-matrix equality-rows))))
     ((vectorp _W) (if (= (length _W) equality-rows)
                       (setq Wa (diagonal _W))
                       (error "length of W (~A) should be ~A~%" (length _W) equality-rows)))
     ((matrixp _W) (if (and (= (array-dimension _W 0) equality-rows) (= (array-dimension _W 1) equality-rows))
                            (setq Wa _W)
                            (error "size of W ~A should be ~A" (array-dimensions _W) equality-rows)))
     )
   (setq Wc (make-matrix inequality-rows inequality-rows))
   (setq check _check)
   (setq check-margin (cond
                        ((numberp _check-margin)
                         (fill (instantiate float-vector equality-rows) _check-margin))
                        ((vectorp _check-margin)
                         (unless (= (length _check-margin) equality-rows)
                           (error "length of check-margin (~A) should be ~A" (length _check-margin) equality-rows))
                         _check-margin)
                        (t
                         (let ((tmp-j (cond ((derivedp j1 joint) j1)
                                            ((derivedp j2 joint) j2)
                                            (t nil))))
                           (if tmp-j
                               (cond ;;sphere jointは線形近似誤差の影響を受けやすい
                                 ((derivedp tmp-j 6dof-joint) (float-vector 1e-5 1e-5 1e-5 1e-3 1e-3 1e-3))
                                 ((derivedp tmp-j sphere-joint) (float-vector 1e-3 1e-3 1e-3))
                                 (t (fill (instantiate float-vector equality-rows) 1e-5)))
                               (fill (instantiate float-vector equality-rows) 1e-5))))))
   t
   )
  (:initialize
   (variable-length variables)
   "
探索変数\it{variables}のうち，該当\it{joint}に影響を与える成分を調べ，記録する．それぞれ0または1個の\it{variables}が発見されなければならない．
   "
   ;;for virtual joint
   (when (or (derivedp j1 cascaded-link)
             (derivedp j1 bodyset-link)
             (derivedp j2 cascaded-link)
             (derivedp j2 bodyset-link))
     (send self :init
                 (cond
                   ((derivedp j1 cascaded-link) (send (car (send j1 :links)) :joint))
                   ((derivedp j1 bodyset-link) (send j1 :joint)))
                 (cond
                   ((derivedp j2 cascaded-link) (send (car (send j2 :links)) :joint))
                   ((derivedp j2 bodyset-link) (send j2 :joint)))
                 :W Wa))

   (send-super :initialize variable-length variables)
   (if (derivedp j1 joint)
       (dolist (variable variables)
         (when (and (derivedp variable joint-variable)
                    (eq (send variable :joint) j1))
           (setq target-variable1 variable)
           )))
   (when target-variable1
     (copy-block-matrix A (unit-matrix (send target-variable1 :dim)) 0 (send target-variable1 :index))
     (copy-block-matrix Asparce (unit-matrix (send target-variable1 :dim)) 0 (send target-variable1 :index))
     )
   (if (derivedp j2 joint)
       (dolist (variable variables)
         (when (and (derivedp variable joint-variable)
                    (eq (send variable :joint) j2))
           (setq target-variable2 variable)
           )))
   (when target-variable2
     (copy-block-matrix A (scale-matrix -1 (unit-matrix (send target-variable2 :dim))) 0 (send target-variable2 :index))
     (copy-block-matrix Asparce (unit-matrix (send target-variable2 :dim)) 0 (send target-variable2 :index))
     )
   t
   )
  (:update
   ()
   "
現在の関節角度に応じて$\bm{d}_l$，$\bm{d}_u$を更新する
   "
   (when (or target-variable1 target-variable2)
     (let ((tmp-joint (if (derivedp j1 joint) j1 j2)))
       (copy-block-vec b (v- (cond ((numberp j2) (float-vector (send tmp-joint :angle-to-speed j2)))
                                   ((vectorp j2) (send tmp-joint :angle-to-speed j2))
                                   ((numberp (send j2 :joint-angle)) (float-vector (send tmp-joint :angle-to-speed (send j2 :joint-angle))))
                                   (t (send tmp-joint :angle-to-speed (send j2 :joint-angle))))
                             (cond ((numberp j1) (float-vector (send tmp-joint :angle-to-speed j1)))
                                   ((vectorp j1) (send tmp-joint :angle-to-speed j1))
                                   ((numberp (send j1 :joint-angle)) (float-vector (send tmp-joint :angle-to-speed (send j1 :joint-angle))))
                                   (t (send tmp-joint :angle-to-speed (send j1 :joint-angle))))))))
   t
   )
  (:is-satisfied
   (&optional force-check)
   "
関節角度が一致しているかどうかを判定
   "
   (if (or force-check check)
       (and
        (every #'(lambda (v) (> v 0)) (v+ b check-margin))
        (every #'(lambda (v) (< v 0)) (v- b check-margin)))
       t)
   )
  (:debug
   ()
   "
現在の角度誤差を表示
   "
   (warning-message (cond
                      ((send self :is-satisfied t) 7);;white
                      (check 1) ;;red
                      (t 3)) ;;yellow
                    "    joint-angle-task: ~A - ~A [mm, deg]~%"
                    (if (derivedp j2 joint)  (send j2 :name) j2)
                    (if (derivedp j1 joint)  (send j1 :name) j1))
   (format-array b)
   t)
  )

;;;;;;;;;;;;;;;;;;;;;;;;;
;; infeasible-angle-vector-task
;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass infeasible-angle-vector-task
  :super inverse-kinematics-task
  :slots ((joints ", target joints")
          (infeasible-angle-vector "")
          (min-distance)
          (targets ", list of (joint infeasible-angle inverse-kinematics-variable)")
          (check "終了判定を行うか否か")
          )
  :documentation
  "
\ \ 対象の複数関節の関節角度列を関節角度空間のある一点からの距離が閾値以上にさせる制約を表現するクラス．\\
距離を$d$とおき，そのヤコビアンを$\mathbf{J}$とすると，\it{prioritized-inverse-kinematics}中で次の問題に変換される．
\begin{eqnarray}
  \min_{\bm{x},\omega} &\ & \omega W \omega \nonumber\\
  \rm{subject\ to} &\ & d_{min} \leqq \mathbf{J} \bm{x} + d + \omega \nonumber
\end{eqnarray}
$d$の単位は\rm{m}，\rm{rad}である．
"
  )

(defmethod infeasible-angle-vector-task
  (:init
   (_joints
    _infeasible-angle-vector
    &key
    ((:min-distance _min-distance) 0.1)
    ((:W _W) 1.0)
    ((:check _check) t)
    )
   "
\begin{itemize}
  \item joints\\
\it{joint}クラスのリスト.対象の関節を表す．仮想関節非対応.
  \item infeasible-angle-vector\\
\it{joint-vector}クラス.回避したい関節角度列を表す．[deg][mm]
  \item min-distance\\
\it{float}クラス.infeasible-angle-vectorからの距離の最小値を表す．[rad][m]
  \item W\\
\it{float}クラス
  \item check : t or nil\\
終了判定を行うか否か
\end{itemize}
   "
   (setq joints _joints)
   (setq infeasible-angle-vector _infeasible-angle-vector)
   (unless (= (reduce #'+ (send-all joints :joint-dof))
              (length infeasible-angle-vector))
     (error "length of angle-vector should be same ~A ~A" joints infeasible-angle-vector))
   (setq min-distance _min-distance)
   (setq equality-rows 0)
   (setq inequality-rows 1)
   (setq Wa (make-matrix equality-rows equality-rows))
   (setq Wc (scale-matrix _W (unit-matrix inequality-rows)))

   (setq check _check)
   t
   )
  (:initialize
   (variable-length variables)
   "
探索変数\it{variables}のうち，該当\it{joint}に影響を与える成分を調べ，記録する．それぞれ0または1個の\it{variables}が発見されなければならない．
   "
   (send-super :initialize variable-length variables)
   (setq targets
         (let ((idx 0))
           (mapcar #'(lambda (j)
                       (let ((variable (find-if #'(lambda (variable) (and (derivedp variable joint-variable) (eq (send variable :joint) j))) variables)))
                         (prog1
                             (list j
                                   (cond ((numberp (send j :joint-angle)) (elt infeasible-angle-vector idx))
                                         (t (subseq infeasible-angle-vector idx (+ idx (send j :joint-dof)))))
                                   variable
                                   )
                           (when variable
                             (fill-block-matrix C 1.0 0 (send variable :index) inequality-rows (send variable :dim))
                             (fill-block-matrix Csparce 1.0 0 (send variable :index) inequality-rows (send variable :dim)))
                           (incf idx (send j :joint-dof))
                           )
                         )
                       )
                   joints)
           )
         )
   (setelt du 0 *inf*)
   t
   )
  (:update
   ()
   "
現在の関節角度に応じて$\bm{d}_l$，$\mathbf{C}$を更新する
   "
   (let ((d (sqrt (reduce #'+ (mapcar #'(lambda (x) (cond ((numberp (elt x 1)) (expt (send (elt x 0) :angle-to-speed (- (elt x 1) (send (elt x 0) :joint-angle))) 2))
                                                          (t (norm2 (send (elt x 0) :angle-to-speed (v- (elt x 1) (send (elt x 0) :joint-angle)))))))
                                      targets)))))
     (setelt dl 0 (- min-distance d))
     (unless (= d 0)
       (dolist (x targets)
         (when (elt x 2)
           (cond ((numberp (elt x 1)) (setf (aref C 0 (send (elt x 2) :index)) (/ (send (elt x 0) :angle-to-speed (- (send (elt x 0) :joint-angle) (elt x 1))) d)))
                 (t (copy-block-rowvector C (scale (/ 1.0 d) (send (elt x 0) :angle-to-speed (v- (send (elt x 0) :joint-angle) (elt x 1)))) 0 (send (elt x 2) :index))))
           )
         )
       )
     )
   t
   )
  (:is-satisfied
   (&optional force-check)
   "
距離が最小値以上かどうかを判定
   "
   (if (or force-check check)
       (<= (elt dl 0) 0)
       t)
   )
  (:debug
   ()
   "
現在の距離を表示
   "
   (warning-message (cond
                      ((send self :is-satisfied t) 7);;white
                      (check 1) ;;red
                      (t 3)) ;;yellow
                    "    infeasible-angle-vector-task: distance = ~A (> ~A [m, rad]) between~%"
                    (- min-distance (elt dl 0))
                    min-distance)
   (format-array infeasible-angle-vector)
   (format-array (concatenate float-vector (flatten (mapcar #'(lambda (j) (cond ((numberp (send j :joint-angle)) (send j :joint-angle))
                                                                                (t (coerce (send j :joint-angle) cons))))
                                                            joints))))
   t)
  )


;;;;;;;;;;;;;;;;;;;;;;;;;
;; avoid-angle-vectors-task
;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass avoid-angle-vectors-task
  :super inverse-kinematics-task
  :slots ((joints ", target joints")
          (avoid-angle-vectors "")
          (min-variable)
          (min-variable-task)
          (targets ", list of list of (joint avoid-angle inverse-kinematics-variable)")
          )
  :documentation
  "
\ \ 対象の複数関節の関節角度列と，関節角度空間の複数点からの距離の最小値を最大化するタスクを表現するクラス．\\
関節角度空間の点$i$からの距離を$d_i$とおき，そのヤコビアンを$\mathbf{J}_i$とすると，\it{prioritized-inverse-kinematics}中で次の問題に変換される．
\begin{eqnarray}
  \min_{\bm{x},d} &\ & - d W d \nonumber\\
  \rm{subject\ to} &\ & 0 \leqq d \nonumber\\
  &\ & d \leqq \mathbf{J}_i \bm{x} + d_i \nonumber
\end{eqnarray}
$d$の単位は\rm{m}，\rm{rad}である．
"
  )

(defmethod avoid-angle-vectors-task
  (:init
   (_joints
    _avoid-angle-vectors
    &key
    ((:W _W) 1.0)
    (value-scale 1.0)
    )
   "
\begin{itemize}
  \item joints\\
\it{joint}クラスのリスト.対象の関節を表す．仮想関節非対応.
  \item infeasible-angle-vector\\
\it{joint-vector}クラス.回避したい関節角度列を表す．[deg][mm]
  \item min-distance\\
\it{float}クラス.infeasible-angle-vectorからの距離の最小値を表す．[rad][m]
  \item W\\
\it{float}クラス
  \item check : t or nil\\
終了判定を行うか否か
\end{itemize}
   "
   (setq joints _joints)
   (setq avoid-angle-vectors _avoid-angle-vectors)
   (dolist (avoid-angle-vector avoid-angle-vectors)
     (unless (= (reduce #'+ (send-all joints :joint-dof))
                (length avoid-angle-vector))
       (error "length of angle-vector should be same ~A ~A" joints avoid-angle-vector)))
   (if avoid-angle-vectors
       (progn (setq equality-rows 1)
              (setq min-variable (instance value-variable :init :value-scale value-scale))
              (setq min-variable-task (instance avoid-angle-vectors-task-task0 :init joints avoid-angle-vectors min-variable)))
       (setq equality-rows 0))
   (setq inequality-rows 0)
   (setq Wa (scale-matrix (- _W) (unit-matrix equality-rows)))
   (setq Wc (make-matrix inequality-rows inequality-rows))

   t
      )
  (:initialize
   (variable-length variables)
   "
探索変数\it{variables}のうち，影響を与える成分を調べ，記録する．
   "
   (send-super :initialize variable-length variables)
   (when min-variable
     (setf (aref A 0 (send min-variable :index)) (send min-variable :value-scale))
     (setf (aref Asparce 0 (send min-variable :index)) 1.0))
   t
   )
  (:update
   ()
   "
現在の関節角度に応じて$\bm{b}$，$\mathbf{A}$を更新する
   "
   (when min-variable
     (setelt b 0 (- (send min-variable :value))))
   t
   )

  (:debug
   ()
   "
現在の距離を表示
   "
   (warning-message 7 "    avoid-angle-vectors-task: min distance = ~A~%"
                    (if min-variable (send min-variable :value) nil))
   (if min-variable
       (send min-variable-task :debug))
   t)
  (:additional-variables () (list min-variable))
  (:additional-task0s () (list min-variable-task))
  )


(defclass avoid-angle-vectors-task-task0
  :super inverse-kinematics-task
  :slots ((joints ", target joints")
          (avoid-angle-vectors "")
          (min-variable)
          (targets ", list of list of (joint avoid-angle inverse-kinematics-variable)")
          )
  :documentation
  "
\ \ avoid-angle-vectors-taskクラスの不等式制約を担当するクラス
  "
  )

(defmethod avoid-angle-vectors-task-task0
  (:init
   (_joints
    _avoid-angle-vectors
    _min-variable
    )
   "
   "
   (setq joints _joints)
   (setq avoid-angle-vectors _avoid-angle-vectors)
   (setq min-variable _min-variable)
   (setq equality-rows 0)
   (setq inequality-rows (1+ (length avoid-angle-vectors)))
   (setq Wa (make-matrix equality-rows equality-rows))
   (setq Wc (scale-matrix 1.0 (unit-matrix inequality-rows)))
   t
   )
  (:initialize
   (variable-length variables)
   "
探索変数\it{variables}のうち，該当\it{joint}に影響を与える成分を調べ，記録する．それぞれ0または1個の\it{variables}が発見されなければならない．
   "
   (send-super :initialize variable-length variables)
   (setq targets
         (let ((idx 0))
           (mapcar #'(lambda (j)
                       (let ((variable (find-if #'(lambda (variable) (and (derivedp variable joint-variable) (eq (send variable :joint) j))) variables)))
                         (prog1
                             (list j
                                   (cond ((numberp (send j :joint-angle)) (mapcar #'(lambda (avoid-angle-vector) (elt avoid-angle-vector idx)) avoid-angle-vectors))
                                         (t (mapcar #'(lambda (avoid-angle-vector) (subseq avoid-angle-vector idx (+ idx (send j :joint-dof)))) avoid-angle-vectors)))
                                   variable
                                   )
                           (when variable
                             (fill-block-matrix C 1.0 0 (send variable :index) (1- inequality-rows) (send variable :dim))
                             (fill-block-matrix Csparce 1.0 0 (send variable :index) (1- inequality-rows) (send variable :dim)))
                           (incf idx (send j :joint-dof))
                           )
                         )
                       )
                   joints)
           )
         )
   (fill-block-matrix C (- (send min-variable :value-scale)) 0 (send min-variable :index) (1- inequality-rows) 1)
   (fill-block-matrix Csparce 1.0 0 (send min-variable :index) (1- inequality-rows) 1)
   (fill-block-matrix C (send min-variable :value-scale) (1- inequality-rows) (send min-variable :index) 1 1)
   (fill-block-matrix Csparce 1.0 (1- inequality-rows) (send min-variable :index) 1 1)
   (fill du *inf*)
   (setelt dl (1- inequality-rows) 0.1)
   t
   )
  (:update
   ()
   "
現在の関節角度に応じて$\bm{d}_l$，$\mathbf{C}$を更新する
   "
   (let ((ds (mapcar #'sqrt
                     (coerce
                      (reduce #'v+
                              (mapcar #'(lambda (x) (cond ((numberp (send (elt x 0) :joint-angle))
                                                           (map float-vector #'(lambda (angle) (expt (send (elt x 0) :angle-to-speed (- angle (send (elt x 0) :joint-angle))) 2))
                                                                (elt x 1)))
                                                          (t
                                                           (map float-vector #'(lambda (angle) (norm2 (send (elt x 0) :angle-to-speed (v- angle (send (elt x 0) :joint-angle)))))
                                                                (elt x 1)))))
                                      targets))
                      cons))))
     (send min-variable :value (apply #'min ds))
     (let ((idx 0))
       (dolist (d ds)
         (setelt dl idx (- (send min-variable :value) d))
         (incf idx)))
     (dolist (x targets)
       (when (elt x 2)
         (cond ((numberp (send (elt x 0) :joint-angle))
                (let ((idx 0))
                  (mapc #'(lambda (angle d)
                            (unless (= d 0)
                              (setf (aref C idx (send (elt x 2) :index)) (/ (send (elt x 0) :angle-to-speed (- (send (elt x 0) :joint-angle) angle)) d)))
                            (incf idx)
                            )
                        (elt x 1)
                        ds)))
               (t
                (let ((idx 0))
                  (mapc #'(lambda (angle d)
                            (unless (= d 0)
                              (copy-block-rowvector C (scale (/ 1.0 d) (send (elt x 0) :angle-to-speed (v- (send (elt x 0) :joint-angle) angle))) idx (send (elt x 2) :index)))
                            (incf idx)
                            )
                        (elt x 1)
                        ds)))
               )
         )
       )
     )
   (setelt dl (1- inequality-rows) (- (send min-variable :value)))
   t
   )
  (:debug
   ()
   "
現在の距離を表示
   "
   (warning-message 7 "        current angle:~%")
   (format-array (concatenate float-vector (flatten (mapcar #'(lambda (j) (cond ((numberp (send j :joint-angle)) (send j :joint-angle))
                                                                                (t (coerce (send j :joint-angle) cons))))
                                                            joints))))
   (let ((idx 0))
     (dolist (avoid-angle-vector avoid-angle-vectors)
       (warning-message 7 "        avoid angle: distance = ~A~%" (- (send min-variable :value) (elt dl idx)))
       (format-array avoid-angle-vector)
       (incf idx)))
   t)
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
          (target-coords-variables ", target-coordsに影響を与えるvariableのリスト")
          (move-target-variables ", move-targetに影響を与えるvariableのリスト")
          (check "終了判定を行うか否か")
          (thre "終了判定時の並進許容誤差")
          (rthre "終了判定時の回転許容誤差")
          (b-raw ", min-max適用前の$\bm{b}$")
          (p-limit "一回の反復計算で動く並進ノルムの大きさの上限[m]")
          (r-limit "一回の反復計算で動く回転ノルムの大きさの上限[rad]")
          (tmp-v0)
          (tmp-v1)
          (tmp-v2)
          (tmp-v3)
          (tmp-v3a)
          (tmp-v3b)
          (tmp-m66)
          (tmp-m33)
          )
  :documentation
  "
\ \ 2つの\it{coordinates}を一致させるタスクを表すクラス．\it{move-target}と\it{target-coords}を一致させる．\it{move-target}と\it{target-coords}はどちらも動いてよい．\\
\it{move-target}と\it{target-coords}の並進誤差を\it{translation-coords}の座標系で表現し，\it{translation-axis}によって抽出された成分を$\bm{e}_{trans}$と表す．\it{move-target}と\it{target-coords}の回転誤差を\it{rotation-coords}の座標系で表現し，\it{rotation-axis}によって抽出された成分を$\bm{e}_{rot}$と表す．$\bm{e}_{trans}$，$\bm{e}_{rot}$のヤコビアンをそれぞれ$\mathbf{J}_{trans}$，$\mathbf{J}_{rot}$とすると，\it{prioritized-inverse-kinematics}中で次の問題に変換される．
\begin{eqnarray}
  \min_{\bm{x}} &\ & (\mathbf{J}_{trans} \bm{x} + \bm{e}_{trans})^\mathrm{T} \mathbf{W}_{trans} (\mathbf{J}_{trans} \bm{x} + \bm{e}_{trans}) + (\mathbf{J}_{rot} \bm{x} + \bm{e}_{rot})^\mathrm{T} \mathbf{W}_{rot} (\mathbf{J}_{rot} \bm{x} + \bm{e}_{rot}) \nonumber
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
    ((:translation-coords _translation-coords) :local)
    ((:rotation-coords _rotation-coords) :local)
    ((:Wtrans _Wtrans) 1.0)
    ((:Wrot _Wrot) 1.0)
    ((:check _check) t)
    ((:thre _thre) 0.001)
    ((:rthre _rthre) (deg2rad 1))
    ((:p-limit _p-limit) 0.1)
    ((:r-limit _r-limit) 0.5)
    )
   "
\begin{itemize}
  \item target-coords, move-target\\
\it{coordinate}クラス.どちらも動いて良い
  \item translation-axis : t :x :y :z :xy :yx :yz :zy :zx :xz nil
  \item rotation-axis : t :x :y :z nil
  \item translation-coords\\
\it{translation-axis}及び\it{Wtrans}は\it{translation-coords}系で表現される.:localならmove-target系，:worldならworld系，\it{coordinates}クラスならその座標系である．
  \item rotation-coords\\
\it{rotation-axis}及び\it{Wrot}は\it{rotation-coords}系で表現される.:localならmove-target系，:worldならworld系，\it{coordinates}クラスならその座標系である．
  \item Wtrans\\
\it{float}クラスの場合$\mathbf{W}_{trans}$はその値を対角成分に並べた行列になる\\
\it{vector}クラスの場合$\mathbf{W}_{trans}$は各値を対角成分に並べた行列になる\\
\it{matrix}クラスの場合$\mathbf{W}_{trans}$としてそのまま使用される．
  \item Wrot\\
\it{float}クラスの場合$\mathbf{W}_{rot}$はその値を対角成分に並べた行列になる\\
\it{vector}クラスの場合$\mathbf{W}_{rot}$は各値を対角成分に並べた行列になる\\
\it{matrix}クラスの場合$\mathbf{W}_{rot}$としてそのまま使用される．
  \item check : t or nil\\
終了判定を行うか否か
  \item thre, rthre\\
\it{float}クラスまたは\it{float-vector}クラス．終了判定時にこの値以下の侵犯を許容する．単位は\rm{m},\rm{rad}
  \item p-limit, r-limit\\
一回の反復計算で動く並進ノルム・回転ノルムの上限．単位は\rm{m},\rm{rad}.特に回転については変位が大き過ぎると線形近似誤差の影響によって計算が収束しない．
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
                            (error "size of Wtrans ~A should be ~A~%" (array-dimensions _Wtrans) rows-trans)))
     )
   (cond
     ((numberp _Wrot) (setq Wrot (scale-matrix _Wrot (unit-matrix rows-rot))))
     ((vectorp _Wrot) (if (= (length _Wrot) rows-rot)
                            (setq Wrot (diagonal _Wrot))
                            (error "length of Wrot (~A) should be ~A~%" (length _Wrot) rows-rot)))
     ((matrixp _Wrot) (if (and (= (array-dimension _Wrot 0) rows-rot) (= (array-dimension _Wrot 1) rows-rot))
                            (setq Wrot _Wrot)
                            (error "size of Wrot ~A should be ~A~%" (array-dimensions _Wrot) rows-rot)))
     )
   (setq inequality-rows 0)
   (setq equality-rows (+ rows-trans rows-rot))
   (setq Wa (concatenate-matrix-diagonal Wtrans Wrot))
   (setq Wc (make-matrix inequality-rows inequality-rows))
   (setq check _check)
   (setq thre (cond ((numberp _thre)
                     (fill (instantiate float-vector rows-trans) _thre))
                    (t
                     (if (= (length _thre) rows-trans)
                         _thre
                         (error "size of thre (~A) should be ~A~%" (length _thre) rows-trans)))))
   (setq rthre (cond ((numberp _rthre)
                     (fill (instantiate float-vector rows-rot) _rthre))
                    (t
                     (if (= (length _rthre) rows-rot)
                         _rthre
                         (error "size of rthre (~A) should be ~A~%" (length _thre) rows-rot)))))
   (setq p-limit _p-limit)
   (setq r-limit _r-limit)
   (setq
    tmp-v0 (instantiate float-vector 0)
    tmp-v1 (instantiate float-vector 1)
    tmp-v2 (instantiate float-vector 2)
    tmp-v3 (instantiate float-vector 3)
    tmp-v3a (instantiate float-vector 3)
    tmp-v3b (instantiate float-vector 3)
    tmp-m66 (make-matrix 6 6)
    tmp-m33 (make-matrix 3 3))
   t
   )
  (:initialize
   (variable-length variables)
   "
探索変数\it{variables}のうち，\it{target-coords},\it{move-target}に影響を与える成分を調べ，記録する．
   "
   (send-super :initialize variable-length variables)
   (setq b-raw (instantiate float-vector (length b)))
   (setq move-target-variables
         (remove-if-not
          #'identity
          (mapcar
           #'(lambda (l)
               (if (send l :joint)
                   (find-if #'(lambda (variable) (and (derivedp variable joint-variable) (eq (send variable :joint) (send l :joint)))) variables)
                   nil))
           (get-link-list move-target))))
   (setq target-coords-variables
         (remove-if-not
          #'identity
          (mapcar
           #'(lambda (l)
               (if (send l :joint)
                   (find-if #'(lambda (variable) (and (derivedp variable joint-variable) (eq (send variable :joint) (send l :joint)))) variables)
                   nil))
           (get-link-list target-coords))))
   (dolist (variable (append move-target-variables target-coords-variables))
     (fill-block-matrix Asparce 1.0 0 (send variable :index) equality-rows (send variable :dim))
     )
   t
   )
  (:update
   ()
   "
現在の状態近傍で線形近似し$\mathbf{A}$, $\bm{b}$を更新する
   "
   ;;position error
   (scale 0.001 (send move-target :difference-position target-coords :translation-axis translation-axis :world translation-coords) tmp-v3)
   (copy-block-vec b-raw (calc-dif-with-axis tmp-v3 translation-axis tmp-v0 tmp-v1 tmp-v2) 0)
   (if (> (norm (subseq b-raw 0 rows-trans)) p-limit)
       (copy-block-vec b (scale p-limit (normalize-vector (subseq b-raw 0 rows-trans))) 0)
       (copy-block-vec b (subseq b-raw 0 rows-trans) 0))
   ;;rotation error
   (copy-block-vec b-raw (calc-dif-with-axis (send move-target :difference-rotation target-coords :rotation-axis rotation-axis :world rotation-coords) rotation-axis tmp-v0 tmp-v1 tmp-v2) rows-trans)
   (if (> (norm (subseq b-raw rows-trans)) r-limit)
       (copy-block-vec b (scale r-limit (normalize-vector (subseq b-raw rows-trans))) rows-trans)
       (copy-block-vec b (subseq b-raw rows-trans) rows-trans))
   ;;calc jacobian of move-target
   (dolist (variable (append move-target-variables target-coords-variables))
     (fill-block-matrix A 0.0 0 (send variable :index) equality-rows (send variable :dim))
     )
   (dolist (variable move-target-variables)
     (let* ((j (send variable :joint))
            (paxis (case (j . axis)
                     (:x #f(1 0 0)) (:y #f(0 1 0)) (:z #f(0 0 1))
                     (:xx #f(1 0 0)) (:yy #f(0 1 0)) (:zz #f(0 0 1))
                     (:-x #f(-1 0 0)) (:-y #f(0 -1 0)) (:-z #f(0 0 -1))
                     (t (j . axis))))
            (child-link (send j :child-link))
            (parent-link (send j :parent-link))
            (default-coords (j . default-coords))
            (world-default-coords (send (send parent-link :copy-worldcoords)
                                        :transform default-coords))
            (transform-coords-trans (cond ((equal translation-coords :local) move-target)
                                          ((equal translation-coords :world) (make-coords))
                                          (t translation-coords)))
            (transform-coords-rot (cond ((equal rotation-coords :local) move-target)
                                        ((equal rotation-coords :world) (make-coords))
                                        (t rotation-coords))))

       ;;translation
       (send j :calc-jacobian
             tmp-m66 ;;fik
             0 ;;row
             0 ;;column
             j ;;joint
             paxis ;;paxis
             child-link ;;child-link
             world-default-coords ;;world-default-coords
             nil ;;child-reverse
             move-target ;;move-target
             transform-coords-trans ;;transform-coords
             rotation-axis ;;rotation-axis
             translation-axis ;;translation-axis
             tmp-v0 tmp-v1 tmp-v2 tmp-v3 tmp-v3a tmp-v3b tmp-m33)
       (dotimes (i rows-trans)
         (dotimes (j (send variable :dim))
           (incf (aref A i (+ (send variable :index) j)) (aref tmp-m66 i j))))
       ;;rotation
       (send j :calc-jacobian
             tmp-m66 ;;fik
             0 ;;row
             0 ;;column
             j ;;joint
             paxis ;;paxis
             child-link ;;child-link
             world-default-coords ;;world-default-coords
             nil ;;child-reverse
             move-target ;;move-target
             transform-coords-rot ;;transform-coords
             rotation-axis ;;rotation-axis
             translation-axis ;;translation-axis
             tmp-v0 tmp-v1 tmp-v2 tmp-v3 tmp-v3a tmp-v3b tmp-m33)
       (dotimes (i rows-rot)
         (dotimes (j (send variable :dim))
           (incf (aref A (+ rows-trans i) (+ (send variable :index) j)) (aref tmp-m66 (+ rows-trans i) j))))
       ))
   ;;calc jacobian of target-coords
   (dolist (variable target-coords-variables)
     (let* ((j (send variable :joint))
            (paxis (case (j . axis)
                     (:x #f(1 0 0)) (:y #f(0 1 0)) (:z #f(0 0 1))
                     (:xx #f(1 0 0)) (:yy #f(0 1 0)) (:zz #f(0 0 1))
                     (:-x #f(-1 0 0)) (:-y #f(0 -1 0)) (:-z #f(0 0 -1))
                     (t (j . axis))))
            (child-link (send j :child-link))
            (parent-link (send j :parent-link))
            (default-coords (j . default-coords))
            (world-default-coords (send (send parent-link :copy-worldcoords)
                                        :transform default-coords))
            (transform-coords-trans (cond ((equal translation-coords :local) move-target)
                                          ((equal translation-coords :world) (make-coords))
                                          (t translation-coords)))
            (transform-coords-rot (cond ((equal rotation-coords :local) move-target)
                                        ((equal rotation-coords :world) (make-coords))
                                        (t rotation-coords))))
       ;;translation
       (send j :calc-jacobian
             tmp-m66 ;;fik
             0 ;;row
             0 ;;column
             j ;;joint
             paxis ;;paxis
             child-link ;;child-link
             world-default-coords ;;world-default-coords
             nil ;;child-reverse
             target-coords ;;move-target
             transform-coords-trans ;;transform-coords
             rotation-axis ;;rotation-axis
             translation-axis ;;translation-axis
             tmp-v0 tmp-v1 tmp-v2 tmp-v3 tmp-v3a tmp-v3b tmp-m33)
       (dotimes (i rows-trans)
         (dotimes (j (send variable :dim))
           (decf (aref A i (+ (send variable :index) j)) (aref tmp-m66 i j))))
       ;;rotation
       (send j :calc-jacobian
             tmp-m66 ;;fik
             0 ;;row
             0 ;;column
             j ;;joint
             paxis ;;paxis
             child-link ;;child-link
             world-default-coords ;;world-default-coords
             nil ;;child-reverse
             target-coords ;;move-target
             transform-coords-rot ;;transform-coords
             rotation-axis ;;rotation-axis
             translation-axis ;;translation-axis
             tmp-v0 tmp-v1 tmp-v2 tmp-v3 tmp-v3a tmp-v3b tmp-m33)
       (dotimes (i rows-rot)
         (dotimes (j (send variable :dim))
           (decf (aref A (+ rows-trans i) (+ (send variable :index) j)) (aref tmp-m66 (+ rows-trans i) j))))
       ))
   t
   )
  (:is-satisfied
   (&optional force-check)
   "
move-targetとtarget-coordsが一致しているかどうかを判定
   "
   (if (or force-check check)
       (and (every #'(lambda (i)
                       (<= (abs (elt b-raw i)) (elt thre i)))
                   (range rows-trans))
            (every #'(lambda (i)
                       (<= (abs (elt b-raw (+ rows-trans i))) (elt rthre i)))
                   (range rows-rot)))
       t)
   )
  (:draw
   ()
   "
move-targetとtarget-coordsを描画
   "
   (send move-target :draw-on :color #F(1 0 0) :width 2)
   (send target-coords :draw-on :color #F(1 0 0) :width 2)
   (let ((pwidth (send *viewer* :viewsurface :line-width))
         (psize (send *viewer* :viewsurface :point-size))
         (pcolor (send *viewer* :viewsurface :color)))
     (unwind-protect
          (progn
            (send *viewer* :viewsurface :color #f(1 0 0))
            (send *viewer* :viewsurface :line-width 1)
            (send *viewer* :viewsurface :point-size 3)
            ;;(send *viewer* :viewsurface :3d-point (elt np 1) :depth-test t)
            (send *viewer* :viewsurface :3d-line (send move-target :worldpos) (send target-coords :worldpos) :depth-test t))
       (progn
         (send *viewer* :viewsurface :line-width pwidth)
         (send *viewer* :viewsurface :point-size psize)
         (send *viewer* :viewsurface :color pcolor)))
     )
   t)
  (:debug
   ()
   "
現在のエラーを表示する
   "
   (warning-message (cond
                      ((send self :is-satisfied t) 7);;white
                      (check 1) ;;red
                      (t 3)) ;;yellow
                    "    move-target-task: ~A - ~A [m, rad]~%" (send target-coords :name) (send move-target :name))
   (format-array b-raw)
   t)
  )



;;;;;;;;;;;;;;;;;;;;;;;;;
;; cog-task
;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass cog-task
  :super inverse-kinematics-task
  :slots ((target "重心を考える対象")
          (target-variables ", 重心に影響を与えるvariableのリスト")
          (target-jacobis ", 各target-variablesに対応した重心ヤコビアンのリスト")
          (tmp-v0)
          (tmp-v1)
          (tmp-v2)
          (tmp-va)
          (tmp-vb)
          (tmp-vc)
          (tmp-vd)
          (tmp-ma)
          (tmp-mb)
          (tmp-mc)
          )
  :documentation
  "
\ \ 重心を扱うための仮想クラス．単位は\rm{m}
"
  )

(defmethod cog-task
  (:init
   (_target
    )
   "
\begin{itemize}
  \item target\\
\it{cascaded-link}クラス.
\end{itemize}
   "
   (send-super :init)
   (setq target _target)
   (setq tmp-v0 (instantiate float-vector 0))
   (setq tmp-v1 (instantiate float-vector 1))
   (setq tmp-v2 (instantiate float-vector 2))
   (setq tmp-va (instantiate float-vector 3))
   (setq tmp-vb (instantiate float-vector 3))
   (setq tmp-vc (instantiate float-vector 3))
   (setq tmp-vd (instantiate float-vector 3))
   (setq tmp-ma (make-matrix 3 3))
   (setq tmp-mb (make-matrix 3 3))
   (setq tmp-mc (make-matrix 3 3))
   t
   )
  (:initialize
   (variable-length variables)
   "
探索変数\it{variables}のうち，\it{target}に影響を与える成分を調べ，記録する．
   "
   (send-super :initialize variable-length variables)
   (setq target-variables
         (append
          (remove-if-not
           #'identity
           (mapcar
            #'(lambda (l)
                (if (send l :joint)
                    (find-if #'(lambda (variable) (eq (send variable :joint) (send l :joint))) variables)
                    nil))
            (get-link-list (car (send target :links)))))
          (remove-if-not
           #'identity
           (mapcar
            #'(lambda (j)
                (find-if #'(lambda (variable) (eq (send variable :joint) j)) variables))
            (send target :joint-list)))))

   (setq target-jacobis
         (mapcar #'(lambda (variable)
                     (make-matrix 3 (send variable :dim)))
                 target-variables))
   t
   )
  (:update
   ()
   "
現在の状態近傍で線形近似しtarget-jacobisに重心ヤコビアンを入れる
   "
   (send target :update-mass-properties
         :tmp-va tmp-va :tmp-vb tmp-vb :tmp-ma tmp-ma :tmp-mb tmp-mb :tmp-mc tmp-mc)
   (let ((all-M (* 0.001 (send target :weight nil)))
         (links (all-child-links (car (send target :links)))))
     (mapc #'(lambda (variable jacobi)
              (when (derivedp variable joint-variable)
                (if (find (send (send variable :joint) :child-link) links) ;;:update-mass-propertiesによってm-til,c-tilが更新されている
                    (let* ((j (send variable :joint))
                           (l (send j :child-link))
                           (paxis (case (j . axis)
                                    (:x (float-vector 1 0 0)) (:-x (float-vector -1 0 0))
                                    (:y (float-vector 0 1 0)) (:-y (float-vector 0 -1 0))
                                    (:z (float-vector 0 0 1)) (:-z (float-vector 0 0 -1))
                                    (t (j . axis))))
                           (world-default-coords (send (send (send j :parent-link) :copy-worldcoords)
                                                       :transform (j . default-coords))))
                      (send j :calc-inertia-matrix
                            jacobi ;;mat
                            0 ;;row
                            0 ;;column
                            paxis ;;paxis
                            (send l :get :m-til) ;;m-til
                            (send l :get :c-til) ;;c-til
                            (send l :get :I-til) ;;I-til
                            (send target :centroid nil) ;;axis-for-angular
                            world-default-coords ;;world-default-coords
                            t ;;translation-axis
                            nil ;;rotation-axis
                            tmp-v0 tmp-v1 tmp-v2 tmp-va tmp-vb tmp-vc tmp-vd tmp-ma)
                      )
                    ;;else
                    (let* ((j (send variable :joint))
                           (l (car (send target :links)))
                           (paxis (case (j . axis)
                                    (:x (float-vector 1 0 0)) (:-x (float-vector -1 0 0))
                                    (:y (float-vector 0 1 0)) (:-y (float-vector 0 -1 0))
                                    (:z (float-vector 0 0 1)) (:-z (float-vector 0 0 -1))
                                    (t (j . axis))))
                           (world-default-coords (send (send (send j :parent-link) :copy-worldcoords)
                                                       :transform (j . default-coords))))
                      (send j :calc-inertia-matrix
                            jacobi ;;mat
                            0 ;;row
                            0 ;;column
                            paxis ;;paxis
                            (send l :get :m-til) ;;m-til
                            (send l :get :c-til) ;;c-til
                            (send l :get :I-til) ;;I-til
                            (send target :centroid nil) ;;axis-for-angular
                            world-default-coords ;;world-default-coords
                            t ;;translation-axis
                            nil ;;rotation-axis
                            tmp-v0 tmp-v1 tmp-v2 tmp-va tmp-vb tmp-vc tmp-vd tmp-ma)
                      )
                    ) ;;if
                (scale-matrix (/ 1.0 all-M) jacobi jacobi)
                );;when
              );;lambda
          target-variables
          target-jacobis
          )
     )
   t
   )
  (:draw
   ()
   "
重心を描画
   "
   (send (send target :centroid nil) :draw-on :color #F(0 0 1) :width 2)
   t)
  )



;;;;;;;;;;;;;;;;;;;;;;;;;
;; target-centroid-pos-task
;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass target-centroid-pos-task
  :super cog-task
  :slots ((target-centroid-pos)
          (cog-translation-axis ",represented in translation-coords")
          (cog-translation-coords)
          (check "終了判定を行うか否か")
          (centroid-thre "終了判定時の並進許容誤差")
          (b-raw ", min-max適用前の$\bm{b}$")
          (p-limit "一回の反復計算で動く並進ノルムの大きさの上限[m]")
          )
  :documentation
  "
\ \ 重心を目標位置に一致させるタスクを表すクラス．\\
重心の目標位置までの並進誤差を\it{cog-translation-coords}の座標系で表現し，\it{cog-translation-axis}によって抽出された成分を$\bm{e}$と表す．$\bm{e}$のヤコビアンをそれぞれ$\mathbf{J}_{cog}$とすると，\it{prioritized-inverse-kinematics}中で次の問題に変換される．
\begin{eqnarray}
  \min_{\bm{x}} &\ & (\mathbf{J}_{cog} \bm{x} + \bm{e})^\mathrm{T} \mathbf{W} (\mathbf{J}_{cog} \bm{x} \nonumber
\end{eqnarray}
$\bm{e}$の単位は\rm{m}である．
"
  )

(defmethod target-centroid-pos-task
  (:init
   (_target
    &key
    ((:target-centroid-pos _target-centroid-pos))
    ((:cog-translation-axis _cog-translation-axis) :z)
    ((:cog-translation-coords _cog-translation-coords) (make-coords))
    ((:W _W) 1.0)
    ((:check _check) t)
    ((:centroid-thre _centroid-thre) 0.001)
    ((:p-limit _p-limit) 0.1)
    )
   "
\begin{itemize}
  \item target\\
\it{cascaded-link}クラス.このオブジェクトの重心を考える．
  \item target-centroid-pos\\
サイズ3の\it{float-vector}クラス．目標重心位置であり，world系で表す．単位は\rm{mm}.
  \item cog-translation-axis : t :x :y :z :xy :yx :yz :zy :zx :xz nil
  \item cog-translation-coords\\
\it{cog-translation-axis}及び\it{W}は\it{translation-coords}系で表現される
  \item W\\
\it{float}クラスの場合$\mathbf{W}$はその値を対角成分に並べた行列になる\\
\it{vector}クラスの場合$\mathbf{W}$は各値を対角成分に並べた行列になる\\
\it{matrix}クラスの場合$\mathbf{W}$としてそのまま使用される．
  \item check : t or nil\\
終了判定を行うか否か
  \item centroid-thre\\
\it{float}クラスまたは\it{float-vector}クラス．終了判定時にこの値以下の侵犯を許容する．単位は\rm{m}
  \item p-limit\\
一回の反復計算で動く並進ノルムの上限．単位は\rm{m}.
\end{itemize}
   "
   (send-super :init _target)
   (setq target-centroid-pos _target-centroid-pos)
   (setq cog-translation-axis _cog-translation-axis)
   (setq cog-translation-coords _cog-translation-coords)
   (setq equality-rows (case cog-translation-axis
                         ((t) 3)
                         ((:x :y :z) 2)
                         ((:xy :yx :yz :zy :zx :xz) 1)
                         (nil 0)))
   (unless (= 3 (length target-centroid-pos))
     (error "length of target-centroid-pos (~A) should be 3" (length target-centroid-pos)))
   (cond
     ((numberp _W) (setq Wa (scale-matrix _W (unit-matrix equality-rows))))
     ((vectorp _W) (if (= (length _W) equality-rows)
                       (setq Wa (diagonal _W))
                       (error "length of W (~A) should be ~A~%" (length _W) equality-rows)))
     ((matrixp _W) (if (and (= (array-dimension _W 0) equality-rows) (= (array-dimension _W 1) equality-rows))
                       (setq Wa _W)
                       (error "size of W ~A should be ~A~%" (array-dimensions _W) equality-rows)))
     )
   (setq inequality-rows 0)
   (setq Wc (make-matrix inequality-rows inequality-rows))
   (setq check _check)
   (setq centroid-thre (cond ((numberp _centroid-thre)
                              (fill (instantiate float-vector equality-rows) _centroid-thre))
                             (t
                              (if (= (length _centroid-thre) equality-rows)
                                  _centroid-thre
                                  (error "size of centroid-thre (~A) should be ~A~%" (length _centroid-thre) equality-rows)))))
   (setq p-limit _p-limit)
   t
   )
  (:initialize
   (variable-length variables)
   "
探索変数\it{variables}のうち，重心に影響を与える成分を調べ，記録する．
   "
   (send-super :initialize variable-length variables)
   (setq b-raw (instantiate float-vector (length b)))
   (dolist (variable target-variables)
     (fill-block-matrix Asparce 1.0 0 (send variable :index) equality-rows (send variable :dim))
     )
   t
   )
  (:update
   ()
   "
現在の状態近傍で線形近似し$\mathbf{A}$, $\bm{b}$を更新する
   "
   ;;calculate cog-jacobian
   (send-super :update)
   ;;position error
   (scale 0.001 (transform (transpose (send cog-translation-coords :worldrot) tmp-ma) (v- target-centroid-pos (send target :centroid nil) tmp-va) tmp-va) tmp-va)
   (copy-block-vec b-raw (calc-dif-with-axis tmp-va cog-translation-axis tmp-v0 tmp-v1 tmp-v2) 0)
   (if (> (norm b-raw) p-limit)
       (copy-block-vec b (scale p-limit (normalize-vector b-raw)) 0)
       (copy-block-vec b b-raw 0))
   ;;calc jacobian
   (mapc #'(lambda (variable jacobi)
             (m* (transpose (send cog-translation-coords :worldrot)) jacobi jacobi)
             (dotimes (i (send variable :dim))
               (copy-block-colvector A (calc-dif-with-axis (matrix-column jacobi i) cog-translation-axis tmp-v0 tmp-v1 tmp-v2) 0 (+ (send variable :index) i))))
         target-variables
         target-jacobis)
   t
   )
  (:is-satisfied
   (&optional force-check)
   "
重心が目標位置と一致しているかどうかを判定
   "
   (if (or force-check check)
       (every #'(lambda (i)
                  (<= (abs (elt b-raw i)) (elt centroid-thre i)))
              (range equality-rows))
       t)
   )
  (:draw
   ()
   "
重心と目標位置を描画
   "
   (send (send target :centroid :nil) :draw-on :color #F(0 0 1) :width 2)
   (send target-centroid-pos :draw-on :color #F(0 0 1) :width 2)
   (let ((pwidth (send *viewer* :viewsurface :line-width))
         (psize (send *viewer* :viewsurface :point-size))
         (pcolor (send *viewer* :viewsurface :color)))
     (unwind-protect
          (progn
            (send *viewer* :viewsurface :color #f(0 0 1))
            (send *viewer* :viewsurface :line-width 1)
            (send *viewer* :viewsurface :point-size 3)
            ;;(send *viewer* :viewsurface :3d-point (elt np 1) :depth-test t)
            (send *viewer* :viewsurface :3d-line (send target :centroid :nil) target-centroid-pos :depth-test t))
       (progn
         (send *viewer* :viewsurface :line-width pwidth)
         (send *viewer* :viewsurface :point-size psize)
         (send *viewer* :viewsurface :color pcolor)))
     )
   t)
  (:debug
   ()
   "
現在のエラーを表示する
   "
   (warning-message (cond
                      ((send self :is-satisfied t) 7);;white
                      (check 1) ;;red
                      (t 3)) ;;yellow
                    "    target-centroid-pos-task: ~A [m, rad]~%" (send target :name))
   (format-array b-raw)
   t)
  )



;;;;;;;;;;;;;;;;;;;;;;;;;
;; centroid-support-polygon-task
;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass centroid-support-polygon-task
  :super cog-task
  :slots ((polygons "各support polygonを表すpolygonのリスト")
          (polygons-variables "各polygonに影響を与えるvariableのリストのリスト")
          (polygons-jacobis)
          (normal "法線ベクトル")
          (margin ", support polygonをこの値だけ縮小する")
          (check "終了判定を行うか否か")
          (centroid-thre "終了判定時の許容誤差")
          (W ", 重み")
          (hull ", 現在のpolygonsのhull")
          )
  :documentation
  "
\ \ 重心をsupport polygonの2次元の範囲内に位置させるタスクを表すクラス．\\
法線ベクトル$\bm{n}$に対して半時計回りに並んだ凸包($\bm{p}_0, \bm{p}_1, \bm{p}_2, ...$)の表す領域内に重心$\bm{c}$を存在させる．$\bm{n}$に垂直な平面に各点を射影し$\bm{n}$をz軸とするlocal座標系で各点を${}^{0}\bm{p}_0, {}^{0}\bm{p}_1, {}^{0}\bm{p}_2, ..., {}^{0}\bm{c}$と表現する.点${}^{0}\bm{p}_i$と点${}^{0}\bm{p}_{i+1}$の距離を$l_i$とすると，直線の左側を正とした${}^{0}\bm{c}$と直線${}^{0}\bm{p}_i {}^{0}\bm{p}_{i+1}$の符号付き距離は，$d_i = \frac{({}^{0}y_i - {}^{0}y_{i+1}){}^{0}x_c+({}^{0}x_{i+1}-{}^{0}x_i){}^{0}y_c+{}^{0}x_i {}^{0}y_{i+1} - {}^{0}x_{i+1} {}^{0}y_i}{l}$である．よって，$\bm{d}$のヤコビアンを$\mathbf{J}$とすると，\it{prioritized-inverse-kinematics}中で次の問題に変換される．
\begin{eqnarray}
  \min_{\bm{x},\bm{\omega}} &\ & \bm{\omega}^\mathrm{T} \mathbf{W} \bm{\omega} \nonumber\\
  \rm{subject\ to} &\ & \bm{d} + \mathbf{J} \bm{x} + \bm{\omega} \geqq margin \nonumber
\end{eqnarray}
なお，
\begin{eqnarray}
  \mathbf{J}_i &=& \frac{1}{l} [({}^{0}\bm{p}_{i+1} - {}^{0}\bm{p}_{i}) \times]_{z行} \frac{\partial}{\partial \bm{x}} \left [\begin{array}{c} {}^{0}x_c\\{}^{0}y_c\\0\end{array}\right] + \left\{\frac{1}{l} [({}^{0}\bm{c} - {}^{0}\bm{p}_{i+1}) \times]_{z行} + \frac{d}{l^2} ({}^{0}\bm{p}_{i+1} - {}^{0}\bm{p}_{i})^\mathrm{T}\right\} \frac{\partial}{\partial \bm{x}} \left [\begin{array}{c} {}^{0}x_i\\{}^{0}y_i\\0\end{array}\right]\nonumber\\
  && + \left\{\frac{1}{l} [({}^{0}\bm{p}_i - {}^{0}\bm{c}) \times]_{z行} + \frac{d}{l^2} ({}^{0}\bm{p}_{i} - {}^{0}\bm{p}_{i+1})^\mathrm{T}\right\} \frac{\partial}{\partial \bm{x}} \left [\begin{array}{c} {}^{0}x_{i+1}\\{}^{0}y_{i+1}\\0\end{array}\right]\nonumber
\end{eqnarray}
である．単位は\rm{m}である．
"
  )

(defmethod centroid-support-polygon-task
  (:init
   (_target
    &key
    ((:polygons _polygons))
    ((:normal _normal) (float-vector 0 0 1))
    ((:margin _margin) 0.0)
    ((:W _W) 1.0)
    ((:check _check) t)
    ((:centroid-thre _centroid-thre) 0.001)
    )
   "
\begin{itemize}
  \item target\\
\it{cascaded-link}クラス.このオブジェクトの重心を考える．
  \item polygons\\
各support polygonを表す\it{polygon}クラスのリスト.woeld座標系で，単位は\rm{m}.各反復ごとにこれらの\it{polygon}のconvex hullを計算し，この範囲内に重心を制限する．
  \item normal\\
サイズ3の\it{float-vector}クラス．上方向を表す法線ベクトルであり，このベクトルで表される2次元平面でsupport polygonを考える．
  \item margin\\
\it{float}クラス．重心をsupport polygonの端からこの値以上離す．単位は[m]．
  \item W\\
\it{float}クラス.$\mathbf{W}$はこの値を対角成分に並べた行列になる\\
  \item check : t or nil\\
終了判定を行うか否か
  \item centroid-thre\\
\it{float}クラス．終了判定時にこの値以下の侵犯を許容する．単位は\rm{m}
\end{itemize}
   "
   (send-super :init _target)
   (setq polygons _polygons)
   (setq normal _normal)
   (setq margin _margin)
   (setq W _W)
   (setq equality-rows 0)
   (setq inequality-rows 0)
   (setq check _check)
   (setq centroid-thre _centroid-thre)
   t
   )
  (:initialize
   (variable-length variables)
   "
探索変数\it{variables}のうち，重心やpolygonsに影響を与える成分を調べ，記録する．
   "
   ;;重心
   (send-super :initialize variable-length variables)
   ;;polygons
   (setq polygons-variables
         (mapcar #'(lambda (p)
                     (remove-if-not
                      #'identity
                      (mapcar
                       #'(lambda (l)
                           (if (send l :joint)
                               (find-if #'(lambda (variable) (eq (send variable :joint) (send l :joint))) variables)
                               nil))
                       (get-link-list (send p :body)))))
                 polygons))
   (setq polygons-jacobis
         (mapcar #'(lambda (polygon-variables)
                     (mapcar #'(lambda (variable)
                                 (make-matrix 3 (send variable :dim)))
                             polygon-variables))
                 polygons-variables))
   t
   )
  (:update
   ()
   "
現在の状態近傍で線形近似し$\mathbf{C}$,$\mathbf{C}_{sparce}$,  $\bm{d}_l$，$\bm{d}_u$, $\mathbf{W}_c$を更新する
   "
   ;;calculate cog-jacobian
   (send-super :update)
   ;;get support polygon
   (send-all (send-all polygons :body) :worldcoords)
   (setq hull (instance polygon :init :vertices (quickhull
                                                 (apply
                                                  #'append
                                                  (mapcar #'(lambda (p)
                                                              (send p :vertices))
                                                          polygons))
                                                 normal)))
   ;;calc
   (setq inequality-rows (length (send hull :edges)))
   (if (= (array-dimension C 0) inequality-rows)
       (progn
         (fill (array-entity C) 0)
         (fill (array-entity Csparce) 0)
         (fill dl 0))
       (progn
         (setq C (make-matrix inequality-rows cols))
         (setq Csparce (make-matrix inequality-rows cols))
         (setq Wc (scale-matrix W (unit-matrix inequality-rows)))
         (setq du (fill (instantiate float-vector inequality-rows) *inf*))
         (setq dl (instantiate float-vector inequality-rows)))
       )
   (let* ((lc (orient-coords-to-axis (make-coords) normal))
          jacobis-i1
          variables-i1
          jacobis-i2
          variables-i2
          (num 0)
          polygon-idx
          (cog (send target :centroid nil))
          (0cog (scale 0.001 (send lc :inverse-transform-vector cog)));;m
          p1
          p2
          (0p1 (instantiate float-vector 3));;m
          (0p2 (instantiate float-vector 3));;m
          l
          d
          (0p2-0p1-cross (make-matrix 1 3))
          (0cog-0p2-cross (make-matrix 1 3))
          (0p1-0cog-cross (make-matrix 1 3))
          (0p2-0p1 (make-matrix 1 3))
          (0p1-0p2 (make-matrix 1 3))
          (sum1 (make-matrix 1 3))
          (sum2 (make-matrix 1 3))
          )
     (setelt 0cog 2 0)
     (dolist (jacobi target-jacobis)
       (m* (transpose (send lc :worldrot)) jacobi jacobi))

     (dolist (e (send hull :edges))
       (setq p1 (elt (send e :vertices) 0))
       (setq p2 (elt (send e :vertices) 1))
       (scale 0.001 (send lc :inverse-transform-vector p1) 0p1)
       (setelt 0p1 2 0)
       (scale 0.001 (send lc :inverse-transform-vector p2) 0p2)
       (setelt 0p2 2 0)
       (setq l (distance 0p1 0p2))
       (setq d (/ (+ (* (- (elt 0p1 1) (elt 0p2 1)) (elt 0cog 0))
                     (* (- (elt 0p2 0) (elt 0p1 0)) (elt 0cog 1))
                     (* (elt 0p1 0) (elt 0p2 1))
                     (- (* (elt 0p2 0) (elt 0p1 1))))
                  l))
       (setf (aref 0p2-0p1-cross 0 0) (/ (- (elt 0p1 1) (elt 0p2 1)) l))
       (setf (aref 0p2-0p1-cross 0 1) (/ (- (elt 0p2 0) (elt 0p1 0)) l))
       (setf (aref 0cog-0p2-cross 0 0) (/ (- (elt 0p2 1) (elt 0cog 1)) l))
       (setf (aref 0cog-0p2-cross 0 1) (/ (- (elt 0cog 0) (elt 0p2 0)) l))
       (setf (aref 0p1-0cog-cross 0 0) (/ (- (elt 0cog 1) (elt 0p1 1)) l))
       (setf (aref 0p1-0cog-cross 0 1) (/ (- (elt 0p1 0) (elt 0cog 0)) l))
       (setf (aref 0p2-0p1 0 0) (/ (* (- (elt 0p2 0) (elt 0p1 0)) d) (expt l 2)))
       (setf (aref 0p2-0p1 0 1) (/ (* (- (elt 0p2 1) (elt 0p1 1)) d) (expt l 2)))
       (setf (aref 0p1-0p2 0 0) (/ (* (- (elt 0p1 0) (elt 0p2 0)) d) (expt l 2)))
       (setf (aref 0p1-0p2 0 1) (/ (* (- (elt 0p1 1) (elt 0p2 1)) d) (expt l 2)))
       (m+ 0cog-0p2-cross 0p2-0p1 sum1)
       (m+ 0p1-0cog-cross 0p1-0p2 sum2)

       ;;lower limit
       (setelt dl num (- margin d))

       ;;cog jacobian
       (mapc #'(lambda (variable jacobi)
                 (copy-block-matrix C (m* 0p2-0p1-cross jacobi) num (send variable :index))
                 (fill-block-matrix Csparce 1.0 num (send variable :index) 1 (send variable :dim)))
             target-variables
             target-jacobis)

       ;;p1 jacobian
       (unless jacobis-i1 ;;first edge
         (setq polygon-idx (position-if #'(lambda (p) (find (elt (send e :vertices) 0) (send p :vertices)))
                                        polygons))
         (setq variables-i1 (elt polygons-variables polygon-idx))
         (setq jacobis-i1 (elt polygons-jacobis polygon-idx))
         (mapc #'(lambda (variable jacobi)
                   (let* ((j (send variable :joint))
                          (paxis (case (j . axis)
                                   (:x #f(1 0 0)) (:y #f(0 1 0)) (:z #f(0 0 1))
                                   (:xx #f(1 0 0)) (:yy #f(0 1 0)) (:zz #f(0 0 1))
                                   (:-x #f(-1 0 0)) (:-y #f(0 -1 0)) (:-z #f(0 0 -1))
                                   (t (j . axis))))
                          (child-link (send j :child-link))
                          (parent-link (send j :parent-link))
                          (default-coords (j . default-coords))
                          (world-default-coords (send (send parent-link :copy-worldcoords)
                                                      :transform default-coords))
                          )

                     ;;translation
                     (send j :calc-jacobian
                           jacobi ;;fik
                           0 ;;row
                           0 ;;column
                           j ;;joint
                           paxis ;;paxis
                           child-link ;;child-link
                           world-default-coords ;;world-default-coords
                           nil ;;child-reverse
                           (make-coords :pos (elt (send e :vertices) 0)) ;;move-target
                           lc ;;transform-coords
                           nil ;;rotation-axis
                           t ;;translation-axis
                           tmp-v0 tmp-v1 tmp-v2 tmp-va tmp-vb tmp-vc tmp-ma)
                     )
                   )
               variables-i1
               jacobis-i1)
         )
       (mapc #'(lambda (variable jacobi)
                 (let ((tmpJ (m* sum1 jacobi)))
                   (dotimes (i (send variable :dim))
                     (incf (aref C num (+ i (send variable :index))) (aref tmpJ 0 i))))
                 (fill-block-matrix Csparce 1.0 num (send variable :index) 1 (send variable :dim)))
             variables-i1
             jacobis-i1)

       ;;p2 jacobian
       (setq polygon-idx (position-if #'(lambda (p) (find (elt (send e :vertices) 1) (send p :vertices)))
                                      polygons))
       (setq variables-i2 (elt polygons-variables polygon-idx))
       (setq jacobis-i2 (elt polygons-jacobis polygon-idx))
       (mapc #'(lambda (variable jacobi)
                 (let* ((j (send variable :joint))
                        (paxis (case (j . axis)
                                 (:x #f(1 0 0)) (:y #f(0 1 0)) (:z #f(0 0 1))
                                 (:xx #f(1 0 0)) (:yy #f(0 1 0)) (:zz #f(0 0 1))
                                 (:-x #f(-1 0 0)) (:-y #f(0 -1 0)) (:-z #f(0 0 -1))
                                 (t (j . axis))))
                        (child-link (send j :child-link))
                        (parent-link (send j :parent-link))
                        (default-coords (j . default-coords))
                        (world-default-coords (send (send parent-link :copy-worldcoords)
                                                    :transform default-coords))
                        )
                   ;;translation
                   (send j :calc-jacobian
                         jacobi ;;fik
                         0 ;;row
                         0 ;;column
                         j ;;joint
                         paxis ;;paxis
                         child-link ;;child-link
                         world-default-coords ;;world-default-coords
                         nil ;;child-reverse
                         (make-coords :pos (elt (send e :vertices) 1)) ;;move-target
                         lc ;;transform-coords
                         nil ;;rotation-axis
                         t ;;translation-axis
                         tmp-v0 tmp-v1 tmp-v2 tmp-va tmp-vb tmp-vc tmp-ma)
                   )
                 )
             variables-i2
             jacobis-i2)
       (mapc #'(lambda (variable jacobi)
                 (let ((tmpJ (m* sum2 jacobi)))
                     (dotimes (i (send variable :dim))
                       (incf (aref C num (+ i (send variable :index))) (aref tmpJ 0 i))))
                 (fill-block-matrix Csparce 1.0 num (send variable :index) 1 (send variable :dim)))
             variables-i2
             jacobis-i2)


       (setq jacobis-i1 jacobis-i2)
       (setq variables-i1 variables-i2)
       (incf num)
       )
     )
   t
   )
  (:is-satisfied
   (&optional force-check)
   "
重心が領域内にあるかどうかを判定
   "
   (if (or force-check check)
       (every #'(lambda (i) (<= i 0))
              (v- dl (fill (instantiate float-vector (length dl)) centroid-thre)))
       t)
   )
  (:draw
   ()
   "
重心と領域を描画
   "
   (send (send target :centroid :nil) :draw-on :color #F(0 0 1) :width 2)
   (send hull :draw-on :color #F(0 0 1) :width 2)
   (let ((pwidth (send *viewer* :viewsurface :line-width))
         (psize (send *viewer* :viewsurface :point-size))
         (pcolor (send *viewer* :viewsurface :color)))
     (unwind-protect
          (progn
            (send *viewer* :viewsurface :color #f(0 0 1))
            (send *viewer* :viewsurface :line-width 1)
            (send *viewer* :viewsurface :point-size 3)
            ;;(send *viewer* :viewsurface :3d-point (elt np 1) :depth-test t)
            (dolist (v (send hull :vertices))
              (send *viewer* :viewsurface :3d-line v (v+ v (scale 50 (normalize-vector normal))) :depth-test t)))
       (progn
         (send *viewer* :viewsurface :line-width pwidth)
         (send *viewer* :viewsurface :point-size psize)
         (send *viewer* :viewsurface :color pcolor)))
     )
   t)
  (:debug
   ()
   "
現在のエラーを表示する
   "
   (warning-message (cond
                      ((send self :is-satisfied t) 7);;white
                      (check 1) ;;red
                      (t 3)) ;;yellow
                    "    centroid-support-polygon-task: ~A [m]~%" (send target :name))
   (format-array (map float-vector #'(lambda (i) (if (> i 0) i 0)) dl))
   t)
  )



;;;;;;;;;;;;;;;;;;;;;;;;;
;; centroid-scfr-task
;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass centroid-scfr-task
  :super cog-task
  :slots ((contact-constraints)
          (contact-coords)
          (margin ", support polygonをこの値だけ縮小する")
          (check "終了判定を行うか否か")
          (centroid-thre "終了判定時の許容誤差")
          (W ", 重み")
          (A_eq)
          (b_eq)
          (A_ineq)
          (b_ineq)
          (epsilon)
          )
  :documentation
  "
\ \ 重心を静的重心実行可能領域内に位置させるタスクを表すクラス．重心の位置は最適化するが，外界接触位置を最適化して静的重心実行可能領域を能動的に変化させるような処理はしない．\\
外界接触部位でロボットが受ける接触レンチ(6自由度,world系,接触部位まわり)を並べたベクトルを$\bm{w}$とし，接触力拘束を$\mathbf{C}\bm{w} + \bm{d} \geqq 0$，力の静的釣り合いを$\mathbf{G} \bm{w} + [\begin{array}{c c c c c c} 0 & 0 & -mg & - c_y mg & c_x mg & 0 \end{array}]^\mathrm{T} = 0$とする．これらの式によって制限される$\bm{w'} = [\begin{array}{c c c} c_x & c_y & \bm{w}^\mathrm{T} \end{array}]^\mathrm{T}$の存在領域は，
\begin{eqnarray}
  \left\{ \bm{w'} \in \mathbb{R}^d : \left [\begin{array}{c c} \begin{array}{c c} \bm{0}_3&\bm{0}_3\\ 0&-mg\\ mg&0\\ 0&0\end{array} & \mathbf{G} \end{array}\right] \bm{w'} = 0, \left[\begin{array}{c c} \mathbf{0}&\mathbf{C}\end{array}\right] \bm{w'} \geqq \bm{d} \right\} \nonumber
\end{eqnarray}
となり，これはd次元空間の凸多面体(閉じていなくともよい)である．この多面体の頂点を考えることで，$\bm{w'}$の存在領域は，
\begin{eqnarray}
  \left\{ \bm{w'} \in \mathbb{R}^d : {}^\exists\bm{y}, {}^\exists\bm{z}, \bm{w'} = \mathbf{V} \bm{y} + \mathbf{R} \bm{z}, \sum_i \bm{y}_i=1, \bm{y}\geqq 0, \bm{z}\geqq 0 \right\} \nonumber
\end{eqnarray}
とも表せる．この頂点$\mathbf{V}$, $\mathbf{R}$を$c_x$ - $c_y$平面に射影した$\mathbf{V'}$, $\mathbf{R'}$を用いることで，静的重心実行可能領域が次のように得られる．
\begin{eqnarray}
  \left\{ c_x, c_y : {}^\exists\bm{y}, {}^\exists\bm{z}, \left[\begin{array}{c} c_x\\c_y\end{array} \right] = \mathbf{V'} \bm{y} + \mathbf{R'} \bm{z}, \sum_i \bm{y}_i=1, \bm{y}\geqq 0, \bm{z}\geqq 0 \right\} \nonumber
\end{eqnarray}
この多面体を不等式制約の形式で表現すれば，静的重心実行可能領域を表す不等式制約が得られる．
\begin{eqnarray}
  \left\{ c_x, c_y : \mathbf{A} \left[\begin{array}{c} c_x\\c_y\end{array} \right] + \bm{b} \geqq  0 \right\} \nonumber
\end{eqnarray}

重心$c_x$,$c_y$のヤコビアンを$\mathbf{J}$とすると，\it{prioritized-inverse-kinematics}中で次の問題に変換される．
\begin{eqnarray}
  \min_{\bm{x},\bm{\omega}} &\ & \bm{\omega}^\mathrm{T} \mathbf{W} \bm{\omega} \nonumber\\
  \rm{subject\ to} &\ & \mathbf{A}(\left[\begin{array}{c} c_x\\c_y\end{array} \right] + \mathbf{J} \bm{x}) + \bm{\omega} \geqq -\bm{b} + margin \nonumber
\end{eqnarray}
単位が\rm{m}となるよう$\mathbf{A}$,$\bm{b}$はスケーリングされる．
"
  )

(defmethod centroid-scfr-task
  (:init
   (_target
    &key
    ((:contact-constraints _contact-constraints))
    ((:contact-coords _contact-coords))
    ((:margin _margin) 0.0)
    ((:W _W) 1.0)
    ((:check _check) t)
    ((:centroid-thre _centroid-thre) 0.001)
    ((:epsilon _epsilon) 5)
    )
   "
\begin{itemize}
  \item target\\
\it{cascaded-link}クラス.このオブジェクトの重心を考える．
  \item contact-constraints\\
\it{contact-constraint}クラスのリスト．これらによって定義される静的重心実行可能領域内に重心を制限する．
  \item contact-coords\\
\it{coordinates}クラスのリスト．それぞれのcontact-constraintの座標系を表す．
  \item margin\\
\it{float}クラス．重心を静的重心実行可能領域の端からこの値以上離す．単位は[m]．
  \item W\\
\it{float}クラス.$\mathbf{W}$はこの値を対角成分に並べた行列になる\\
  \item check : t or nil\\
終了判定を行うか否か
  \item centroid-thre\\
\it{float}クラス．終了判定時にこの値以下の侵犯を許容する．単位は\rm{m}
  \item epsilon\\
\it{integer}クラス．多面体計算時の離散化に用いる．
\end{itemize}
   "
   (send-super :init _target)
   (setq contact-constraints _contact-constraints)
   (setq contact-coords _contact-coords)
   (setq margin _margin)
   (setq W _W)
   (setq equality-rows 0)
   (setq inequality-rows 0)
   (setq check _check)
   (setq centroid-thre _centroid-thre)
   (setq epsilon _epsilon)
   t
   )
  (:initialize
   (variable-length variables)
   "
探索変数\it{variables}のうち，重心に影響を与える成分を調べ，記録する．
   "
   ;;重心
   (send-super :initialize variable-length variables)
   t
   )
  (:update
   ()
   "
現在の状態近傍で線形近似し$\mathbf{A}$,$\mathbf{A}_{sparce}$,  $\bm{b}$， $\mathbf{W}_a$,$\mathbf{C}$,$\mathbf{C}_{sparce}$,  $\bm{d}_l$，$\bm{d}_u$, $\mathbf{W}_c$を更新する
   "
   ;;calculate cog-jacobian
   (send-super :update)
   ;;get support polygon
   (let* ((mg (* 1e-6 (send target :weight nil) (elt *g-vec* 2)))
          (_C (apply
               #'concatenate-matrix-diagonal
               (mapcar #'(lambda (constraint coords)
                           (send constraint :calc-constraint-matrix (send coords :worldcoords)))
                      contact-constraints contact-coords)))
          (_d (scale (/ 1.0 mg) (v- (apply
                                     #'concatenate
                                     float-vector
                                     (mapcar #'(lambda (constraint)
                                                 (send constraint :get-constraint-vector))
                                             contact-constraints)))))
          (_G (send target :calc-grasp-matrix
                    (mapcar #'(lambda (coords)
                                (send coords :worldpos))
                            contact-coords)))
          (total-wrench (float-vector 0
                                      0
                                      (- 1 #|mg|#)
                                      0
                                      0
                                      0))
          (_C2  (concatenate-matrix-row _C (make-matrix (array-dimension _C 0) 2)))
          (_G2 (concatenate-matrix-row _G (matrix (float-vector 0 0)
                                                  (float-vector 0 0)
                                                  (float-vector 0 0)
                                                  (float-vector 0 (- 1 #|mg|#))
                                                  (float-vector 1 #|mg|# 0)
                                                  (float-vector 0 0))))
          (ret (cddlib-H-to-V :A_ineq _C2 :b_ineq _d :A_eq _G2 :b_eq total-wrench :verbose 0))
          (V (block-matrix (elt ret 0) (- (array-dimension (elt ret 0) 0) 2) 0 2 (array-dimension (elt ret 0) 1)))
          (R_nonneg (block-matrix (elt ret 1) (- (array-dimension (elt ret 1) 0) 2) 0 2 (array-dimension (elt ret 1) 1)))
          (R_free (block-matrix (elt ret 2) (- (array-dimension (elt ret 2) 0) 2) 0 2 (array-dimension (elt ret 2) 1)))
          (ret2 (cddlib-V-to-H :V V :R_nonneg R_nonneg :R_free R_free :verbose 0))
          (_A_eq (elt ret2 0))
          (_b_eq (elt ret2 1))
          (_A_ineq (elt ret2 2))
          (_b_ineq (elt ret2 3))
          )
     (setq A_eq (copy-object _A_eq))
     (setq b_eq (copy-object _b_eq))
     (setq A_ineq (copy-object _A_ineq))
     (setq b_ineq (copy-object _b_ineq))

     (setq equality-rows (length b_eq))
     (if (= (array-dimension A 0) equality-rows)
         (progn
           (fill (array-entity A) 0)
           (fill (array-entity Asparce) 0)
           (fill b 0))
         (progn
           (setq A (make-matrix equality-rows cols))
           (setq Asparce (make-matrix equality-rows cols))
           (setq Wa (scale-matrix W (unit-matrix equality-rows)))
           (setq b (instantiate float-vector equality-rows)))
         )
     (setq inequality-rows (length b_ineq))
     (if (= (array-dimension C 0) inequality-rows)
         (progn
           (fill (array-entity C) 0)
           (fill (array-entity Csparce) 0)
           (fill dl 0))
         (progn
           (setq C (make-matrix inequality-rows cols))
           (setq Csparce (make-matrix inequality-rows cols))
           (setq Wc (scale-matrix W (unit-matrix inequality-rows)))
           (setq du (fill (instantiate float-vector inequality-rows) *inf*))
           (setq dl (instantiate float-vector inequality-rows)))
         )

     (dotimes (i equality-rows)
       (let ((n (sqrt (+ (* (aref _A_eq i 0) (aref _A_eq i 0)) (* (aref _A_eq i 1) (aref _A_eq i 1))))))
         (unless (= n 0)
           (setf (aref _A_eq i 0) (/ (aref _A_eq i 0) n))
           (setf (aref _A_eq i 1) (/ (aref _A_eq i 1) n))
           (setelt _b_eq i (/ (elt _b_eq i) n)))
         ))
     (dotimes (i inequality-rows)
       (let ((n (sqrt (+ (* (aref _A_ineq i 0) (aref _A_ineq i 0)) (* (aref _A_ineq i 1) (aref _A_ineq i 1))))))
         (unless (= n 0)
           (setf (aref _A_ineq i 0) (/ (aref _A_ineq i 0) n))
           (setf (aref _A_ineq i 1) (/ (aref _A_ineq i 1) n))
           (setelt _b_ineq i (/ (elt _b_ineq i) n)))
         ))

     ;;calc error
     (transform _A_eq (v- (scale 1e-3 (subseq (send target :centroid nil) 0 2))) b)
     (v- b _b_eq b)
     (fill dl margin)
     (v- dl (transform _A_ineq (scale 1e-3 (subseq (send target :centroid nil) 0 2))) dl)
     (v- dl _b_ineq dl)

     ;;calc jacobian
     (mapc #'(lambda (variable jacobi)
               (dotimes (i (send variable :dim))
                 (copy-block-colvector A (transform A_eq (calc-dif-with-axis (matrix-column jacobi i) :z tmp-v0 tmp-v1 tmp-v2)) 0 (+ (send variable :index) i))
                 (copy-block-colvector C (transform A_ineq (calc-dif-with-axis (matrix-column jacobi i) :z tmp-v0 tmp-v1 tmp-v2)) 0 (+ (send variable :index) i))
                 ))
           target-variables
           target-jacobis)
     )
   t
   )
  (:is-satisfied
   (&optional force-check)
   "
重心が領域内にあるかどうかを判定
   "
   (if (or force-check check)
       (and
        (every #'(lambda (i) (<= (abs (elt b i)) centroid-thre))
               (range equality-rows))
        (every #'(lambda (i) (<= i 0))
               (v- dl (fill (instantiate float-vector (length dl)) centroid-thre))))
       t)
   )
  (:draw
   ()
   "
重心と領域を描画
   "
   (send (send target :centroid :nil) :draw-on :color #F(0 0 1) :width 2)
   (dolist (cc contact-constraints)
     (send cc :draw-on :flush nil))

   (let* ((ret3 (cddlib-H-to-V :A_ineq A_ineq :b_ineq b_ineq :A_eq A_eq :b_eq b_eq :verbose 0))
          (V (elt ret3 0))
          (R_nonneg (elt ret3 1))
          (R_free (elt ret3 2))
          (z (elt (send target :centroid nil) 2))
          )
     (cond
       ((= (array-dimension R_free 1) 2)
        (let ((pwidth (send *viewer* :viewsurface :line-width))
              (psize (send *viewer* :viewsurface :point-size))
              (pcolor (send *viewer* :viewsurface :color))
              (vec (if (= (array-dimension V 1) 0)
                       (float-vector 0 0 z)
                       (float-vector (* 1e3 (aref V 0 0))
                                     (* 1e3 (aref V 1 0))
                                     z))))
          (unwind-protect
               (progn
                 (send *viewer* :viewsurface :color #f(0 0 1))
                 (send *viewer* :viewsurface :line-width 3)
                 (send *viewer* :viewsurface :point-size 3)
                 (send *viewer* :viewsurface :3d-line
                       (v+ vec (float-vector (* 1e6 (aref R_free 0 0))
                                             (* 1e6 (aref R_free 1 0))
                                             0))
                       (v+ vec (float-vector (* -1e6 (aref R_free 0 0))
                                             (* -1e6 (aref R_free 1 0))
                                             0))
                       :depth-test t)
                 (send *viewer* :viewsurface :3d-line
                       (v+ vec (float-vector (* 1e6 (aref R_free 0 1))
                                             (* 1e6 (aref R_free 1 1))
                                             0))
                       (v+ vec (float-vector (* -1e6 (aref R_free 0 1))
                                             (* -1e6 (aref R_free 1 1))
                                             0))
                       :depth-test t)
                 )
            (progn
              (send *viewer* :viewsurface :line-width pwidth)
              (send *viewer* :viewsurface :point-size psize)
              (send *viewer* :viewsurface :color pcolor)))))
       ((and (= (array-dimension R_free 1) 1)
             (= (array-dimension R_nonneg 1) 1))
        (let ((pwidth (send *viewer* :viewsurface :line-width))
              (psize (send *viewer* :viewsurface :point-size))
              (pcolor (send *viewer* :viewsurface :color))
              (vec (if (= (array-dimension V 1) 0)
                       (float-vector 0 0 z)
                       (float-vector (+ 1e3 (aref V 0 0))
                                     (+ 1e3 (aref V 1 0))
                                     z))))
          (unwind-protect
               (progn
                 (send *viewer* :viewsurface :color #f(0 0 1))
                 (send *viewer* :viewsurface :line-width 3)
                 (send *viewer* :viewsurface :point-size 3)
                 (send *viewer* :viewsurface :3d-line
                       (v+ vec (float-vector (* 1e6 (aref R_free 0 0))
                                             (* 1e6 (aref R_free 1 0))
                                             0))
                       (v+ vec (float-vector (* -1e6 (aref R_free 0 0))
                                             (* -1e6 (aref R_free 1 0))
                                             0))
                       :depth-test t)
                 (send *viewer* :viewsurface :3d-line
                       vec
                       (v+ vec (float-vector (* 1e6 (aref R_nonneg 0 0))
                                             (* 1e6 (aref R_nonneg 1 0))
                                             0))
                       :depth-test t)
                 )
            (progn
              (send *viewer* :viewsurface :line-width pwidth)
              (send *viewer* :viewsurface :point-size psize)
              (send *viewer* :viewsurface :color pcolor)))))
       ((and  (= (array-dimension R_free 1) 1)
              (= (array-dimension R_nonneg 1) 0))
        (let ((pwidth (send *viewer* :viewsurface :line-width))
              (psize (send *viewer* :viewsurface :point-size))
              (pcolor (send *viewer* :viewsurface :color))
              (vec (if (= (array-dimension V 1) 0)
                       (list (float-vector 0 0 z))
                       (let ((vec))
                         (dotimes (i (array-dimension V 1))
                           (push (float-vector (+ 1e3 (aref V 0 i))
                                               (+ 1e3 (aref V 1 i))
                                               z) vec))
                         vec))))
          (unwind-protect
               (progn
                 (send *viewer* :viewsurface :color #f(0 0 1))
                 (send *viewer* :viewsurface :line-width 3)
                 (send *viewer* :viewsurface :point-size 3)
                 (dolist (vv vec)
                   (send *viewer* :viewsurface :3d-line
                         (v+ vv (float-vector (* 1e6 (aref R_free 0 0))
                                              (* 1e6 (aref R_free 1 0))
                                              0))
                         (v+ vv (float-vector (* -1e6 (aref R_free 0 0))
                                              (* -1e6 (aref R_free 1 0))
                                              0))
                         :depth-test t))
                 )
            (progn
              (send *viewer* :viewsurface :line-width pwidth)
              (send *viewer* :viewsurface :point-size psize)
              (send *viewer* :viewsurface :color pcolor)))))
       (t
        (let* ((points (let ((points nil))
                         (dotimes (i (array-dimension V 1))
                           (push (float-vector (* 1e3 (aref V 0 i)) (* 1e3 (aref V 1 i)) z) points))
                         points))
               (hull (instance polygon :init :vertices (if points (quickhull points) (list (float-vector 0 0 z)))))
               (vec (cond ((= (array-dimension R_nonneg 1) 0)
                           nil)
                          ((= (array-dimension R_nonneg 1) 1)
                           (list (float-vector (aref R_nonneg 0 0) (aref R_nonneg 1 0) 0)
                                 (float-vector (aref R_nonneg 0 0) (aref R_nonneg 1 0) 0)))
                          (t
                           (let ((v1 (float-vector (aref R_nonneg 0 0) (aref R_nonneg 1 0) 0))
                                 (v2 (float-vector (aref R_nonneg 0 1) (aref R_nonneg 1 1) 0)))
                             (if (> (elt (v* v1 v2) 2) 0)
                                 (list v1 v2)
                                 (list v2 v1))))))
               (ext (if vec
                        (let ((v1 (v* (elt vec 0) #F(0 0 1)))
                              (v2 (v* #F(0 0 1) (elt vec 0))))
                          (list
                           (find-extream (send hull :vertices) #'(lambda (v) (v. v v1)) #'>)
                           (find-extream (send hull :vertices) #'(lambda (v) (v. v v2)) #'>)))
                        nil))
               )
          (let ((pwidth (send *viewer* :viewsurface :line-width))
                (psize (send *viewer* :viewsurface :point-size))
                (pcolor (send *viewer* :viewsurface :color)))
            (unwind-protect
                 (progn
                   (send *viewer* :viewsurface :color #f(0 0 1))
                   (send *viewer* :viewsurface :line-width 3)
                   (send *viewer* :viewsurface :point-size 3)
                   (mapc #'(lambda (e)
                             (unless (and (find (elt (send e :vertices) 0) vec) (find (elt (send e :vertices) 1) vec))
                               (send *viewer* :viewsurface :3d-line
                                     (elt (send e :vertices) 0)
                                     (elt (send e :vertices) 1)
                                     :depth-test t)))
                         (send hull :edges))
                   (mapc #'(lambda (v e)
                             (send *viewer* :viewsurface :3d-line
                                   e
                                   (v+ e (scale 1e6 v))
                                   :depth-test t))
                         vec
                         ext)
                   )
              (progn
                (send *viewer* :viewsurface :line-width pwidth)
                (send *viewer* :viewsurface :point-size psize)
                (send *viewer* :viewsurface :color pcolor)))
            )
          )
        )
       )
     )
   t)
  (:debug
   ()
   "
現在のエラーを表示する
   "
   (warning-message (cond
                      ((send self :is-satisfied t) 7);;white
                      (check 1) ;;red
                      (t 3)) ;;yellow
                    "    centroid-scfr-task: ~A [m]~%" (send target :name))
   (format-array (concatenate float-vector
                              b
                              (map float-vector #'(lambda (i) (if (> i 0) i 0)) dl)))
   t)
  )
