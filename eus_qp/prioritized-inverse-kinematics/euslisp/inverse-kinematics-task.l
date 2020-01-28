(require (format nil "~A/prioritized-inverse-kinematics/euslisp/util" (ros::resolve-ros-path "package://eus_qp")))
(require (format nil "~A/prioritized-inverse-kinematics/euslisp/inverse-kinematics-variable" (ros::resolve-ros-path "package://eus_qp")))

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
   ()
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
   ()
   "
関節角度上下限を満足しているかどうかを判定
   "
   (if check
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
                      ((send self :is-satisfied) 7);;white
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
   ()
   "
関節角度が一致しているかどうかを判定
   "
   (if check
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
                      ((send self :is-satisfied) 7);;white
                      (check 1) ;;red
                      (t 3)) ;;yellow
                    "    joint-angle-task: ~A - ~A [mm, deg]~%"
                    (if (derivedp j2 joint)  (send j2 :name) j2)
                    (if (derivedp j1 joint)  (send j1 :name) j1))
   (format-array b)
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
                   (find-if #'(lambda (variable) (eq (send variable :joint) (send l :joint))) variables)
                   nil))
           (get-link-list move-target))))
   (setq target-coords-variables
         (remove-if-not
          #'identity
          (mapcar
           #'(lambda (l)
               (if (send l :joint)
                   (find-if #'(lambda (variable) (eq (send variable :joint) (send l :joint))) variables)
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
   (copy-block-vec b-raw (calc-dif-with-axis (send move-target :difference-rotation target-coords :rotation-axis rotation-axis :world rotation-coords) rotation-axis tmp-v0 tmp-v1 tmp-v3) rows-trans)
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
   ()
   "
move-targetとtarget-coordsが一致しているかどうかを判定
   "
   (if check
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
                      ((send self :is-satisfied) 7);;white
                      (check 1) ;;red
                      (t 3)) ;;yellow
                    "    move-target-task: ~A - ~A [m, rad]~%" (send target-coords :name) (send move-target :name))
   (format-array b-raw)
   t)
  )
