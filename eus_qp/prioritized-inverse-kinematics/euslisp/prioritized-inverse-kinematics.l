(require (format nil "~A/prioritized-inverse-kinematics/euslisp/inverse-kinematics-variable" (ros::resolve-ros-path "package://eus_qp")))
(require (format nil "~A/prioritized-inverse-kinematics/euslisp/inverse-kinematics-task" (ros::resolve-ros-path "package://eus_qp")))

(defun prioritized-inverse-kinematics
    (tasks variables
     &key
       (regular 1e-8)
       (task0 nil)
       (stop 10)
       (min-loop 0)
       (revert-if-fail t)
       (debug-view :no-message)
       (qp-solver #'solve-osqp-qp)
       (qp-args (list :solve-qp-mode :sqp-with-hotstart))
     )
  "
\begin{itemize}
\item tasks : (list (list task1-1 task1-2 ... )\\
\hspace{5em}        (list task2-1 task2-2 ... )\\
\hspace{5em}        (list task3-1 task3-2 ... )\\
\hspace{5em}        ...)\\
\ \ 各\it{task}は\it{inverse-kinematics-task}クラスである．\it{prioritized-inverse-kinematics}は次の問題を解く．\rm{priority 1}のタスクが最も優先度が高い．
\begin{eqnarray}
  \rm{priority 1} &:& \min \ \rm{task1\mathchar`-1} + \rm{task1\mathchar`-2} + ...\nonumber \\
  \rm{priority 2} &:& \min \ \rm{task2\mathchar`-1} + \rm{task2\mathchar`-2} + ...\nonumber \\
  \rm{priority 3} &:& \min \ \rm{task3\mathchar`-1} + \rm{task3\mathchar`-2} + ...\nonumber \\
  : \nonumber
\end{eqnarray}

\item variables : (list variable1 variable2 ...)\\
\ \ 探索変数のリストである．各\it{variable}は\it{joint}クラスまたは\it{inverse-kinematics-variable}クラスである．\it{joint}クラスが与えられた場合，そのjoint-angleが探索変数となる．

\item regular : weight or (list weight1 weight2 weight3 ...)\\
\ \ 各\rm{priority}のQPにおいて解を一意に定めるための正則化重みの大きさである．各\it{weight}は\it{float}クラスである．\it{regular}として\it{float}が与えられた場合は全\rm{priority}共通でその重みを用い，\it{list}が与えられた場合は各\rm{priority}ごとに指定された重みを用いる．

\item task0 : (list task0-1 task0-2 ... )\\
\ \ 各\it{task}は\it{inverse-kinematics-task}クラスである．\it{task0}が与えられた場合，\it{prioritized-inverse-kinematics}は次の問題を解く．
\begin{eqnarray}
  \rm{priority 0} &:& \min \ \rm{task0\mathchar`-1} + \rm{task0\mathchar`-2} + ...\nonumber \\
  \rm{priority 1} &:& \min \ \rm{task1\mathchar`-1} + \rm{task1\mathchar`-2} + ...\nonumber \\
  \rm{priority 2} &:& \min \ \rm{task2\mathchar`-1} + \rm{task2\mathchar`-2} + ...\nonumber \\
  \rm{priority 3} &:& \min \ \rm{task3\mathchar`-1} + \rm{task3\mathchar`-2} + ...\nonumber \\
  : \nonumber
\end{eqnarray}
ただし，\rm{priority 0}の各タスクの最適値を同時に満たす解が必ず存在すると仮定し，\rm{priority 0}ではQPを解かないことで高速化を図る．したがって，\it{task0}としてこの仮定が成り立たないタスクを与えるべきでない．

\item stop\\
\ \ \it{stop}回の反復計算後，直ちに終了する．\it{stop}は\it{interger}クラスである.

\item min-loop\\
\ \ \it{min-loop}回の反復計算後から，各反復終了時に全\it{task}の終了条件を満たすなら直ちに終了する．\it{min-loop}は\it{interger}クラスである.

\item revert-if-fail : t or nil\\
\ \ \it{t}ならば，終了時にある\rm{task}の終了条件を満たして無い場合に，初期状態に戻してから\it{prioritized-inverse-kinematics}から返る

\item debug-view\\
\ \ \it{t}ならば反復計算中に状態を描画しデバッグメッセージを表示する．\it{:no-message}ならば反復計算中に状態を描画する．\it{nil}ならば何もしない．

\item qp-solver\\
\ \ QPのソルバを指定する．\it{qp-solver}は\it{function}クラスである．

\item qp-args\\
\ \ \it{prioritized-inverse-kinematics}内で，(apply qp-solver ... qp-args)の形でqp-solverが呼ばれる．

\end{itemize}
  "
  (unwind-protect
   (let ((variable-length 0)
         x)
     (dolist (variable variables)
       (send variable :init-form)
       (send variable :index num)
       (incf num (send variable :dim)))
     (dolist (task (append task0 tasks)) (send task :initialize variable-length variables))
     (setq x (instantiate float-vector variable-length))
     (dotimes (i stop)
       (fill x 0)
       (dolist (task (append task0 tasks)) (send task :update))
       ;;描画
       ;;終了判定
       ;;QP
       ;;QPfail判定
       (dolist (variable variables) (send variable :apply-x x))
       )
     (if nil ;;revert-if-fail判定
         (progn
           (dolist (variable variables) (send variable :revert))
           nil)
         t)
     )
    ;;cleanup-form
    (dolist (variable variables) (send variable :cleanup-form)))
  )
