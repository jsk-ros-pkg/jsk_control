(require (format nil "~A/euslisp/eus-osqp-compiled.l" (ros::resolve-ros-path "package://eus_osqp")))
(require (format nil "~A/prioritized-inverse-kinematics/euslisp/inverse-kinematics-variable" (ros::resolve-ros-path "package://eus_qp")))
(require (format nil "~A/prioritized-inverse-kinematics/euslisp/inverse-kinematics-task" (ros::resolve-ros-path "package://eus_qp")))
(require (format nil "~A/prioritized-inverse-kinematics/euslisp/util" (ros::resolve-ros-path "package://eus_qp")))

(defun prioritized-inverse-kinematics
    (tasks variables
     &key
       (regular 1e-6)
       (regular-rel 5e-0)
       (regular-max 1e-1)
       (task0 nil)
       (stop 50)
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
  \rm{priority 1} &:& \min_{\bm{x}} \ \rm{task1\mathchar`-1} + \rm{task1\mathchar`-2} + ... + \bm{x}^\mathrm{T} \mathbf{W} \bm{x} \nonumber \\
  \rm{priority 2} &:& \min_{\bm{x}} \ \rm{task2\mathchar`-1} + \rm{task2\mathchar`-2} + ... + \bm{x}^\mathrm{T} \mathbf{W} \bm{x} \nonumber \\
  \rm{priority 3} &:& \min_{\bm{x}} \ \rm{task3\mathchar`-1} + \rm{task3\mathchar`-2} + ... + \bm{x}^\mathrm{T} \mathbf{W} \bm{x} \nonumber \\
  : \nonumber
\end{eqnarray}

\item variables : (list variable1 variable2 ...)\\
\ \ 探索変数のリストである．各\it{variable}は\it{joint}クラスまたは\it{inverse-kinematics-variable}クラスである．\it{joint}クラスが与えられた場合，そのjoint-angleが探索変数となる．同一の探索変数を意味する\it{variable}を2回以上与えてはならない．

\item regular, regular-rel, regular-max : weight or (list weight1 weight2 weight3 ...)\\
\ \ 各\rm{priority}のQPにおいて解を一意に定めるための正則化重みの大きさである．各\it{weight}は\it{float}クラスである．\it{regular}として\it{float}が与えられた場合は全\rm{priority}共通でその重みを用い，\it{list}が与えられた場合は各\rm{priority}ごとに指定された重みを用いる．$w=\rm{regular}$，$w_r=\rm{regular\mathchar`-rel}$，$w_{max}=\rm{regular\mathchar`-max}$とし，各\rm{priority}の現在のエラーの2乗和を$e$とおくと，重みは$\min(w+w_r e, w_{max})$となる．

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
     (when (numberp regular)
       (setq regular (make-list (length tasks) :initial-element regular)))
     (when (numberp regular-rel)
       (setq regular-rel (make-list (length tasks) :initial-element regular-rel)))
     (when (numberp regular-max)
       (setq regular-max (make-list (length tasks) :initial-element regular-max)))
     (dolist (variable variables)
       (send variable :init-form)
       (send variable :index variable-length)
       (incf variable-length (send variable :dim)))
     (dolist (task-p (append (list task0) tasks)) (dolist (task task-p) (send task :initialize variable-length variables)))
     (setq x (instantiate float-vector variable-length))

     (dotimes (i stop)
       (fill x 0)
       (dolist (task-p (append (list task0) tasks)) (dolist (task task-p) (send task :update)))
       ;;描画
       (when debug-view
         (send *irtviewer* :draw-objects :flush nil)
         (dolist (task-p (append (list task0) tasks)) (dolist (task task-p) (send task :draw)))
         (send *viewer* :viewsurface :flush)
         )
       (when (eq debug-view t)
         (warning-message 4 "~%Iteration: ~A~%" i)
         (warning-message 2 "  Priority: 0~%")
         (dolist (task task0) (send task :debug)))
       ;;終了判定
       (when (and
              (>= i min-loop)
              (every #'(lambda (task-p) (every #'identity (send-all task-p :is-satisfied))) (append (list task0) tasks)))
         (when (eq debug-view t)
           (let ((priority 1))
             (dolist (task-p tasks)
               (warning-message 2 "  Priority: ~A~%" priority)
               (dolist (task task-p) (send task :debug))
               (incf priority))))
         (return-from prioritized-inverse-kinematics t))
       ;;QP
       ;;task0
       (let ((A (apply #'concatenate-matrix-column
                        (make-matrix 0 variable-length)
                        (send-all task0 :A)))
             (Asparce (apply #'concatenate-matrix-column
                             (make-matrix 0 variable-length)
                             (send-all task0 :Asparce)))
             (b (apply #'concatenate float-vector
                       (send-all task0 :b)))
             (C (apply #'concatenate-matrix-column
                       (make-matrix 0 variable-length)
                       (send-all task0 :C)))
             (Csparce (apply #'concatenate-matrix-column
                             (make-matrix 0 variable-length)
                             (send-all task0 :Csparce)))
             (dl (apply #'concatenate float-vector
                        (send-all task0 :dl)))
             (du (apply #'concatenate float-vector
                        (send-all task0 :du)))
             (priority 0)
             )
         (dolist (task-p tasks)
           (let ((this-A (apply #'concatenate-matrix-column
                                (make-matrix 0 variable-length)
                                (send-all task-p :A)))
                 (this-Asparce (apply #'concatenate-matrix-column
                                      (make-matrix 0 variable-length)
                                      (send-all task-p :Asparce)))
                 (this-b (apply #'concatenate float-vector
                                (instantiate float-vector 0)
                                (send-all task-p :b)))
                 (this-C (apply #'concatenate-matrix-column
                                (make-matrix 0 variable-length)
                                (send-all task-p :C)))
                 (this-Csparce (apply #'concatenate-matrix-column
                                      (make-matrix 0 variable-length)
                                      (send-all task-p :Csparce)))
                 (this-dl (apply #'concatenate float-vector
                                 (instantiate float-vector 0)
                                 (send-all task-p :dl)))
                 (this-du (apply #'concatenate float-vector
                                 (instantiate float-vector 0)
                                 (send-all task-p :du)))
                 (this-Wa (apply #'concatenate-matrix-diagonal
                                 (send-all task-p :Wa)))
                 (this-Wc (apply #'concatenate-matrix-diagonal
                                 (send-all task-p :Wc)))
                 )
             (when (eq debug-view t)
               (warning-message 2 "  Priority: ~A~%" (1+ priority))
               (dolist (task task-p) (send task :debug)))
             (let* ((omega-length (array-dimension this-C 0))
                    (this-error (+ (norm2 this-b)
                                   (norm2 (remove-if #'(lambda (v) (< v 0)) this-dl))
                                   (norm2 (remove-if #'(lambda (v) (> v 0)) this-du))))
                    (this-regular (min (+ (elt regular priority)
                                          (* (elt regular-rel priority)
                                             this-error))
                                       (elt regular-max priority)))
                    (status (float-vector 0))
                    (this-x (apply
                             qp-solver
                             :eval-weight-matrix (concatenate-matrix-diagonal
                                                  (m+ (m* (m* (transpose this-A) this-Wa) this-A)
                                                      (scale-matrix this-regular (unit-matrix variable-length)))
                                                  this-Wc)
                             :eval-coeff-vector (concatenate float-vector
                                                             (v- (transform (m* (transpose this-A) this-Wa) this-b))
                                                             (instantiate float-vector omega-length))
                             :state-dim (+ variable-length omega-length)
                             :equality-matrix (concatenate-matrix-row
                                               A
                                               (make-matrix (array-dimension A 0) omega-length))
                             :equality-vector b
                             :inequality-matrix (concatenate-matrix-column
                                                 (concatenate-matrix-row C (make-matrix (array-dimension C 0) omega-length))
                                                 (concatenate-matrix-row this-C (unit-matrix omega-length)))
                             :inequality-min-vector (concatenate float-vector dl this-dl)
                             :inequality-max-vector (concatenate float-vector du this-du)
                             :inequality-dim (+ (array-dimension C 0) (array-dimension this-C 0))
                             :status status
                             :eval-weight-matrix-sparce (concatenate-matrix-diagonal
                                                         (m+ (m* (m* (transpose this-Asparce) this-Wa) this-Asparce)
                                                             (scale-matrix this-regular (unit-matrix variable-length)))
                                                         this-Wc)
                             :equality-matrix-sparce (concatenate-matrix-row
                                                      Asparce
                                                      (make-matrix (array-dimension Asparce 0) omega-length))
                             :inequality-matrix-sparce (concatenate-matrix-column
                                                        (concatenate-matrix-row Csparce (make-matrix (array-dimension Csparce 0) omega-length))
                                                        (concatenate-matrix-row this-Csparce (unit-matrix omega-length)))
                             qp-args
                             ))
                    )
               (when (eq debug-view t)
                 (format t "    e: ~A~%" this-error))
               ;;QPfail判定
               (unless this-x
                 (warn "priority ~A: QP could not be colved (status ~A)~%" (1+ priority) (round (elt status 0)))
                 (return-from prioritized-inverse-kinematics nil)
                 )
               ;;今タスクの結果の反映
               (dotimes (i variable-length) (setelt x i (elt this-x i)))
               (let ((tmp-b (transform this-A x))
                     (tmp-d (transform this-C x)))
                 (dotimes (i (length tmp-d))
                   (if (> (elt tmp-d i) (elt this-du i)) (setelt this-du i (elt tmp-d i)))
                   (if (< (elt tmp-d i) (elt this-dl i)) (setelt this-dl i (elt tmp-d i)))
                   )
                 (setq A (concatenate-matrix-column A this-A))
                 (setq Asparce (concatenate-matrix-column Asparce this-Asparce))
                 (setq b (concatenate float-vector b tmp-b))
                 (setq C (concatenate-matrix-column C this-C))
                 (setq Csparce (concatenate-matrix-column Csparce this-Csparce))
                 (setq dl (concatenate float-vector dl this-dl))
                 (setq du (concatenate float-vector du this-du))
                 )
               )
             )
           (incf priority 1)
           ) ;;dolist (task-p tasks)
         ;;xを適用
         (dolist (variable variables) (send variable :apply-x x))
         (when (eq debug-view t)
           (format t "  x:~%")
           (format-array x))
         ) ;;let ((A
       ) ;;dotimes (i stop)
     ;;debug
     (when debug-view
       (dolist (task-p (append (list task0) tasks)) (dolist (task task-p) (send task :update)))
       ;;描画
       (send *irtviewer* :draw-objects :flush nil)
       (dolist (task-p (append (list task0) tasks)) (dolist (task task-p) (send task :draw)))
       (send *viewer* :viewsurface :flush)
       (when (eq debug-view t)
         (warning-message 4 "~%Max Iteration:~%")
         (let ((priority 0))
           (dolist (task-p (append (list task0) tasks))
             (warning-message 2 "  Priority: ~A~%" priority)
             (dolist (task task-p) (send task :debug))
             (incf priority))))
       )
     ;;revert-if-fail判定
     (if revert-if-fail
         (if (every #'(lambda (task-p) (every #'identity (send-all task-p :is-satisfied))) tasks)
             t
             (progn
               (dolist (variable variables) (send variable :revert))
               nil))
         (if (every #'(lambda (task-p) (every #'identity (send-all task-p :is-satisfied))) tasks)
             t
             nil))
     )
    ;;cleanup-form
    (dolist (variable variables) (send variable :cleanup-form)))
  )
