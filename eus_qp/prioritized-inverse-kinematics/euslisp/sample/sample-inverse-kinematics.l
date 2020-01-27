(load "package://eus_qp/prioritized-inverse-kinematics/euslisp/prioritized-inverse-kinematics-compiled.l")

(defun setup ()
  (unless (boundp '*robot*)
    (load "irteus/demo/sample-robot-model.l")
    (setq *robot* (instance sample-robot :init)))
  (unless (boundp '*irtviewer*)
    (make-irtviewer))
  (objects (list *robot*))
  )
(print "(setup)")

;;左手を目標に伸ばす
(defun test-inverse-kinematics
 (&rest
    args
    )
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords))

  (apply
   #'prioritized-inverse-kinematics
   (list (list (instance move-target-task :init
                         (send *robot* :larm :end-coords)
                         (make-coords :pos #F(300 100 600)))))
   (mapcar #'(lambda (j) (instance joint-variable :init j))
           (send *robot* :larm :joint-list))
   args
   )
  )
(print "(test-inverse-kinematics)")

;;左手を目標に伸ばす
;;関節上下限を考慮する
(defun test-inverse-kinematics-minmax
 (&rest
    args
    )
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords))

  (apply
   #'prioritized-inverse-kinematics
   (list (list (instance move-target-task :init
                         (send *robot* :larm :end-coords)
                         (make-coords :pos #F(300 100 600)))))
   (mapcar #'(lambda (j) (instance joint-variable :init j))
           (send *robot* :larm :joint-list))
   :task0 (mapcar #'(lambda (j) (instance minmax-angle-task :init j))
                  (send *robot* :larm :joint-list))
   args
   )
  )
(print "(test-inverse-kinematics-minmax)")


;;両手を目標に伸ばす
;;関節上下限を考慮する
(defun test-inverse-kinematics-dual
 (&rest
    args
    )
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords))

  (apply
   #'prioritized-inverse-kinematics
   (list (list (instance move-target-task :init
                         (send *robot* :rarm :end-coords)
                         (make-coords :pos #F(350 -200 700)))
               (instance move-target-task :init
                         (send *robot* :larm :end-coords)
                         (make-coords :pos #F(300 100 600)))))
   (mapcar #'(lambda (j) (instance joint-variable :init j))
           (append (send *robot* :torso :joint-list) (send *robot* :rarm :joint-list) (send *robot* :larm :joint-list)))
   :task0 (mapcar #'(lambda (j) (instance minmax-angle-task :init j))
                  (append (send *robot* :torso :joint-list) (send *robot* :rarm :joint-list) (send *robot* :larm :joint-list)))
   args
   )
  )
(print "(test-inverse-kinematics-dual)")


;;浮遊リンク系で両手両足を目標に伸ばす
;;関節上下限を考慮する
(defun test-inverse-kinematics-fullbody
 (&rest
    args
    )
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords))

  (apply
   #'prioritized-inverse-kinematics
   (list (list (instance move-target-task :init
                         (send *robot* :rarm :end-coords)
                         (make-coords :pos #F(350 -200 700)))
               (instance move-target-task :init
                         (send *robot* :larm :end-coords)
                         (make-coords :pos #F(300 100 600)))
               (instance move-target-task :init
                         (send *robot* :rleg :end-coords)
                         (make-coords :pos #F(50 -100 50)))
               (instance move-target-task :init
                         (send *robot* :lleg :end-coords)
                         (make-coords :pos #F(-50 100 -50) :rpy #F(0.3 0 0)))
               ))
   (append
    (list (instance virtual-joint-variable :init (car (send *robot* :links))))
    (mapcar #'(lambda (j) (instance joint-variable :init j))
            (send *robot* :joint-list)))
   :task0 (mapcar #'(lambda (j) (instance minmax-angle-task :init j))
                  (send *robot* :joint-list))
   args
   )
  )
(print "(test-inverse-kinematics-fullbody)")


;;浮遊リンク系で両手両足を目標に伸ばす
;;関節上下限を考慮する
;;腕は届かない位置を指定し，脚を優先に解く．
(defun test-inverse-kinematics-fullbody-priority
 (&rest
    args
    )
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords))

  (apply #'prioritized-inverse-kinematics
   (list (list (instance move-target-task :init
                         (send *robot* :rleg :end-coords)
                         (make-coords :pos #F(50 -100 50)))
               (instance move-target-task :init
                         (send *robot* :lleg :end-coords)
                         (make-coords :pos #F(-50 100 -50) :rpy #F(0.3 0 0))))
         (list (instance move-target-task :init
                         (send *robot* :rarm :end-coords)
                         (make-coords :pos #F(1000 -200 700))
                         :check nil)
               (instance move-target-task :init
                         (send *robot* :larm :end-coords)
                         (make-coords :pos #F(300 100 1500))
                         :check nil))
         )
   (append
    (list (instance virtual-joint-variable :init *robot*))
    (mapcar #'(lambda (j) (instance joint-variable :init j))
            (send *robot* :joint-list)))
   :task0 (mapcar #'(lambda (j) (instance minmax-angle-task :init j))
                  (send *robot* :joint-list))
   :min-loop 30
   args
   )
  )
(print "(test-inverse-kinematics-fullbody-priority)")
