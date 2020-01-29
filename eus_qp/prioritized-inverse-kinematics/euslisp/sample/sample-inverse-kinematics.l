(load "package://eus_qp/prioritized-inverse-kinematics/euslisp/prioritized-inverse-kinematics-compiled.l")

(defun setup ()
  (unless (boundp '*robot*)
    (load "irteus/demo/sample-robot-model.l")
    (setq *robot* (instance sample-robot :init))
    (send *robot* :set-val 'support-polygons (append
                                              (mapcar #'(lambda (l) (send *robot* :make-sole-polygon-ex l))
                                                      (list :rleg-link5 :lleg-link5 :rarm-link6 :larm-link6))
                                              (send *robot* :get-val 'support-polygons)))
    )
  (unless (boundp '*irtviewer*)
    (make-irtviewer))
  (objects (list *robot*))

  (dotimes (i 10)
    (unless (boundp (read-from-string (format nil "*robot~A*" i)))
      (eval `(setq ,(read-from-string (format nil "*robot~A*" i)) (instance sample-robot :init)))
      (let ((robot (eval (read-from-string (format nil "*robot~A*" i)))))
        (send robot :set-val 'support-polygons (append
                                                (mapcar #'(lambda (l) (send robot :make-sole-polygon-ex l))
                                                        (list :rleg-link5 :lleg-link5 :rarm-link6 :larm-link6))
                                                (send robot :get-val 'support-polygons)))
        )))
  )
(print "(setup)")

;;左手を目標に伸ばす
(defun test-inverse-kinematics
 (&rest
    args
    )
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords))

  (if (set-exclusive-or (send *irtviewer* :objects) (list *robot*))
      (objects (list *robot*)))

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

  (if (set-exclusive-or (send *irtviewer* :objects) (list *robot*))
      (objects (list *robot*)))

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


;;左手を目標に伸ばす
;;関節上下限を考慮する
;;関節速度上下限を考慮する
(defun test-inverse-kinematics-minmax-vel
 (&rest
    args
    )
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords))

  (if (set-exclusive-or (send *irtviewer* :objects) (list *robot*))
      (objects (list *robot*)))

  (apply
   #'prioritized-inverse-kinematics
   (list (list (instance move-target-task :init
                         (send *robot* :larm :end-coords)
                         (make-coords :pos #F(300 100 600)))))
   (mapcar #'(lambda (j) (instance joint-variable :init j))
           (send *robot* :larm :joint-list))
   :task0 (append
           (mapcar #'(lambda (j) (instance minmax-angle-task :init j))
                   (send *robot* :larm :joint-list))
           (mapcar #'(lambda (j) (instance joint-velocity-task :init j :max-vel 0.3 :min-vel -0.3 :periodic-time 0.05));;1回の反復計算で最大0.3rad/s * 0.05sだけ動く
                   (send *robot* :larm :joint-list)))
   :stop 100
   args
   )
  )
(print "(test-inverse-kinematics-minmax-vel)")


;;両手を目標に伸ばす
;;関節上下限を考慮する
(defun test-inverse-kinematics-dual
 (&rest
    args
    )
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords))

  (if (set-exclusive-or (send *irtviewer* :objects) (list *robot*))
      (objects (list *robot*)))

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


;;両手を5cm横に並べる
;;関節上下限を考慮する
(defun test-inverse-kinematics-dual-lineup
 (&rest
    args
    )
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords))

  (if (set-exclusive-or (send *irtviewer* :objects) (list *robot*))
      (objects (list *robot*)))

  (let ((larm-tmp-coords (make-cascoords :name :larm-tmp-coords
                                              :parent (send *robot* :larm :end-coords)
                                              :coords (send (send *robot* :larm :end-coords :copy-worldcoords) :translate #F(0 -50 0):local))))
    (unwind-protect
         (apply
          #'prioritized-inverse-kinematics
          (list (list (instance move-target-task :init
                                larm-tmp-coords
                                (send *robot* :rarm :end-coords))))
          (mapcar #'(lambda (j) (instance joint-variable :init j))
                  (append (send *robot* :torso :joint-list) (send *robot* :rarm :joint-list) (send *robot* :larm :joint-list)))
          :task0 (mapcar #'(lambda (j) (instance minmax-angle-task :init j))
                         (append (send *robot* :torso :joint-list) (send *robot* :rarm :joint-list) (send *robot* :larm :joint-list)))
          args
          )
      (send (send *robot* :larm :end-coords) :dissoc larm-tmp-coords)
      )
    )
  )
(print "(test-inverse-kinematics-dual-lineup)")


;;両手を5cm横に並べ，出来る限り上に上げる
;;関節上下限を考慮する
(defun test-inverse-kinematics-dual-lineup-priority
 (&rest
    args
    )
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords))

  (if (set-exclusive-or (send *irtviewer* :objects) (list *robot*))
      (objects (list *robot*)))

  (let ((larm-tmp-coords (make-cascoords :name :larm-tmp-coords
                                              :parent (send *robot* :larm :end-coords)
                                              :coords (send (send *robot* :larm :end-coords :copy-worldcoords) :translate #F(0 -50 0):local))))
    (unwind-protect
         (apply
          #'prioritized-inverse-kinematics
          (list (list (instance move-target-task :init
                                larm-tmp-coords
                                (send *robot* :rarm :end-coords)))
                (list (instance move-target-task :init
                                (send *robot* :rarm :end-coords)
                                (make-coords :pos #F(0 0 1500))
                                :translation-axis :xy
                                :rotation-axis nil
                                :translation-coords :world
                                :check nil)))
          (mapcar #'(lambda (j) (instance joint-variable :init j))
                  (append (send *robot* :torso :joint-list) (send *robot* :rarm :joint-list) (send *robot* :larm :joint-list)))
          :task0 (mapcar #'(lambda (j) (instance minmax-angle-task :init j))
                         (append (send *robot* :torso :joint-list) (send *robot* :rarm :joint-list) (send *robot* :larm :joint-list)))
          :min-loop 30
          args
          )
      (send (send *robot* :larm :end-coords) :dissoc larm-tmp-coords)
      )
    )
  )
(print "(test-inverse-kinematics-dual-lineup-priority)")

;;左手を目標に伸ばす10のキーポーズを作成
;;関節上下限を考慮する
(defun test-inverse-kinematics-keypose
 (&rest
    args
    )

  (let ((robot-list (mapcar #'(lambda (i) (eval (read-from-string (format nil "*robot~A*" i))))
                            (range 10))))
    (dolist (robot robot-list)
      (send robot :reset-pose)
      (send robot :fix-leg-to-coords (make-coords)))

    (if (set-exclusive-or (send *irtviewer* :objects) robot-list)
      (objects robot-list))

    (apply
     #'prioritized-inverse-kinematics
     (list (list (instance move-target-task :init
                           (send *robot0* :larm :end-coords)
                           (send *robot0* :larm :end-coords :copy-worldcoords))
                 (instance move-target-task :init
                           (send *robot9* :larm :end-coords)
                           (make-coords :pos #F(300 100 600))))
           (mapcan #'(lambda (i)
                       (mapcar #'(lambda (j1 j2)
                                   (instance joint-angle-task :init
                                             j1 j2
                                             :check nil))
                               (send (elt robot-list i) :larm :joint-list)
                               (send (elt robot-list (1+ i)) :larm :joint-list)))
                   (range 9)))
     (mapcan #'(lambda (robot)
                 (mapcar #'(lambda (j) (instance joint-variable :init j))
                         (send robot :larm :joint-list)))
             robot-list)
     :task0 (mapcan #'(lambda (robot)
                        (mapcar #'(lambda (j) (instance minmax-angle-task :init j))
                                (send robot :larm :joint-list)))
                    robot-list)
     args
     )
    )
  )
(print "(test-inverse-kinematics-keypose)")

;;浮遊リンク系で両手両足を目標に伸ばす
;;関節上下限を考慮する
(defun test-inverse-kinematics-fullbody
 (&rest
    args
    )
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords))

  (if (set-exclusive-or (send *irtviewer* :objects) (list *robot*))
      (objects (list *robot*)))

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
    (list (instance virtual-joint-variable :init *robot*))
    (mapcar #'(lambda (j) (instance joint-variable :init j))
            (send *robot* :joint-list)))
   :task0 (append
           (list (instance minmax-angle-task :init *robot*
                           :min-angle #F(-200 -200 -200 -20 -20 -10)
                           :max-angle #F(200 200 200 20 20 10)))
           (mapcar #'(lambda (j) (instance minmax-angle-task :init j))
                   (send *robot* :joint-list)))
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

  (if (set-exclusive-or (send *irtviewer* :objects) (list *robot*))
      (objects (list *robot*)))

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
   :task0 (append
           (list (instance minmax-angle-task :init *robot*
                           :min-angle (float-vector -200 -200 -200 -20 -20 -10)
                           :max-angle #F(200 200 200 20 20 10)))
           (mapcar #'(lambda (j) (instance minmax-angle-task :init j))
                  (send *robot* :joint-list)))
   :min-loop 30
   args
   )
  )
(print "(test-inverse-kinematics-fullbody-priority)")


;;浮遊リンク系で両手両足を目標に伸ばし，重心を目標位置へ
;;関節上下限を考慮する
(defun test-inverse-kinematics-cog
 (&rest
    args
    )
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords))

  (if (set-exclusive-or (send *irtviewer* :objects) (list *robot*))
      (objects (list *robot*)))

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
               (instance target-centroid-pos-task :init
                         *robot*
                         :target-centroid-pos #F(0 0 0))
               ))
   (append
    (list (instance virtual-joint-variable :init *robot*))
    (mapcar #'(lambda (j) (instance joint-variable :init j))
            (send *robot* :joint-list)))
   :task0 (append
           (list (instance minmax-angle-task :init *robot*
                           :min-angle #F(-200 -200 -200 -20 -20 -10)
                           :max-angle #F(200 200 200 20 20 10)))
           (mapcar #'(lambda (j) (instance minmax-angle-task :init j))
                   (send *robot* :joint-list)))
   args
   )
  )
(print "(test-inverse-kinematics-cog)")



;;浮遊リンク系で両手両足を目標に伸ばし，重心は可能な限り満たす
;;関節上下限を考慮する
(defun test-inverse-kinematics-cog-priority
 (&rest
    args
    )
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords))

  (if (set-exclusive-or (send *irtviewer* :objects) (list *robot*))
      (objects (list *robot*)))

  (apply
   #'prioritized-inverse-kinematics
   (list (list (instance move-target-task :init
                         (send *robot* :rarm :end-coords)
                         (make-coords :pos #F(700 -200 700)))
               (instance move-target-task :init
                         (send *robot* :larm :end-coords)
                         (make-coords :pos #F(700 100 700)))
               (instance move-target-task :init
                         (send *robot* :rleg :end-coords)
                         (send *robot* :rleg :end-coords :copy-worldcoords))
               (instance move-target-task :init
                         (send *robot* :lleg :end-coords)
                         (send *robot* :lleg :end-coords :copy-worldcoords))
               )
         (list (instance target-centroid-pos-task :init
                         *robot*
                         :target-centroid-pos #F(0 0 0)
                         :check nil)))
   (append
    (list (instance virtual-joint-variable :init *robot*))
    (mapcar #'(lambda (j) (instance joint-variable :init j))
            (send *robot* :joint-list)))
   :task0 (append
           (list (instance minmax-angle-task :init *robot*
                           :min-angle #F(-200 -200 -200 -20 -20 -10)
                           :max-angle #F(200 200 200 20 20 10)))
           (mapcar #'(lambda (j) (instance minmax-angle-task :init j))
                   (send *robot* :joint-list)))
   :min-loop 30
   args
   )
  )
(print "(test-inverse-kinematics-cog-priority)")



;;浮遊リンク系で両手両足を目標に伸ばし，重心を支持領域内へ
;;関節上下限を考慮する
(defun test-inverse-kinematics-cog-support
 (&rest
    args
    )
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords))

  (if (set-exclusive-or (send *irtviewer* :objects) (list *robot*))
      (objects (list *robot*)))

  (apply
   #'prioritized-inverse-kinematics
   (list (list (instance move-target-task :init
                         (send *robot* :rarm :end-coords)
                         (make-coords :pos #F(600 -200 700)))
               (instance move-target-task :init
                         (send *robot* :larm :end-coords)
                         (make-coords :pos #F(600 100 700)))
               (instance move-target-task :init
                         (send *robot* :rleg :end-coords)
                         (send *robot* :rleg :end-coords :copy-worldcoords))
               (instance move-target-task :init
                         (send *robot* :lleg :end-coords)
                         (send *robot* :lleg :end-coords :copy-worldcoords))
               (instance centroid-support-polygon-task :init
                         *robot*
                         :polygons (mapcar #'(lambda (l) (send *robot* :support-polygon l))
                                           (list :rleg-link5 :lleg-link5)))
               ))
   (append
    (list (instance virtual-joint-variable :init *robot*))
    (mapcar #'(lambda (j) (instance joint-variable :init j))
            (send *robot* :joint-list)))
   :task0 (append
           (list (instance minmax-angle-task :init *robot*
                           :min-angle #F(-200 -200 -200 -20 -20 -10)
                           :max-angle #F(200 200 200 20 20 10)))
           (mapcar #'(lambda (j) (instance minmax-angle-task :init j))
                   (send *robot* :joint-list)))
   args
   )
  )
(print "(test-inverse-kinematics-cog-support)")



;;浮遊リンク系で両手を目標に伸ばす．脚は脚どうしの相対位置は固定で，重心が支持領域に入るように配置する
;;関節上下限を考慮する
(defun test-inverse-kinematics-cog-support-leg
 (&rest
    args
    )
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords))

  (if (set-exclusive-or (send *irtviewer* :objects) (list *robot*))
      (objects (list *robot*)))

  (let ((lleg-tmp-coords (make-cascoords :name :lleg-tmp-coords
                                         :parent (send *robot* :lleg :end-coords)
                                         :coords (send *robot* :rleg :end-coords :copy-worldcoords))))
    (unwind-protect
         (apply
          #'prioritized-inverse-kinematics
          (list (list (instance move-target-task :init
                                (send *robot* :rarm :end-coords)
                                (make-coords :pos #F(800 -400 700)))
                      (instance move-target-task :init
                                (send *robot* :larm :end-coords)
                                (make-coords :pos #F(600 300 400)))
                      (instance move-target-task :init
                                (send *robot* :rleg :end-coords)
                                (make-coords)
                                :translation-axis :xy
                                :rotation-axis :z)
                      (instance move-target-task :init
                                (send *robot* :rleg :end-coords)
                                lleg-tmp-coords)
                      (instance centroid-support-polygon-task :init
                                *robot*
                                :polygons (mapcar #'(lambda (l) (send *robot* :support-polygon l))
                                                  (list :rleg-link5 :lleg-link5))
                                :margin 0.002)
                      ))
          (append
           (list (instance virtual-joint-variable :init *robot*))
           (mapcar #'(lambda (j) (instance joint-variable :init j))
                   (send *robot* :joint-list)))
          :task0 (append
                  (list (instance minmax-angle-task :init *robot*
                                  :min-angle #F(-500 -500 -200 -20 -20 -10)
                                  :max-angle #F(500 500 200 20 20 10)))
                  (mapcar #'(lambda (j) (instance minmax-angle-task :init j))
                          (send *robot* :joint-list)))
          args
          )
      (send (send *robot* :lleg :end-coords) :dissoc lleg-tmp-coords)
      )
    )
  )
(print "(test-inverse-kinematics-cog-support-leg)")



;;浮遊リンク系で右足を固定し，左足をつく位置は自由に決めて重心が支持領域上にある範囲内で左手を可能な限り前方に伸ばす
;;関節上下限を考慮する
(defun test-inverse-kinematics-cog-support-priority
 (&rest
    args
    )
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords))

  (if (set-exclusive-or (send *irtviewer* :objects) (list *robot*))
      (objects (list *robot*)))

  (apply
   #'prioritized-inverse-kinematics
   (list (list (instance move-target-task :init
                         (send *robot* :rleg :end-coords)
                         (send *robot* :rleg :end-coords :copy-worldcoords))
               (instance move-target-task :init
                         (send *robot* :lleg :end-coords)
                         (send *robot* :lleg :end-coords :copy-worldcoords)
                         :translation-coords :world
                         :translation-axis :xy
                         :rotation-axis :z)
               (instance centroid-support-polygon-task :init
                         *robot*
                         :polygons (mapcar #'(lambda (l) (send *robot* :support-polygon l))
                                           (list :rleg-link5 :lleg-link5))
                         :margin 0.02)
               )
         (list (instance move-target-task :init
                         (send *robot* :larm :end-coords)
                         (make-coords :pos #F(3000 0 500))
                         :translation-axis :yz
                         :translation-coords :world
                         :check nil)))
   (append
    (list (instance virtual-joint-variable :init *robot*))
    (mapcar #'(lambda (j) (instance joint-variable :init j))
            (send *robot* :joint-list)))
   :task0 (append
           (list (instance minmax-angle-task :init *robot*
                           :min-angle #F(-500 -500 -500 -20 -20 -10)
                           :max-angle #F(500 500 500 20 20 10)))
           (mapcar #'(lambda (j) (instance minmax-angle-task :init j))
                   (send *robot* :joint-list)))
   :min-loop 30
   args
   )
  )
(print "(test-inverse-kinematics-cog-support-priority)")
