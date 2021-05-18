(load "package://eus_qp/prioritized-inverse-kinematics/euslisp/prioritized-inverse-kinematics-compiled.l")
(load "package://eus_qp/euslisp/contact-optimization.l")

(defun setup ()
  ;;ここで生成するオブジェクトや設定は，test関数で明示的に用いられていない限りはprioritized-inverse-kinematics関数の中で使用されることはない．
  (unless (boundp '*robot*)
    (load "irteus/demo/sample-robot-model.l")
    (setq *robot* (instance sample-robot :init))
    (send *robot* :set-val 'support-polygons (append
                                              (mapcar #'(lambda (l) (send *robot* :make-sole-polygon-ex l))
                                                      (list :rleg-link5 :lleg-link5 :rarm-link6 :larm-link6))
                                              (send *robot* :get-val 'support-polygons)))
    (mapcar #'(lambda (l ee)
      (send (send *robot* :link l) :put :contact-constraint
            (instance* default-contact-constraint
                       :init
                       :name l
                       :mu-margin-ratio 1.0 :cop-margin-ratio 1.0
                       :mu-trans 0.5
                       :mu-rot 0.05
                       (let* ((vs (mapcar #'(lambda (v) (send (send *robot* ee :end-coords) :inverse-transform-vector v)) (send (send *robot* :support-polygon l) :vertices))))
                         (list :l-min-x (elt (find-extream vs #'(lambda (v) (elt v 0)) #'<) 0)
                               :l-max-x (elt (find-extream vs #'(lambda (v) (elt v 0)) #'>) 0)
                               :l-min-y (elt (find-extream vs #'(lambda (v) (elt v 1)) #'<) 1)
                               :l-max-y (elt (find-extream vs #'(lambda (v) (elt v 1)) #'>) 1)
                               )))
            ))
            (list :rleg-link5 :lleg-link5)
            (list :rleg :lleg))
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
        (mapcar #'(lambda (l ee)
                    (send (send robot :link l) :put :contact-constraint
                          (instance* default-contact-constraint
                                     :init
                                     :name l
                                     :mu-margin-ratio 1.0 :cop-margin-ratio 1.0
                                     :mu-trans 0.5
                                     :mu-rot 0.05
                                     (let* ((vs (mapcar #'(lambda (v) (send (send robot ee :end-coords) :inverse-transform-vector v)) (send (send robot :support-polygon l) :vertices))))
                                       (list :l-min-x (elt (find-extream vs #'(lambda (v) (elt v 0)) #'<) 0)
                                             :l-max-x (elt (find-extream vs #'(lambda (v) (elt v 0)) #'>) 0)
                                             :l-min-y (elt (find-extream vs #'(lambda (v) (elt v 1)) #'<) 1)
                                             :l-max-y (elt (find-extream vs #'(lambda (v) (elt v 1)) #'>) 1)
                                             )))
                          ))
                (list :rleg-link5 :lleg-link5)
                (list :rleg :lleg))
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



;;浮遊リンク系で両手両足を目標に伸ばし，重心を静的重心実行可能領域内へ
;;関節上下限を考慮する
(defun test-inverse-kinematics-cog-scfr
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
                         (make-coords :pos #F(500 -200 700)))
               (instance move-target-task :init
                         (send *robot* :larm :end-coords)
                         (make-coords :pos #F(500 100 700)))
               (instance move-target-task :init
                         (send *robot* :rleg :end-coords)
                         (send *robot* :rleg :end-coords :copy-worldcoords))
               (instance move-target-task :init
                         (send *robot* :lleg :end-coords)
                         (send *robot* :lleg :end-coords :copy-worldcoords))
               (instance centroid-scfr-task :init
                         *robot*
                         :contact-constraints (mapcar #'(lambda (l) (send (send *robot* :link l) :get :contact-constraint))
                                                      (list :rleg-link5 :lleg-link5))
                         :contact-coords (mapcar #'(lambda (l) (send *robot*  l :end-coords))
                                                 (list :rleg :lleg)))
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
(print "(test-inverse-kinematics-cog-scfr)")


;;浮遊リンク系で両手両足を目標に伸ばし，重心を静的重心実行可能領域内へ
;;関節上下限を考慮する
(defun test-inverse-kinematics-cog-scfr2
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
                         (make-coords :pos #F(500 -400 700)))
               (instance move-target-task :init
                         (send *robot* :larm :end-coords)
                         (make-coords :pos #F(500 -100 700)))
               (instance move-target-task :init
                         (send *robot* :rleg :end-coords)
                         (make-coords :pos #F(200 -400 100) :rpy #F(0.1 0.1 0.1)))
               (instance move-target-task :init
                         (send *robot* :lleg :end-coords)
                         (make-coords :pos #F(-200 -100 200) :rpy #F(0.4 0.7 -0.2)))
               )
         (list
          (instance centroid-scfr-task :init
                    *robot*
                    :contact-constraints (mapcar #'(lambda (l) (send (send *robot* :link l) :get :contact-constraint))
                                                 (list :rleg-link5 :lleg-link5))
                    :contact-coords (mapcar #'(lambda (l) (send *robot*  l :end-coords))
                                            (list :rleg :lleg)))))
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
(print "(test-inverse-kinematics-cog-scfr2)")



;;今の関節角度から離れる
;;(send *robot* :reset-pose)
;;(send *robot* :fix-leg-to-coords (make-coords))
(defun test-inverse-kinematics-avoid ()
  (prioritized-inverse-kinematics
   (list (list (instance move-target-task :init
                         (send *robot* :larm :end-coords)
                         (make-coords :pos #F(300 100 600))))
         (mapcar #'(lambda (j ang)
                     (instance joint-angle-task :init
                               j ang
                               :W -5e-2
                               :check nil))
                 (send *robot* :larm :joint-list)
                 (coerce (v+ (scale 1.1 (v- (send *robot* :larm :angle-vector) #F(10.0 20.0 0.0 -20.0 10.0 0.0 0.0))) #F(10.0 20.0 0.0 -20.0 10.0 0.0 0.0)) cons)
                 )
         )
   (mapcar #'(lambda (j) (instance joint-variable :init j))
           (send *robot* :larm :joint-list))
   :min-loop 100 :regular (list 1e-6 1)
   :task0 (mapcar #'(lambda (j) (instance minmax-angle-task :init j))
                  (send *robot* :larm :joint-list))
   )
  )
(print "(test-inverse-kinematics-avoid)")



;;前回までのIK結果の関節角度から一定距離以上離れる
(defun test-inverse-kinematics-infeasible-prev
 (&rest
    args
    )

  (if (set-exclusive-or (send *irtviewer* :objects) (list *robot*))
      (objects (list *robot*)))

  (let ((infeasible-angle-vectors nil))
    (dotimes (i 30)
      (send *robot* :reset-pose)
      (send *robot* :fix-leg-to-coords (make-coords))
      (if
       (apply
        #'prioritized-inverse-kinematics
        (list (list (instance move-target-task :init
                              (send *robot* :larm :end-coords)
                              (make-coords :pos #F(300 100 650))))
              (mapcar #'(lambda (infeasible-angle-vector)
                          (instance infeasible-angle-vector-task :init
                                    (send *robot* :larm :joint-list)
                                    infeasible-angle-vector
                                    :min-distance 0.2
                                    )
                          )
                      infeasible-angle-vectors)
              )
        (mapcar #'(lambda (j) (instance joint-variable :init j))
                (send *robot* :larm :joint-list))
        :task0 (append
                (mapcar #'(lambda (j) (instance minmax-angle-task :init j))
                        (send *robot* :larm :joint-list))
                (mapcar #'(lambda (j) (instance joint-velocity-task :init j :max-vel 0.01 :min-vel -0.01 :periodic-time 1.0));;1回の反復計算で最大0.01rad/s * 1.0sだけ動く
                        (send *robot* :larm :joint-list))
                )
        :stop 1000
        :revert-if-fail nil
        :regular 1e-2
        :regular-rel 5e4
        :regular-max 1e0
        args
        )
       (print "ik success")
       (print "ik fail"))
      (push (send *robot* :larm :angle-vector) infeasible-angle-vectors)
      (print "press enter")
      (read-char)
      )
    )
  )
(print "(test-inverse-kinematics-infeasible-prev)")


;;前回までのIK結果の関節角度から離れる
(defun test-inverse-kinematics-avoid-prev
 (&rest
    args
    )

  (if (set-exclusive-or (send *irtviewer* :objects) (list *robot*))
      (objects (list *robot*)))

  (let ((infeasible-angle-vectors nil))
    (dotimes (i 5)
      (send *robot* :reset-pose)
      (send *robot* :fix-leg-to-coords (make-coords))
      (if
       (apply
        #'prioritized-inverse-kinematics
        (list (list (instance move-target-task :init
                              (send *robot* :larm :end-coords)
                              (make-coords :pos #F(300 100 650))))
              (list (instance avoid-angle-vectors-task :init
                              (send *robot* :larm :joint-list)
                              infeasible-angle-vectors
                              :W 1e-2))
              )
        (mapcar #'(lambda (j) (instance joint-variable :init j))
                (send *robot* :larm :joint-list))
        :task0 (append
                (mapcar #'(lambda (j) (instance minmax-angle-task :init j))
                        (send *robot* :larm :joint-list))
                (mapcar #'(lambda (j) (instance joint-velocity-task :init j :max-vel 0.01 :min-vel -0.01 :periodic-time 1.0));;1回の反復計算で最大0.01rad/s * 1.0sだけ動く
                        (send *robot* :larm :joint-list))
                )
        :stop 1000
        :revert-if-fail nil
        :regular 1e-2
        :regular-rel 5e4
        :regular-max 1e0
        args
        )
       (print "ik success")
       (print "ik fail"))
      (push (send *robot* :larm :angle-vector) infeasible-angle-vectors)
      (print "press enter")
      (read-char)
      )
    )
  )
(print "(test-inverse-kinematics-avoid-prev)")
