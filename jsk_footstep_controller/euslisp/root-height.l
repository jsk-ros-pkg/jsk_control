(ros::roseus-add-msgs "jsk_recognition_msgs")

;; manipulability ;;;;;;;;;;;;;;;;;;
(defun calc-manipulability-from-limb
  (&optional (limb :lleg))
  (send *robot* :update-descendants)
  (let* ((move-target (send *robot* limb :end-coords))
         (link-list (send *robot* :link-list (send move-target :parent)))
         (jacobi (send *robot* :calc-jacobian-from-link-list
                       link-list
                       :move-target move-target
                       :rotation-axis t
                       :translation-axis t))
         )
    (manipulability jacobi)
    )
  )

(defun test-manipulability
  (&key (limb :lleg) (wait? nil))
  (send *robot* :init-pose)
  (do-until-key
   (unless (send *robot* limb :move-end-pos #f(0 0 10) :local)
     (return-from nil nil))
   (send *irtviewer* :draw-objects)
   (print (calc-manipulability-from-limb limb))
   (when wait? (read-line))
   )
  )


;; joint torque ;;;;;;;;;;;;;;;;;;
(defun calc-joint-torque-from-limb
  (&optional (limb :lleg))
  (send *robot* :update-descendants)
  (calc-self-wieght-torque)
  (let* ((move-target (send *robot* limb :end-coords))
         (link-list (send *robot* :link-list (send move-target :parent)))
         (joint-list (send-all link-list :joint))
         (torque-list (send-all joint-list :joint-torque))
         (normalized-torque-list (mapcar #'(lambda (v j) (/ v (send j :max-joint-torque))) torque-list
                                         joint-list))
         )
    (expt (norm (coerce normalized-torque-list float-vector)) 2)
    )
  )


(defun calc-self-wieght-torque
  ()
  (let* ((leg-list (list :lleg :rleg))
         (leg-coords-list (mapcar #'(lambda (limb) (send *robot* limb :end-coords)) leg-list))
         (leg-pos-list (send-all leg-coords-list :worldpos))
         (force-moment-list (send *robot* :calc-contact-wrenches-from-total-wrench leg-pos-list))
         )
    (send *robot* :calc-torque
          :force-list
          (elt force-moment-list 0)
          :moment-list
          (elt force-moment-list 1)
          :target-coords
          leg-coords-list
          )
    )
  )

(defun test-joint-torque
  (&key (limb :lleg) (wait? nil))
  (send *robot* :init-pose)
  (do-until-key
   (unless (send *robot* limb :move-end-pos #f(0 0 10) :local)
     (return-from nil nil))
   (send *irtviewer* :draw-objects)
   (print (calc-joint-torque-from-limb limb))
   (when wait? (read-line))
   )
  )


;; index ;;;;;;;;;;;;;;;;;;
(defun calc-torque-manipulability-index ;; smaller is better
    (&optional (limb :lleg))
  "(joint-value manipulability-value torque-value)"
  (let* ((manipulability-gain 1.0)
         (torque-gain 0.00005) ;; make larger if you want knee to be more stretched
         )
    (list
     (+ (- (* manipulability-gain (calc-manipulability-from-limb limb)))
        (* torque-gain (calc-joint-torque-from-limb limb)))
     (calc-manipulability-from-limb limb)
     (calc-joint-torque-from-limb limb))
    )
  )

(defun test-torque-manipulability-index
  (&key (limb :lleg) (wait? nil))
  (send *robot* :init-pose)
  (let ((step 10) (i 0)
        (min-index 1e10) (min-pos 0)
        index
        )
    ;; loop
    (do-until-key
     (incf i)
     (unless (send *robot* limb :move-end-pos (float-vector 0 0 step) :local :warnp nil)
       (return-from nil nil))
     (send *irtviewer* :draw-objects)
     (warning-message 2 "pos: ~a  /  manip: ~a  torq: ~a  index: ~a~%"
                      (* step i)
                      (calc-manipulability-from-limb limb)
                      (calc-joint-torque-from-limb limb)
                      (calc-torque-manipulability-index limb))
     (setq index (calc-torque-manipulability-index limb))
     (when (< index min-index)
       (setq min-index index)
       (setq min-pos (* i step))
       )
     (when wait? (read-line))
     )
    ;; go to best pose
    (send *robot* :init-pose)
    (send *robot* limb :move-end-pos (float-vector 0 0 min-pos) :local)
    (send *irtviewer* :draw-objects)
    (warning-message 1 "min-pos: ~a  min-index: ~a~%"
                     min-pos min-index)
    )
  )


;; calc best pose considering both legs ;;;;;;;;;;;;;;;;;;
(defun calc-torque-manipulability-index-for-legs
  ()
  (mapcar #'+
          (calc-torque-manipulability-index :lleg)
          (calc-torque-manipulability-index :rleg))
   )

(defun determine-best-root-height-from-footcoords-with-publish
    (robot lleg-coords rleg-coords &key (draw? nil) (debug? nil))
  (multiple-value-bind
   (joint-plot-data torque-plot-data manipulability-plot-data)
   (determine-best-root-height-from-footcoords robot lleg-coords rleg-coords
                                               :draw? draw? :debug? debug?)
   (ros::publish "/joint_plot" joint-plot-data)
   (ros::publish "/torque_plot" torque-plot-data)
   (ros::publish "/manipulability_plot" manipulability-plot-data)
   ))

;; return best root height from current robot pose
(defun determine-best-root-height-from-footcoords
    (robot lleg-coords rleg-coords &key (draw? nil) (debug? nil))
  ;; First move robot model to midcoords
  (let ((mid-coords (midcoords 0.5 lleg-coords rleg-coords)))
    (send robot :fix-leg-to-coords mid-coords))
  ;; Next, solve inverse kinematics for each coords
  (when (not (and (send robot :lleg :inverse-kinematics lleg-coords)
                  (send robot :rleg :inverse-kinematics rleg-coords)))
    (warning-message 2 "Failed to solve ik")
    (return-from determine-best-root-height-from-footcoords nil))
  (let ((step 10) (i 0)
        (min-index 1e10) (min-pos 0)
        (joint-plot-data (instance jsk_recognition_msgs::PlotData :init))
        (torque-plot-data (instance jsk_recognition_msgs::PlotData :init))
        (manipulability-plot-data (instance jsk_recognition_msgs::PlotData :init))
        index
        (original-av (copy-seq (send robot :angle-vector)))
        (original-foot-midcoords (send (send robot :foot-midcoords) :copy-worldcoords))
        )
    ;; loop for bending direction
    (while t
      (when draw? (send *irtviewer* :draw-objects))
      ;; build data for plotting
      (let ((tmp (calc-torque-manipulability-index-for-legs)))
        (let ((height (abs (elt (send (send (send *robot* :body_lk) :transformation
                                            (send *robot* :foot-midcoords))
                                      :worldpos) 2))))
          (send joint-plot-data :ys
                (append (send joint-plot-data :ys)
                        (list (car tmp))))
          (send torque-plot-data :ys
                (append (send torque-plot-data :ys)
                        (list (caddr tmp))))
          (send manipulability-plot-data :ys
                (append (send manipulability-plot-data :ys)
                        (list (cadr tmp))))
          (send joint-plot-data :xs
                (append (send joint-plot-data :xs) (list height)))
          (send manipulability-plot-data :xs
                (append (send manipulability-plot-data :xs) (list height)))
          (send torque-plot-data :xs
                (append (send torque-plot-data :xs) (list height)))
          (setq index (car tmp))))
      (when (< index min-index)
        (setq min-index index)
        (setq min-pos (* i step))
        )
      (when debug? (warning-message 2 "pos: ~a  /  index: ~a~%" (* step i) index))
      (unless (every #'eval (send robot :legs :move-end-pos (float-vector 0 0 step) :local :warnp nil))
        (return-from nil nil))
      (incf i)
      )
    ;; restore original state
    (send robot :angle-vector original-av)
    (send robot :fix-leg-to-coords original-foot-midcoords)
    ;; loop for stretching direction
    (setq i 0)
    (setq step (- step))
    (while t
      (when draw? (send *irtviewer* :draw-objects))
            ;; build data for plotting
      (let ((tmp (calc-torque-manipulability-index-for-legs)))
        (let ((height (abs (elt (send (send (send *robot* :body_lk) :transformation
                                            (send *robot* :foot-midcoords))
                                      :worldpos) 2))))
          (send joint-plot-data :ys
                (append (send joint-plot-data :ys)
                        (list (car tmp))))
          (send torque-plot-data :ys
                (append (send torque-plot-data :ys)
                        (list (caddr tmp))))
          (send manipulability-plot-data :ys
                (append (send manipulability-plot-data :ys)
                        (list (cadr tmp))))
          (send joint-plot-data :xs
                (append (send joint-plot-data :xs) (list height)))
          (send manipulability-plot-data :xs
                (append (send manipulability-plot-data :xs) (list height)))
          (send torque-plot-data :xs
                (append (send torque-plot-data :xs) (list height)))
          (setq index (car tmp))))
      (when (< index min-index)
        (setq min-index index)
        (setq min-pos (* i step))
        )
      (when debug? (warning-message 2 "pos: ~a  /  index: ~a~%" (* step i) index))
      (unless (every #'eval (send robot :legs :move-end-pos (float-vector 0 0 step) :local :warnp nil))
        (return-from nil nil))
      (incf i)
      )
    ;; go to best pose
    (send robot :angle-vector original-av)
    (send robot :fix-leg-to-coords original-foot-midcoords)
    (send robot :legs :move-end-pos (float-vector 0 0 min-pos) :local)
    (when draw? (send *irtviewer* :draw-objects))
    (warning-message 1 "min-pos: ~a  min-index: ~a~%"
                     min-pos min-index)
    ;; sort plot data
    (list joint-plot-data torque-plot-data manipulability-plot-data)))

(defun test-root-height-init ()
  (ros::roseus "test_root_height")
  (ros::advertise "/joint_plot" jsk_recognition_msgs::PlotData)
  (ros::advertise "/torque_plot" jsk_recognition_msgs::PlotData)
  (ros::advertise "/manipulability_plot" jsk_recognition_msgs::PlotData)
  )

(defun test-root-height0
  ()
  (send *robot* :init-pose)
  (determine-best-root-height-from-footcoords-with-publish
   *robot*
   (send *robot* :lleg :end-coords :copy-worldcoords)
   (send *robot* :rleg :end-coords :copy-worldcoords)
   :draw? t)
  )

(defun test-root-height1
  ()
  (send *robot* :reset-pose)
  (determine-best-root-height-from-footcoords-with-publish *robot* (send *robot* :lleg :end-coords :copy-worldcoords) (send *robot* :rleg :end-coords :copy-worldcoords) :draw? t)
  )

(defun test-root-height2
  ()
  (send *robot* :reset-pose)
  (send *robot* :lleg :move-end-pos (float-vector 100 100 0))
  (determine-best-root-height-from-footcoords-with-publish *robot* (send *robot* :lleg :end-coords :copy-worldcoords) (send *robot* :rleg :end-coords :copy-worldcoords) :draw? t)
  )

(defun test-root-height3
  ()
  (send *robot* :reset-pose)
  (send *robot* :lleg :move-end-pos (float-vector 100 100 0))
  (send *robot* :lleg :move-end-rot 20 :z :local)
  (determine-best-root-height-from-footcoords-with-publish *robot* (send *robot* :lleg :end-coords :copy-worldcoords) (send *robot* :rleg :end-coords :copy-worldcoords) :draw? t)
  )

(defun test-root-height4
  ()
  (send *robot* :reset-pose)
  (send *robot* :lleg :move-end-pos (float-vector 100 100 0))
  (send *robot* :lleg :move-end-rot 20 :z :local)
  (send *robot* :rleg :move-end-pos (float-vector -100 -100 0))
  (send *robot* :rleg :move-end-rot -20 :z :local)
  (determine-best-root-height-from-footcoords-with-publish *robot* (send *robot* :lleg :end-coords :copy-worldcoords) (send *robot* :rleg :end-coords :copy-worldcoords) :draw? t)
  )

(defun test-root-height5
  ()
  (send *robot* :reset-pose)
  (send *robot* :lleg :move-end-pos (float-vector 0 0 100))
  (determine-best-root-height-from-footcoords-with-publish *robot* (send *robot* :lleg :end-coords :copy-worldcoords) (send *robot* :rleg :end-coords :copy-worldcoords) :draw? t)
  )

(defun test-root-height6
  ()
  (send *robot* :reset-pose)
  (send *robot* :lleg :move-end-pos (float-vector 0 0 200))
  (determine-best-root-height-from-footcoords-with-publish *robot* (send *robot* :lleg :end-coords :copy-worldcoords) (send *robot* :rleg :end-coords :copy-worldcoords) :draw? t)
  )

(defun test-root-height7
  ()
  (send *robot* :reset-pose)
  (send *robot* :lleg :move-end-pos (float-vector 0 0 300))
  (determine-best-root-height-from-footcoords-with-publish *robot* (send *robot* :lleg :end-coords :copy-worldcoords) (send *robot* :rleg :end-coords :copy-worldcoords) :draw? t)
  )

(defun test-root-height8
  ()
  (send *robot* :reset-pose)
  (send *robot* :lleg :move-end-pos (float-vector 200 0 200))
  (determine-best-root-height-from-footcoords-with-publish *robot* (send *robot* :lleg :end-coords :copy-worldcoords) (send *robot* :rleg :end-coords :copy-worldcoords) :draw? t)
  )

(defun test-root-height9
  ()
  (send *robot* :reset-pose)
  (send *robot* :lleg :move-end-pos (float-vector 200 0 200))
  (send *robot* :lleg :move-end-rot 20 :z :local)
  (send *robot* :rleg :move-end-rot -20 :z :local)
  (determine-best-root-height-from-footcoords-with-publish *robot* (send *robot* :lleg :end-coords :copy-worldcoords) (send *robot* :rleg :end-coords :copy-worldcoords) :draw? t)
  )

(defun test-root-height-all
  (&optional (read-line t))
  (dotimes (i 10)
    (warning-message 2 "try (~a)~%" (format nil "test-root-height~a" i))
    (funcall (read-from-string (format nil "test-root-height~a" i)))
    (if read-line (read-line))
    )
  )



;; sample code
;; (load "root-height.l")
;; (load "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknt-interface.l")
;; (setq *robot* (instance hrp2jsknt-robot :init))
;; (objects (list *robot*))
;; (test-root-height-init)
;; (test-root-height-all)
;; In order to plot data, please run following command in other terminals:
;;   $ rosrun jsk_rqt_plugins rqt_2d_plot /joint_plot
;;   $ rosrun jsk_rqt_plugins rqt_2d_plot /torque_plot
;;   $ rosrun jsk_rqt_plugins rqt_2d_plot /manipulability_plot
