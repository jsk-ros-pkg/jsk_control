(defvar *model-file* "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknt-interface.l")
(defvar *model-class* 'hrp2jsknt-robot)
;; default hrp3 hand offset is (float-vector 100 70 0)
(defvar *board-position-offset* (float-vector (+ 100) (+ 70 65) 0))
(defvar *board-orientation-offset* (float-vector 1 0 0 0))

(defun setup-model ()
  (load *model-file*)
  (setq *robot* (instance (eval *model-class*) :init))
  (objects (list *robot*)))

(defun setup-pose ()
  (send *robot* :reset-pose)
  (setq *view-coords-l*
        (make-cascoords :coords (send *robot* :larm :end-coords :copy-worldcoords)))
  (let ((offset (make-coords :pos *board-position-offset*
                             :rot (quaternion2matrix *board-orientation-offset*))))
    (send offset :rotate (deg2rad -90) :y)
    (send *view-coords-l* :transform offset))
  (send (send *robot* :larm :end-coords :parent) :assoc *view-coords-l*)
  
  (setq *view-coords-r*
        (make-cascoords :coords (send *robot* :rarm :end-coords :copy-worldcoords)))
  (send *view-coords-r* :translate #f(90 75 0))
  (send (send *robot* :rarm :end-coords :parent) :assoc *view-coords-r*)
  )


(defun next-best-direction (z)
  (if (< (abs (elt z 1)) (abs (elt z 0)))
      (if (< (abs (elt z 1)) (abs (elt z 2)))
          (if (<= (elt z 1) 0) (float-vector 0 1 0) (float-vector 0 -1 0))
        (if (<= (elt z 2) 0) (float-vector 0 0 1) (float-vector 0 0 -1)))
    (if (< (abs (elt z 0)) (abs (elt z 2)))
        (if (<= (elt z 0) 0) (float-vector 1 0 0) (float-vector -1 0 0))
      (if (<= (elt z 2) 0) (float-vector 0 0 1) (float-vector 0 0 -1)))))

(defun make-rotation-mat (z)
  (let* ((z-axis (normalize-vector z))
         (tmp (next-best-direction z-axis))
         (y-axis (normalize-vector (v* z-axis tmp)))
         (x-axis (normalize-vector (v* y-axis z-axis))))
    (transpose (matrix x-axis y-axis z-axis))))

(defun set-hrp2-pose (r)
  (multiple-value-bind (y p l arm ret) r
    (send *robot* :reset-pose)
    (send *robot* :head :neck-p :joint-angle p)
    (send *robot* :head :neck-y :joint-angle (- y))
    (if ret (send *robot* arm :angle-vector ret))
    (send *irtviewer* :draw-objects)
    ))

(defun inverse-kinematics (r &key (rotation-axis :z) (angle))
  (set-hrp2-pose r)
  (multiple-value-bind (y p l arm ret) r
    (let* ((cds (hrp2-camera-coords))
           (pos (send cds :transform-vector (float-vector 0 0 l)))
           (n (normalize-vector (v- (send (hrp2-camera-coords) :worldpos) pos)))
           (m (make-rotation-mat n))
           (c (make-coords :pos pos :rot m)) rr)
      (if angle (send* c :rotate angle))
      (if rotation-axis
          (send *robot* arm :inverse-kinematics c
                :move-target (case arm (:larm *view-coords-l*) (:rarm *view-coords-r*))
                :step 500 :rotation-axis nil)
        (send *robot* :reset-pose))
      (if (setq rr (send *robot* arm :inverse-kinematics c
                         :move-target (case arm (:larm *view-coords-l*) (:rarm *view-coords-r*))
                         :stop 500 :rotation-axis rotation-axis))
    (progn
      (send *irtviewer* :draw-objects)
      (send c :draw-on :flush t :color (float-vector 1 0 0))
      (list y p l arm (send *robot* arm :angle-vector)))))))

(defun make-viewing-pose (&key (arm :larm) (step 60)
                               (yaw-list (list 40 -28 -16 -2 6 -40))
                               (pitch-list (list -18 3 18 30))
                               (add-extra-head-move t))
  (let (lst ret ext (sign 1))
    (dolist (y yaw-list)
      (dolist (p pitch-list)
        (dolist (l ;;(list 420 540 460 580 380 340 620 660))
                 (list 720 680 640 600 560 520 480 440 400))
          (when (setq ret (inverse-kinematics (list y p l arm nil)
                                              :angle (list (* 0.15 (random 1.0))
                                                           (normalize-vector (random-vector)))))
            (let (ll ang0 (ang1 (float-vector 0 0 0))
                     (ang (+ (random 0.2) 0.3)))
              (push ret ll)
              ;;
              (when add-extra-head-move
                (let ((py (copy-object (car ll)))
                      (pp (copy-object (car ll))))
                  (setf (elt py 0) (case arm (:larm (+ (elt py 0) (* sign -4))) (:rarm (+ (elt py 0) (* sign 4)))))
                  (setf (elt pp 1) (+ (elt pp 1) (* sign 4)))
                  (push py ll)
                  (push pp ll)))
              (setq sign (* -1 sign))
              (setq ang0 (random-vector))
              (setf (elt ang0 2) 0)

              (setf (elt ang1 0) (elt ang0 1))
              (setf (elt ang1 1) (- (elt ang0 0)))

              (if (setq ret (inverse-kinematics (list y p (- l step) arm (car (last ret)))
                                                :angle (list (* sign ang) ang0)))
                  (push ret ll)
                (if (setq ret (inverse-kinematics (list y p (- l step) arm (car (last ret)))
                                                  :angle (list (- (* sign ang)) ang0)))
                    (push ret ll)
                  (warn "~%~%~%~A~%~%~%" (list y p l :x))))
              (if (setq ret (inverse-kinematics (list y p (- l step) arm (car (last ret)))
                                                :angle (list (* sign ang) ang1)))
                  (push ret ll)
                (if (setq ret (inverse-kinematics (list y p (- l step) arm (car (last ret)))
                                                  :angle (list (- (* sign ang)) ang1)))
                    (push ret ll)
                  (warn "~%~%~%~A~%~%~%" (list y p l :y))))
              ;;
              (setq ll (sort ll #'(lambda (x y) (>= (elt x 2) (elt y 2)))))
              (dolist (l ll) (push l lst))
              (return)))
          )))
    lst))

;; (defun make-pose-list ()
;;   (setup-pose)
;;   (setq *larm-pose-list* (make-viewing-pose)) ;; for :larm
;;   (setq *rarm-pose-list* (make-viewing-pose :arm :rarm :yaw-list (list 2 10 20 28)))
;;   (send *robot* :reset-pose)
;;   )

(defun hrp2-camera-coords ()
  (send (send *robot* :carmine) :copy-worldcoords))


(setup-model)
(setup-pose)
(objects (list *robot* *view-coords-l*))
;;(make-pose-list)
;;(setq *larm-pose-list* (make-viewing-pose)) ;; for :larm
