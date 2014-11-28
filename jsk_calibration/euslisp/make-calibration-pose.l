
;; class to store calibration motion for one link
(defclass calibration-motion
  :super propertied-object
  :slots (robot-model sampling-joints distances end-effector-offset limb
          reset-method motion-anglevectors camera-method filter-sampling))

(defmethod calibration-motion
  (:init (&key ((:robot-model arobot-model) nil)
               ((:sampling-joint asampling-joint) nil)
               ((:distances adistances) nil)
               ((:camera-method acamera-method) nil)
               ((:end-effector-offset aend-effector-offset) nil)
               ((:reset-method areset-method) nil)
               ((:filter-sampling afilter-sampling) nil)
               ((:limb alimb) nil))
    (setq filter-sampling afilter-sampling)
    (setq camera-method acamera-method)
    (setq reset-method areset-method)
    (setq limb alimb)
    (setq robot-model arobot-model)
    (setq sampling-joints asampling-joint)
    (setq distances adistances)
    (setq end-effector-offset aend-effector-offset)
    self)
  (:sampling-joint-angle-combinations ()
    ;; '((joint-a . angle) (joint-a . angle) (joint-a . angle) ...)
    (let ((angles-set
           (mapcar #'(lambda (j)
                       (mapcar #'(lambda (angle)
                                   (cons (send j :joint)
                                         angle))
                               (send j :angles)))
                   sampling-joints)))
      ;; build combination
      (combinate-angles angles-set)))
  (:generate-motion ()
    (let* ((sampling-angle-set (send self :sampling-joint-angle-combinations))
           (success-count 0)
           (test-counter 0)
           (test-num (* (length distances)
                        (length sampling-angle-set)))
           (success-set nil))
      (format t "~A sampling joint~%" (length sampling-joints))
      (format t "~A combinated sampling joint angles~%" test-num)
      (objects (list robot-model))
      (dolist (sampling-angle sampling-angle-set)
        (let ((possiblep t))
          (when filter-sampling
            (setq possiblep
                  (apply filter-sampling
                         (mapcar #'cdr sampling-angle))))
          (if possiblep
            (dolist (distance distances)
              (send robot-model reset-method)
              (send self :set-model-sampling-angle sampling-angle)
              ;;(send *irtviewer* :draw-objects)
              (let ((ik-result (send self :solve-ik distance
                                     :all-test-num test-num
                                     :success-count success-count
                                     :current-test-no test-counter)))
                (incf test-counter)
                (when ik-result
                  (incf success-count)
                  (push ik-result success-set))
                ))
            (progn
              (format t "~A/~A skip test by filtering~%" test-counter test-num)
              (incf test-counter)))))
      (format t "motion result~%")
      (format t " motions:: ~A/~A~%" (length success-set) test-num)
      ;; animation
      (setq motion-anglevectors success-set)
      (send self :animate)))
  (:animate ()
    (dolist (av motion-anglevectors)
      (send robot-model :angle-vector av)
      (send *irtviewer* :draw-objects)
      ))
  (:solve-ik (l &key (rotation-axis :z)
                (all-test-num) (success-count) (current-test-no))
    ;; setup move-target
    (let ((move-target
           (make-cascoords :coords (send robot-model limb :end-coords
                                         :copy-worldcoords)))
          (ik-successp nil)
          (collision-freep nil))
      (send move-target :transform end-effector-offset)
      (send (send robot-model limb :end-coords :parent) :assoc move-target)
      (let* ((cds (send (send robot-model camera-method) :copy-worldcoords))
             (pos (send cds :transform-vector (float-vector 0 0 l)))
             (n (normalize-vector
                 (v- (send (send (send robot-model camera-method)
                                 :copy-worldcoords)
                           :worldpos) pos)))
             (m (make-rotation-mat n))
             (c (make-coords :pos pos :rot m)) rr)
        ;;(if angle (send* c :rotate angle))
        (if rotation-axis
            (send robot-model limb :inverse-kinematics c
                  :move-target move-target
                  :debug-view nil
                  :dump-command nil
                  :warnp nil
                  ;;:step 500
                  :rotation-axis nil))
        (setq rr (send robot-model limb :inverse-kinematics c
                       :move-target move-target
                       :debug-view nil
                       :dump-command nil
                       :warnp nil
                       ;;:stop 500
                       :rotation-axis rotation-axis))
        (if rr (setq ik-successp t))
        (when ik-successp
          (setq collision-freep (not (send robot-model :self-collision-check))))
        (cond ((not ik-successp)
               (format t "~A/~A fail IK~%" current-test-no all-test-num))
              ((and ik-successp (not collision-freep))
               (format t "~A/~A success IK but NOT collision free~%"
                       current-test-no all-test-num))
              ((and ik-successp collision-freep)
               (format t "~A/~A success IK and collision free~%"
                       current-test-no all-test-num)))
        (when rr
          (send *irtviewer* :draw-objects)
          (send c :draw-on :flush t :color (float-vector 1 0 0))
          )
        rr)))
  (:set-model-sampling-angle (sample-angles)
    (dolist (sample-angle sample-angles)
      (send (car sample-angle) :joint-angle (cdr sample-angle))))
  )

;; angles := '(((joint-a . angle0) (joint-a . angle1) (joint-a . angle2))
;;             ((joint-b . angle0) (joint-b . angle1) (joint-b . angle2))
;;             ((joint-c . angle0) (joint-c . angle1) (joint-c . angle2))
;;             ((joint-d . angle0) (joint-d . angle1) (joint-d . angle2)))
;; return := (((joint-a . angle0) (joint-b . angle0) (joint-c . angle0) (joint-d . angle0))
;;            ((joint-a . angle0) (joint-b . angle0) (joint-c . angle0) (joint-d . angle1))
;;            ...)
;; test code
#|
(combinate-angles '(((joint-a . angle0) (joint-a . angle1) (joint-a . angle2))
                    ((joint-b . angle0) (joint-b . angle1) (joint-b . angle2))
                    ((joint-c . angle0) (joint-c . angle1) (joint-c . angle2))
                   ((joint-d . angle0) (joint-d . angle1) (joint-d . angle2))))
|#
(defun combinate-angles (angles)
  (cond ((null angles)
         nil)
        ((null (cdr angles))
         (mapcar #'list (car angles)))
        (t
         (let ((target-angles (car angles)))
           ;; append all the elements of target-angles to
           ;; (combinate-angles (cdr angles))
           (let ((child-results (combinate-angles (cdr angles)))
                 (ret nil))
             (dolist (target-angle target-angles)
               (dolist (child-result child-results)
                 (push (append (list target-angle) child-result) ret)))
             (reverse ret))))))

;; class to store set of calibration motion
(defclass calibration-config
  :super propertied-object
  :slots (motions model-file model-class robot))

(defmethod calibration-config
  (:init (&key ((:model-file amodel-file) nil)
               ((:model-class amodel-class) nil))
    (setq model-file amodel-file)
    (setq model-class amodel-class)
    (setq motions nil)
    (send self :generate-model)
    self)
  (:generate-model ()
    (load model-file)
    (setq robot (instance (eval model-class) :init)))
  (:add-motion (name motion)
    (push (cons name motion) motions)
    (car motions))
  (:motions ()
    motions)
  )

(defun range-equivalents (min max step)
  (let* ((diff (- max min))
         (count (floor (/ diff step)))
         (margin (/ (- diff (* count step)) 2.0))
         (ret nil))
    (dotimes (i (1+ count))
      (push (+ min margin (* i step)) ret))
    ret))
  

(defclass sample-joint-information
  :super propertied-object
  :slots (j angles))

(defmethod sample-joint-information
  (:init (&key ((:joint ajoint))
               (min-angle nil)
               (max-angle nil)
               (step-angle 5))
    (setq j ajoint)
    (if (null min-angle)
        (setq min-angle (send ajoint :min-angle)))
    (if (null max-angle)
        (setq max-angle (send ajoint :max-angle)))
    ;; compute angles
    (setq angles (range-equivalents min-angle max-angle step-angle))
    self)
  (:joint ()
    j)
  (:angles ()
    angles)
  )

(defclass hrp2-calibration-config
  :super calibration-config
  :slots (reset-angles))

(defmethod hrp2-calibration-config
  (:init (&key ((:model-file amodel-file) nil)
               ((:model-class amodel-class) nil)
               (camera-method nil))
    (send-super :init :model-file amodel-file :model-class amodel-class)
    (send self :add-motion :larm
          (instance calibration-motion
                    :init
                    :limb :larm
                    :robot-model robot
                    :reset-method :reest-pose
                    :camera-method camera-method
                    :sampling-joint
                    (list
                     (instance sample-joint-information :init
                               :joint (send robot :head :neck-p)
                               :step-angle 10)
                     (instance sample-joint-information :init
                               :joint (send robot :head :neck-y)
                               :step-angle 10))
                    :distances
                    (list 720 680 640 600 560 520)
                    :end-effector-offset
                    (let ((c (make-coords :pos (float-vector (+ 100) (+ 70 65) 0))))
                      (send c :rotate (deg2rad -90) :y)
                      c)))
    (send self :add-motion :rarm
          (instance calibration-motion
                    :init
                    :limb :rarm
                    :robot-model robot
                    :reset-method :reset-pose
                    :camera-method camera-method
                    :sampling-joint
                    (list
                     (instance sample-joint-information :init
                               :joint (send robot :head :neck-p)
                               :step-angle 10)
                     (instance sample-joint-information :init
                               :joint (send robot :head :neck-y)
                               :step-angle 10))
                    :distances
                    (list 720 680 640 600 560 520)
                    :end-effector-offset
                    (let ((c (make-coords :pos (float-vector (+ 100) (- (+ 70 65)) 0))))
                      (send c :rotate (deg2rad -90) :y)
                      c)))
    (send self :add-motion :lleg
          (instance calibration-motion
                    :init
                    :limb :lleg
                    :robot-model robot
                    :reset-method :reset-manip-pose
                    :camera-method camera-method
                    :filter-sampling
                    '(lambda (neck-p neck-y waist-y waist-p)
                       (>= (+ neck-y waist-y) 0))
                    :sampling-joint
                    (list
                     (instance sample-joint-information :init
                               :joint (send robot :head :neck-p)
                               :step-angle 10)
                     (instance sample-joint-information :init
                               :joint (send robot :head :neck-y)
                               :step-angle 10)
                     (instance sample-joint-information :init
                               :joint (send robot :torso :waist-y)
                               :step-angle 10)
                     (instance sample-joint-information :init
                               :joint (send robot :torso :waist-p)
                               :step-angle 10))
                    :distances
                    (range-equivalents 1200 1500 40)
                    :end-effector-offset
                    (let ((c (make-coords :pos (float-vector (+ 150 70 65) 0 0))))
                      c)))
    (send self :add-motion :rleg
          (instance calibration-motion
                    :init
                    :limb :rleg
                    :robot-model robot
                    :reset-method :reset-manip-pose
                    :camera-method camera-method
                    :filter-sampling
                    '(lambda (neck-p neck-y waist-y waist-p)
                       (<= (+ neck-y waist-y) 0))
                    :sampling-joint
                    (list
                     (instance sample-joint-information :init
                               :joint (send robot :head :neck-p)
                               :step-angle 10)
                     (instance sample-joint-information :init
                               :joint (send robot :head :neck-y)
                               :step-angle 10)
                     (instance sample-joint-information :init
                               :joint (send robot :torso :waist-y)
                               :step-angle 10)
                     (instance sample-joint-information :init
                               :joint (send robot :torso :waist-p)
                               :step-angle 10))
                    :distances
                    (range-equivalents 1200 1500 40)
                    :end-effector-offset
                    (let ((c (make-coords :pos (float-vector (+ 150 70 65) 0 0))))
                      c)))
    self)
  )

(defun make-rotation-mat (z)
  (let* ((z-axis (normalize-vector z))
         (tmp (next-best-direction z-axis))
         (y-axis (normalize-vector (v* z-axis tmp)))
         (x-axis (normalize-vector (v* y-axis z-axis))))
    (transpose (matrix x-axis y-axis z-axis))))
(defun next-best-direction (z)
  (if (< (abs (elt z 1)) (abs (elt z 0)))
      (if (< (abs (elt z 1)) (abs (elt z 2)))
          (if (<= (elt z 1) 0) (float-vector 0 1 0) (float-vector 0 -1 0))
        (if (<= (elt z 2) 0) (float-vector 0 0 1) (float-vector 0 0 -1)))
    (if (< (abs (elt z 0)) (abs (elt z 2)))
        (if (<= (elt z 0) 0) (float-vector 1 0 0) (float-vector -1 0 0))
      (if (<= (elt z 2) 0) (float-vector 0 0 1) (float-vector 0 0 -1)))))


(setq *calib* (instance hrp2-calibration-config
                        :init
                        :model-file "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknt-interface.l"
                        :model-class 'hrp2jsknt-robot
                        :camera-method :carmine))

;; (load "make-calibration-pose.l")
;; (send (cdr (car (send *calib* :motions))) :generate-motion)
;; (send (cdr (cadr (send *calib* :motions))) :generate-motion)
