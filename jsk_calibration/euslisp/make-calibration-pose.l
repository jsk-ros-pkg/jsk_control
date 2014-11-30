(load "package://euscollada/src/euscollada-robot.l")

;; class to store calibration motion for one link
(defclass calibration-motion
  :super propertied-object
  :slots (robot-model sampling-joints distances end-effector-offset limb
          reset-method motion-anglevectors camera-method filter-sampling
          collision-links board-object))

(defmethod calibration-motion
  (:init (&key ((:robot-model arobot-model) nil)
               ((:sampling-joint asampling-joint) nil)
               ((:distances adistances) nil)
               ((:camera-method acamera-method) nil)
               ((:end-effector-offset aend-effector-offset) nil)
               ((:reset-method areset-method) nil)
               ((:filter-sampling afilter-sampling) nil)
               ((:collision-links acollision-links) nil)
               ((:board board) nil)
               ((:limb alimb) nil))
    (setq board-object board)
    (setq filter-sampling afilter-sampling)
    (setq camera-method acamera-method)
    (setq reset-method areset-method)
    (setq limb alimb)
    (setq robot-model arobot-model)
    (setq sampling-joints asampling-joint)
    (setq distances adistances)
    (setq end-effector-offset aend-effector-offset)
    ;; assoc
    (send board-object :transform
          (send (send robot-model limb :end-coords :copy-worldcoords)
                :transform end-effector-offset))
    (send (send robot-model limb :end-coords :parent) :assoc board-object)
    (setq collision-links (append acollision-links
                                  (mapcar #'(lambda (l)
                                              (cons board-object l))
                                          (send arobot-model :links))))
    self)
  (:test-with-real-robot (ri &optional (not-use-y-or-n nil))
    ;; :reset-pose
    ;; reset-method
    (objects (list robot-model board-object))
    (send robot-model reset-method)
    (send *irtviewer* :draw-objects)
    (let ((start-pose (send robot-model :angle-vector)))
      (if (or not-use-y-or-n (y-or-n-p "Go to start pose with 10 sec, OK?"))
          (progn
            (send ri :angle-vector start-pose 10000)
            (send ri :wait-interpolation)
            )
        (progn
          (return-from :test-with-real-robot nil))))
    (let ((sequence (append (list (send robot-model reset-method))
                            motion-anglevectors
                            (list (send robot-model reset-method)))))
      (dotimes (i (1- (length sequence)))
        ;; i -> i + 1
        (let* ((start-pose (elt sequence i))
               (end-pose (elt sequence (1+ i)))
               (tm (angle-vector-duration robot-model start-pose end-pose)))
          (send robot-model :angle-vector end-pose)
          (send *irtviewer* :draw-objects)
          (if (or not-use-y-or-n (y-or-n-p (format nil "move to this pose with ~A sec, OK?" tm)))
              (progn
                (send ri :angle-vector end-pose (* 1000.0 tm))
                (send ri :wait-interpolation)
                )
            (progn
              (return-from :test-with-real-robot nil)))
          ))
      ))
  (:robot-model ()
    robot-model)
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
  ;; refine motion by checking collision of interporated motions
  (:refine-motion ()
    (objects (list robot-model board-object))
    (let ((collision-freep nil)
          (angle-vector-sequence motion-anglevectors))
      (while (not (eq collision-freep :free))
        (let ((collision-indices (send self :check-motion angle-vector-sequence)))
          (if (not (eq collision-indices :free))
              (setq angle-vector-sequence (remove-indices angle-vector-sequence
                                                          collision-indices)))
          (setq collision-freep collision-indices)))
      (format t "refined motion is ~A~%" (length motion-anglevectors))))
  (:check-motion (angle-vectors)
    (let ((angle-vector-sequence
           (append (list (send robot-model reset-method))
                   angle-vectors
                   (list (send robot-model reset-method)))))
      (dotimes (i (1- (length angle-vector-sequence)))
        (let ((start (elt angle-vector-sequence i))
              (end (elt angle-vector-sequence (1+ i))))
          (when (send self :check-interpolation-collision start end)
            (format t "not collision free!~%")
            (return-from :check-motion (list (1- i) i)))))
      :free))
  (:generate-motion ()
    (send self :generate-primitive-motion)
    (send self :refine-motion))
  (:generate-primitive-motion ()
    (let* ((sampling-angle-set (send self :sampling-joint-angle-combinations))
           (success-count 0)
           (test-counter 0)
           (test-num (* (length distances)
                        (length sampling-angle-set)))
           (success-set nil))
      (format t "~A sampling joint~%" (length sampling-joints))
      (format t "~A combinated sampling joint angles~%" test-num)
      (objects (list robot-model board-object))
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
              (format t "~A-~A/~A skip test by filtering~%"
                      test-counter
                      (+ test-counter (length distances))
                      test-num)
              (incf test-counter (length distances))))))
      (format t "motion result~%")
      (format t " motions:: ~A/~A~%" (length success-set) test-num)
      ;; animation
      (setq motion-anglevectors success-set)
      (send self :animate)))
  (:animate ()
    (objects (list robot-model board-object))
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
        (when rr
          (send *irtviewer* :draw-objects)
          (x::window-main-one)
          (send c :draw-on :flush t :color (float-vector 1 0 0)))
        (when ik-successp
          (setq collision-freep (not (send self :check-collision))))
        (cond ((not ik-successp)
               (format t "~A/~A fail IK~%" current-test-no all-test-num))
              ((and ik-successp (not collision-freep))
               (format t "~A/~A success IK but NOT collision free~%"
                       current-test-no all-test-num))
              ((and ik-successp collision-freep)
               (format t "~A/~A success IK and collision free~%"
                       current-test-no all-test-num)))
        (if (and ik-successp collision-freep)
            rr))))
  (:check-interpolation-collision (start end)
    (let ((avs (interpolate-motions start end 10)))
      (dolist (av avs)
        (send robot-model :angle-vector av)
        (send *irtviewer* :draw-objects)
        (x::window-main-one)
        (let ((collision (send self :check-collision)))
          (when collision
            (print av)
            (pprint collision)
            (return-from :check-interpolation-collision t)))
      ))
    nil)
  (:check-collision ()
    (send robot-model :self-collision-check :pairs collision-links))
  (:set-model-sampling-angle (sample-angles)
    (dolist (sample-angle sample-angles)
      (send (car sample-angle) :joint-angle (cdr sample-angle))))
  )

(defun remove-indices (list indices)
  (let ((ret nil))
    (dotimes (i (length list))
      (unless (member i indices)
        (push (elt list i) ret)))
    (reverse ret)))
    
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
    (setq motions (append motions
                          (list (cons name motion))))
    ;;(push (cons name motion) motions)
    motion)
  (:motions ()
    motions)
  (:animate ()
    (send-all (mapcar #'(lambda (m) (cdr m)) (send self :motions))
              :animate))
  (:generate-motions ()
    (send-all (mapcar #'(lambda (m) (cdr m)) (send self :motions))
              :generate-motion))
  )

(defun range-equivalents (min max step)
  (let* ((diff (- max min))
         (count (floor (/ diff step)))
         (margin (/ (- diff (* count step)) 2.0))
         (ret nil))
    (dotimes (i (1+ count))
      (push (+ min margin (* i step)) ret))
    ret))

(defun mid-float-vector (a b r)
  (v+ (scale r a) (scale (- 1.0 r) b)))

(defun interpolate-motions (start end num)
  (let ((ret nil))
    (dotimes (i num)
      ;; num := 9
      ;;  rate -> 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9
      (let ((rate (* (1+ i) (/ 1.0 (1+ num)))))
        (push (mid-float-vector start end rate) ret)))
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

(defmethod euscollada-robot
  (:reset-pose-for-leg-calibration ()   ;only for hrp2 robots
    (send self :reset-manip-pose)
    (send self :larm :elbow-p :joint-angle 0)
    (send self :rarm :elbow-p :joint-angle 0)
    (send self :angle-vector)))

(defclass hrp2-calibration-config
  :super calibration-config
  :slots (reset-angles))

(defmethod hrp2-calibration-config
  (:init (&key ((:model-file amodel-file) nil)
               ((:model-class amodel-class) nil)
               (camera-method nil))
    (send-super :init :model-file amodel-file :model-class amodel-class)
    ;; build good collision links
    (let ((collision-links (send robot :collision-check-pairs))
          (ignore-link-pairs nil))
          ;; (ignore-link-pairs '(("LARM_LINK6" . "LARM_LINK4")
          ;;                      ("RARM_LINK6" . "RARM_LINK4"))))
      (setq collision-links
            (remove-if #'(lambda (pair)
                           (let ((a (send (car pair) :name))
                                 (b (send (cdr pair) :name)))
                             (or (member (cons a b) ignore-link-pairs
                                         :test #'equal)
                                 (member (cons b a) ignore-link-pairs
                                         :test #'equal))))
                       collision-links))
      (send self :add-motion :larm
            (instance calibration-motion
                      :init
                      :limb :larm
                      :collision-links collision-links
                      :robot-model robot
                      :reset-method :reset-pose
                      :camera-method camera-method
                      :sampling-joint
                      (list
                       (instance sample-joint-information :init
                                 :joint (send robot :head :neck-p)
                                 :step-angle 15)
                       (instance sample-joint-information :init
                                 :joint (send robot :head :neck-y)
                                 :step-angle 15))
                      :distances
                      (range-equivalents 520 720 60)
                      :board (make-mmurooka-board)
                      :end-effector-offset
                      (let ((c (make-coords :pos (float-vector (+ 100) (+ 70 65 80) 0))))
                        (send c :rotate (deg2rad -90) :y)
                        (send c :rotate (deg2rad 90) :z)
                        c)))
      (send self :add-motion :rarm
            (instance calibration-motion
                      :init
                      :limb :rarm
                      :collision-links collision-links
                      :robot-model robot
                      :reset-method :reset-pose
                      :camera-method camera-method
                      :sampling-joint
                      (list
                       (instance sample-joint-information :init
                                 :joint (send robot :head :neck-p)
                                 :step-angle 15)
                       (instance sample-joint-information :init
                                 :joint (send robot :head :neck-y)
                                 :step-angle 15))
                      :distances
                      (range-equivalents 520 720 60)
                      :board (make-mmurooka-board)
                      :end-effector-offset
                      (let ((c (make-coords
                                :pos (float-vector (+ 100) (- (+ 70 65 80)) 0))))
                        (send c :rotate (deg2rad -90) :y)
                        (send c :rotate (deg2rad 90) :z)
                        c)))
      (send self :add-motion :lleg
            (instance calibration-motion
                      :init
                      :limb :lleg
                      :collision-links collision-links
                      :robot-model robot
                      :reset-method :reset-pose-for-leg-calibration
                      :camera-method camera-method
                      :filter-sampling
                      '(lambda (neck-p neck-y waist-y waist-p)
                         (>= (+ neck-y waist-y) 0))
                      :sampling-joint
                      (list
                       (instance sample-joint-information :init
                                 :joint (send robot :head :neck-p)
                                 :min-angle 20
                                 :step-angle 15)
                       (instance sample-joint-information :init
                                 :joint (send robot :head :neck-y)
                                 :step-angle 15)
                       (instance sample-joint-information :init
                                 :joint (send robot :torso :waist-y)
                                 :step-angle 15)
                       (instance sample-joint-information :init
                                 :joint (send robot :torso :waist-p)
                                 :step-angle 15))
                      :distances
                      (range-equivalents 1200 1500 100)
                      :board (make-mmurooka-board)
                      :end-effector-offset
                      (let ((c (make-coords :pos (float-vector (+ 120 70 65) 0 0))))
                        c)))
      (send self :add-motion :rleg
            (instance calibration-motion
                      :init
                      :limb :rleg
                      :collision-links collision-links
                      :robot-model robot
                      :reset-method :reset-pose-for-leg-calibration
                      :camera-method camera-method
                      :filter-sampling
                      '(lambda (neck-p neck-y waist-y waist-p)
                         (<= (+ neck-y waist-y) 0))
                      :sampling-joint
                      (list
                       (instance sample-joint-information :init
                                 :joint (send robot :head :neck-p)
                                 :min-angle 20
                                 :step-angle 15)
                       (instance sample-joint-information :init
                                 :joint (send robot :head :neck-y)
                                 :step-angle 15)
                       (instance sample-joint-information :init
                                 :joint (send robot :torso :waist-y)
                                 :step-angle 15)
                       (instance sample-joint-information :init
                                 :joint (send robot :torso :waist-p)
                                 :step-angle 15))
                      :distances
                      (range-equivalents 1200 1500 100)
                      :board (make-mmurooka-board)
                      :end-effector-offset
                      (let ((c (make-coords :pos (float-vector (+ 120 70 65) 0 0))))
                        c)))
      self))
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

(defun make-mmurooka-board (&optional (padding 20))
  (let ((cube (make-cube (+ padding 320) (+ padding 240) 5)))
    (send cube :translate-vertices (float-vector 0 0 (- (+ 1.0 (/ 5 2.0)))))
    cube))

(defun test ()
  (setq *calib* (instance hrp2-calibration-config
                          :init
                          :model-file "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknt-interface.l"
                          :model-class 'hrp2jsknt-robot
                          :camera-method :carmine))
  (send *calib* :generate-motions))

(defun angle-vector-duration (model start end &optional (scale 5.0) (min-time 0.5))
  (let ((joint-list (send model :joint-list)))
    (let ((diff (coerce (v- end start) cons)))
      (let ((time-list (mapcar #'(lambda (d j)
                                   (* scale (/ (deg2rad (abs d)) (send j :max-joint-velocity))))
                               diff joint-list)))
        (let ((max-time (apply #'max time-list)))
          (max max-time min-time))))))

;; (load "make-calibration-pose.l")
;; (test)
;; (send *calib* :generate-motions)
;; (send *calib* :refine-motions)
;; (send *calib* :animate)
;; (send-all (mapcar #'(lambda (m) (cdr m)) (send *calib* :motions)) :animate)
;; (send (cdr (car (send *calib* :motions))) :generate-motion)
;; (setq *hrp2* (send (cdr (car (send *calib* :motions))) :robot-model))
;; (send (cdr (car (send *calib* :motions))) :refine-motion)
;; (send (cdr (car (send *calib* :motions))) :animate)
;; (send (cdr (cadr (send *calib* :motions))) :generate-motion)
;; (send (cdr (car (send *calib* :motions))) :test-with-real-robot *ri*)
