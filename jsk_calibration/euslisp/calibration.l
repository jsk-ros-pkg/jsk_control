(load "package://euscollada/src/euscollada-robot.l")

;; class to store calibration motion for one link
(defclass calibration-motion
  :super propertied-object
  :slots (robot-model sampling-joints distances end-effector-offset limb
          reset-method motion-anglevectors camera-method filter-sampling
          filter-sampling-coords
          collision-links board-object prompt chain-ids))

(defmethod calibration-motion
  (:init (&key ((:robot-model arobot-model) nil)
               ((:chain-ids achain-ids) nil)
               ((:prompt aprompt) nil)
               ((:sampling-joint asampling-joint) nil)
               ((:distances adistances) nil)
               ((:camera-method acamera-method) nil)
               ((:end-effector-offset aend-effector-offset) nil)
               ((:reset-method areset-method) nil)
               ((:filter-sampling afilter-sampling) nil)
               ((:filter-sampling-coords afilter-sampling-coords) nil)
               ((:collision-links acollision-links) nil)
               ((:board board) nil)
               ((:limb alimb) nil))
    (setq chain-ids achain-ids)
    (setq prompt aprompt)
    (setq board-object board)
    (setq filter-sampling afilter-sampling)
    (setq filter-sampling-coords afilter-sampling-coords)
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
  (:generate-config-yaml (root-directory arm-name)
    (with-open-file
     (f (format nil "~A/config.yaml" root-directory) :direction :output)
     (format f "group: ~A~%" (string arm-name))
     (format f "prompt: ~A~%" prompt)
     (format f "finish: ~A~%" "Skipping arm samples")
     (format f "repeat: False~%")
     ))
  (:generate-motion-yaml (root-directory arm angle-vector index tm)
    (let* ((fname (format nil "~A_~0,4d.yaml" (string arm) index))
           (full-path (format nil "~A/~A" root-directory fname)))
      (with-open-file
       (f full-path :direction :output)
       (format f "camera_measurements:~%")
       (format f "- {cam_id: head_camera, config: ~A}~%"
               (string-downcase (string (get board-object :name))))
       (format f "joint_commands:~%")
       (format f "- controller: fullbody_controller~%")
       (format f "  segments:~%")
       (format f "  - duration: ~A~%" tm)
       (format f "    positions: ~A~%" (python-array-format ;convert to radian
                                        (mapcar #'deg2rad (coerce angle-vector cons))))
       (format f "joint_measurements:~%")
       (dolist (chain-id chain-ids)
         (format f "- {chain_id: ~A,  config: tight_tol}~%" chain-id))
       (format f "sample_id: ~A_~0,4d~%" (string arm) index)
       (format f "target: {chain_id: ~A_chain, target_id: ~A}~%"
               (string arm) (string-downcase (string (get board-object :name))))
       )))
  (:generate-files (root-directory arm-name)
    (format t "generating ~A files under ~A" (length motion-anglevectors) root-directory)
    ;; config.yaml
    (send self :generate-config-yaml root-directory arm-name)
    (let ((angle-vector-sequence (append (list (send robot-model reset-method))
                                         motion-anglevectors
                                         (list (send robot-model reset-method)))))
      ;; ${arm-name}_%04d.yaml
      (dotimes (i (length motion-anglevectors))
        (let ((av (elt motion-anglevectors i)))
          (let ((tm (angle-vector-duration
                     robot-model (elt angle-vector-sequence i)
                     av)))
            (send self :generate-motion-yaml root-directory arm-name av i tm)))
        )))
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
        (if rr
            (if (not filter-sampling-coords)
                (setq ik-successp t)
              (setq ik-successp
                    (funcall filter-sampling-coords
                             (send (send board-object :copy-worldcoords)
                                   :copy-worldcoords)))))
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
  :slots (motions model-file model-class robot chain-configs
          camera-configs board-object))

(defmethod calibration-config
  (:init (&key ((:model-file amodel-file) nil)
               ((:model-class amodel-class) nil))
    (setq model-file amodel-file)
    (setq model-class amodel-class)
    (setq motions nil)
    (send self :generate-model)
    self)
  (:add-board-object (bo)
    (setq board-object bo))
  (:add-chain-configs (configs)
    (setq chain-configs configs))
  (:head-camera-config (camera-topic)
    (push (cons "head_camera" camera-topic)
          camera-configs))
  (:generate-directories (root-directory)
    (let ((directories (append (list "capture_data"
                                     "capture_data/hardware_config"
                                     "capture_data/samples")
                               (list (format nil "capture_data/samples/~A"
                                             (string (car (car motions)))))
                               ;; (mapcar #'(lambda (m i)
                               ;;             (format nil "capture_data/samples/~0,2d_~A" i (string (car m))))
                               ;;         motions (range 0 (length motions)))
                               (list
                                "estimate_params"
                                "estimate_params/config"
                                "view_results"))))
      (warning-message 1 "creating following directory:~%")
      (dolist (d directories)
        (warning-message 1 "  ~A/~A~%" root-directory d))
      (dolist (d directories)
        (let ((full-path (format nil "~A/~A" root-directory d)))
          (unless (probe-file full-path)
            (warning-message 1 "creating ~A~%" full-path)
            (unix::mkdir full-path))
          ))
      ))
  (:generate-motion-files (resolved-dir)
    (let ((motion (car motions)))
      (send (cdr motion)
            :generate-files
            (format nil "~A/capture_data/samples/~A" resolved-dir (string (car motion)))
            (car motion))
      )
    )
  (:generate-controller-config (root-directory)
    (let ((full-path (format nil "~A/capture_data/hardware_config/controller_config.yaml" root-directory)))
      (warning-message 1 "generating ~A~%" full-path)
      (with-open-file
       (f full-path :direction :output)
       (format f "fullbody_controller:~%")
       (format f "  topic: fullbody_controller/command~%")
       (format f "  joint_names:~%")
       (dolist (j (send robot :joint-list))
         (format f
                 "    - ~A~%" (send j :name))))))
  (:generate-chain-config (root-directory)
    (let ((full-path (format nil "~A/capture_data/hardware_config/chain_config.yaml" root-directory)))
      (warning-message 1 "generating ~A~%" full-path)
      (with-open-file
       (f full-path :direction :output)
       (dolist (chain-config chain-configs)
         (let ((chain-name (car chain-config))
               (chain-links (cdr chain-config)))
           (format f "~A_chain:~%" chain-name)
           (format f "  settler_config: /~A_chain/settler_config~%" chain-name)
           (format f "  configs:~%")
           (format f "    tight_tol:~%")
           (format f "      settler:~%")
           (format f "        joint_names:~%")
           (dolist (l chain-links)
             (format f
                     "          - ~A~%" (send (send l :joint) :name)))
           (format f "        tolerances:~%")
           (dotimes (i (length chain-links))
             (format f
                     "          - 0.002~%"))
           (format f "        max_step: 1.0~%")
           (format f "        cache_size: 1500~%")
           )
       ))))
  (:generate-settler-launch (root-directory)
    (let ((full-path (format nil "~A/capture_data/settler.launch" root-directory)))
      (warning-message 1 "generating ~A~%" full-path)
      (with-open-file
       (f full-path :direction :output)
       (format f "<launch>~%")
       (dolist (chain-config chain-configs)
         (format f "  <include file=\"$(find calibration_launch)/capture_data/chain_settler.launch\"~%")
         (format f "           ns=\"~A_chain\" />~%" (car chain-config)))
       (format f "</launch>~%")
       )))
  (:generate-interval-launch (root-directory)
    (let ((full-path (format nil "~A/capture_data/interval.launch" root-directory)))
      (warning-message 1 "generating ~A~%" full-path)
      (with-open-file
       (f full-path :direction :output)
       (format f "<launch>~%")
       (format f "  <node type=\"interval_intersection_action\"~%")
       (format f "        pkg=\"interval_intersection\"~%")
       (format f "        name=\"interval_intersection\"~%")
       (format f "        output=\"screen\">~%")
       (dolist (chain-config chain-configs)
         (format f 
                 "    <remap from=\"~A_chain\" to=\"~A_chain/settled_interval\" />~%"
                 (car chain-config) (car chain-config)))
       (dolist (camera-config camera-configs)
         (format f 
                 "    <remap from=\"~A\" to=\"~A/settled_interval\" />~%"
                 (car camera-config) (cdr camera-config)))
       (format f "  </node>~%")
       (format f "</launch>~%")
      )))
  (:generate-cam-config (root-directory)
    (let ((full-path (format nil "~A/capture_data/hardware_config/cam_config.yaml" root-directory)))
      (warning-message 1 "generating ~A~%" full-path)
      (with-open-file
       (f full-path :direction :output)
       (dolist (camera-config camera-configs)
         (let ((camera-name (car camera-config))
               (camera-topic (cdr camera-config)))
           (format f "~A:~%" camera-name)
           (format f "  cb_detector_config: ~A/cb_detector_config~%" camera-topic)
           (format f "  led_detector_config: ~A/led_detector~%" camera-topic)
           (format f "  settler_config: ~A/monocam_settler_config~%" camera-topic)
           (format f "  configs:~%")
           (format f "    ~A:~%" (string-downcase (string (get board-object :name))))
           (format f "      settler:~%")
           (format f "        tolerance: 2.00~%")
           (format f "        ignore_failures: True~%")
           (format f "        max_step: 3.0 ~%")
           (format f "        cache_size: 100~%")
           (format f "      cb_detector:~%")
           (format f "        active: True~%")
           (format f "        num_x: ~A~%" (get board-object :num_x))
           (format f "        num_y: ~A~%" (get board-object :num_y))
           (format f "        width_scaling: 1~%")
           (format f "        height_scaling: 1~%")
           (format f "        subpixel_window: 4~%")
           (format f "        subpixel_zero_zone: 1~%")
           (format f "      led_detector:~%")
           (format f "        active: False~%")
           )))))
  (:generate-files (root-directory)
    ;; directory
    ;; root
    ;;  +---capture_data
    ;;           +---hardware_config
    ;;           +---samples
    ;;                  +---arm0
    ;;                  +---arm1
    ;;  +---estimate_params
    ;;           +---config
    ;;  +---view_results
    (let ((resolved-dir (ros::resolve-ros-path root-directory)))
      (send self :generate-directories resolved-dir)
      (send self :generate-motion-files resolved-dir)
      (send self :generate-controller-config resolved-dir)
      (send self :generate-chain-config resolved-dir)
      (send self :generate-settler-launch resolved-dir)
      (send self :generate-interval-launch resolved-dir)
      (send self :generate-cam-config resolved-dir)
      (let ((not-generated-files '("capture_data/all_pipelines.launch"
                                   "capture_data/all_viewers.launch"
                                   "capture_data/capture_data.launch"
                                   "capture_data/capture_exec.launch"
                                   "capture_data/hardware_config/laser_config.yaml"
                                   "view_results/scatter_config.yaml"
                                   "view_results/scatter.sh"
                                   )))
        (warning-message 3 "following files are not generated, please check by yourself~%")
        (dolist (f not-generated-files)
          (warning-message 3 "  ~A~%" f))
        )))
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

(defun range (start end)
  (let ((ret nil))
    (dotimes (i (- end start))
      (push (+ i start) ret)
      )
    (reverse ret)))

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
        (setq min-angle (- (send ajoint :min-angle) (deg2rad 1))))
    (if (null max-angle)
        (setq max-angle (- (send ajoint :max-angle) (deg2rad 1))))
    ;; compute angles
    (setq angles (range-equivalents min-angle max-angle step-angle))
    self)
  (:joint ()
    j)
  (:angles ()
    angles)
  )

(defun python-array-format (av)
  (let ((ss (make-string-output-stream)))
    (format ss "[")
    (dotimes (i (length av))
      (format ss "~A" (elt av i))
      (unless (= i (1- (length av)))
        (format ss ", "))
      )
    (format ss "]")
    (get-output-stream-string ss)))

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
    ;; add property to the board
    (setf (get cube :name) :mmurooka_board)
    (setf (get cube :num_x) 5)
    (setf (get cube :num_y) 6)
    cube))

(defun angle-vector-duration (model start end &optional (scale 5.0) (min-time 0.5))
  (let ((joint-list (send model :joint-list)))
    (let ((diff (coerce (v- end start) cons)))
      (let ((time-list (mapcar #'(lambda (d j)
                                   (* scale (/ (deg2rad (abs d)) (send j :max-joint-velocity))))
                               diff joint-list)))
        (let ((max-time (apply #'max time-list)))
          (max max-time min-time))))))

