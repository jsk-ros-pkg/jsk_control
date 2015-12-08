(load "package://euscollada/src/euscollada-robot.l")

;; class to store calibration motion for one link
(defclass calibration-motion
  :super propertied-object
  :slots (robot-model sampling-joints distances end-effector-offset limb
          angles x-angles y-angles
          reset-method motion-anglevectors camera-method filter-sampling
          filter-sampling-coords duration-scale
          collision-links board-object prompt chain-ids
          robust-motion root-link))

(defmethod calibration-motion
  (:init (&key ((:robot-model arobot-model) nil)
               ((:chain-ids achain-ids) nil)
               ((:prompt aprompt) nil)
               ((:root-link aroot-link) "BODY")
               ((:sampling-joint asampling-joint) nil)
               ((:distances adistances) nil)
               ((:camera-method acamera-method) nil)
               ((:end-effector-offset aend-effector-offset) nil)
               ((:reset-method areset-method) nil)
               ((:filter-sampling afilter-sampling) nil)
               ((:filter-sampling-coords afilter-sampling-coords) nil)
               ((:collision-links acollision-links) nil)
               ((:angles aangles) '(0))
               ((:x-angles ax-angles) '(0))
               ((:y-angles ay-angles) '(0))
               ((:board board) nil)
               ((:limb alimb) nil)
               ((:duration-scale dscl) 1.0)
               ((:robust-motion rob-motion) nil))
    (setq root-link aroot-link)
    (setq chain-ids achain-ids)
    (setq angles aangles)
    (setq x-angles ax-angles)
    (setq y-angles ay-angles)
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
    (setq robust-motion rob-motion)
    (setq duration-scale dscl)
    ;; assoc
    (send board-object :transform
          (send (send robot-model limb :end-coords :copy-worldcoords)
                :transform end-effector-offset))
    (send (send robot-model limb :end-coords :parent) :assoc board-object)
    (setq collision-links (append acollision-links
                                  (mapcar #'(lambda (l)
                                              (cons board-object l))
                                          (remove (send robot-model limb :end-coords :parent)
                                                  (send arobot-model :links))
                                          )))
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
  (:write-initial-poses (strm)
    (let ((cntr 0))
    (dolist (angle-vector motion-anglevectors)
     (send robot-model :angle-vector angle-vector)
     (let ((root (send robot-model :link root-link))
           (move-target (send robot-model limb :end-coords :copy-worldcoords)))
       (send move-target :transform end-effector-offset)
       (let ((trans (send (send root :copy-worldcoords) :transformation
                          (send move-target :worldcoords))))
         (let ((pos (scale 0.001 (send trans :worldpos)))
               (ypr (send trans :rpy-angle)))
           ;; select least rpy angle
           (let ((a (mapcar #'(lambda (x) (if (> x pi) (- x 2pi) (if (< x -pi) (+ x 2pi) x))) (car ypr)))
                 (b (mapcar #'(lambda (x) (if (> x pi) (- x 2pi) (if (< x -pi) (+ x 2pi) x))) (cadr ypr))))
             (if (> (apply #'max (mapcar #'(lambda (x) (abs x)) a))
                    (apply #'max (mapcar #'(lambda (x) (abs x)) b)))
                 (setq ypr b) (setq ypr a)))
           (format strm "## ~A ~D~%" limb cntr)
           (incf cntr)
           (format strm "- [~8,8f, ~8,8f, ~8,8f, ~8,8f, ~8,8f, ~8,8f]~%"
                   (elt pos 0) (elt pos 1) (elt pos 2)
                   (elt ypr 2) (elt ypr 1) (elt ypr 0))))))))
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
       (format f "  - duration: ~A~%" (* duration-scale tm)) ;; 
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
    (format t "generating ~A files under ~A~%" (length motion-anglevectors) root-directory)
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
            (if (= i 0)                 ;first time, use 10 sec
                (send self :generate-motion-yaml root-directory arm-name av i 10)
              (send self :generate-motion-yaml root-directory arm-name av i tm)))
          ))))
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
  (:choose-motions (N)
    (format t "choose ~A motions out of ~A motions~%"
            N (length motion-anglevectors))
    (let ((indices (random-sampling-with-variant
                    motion-anglevectors N :return-type :element)))
      (elt-indices motion-anglevectors indices)))
  (:refine-motion ()
    (let ((candidate-N 50)
          (max-N 30))
      (let ((collision-freep nil)
            (angle-vector-sequence
             (if (> (length motion-anglevectors) candidate-N)
               (send self :choose-motions candidate-N)
               motion-anglevectors)))
        (while (> (length angle-vector-sequence) max-N)
          (setq collision-freep nil)
          (objects (list robot-model board-object))
          (while (not (eq collision-freep :free))
            (let ((collision-indices (send self :check-motion angle-vector-sequence)))
              (if (not (eq collision-indices :free))
                  (setq angle-vector-sequence (remove-indices angle-vector-sequence
                                                              collision-indices)))
              (setq collision-freep collision-indices)))
          (when (> (length angle-vector-sequence) max-N)
            (format t "motion length is ~A, try to reduce them to ~A~%"
                    (length angle-vector-sequence) max-N)
            (dotimes (i (- (length angle-vector-sequence) max-N))
              (setq angle-vector-sequence (remove-nearest-one angle-vector-sequence)))))
        (setq motion-anglevectors angle-vector-sequence)
        (format t "refined motion is ~A~%" (length motion-anglevectors)))))
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
                        (length angles)
                        (length x-angles)
                        (length y-angles)
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
              (dolist (angle angles)
                ;; first trial
                (send robot-model reset-method)
                (send self :set-model-sampling-angle sampling-angle)
                (if (send self :solve-ik distance
                            angle (car x-angles) (car y-angles)
                            :all-test-num test-num
                            :success-count success-count
                            :current-test-no test-counter)
                  (dolist (x-angle x-angles)
                    (dolist (y-angle y-angles)
                      (send robot-model reset-method)
                      (send self :set-model-sampling-angle sampling-angle)
                      ;;(send *irtviewer* :draw-objects)
                      (let ((ik-result (send self :solve-ik distance
                                             angle x-angle y-angle
                                             :all-test-num test-num
                                             :success-count success-count
                                             :current-test-no test-counter)))
                        (incf test-counter)
                        (when ik-result
                          (incf success-count)
                          (push ik-result success-set))
                        )))
                  (incf test-counter (* (length x-angles) (length y-angles)))
                  )))
            (progn
              (format t "~A-~A/~A skip test by filtering~%"
                      test-counter
                      (+ test-counter (* (length distances) (length angles)
                                         (length x-angles) (length y-angles)))
                      test-num)
              (incf test-counter (* (length distances) (length angles)
                                    (length x-angles) (length y-angles)))))))
      (format t "motion result~%")
      (format t " motions:: ~A/~A~%" (length success-set) test-num)
      ;; animation
      (setq motion-anglevectors success-set)
      (send self :animate 1)))
  (:animate (&optional (N 10))
    (objects (list robot-model board-object))
    (dotimes (i (1- (length motion-anglevectors)))
      (let ((from (elt motion-anglevectors i))
            (to (elt motion-anglevectors (1+ i))))
        (let ((avs (interpolate-motions from to N)))
          (dolist (av avs)
            (send robot-model :angle-vector av)
            (send *irtviewer* :draw-objects)
            ))))
    )
  (:avoid-min-max ()
    (let ((links (send robot-model limb)))
      (dolist (l links)
        (let ((j (send (send l :joint) :joint-angle))
              (min (send (send l :joint) :min-angle))
              (max (send (send l :joint) :max-angle)))
          (cond ((< j (+ min 1.0))
                 (send (send l :joint) :joint-angle (+ j 1)))
                ((> j (- max 1.0))
                 (send (send l :joint) :joint-angle (- j 1))))
          ))))
  (:solve-ik (l ang x-ang y-ang &key (rotation-axis t)
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
        (send c :rotate ang :z)
        (send c :rotate x-ang :x)
        (send c :rotate y-ang :y)
        (send c :worldpos)
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
        ;; filter min-max
        (if rr (send self :avoid-min-max))
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
               (send c :draw-on :flush t :color (float-vector 1 0 0))
               (send *irtviewer* :draw-objects)
               (x::window-main-one)
               (format t "~A/~A ~A ~A ~A ~A fail IK~%"
                       current-test-no all-test-num
                       l (rad2deg ang) (rad2deg x-ang) (rad2deg y-ang)))
              ((and ik-successp (not collision-freep))
               (format t "~A/~A ~A ~A ~A ~A success IK but NOT collision free~%"
                       current-test-no all-test-num
                       l (rad2deg ang) (rad2deg x-ang) (rad2deg y-ang)))
              ((and ik-successp collision-freep)
               (format t "~A/~A ~A ~A ~A ~A success IK and collision free~%"
                       current-test-no all-test-num
                       l (rad2deg ang) (rad2deg x-ang) (rad2deg y-ang)))
              )
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
            (return-from :check-interpolation-collision t)))
      ))
    nil)
  (:check-collision ()
    (let ((result (send robot-model :self-collision-check :pairs collision-links)))
      (if result (return-from :check-collision result))
      (unless robust-motion
        (return-from :check-collision result)))
    (let ((links (send robot-model limb)))
      ;; check collision with small difference to the joints
      (let* ((joints (send-all links :joint))
             (initial-joint-anlges (send-all joints :joint-angle)))
        (let ((test-samples (make-3bit-vector-combinations (length joints))))
          (format t ">> checking robust collision-free pose~%")
          (dolist (sample test-samples)
            ;; move
            (mapcar #'(lambda (j a d) (send j :joint-angle (+ a d)))
                    joints initial-joint-anlges sample)
            (let ((result (send robot-model :self-collision-check :pairs collision-links)))
              (when result
                ;; reset to the original
                (mapcar #'(lambda (j a) (send j :joint-angle a))
                        joints initial-joint-anlges)
                (return-from :check-collision result)))))))
    nil)
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
                               ;; (list (format nil "capture_data/samples/~A"
                               ;;               (string (car (car motions)))))
                               (mapcar #'(lambda (m i)
                                           (format nil "capture_data/samples/~0,2d_~A" i (string (car m))))
                                       motions (range 0 (length motions)))
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
    (dotimes (i (length motions))
      (let ((motion (elt motions i)))
        (send (cdr motion)
              :generate-files
              (format nil "~A/capture_data/samples/~0,2d_~A" resolved-dir i (string (car motion)))
              (car motion)))))
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
  (:generate-initial-poses (root-directory)
    (let ((full-path (format nil "~A/capture_data/samples/initial_poses.yaml" root-directory)))
      (with-open-file (f full-path :direction :output)
        (dolist (motion motions)
          (send (cdr motion) :write-initial-poses f)
          )
        )))
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
      (send self :generate-initial-poses resolved-dir)
      (let ((not-generated-files '("capture_data/all_pipelines.launch"
                                   "capture_data/all_viewers.launch"
                                   "capture_data/capture_data.launch"
                                   "capture_data/capture_exec.launch"
                                   "capture_data/hardware_config/laser_config.yaml"
                                   "capture_data/samples/initial_poses.yaml"
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
         ;;(tmp (next-best-direction z-axis))
         (tmp (float-vector 0 -1 0))
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
    (setf (get cube :num_x) 6)
    (setf (get cube :num_y) 5)
    cube))

(defun angle-vector-duration (model start end &optional (scale 10.0) (min-time 1.0))
  (let ((joint-list (send model :joint-list)))
    (let ((diff (coerce (v- end start) cons)))
      (let ((time-list (mapcar #'(lambda (d j)
                                   (* scale (/ (deg2rad (abs d)) (send j :max-joint-velocity))))
                               diff joint-list)))
        (let ((max-time (apply #'max time-list)))
          (max max-time min-time))))))

(defun remove-element-from-list (original element)
  (remove element original))

(defun compute-distance (a b)
  (abs (- a b)))

(defun compute-nearest-neighbor-element (element selected-list)
  ;; linear search
  (let ((nearest-index 0)
        (nearest-value (compute-distance element (elt selected-list 0))))
    (dotimes (i (length selected-list))
      (let ((diff-value (compute-distance element (elt selected-list i))))
        (when (> nearest-value diff-value)
          (setq nearest-index i)
          (setq nearest-value diff-value))))
    nearest-index))

(defun compute-probability (v nearest-value &optional (scale 1))
  ;; 1 - exp(-x)
  (let ((x (abs (- v nearest-value))))
    (let ((v (- 1.0 (exp (* scale (- x))))))
      (if (< v 0.1)
          0.1
        v))))

(defun compute-probability-element (element selected-set)
  (let ((element-values (coerce element cons)))
    (let ((probabilities
           (mapcar #'(lambda (v i)
                       (let ((nearest (compute-nearest-neighbor-element
                                       v (mapcar #'(lambda (e) (elt e (floor i)))
                                                 selected-set))))
                         (let ((nearest-value (elt selected-set nearest)))
                           (compute-probability v (deg2rad (elt nearest-value (floor i)))))))
                   element-values
                   (reverse (range-equivalents 0 (length element-values) 1)))))
      (reduce #'* probabilities))))

;; (probability-map-for-sampling '(#f(0) #f(1) #f(2) #f(3) #f(4)) '(#f(2)))
(defun probability-map-for-sampling (original-set selected-set)
  (let ((probability-map (make-list (length original-set))))
    (dotimes (i (length original-set))
      (let ((element (elt original-set i)))
        (if (member element selected-set)
            (setf (elt probability-map i) 0.0)
          (progn
            (setf (elt probability-map i)
                  (compute-probability-element element selected-set))
            )
          )))
    (normalize-probability-map probability-map)))
;; (probability-map-for-sampling '(#f(0) #f(1) #f(2) #f(3) #f(4)) '(#f(2)))

(defun normalize-probability-map (ps)
  ;; \sum{ps} = 1
  (let ((sum (reduce #'+ ps)))
    (mapcar #'(lambda (x) (/ x sum)) ps)))

;; (probability-map-for-sampling '(#f(0) #f(1) #f(2) #f(3) #f(4)) '(#f(2)))
(defun choose-index-from-probability-map (xs probability-map)
  (let ((random-value (random 1.0)))
    (let ((positioned-probability-map
           (mapcar #'(lambda (x i)
                       (+ x (reduce #'+ (subseq probability-map 0 (floor i)))))
                   probability-map
                   (reverse (range-equivalents 0 (length probability-map) 1)))))
      (dotimes (i (length positioned-probability-map))
        (let ((right-value (elt positioned-probability-map i)))
          (if (>= right-value random-value)
              (return-from choose-index-from-probability-map i)))))))

;; 
;; (sort-element-as-original '(1 3 0) '(0 1 2 3 4 5 6))
;;   => '(0 1 3)
(defun sort-element-as-original (selected original)
  (let ((position-list (mapcar #'(lambda (x) (position x original)) selected)))
    (let ((concatenated-list (mapcar #'(lambda (x i)
                                         (cons x i))
                                     selected position-list)))
      (mapcar #'car (sort concatenated-list #'< #'(lambda (x) (cdr x)))))))

(defun elt-indices (v indices)
  (let ((ret nil))
    (dolist (i indices)
      (push (elt v i) ret))
    (reverse ret)))

;; (make-3bit-vector-combinations 1) => '((-1) (0) (1))
;; (make-3bit-vector-combinations 2) => '((-1 -1) (-1 0) (-1 1) (0 -1) (0 0) (0 1) (1 0) (1 0) (1 1))
(defun make-3bit-vector-combinations (num &optional (diff-angle 1.0))
  ;; use combinate-angles
  (combinate-angles (mapcar #'(lambda (i) (list (- diff-angle) 0 diff-angle)) (make-list num))))


;; xs is a list of 6 float-vector
;; (x, y, z, roll, pitch, yaw)
(defun random-sampling-with-variant (xs N &key (return-type :value))
  (let ((sampled-elements nil)
        (rest-elements xs))
    (let ((seed-element (elt xs (random (length xs)))))
      (push seed-element sampled-elements)
      (setq rest-elements (remove-element-from-list xs seed-element))
      (while (< (length sampled-elements) N)
        (let ((probability-map (probability-map-for-sampling xs sampled-elements)))
          (let ((index (choose-index-from-probability-map xs probability-map)))
            (let ((selected-element (elt xs index)))
              (push selected-element sampled-elements)
              (setq rest-elements (remove-element-from-list rest-elements selected-element))
              ))))
      (cond ((eq return-type :element)
             (sort (mapcar #'(lambda (x) (position x xs)) sampled-elements) #'<))
            (t
             (sort-element-as-original sampled-elements xs))))))

(defun remove-nearest-one (avs)
  ;; compute distance matrix
  (let ((distance-map (mapcar #'(lambda (x) (make-list (length avs))) avs)))
    (dotimes (i (length avs))
      (dotimes (j (length avs))
        (if (= i j)
            (progn
              (setf (elt (elt distance-map i) j) nil)
              (setf (elt (elt distance-map j) i) nil))
          (progn
            (let ((d (distance (elt avs i) (elt avs j))))
              (setf (elt (elt distance-map i) j) d)
              (setf (elt (elt distance-map j) i) d))
            ))))
    ;; find smallest d, not nil
    (let ((smallest-i nil)
          (smallest-j nil)
          (smallest 1000000.0))         ;large enough
      (dotimes (i (length avs))
        (dotimes (j (length avs))
          (let ((v (elt (elt distance-map i) j)))
            (when (and v (< v smallest))
              (setq smallest v)
              (setq smallest-i i)
              (setq smallest-i j)))))
      ;; remove smallest-i or smallest-j
      (if (= (1+ smallest-i) (length avs))
          (butlast avs)
        (append (subseq avs 0 smallest-i) (subseq avs (1+ smallest-i) (length avs)))))))

;; (load "calibration.l")
;; (random-sampling-with-variant (mapcar #'float-vector (range-equivalents 0 10 0.1)) 10 :return-type :element)
