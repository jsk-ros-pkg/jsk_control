#!/usr/bin/env roseus

(load "models/arrow-object.l")

(ros::load-ros-manifest "geometry_msgs")
(ros::load-ros-manifest "sensor_msgs")
(ros::load-ros-manifest "omni_msgs")
(ros::roseus-add-msgs "geometry_msgs")
(ros::roseus-add-msgs "sensor_msgs")
(ros::roseus-add-msgs "omni_msgs")


(defvar *jgt-robot* nil)
(defvar *jgt-arms* '(:larm :rarm))
(defvar *jgt-device-type* :omni)
(defvar *jgt-ratio* 1.0)
(defvar *jgt-send-time* 200)
(defvar *jgt-use-torso* :unset)
(defvar *jgt-rotation-axis* t)
(defvar *jgt-stop* 10)
(defvar *jgt-revert-if-fail* t)
(defvar *jgt-check-self-collision* t)
(defvar *jgt-debug-view* nil)
(defvar *jgt-debug-target-arrow-alist* nil)
(defvar *jgt-debug-end-arrow-alist* nil)
(defvar *jgt-origin-coords-alist* nil)
(defvar *jgt-target-coords-alist* nil)
(defvar *jgt-updated-arms* nil)
(defvar *jgt-absolute-input-alist* nil)
(defvar *jgt-relative-joy* nil)
(defvar *jgt-relative-prev-joy* nil)
(defvar *jgt-active-arm* :rarm)
(defvar *jgt-relative-translation-scale* 50.0)
(defvar *jgt-relative-rotation-scale* (/ pi 16.0))
(defvar *jgt-relative-deadzone* 0.2)
(defvar *jgt-absolute-rotation-map*
  (matrix (float-vector 0 -1 0)
          (float-vector 1 0 0)
          (float-vector 0 0 1)))

(defun jgt-get-param (name default)
  (if (ros::has-param name)
      (ros::get-param name)
    default))

(defun jgt-arm-keyword (arm suffix)
  (read-from-string (format nil "~A-~A" (string-upcase arm) (string-upcase suffix))))

(defun jgt-alist-get (key alist &optional default)
  (let ((cell (assoc key alist)))
    (if cell (cdr cell) default)))

(defun jgt-alist-set (key value alist)
  (let ((cell (assoc key alist)))
    (if cell
        (progn (rplacd cell value) alist)
      (cons (cons key value) alist))))

(defun jgt-mark-updated (arm)
  (if (not (member arm *jgt-updated-arms*))
      (setq *jgt-updated-arms* (cons arm *jgt-updated-arms*))))

(defun jgt-relative-device-p ()
  (or (eq *jgt-device-type* :spacenav) (eq *jgt-device-type* :joy)))

(defun jgt-copy-coords-alist (alist)
  (mapcar #'(lambda (cell)
              (cons (car cell) (send (cdr cell) :copy-worldcoords)))
          alist))

(defun jgt-to-float-vector3 (value &optional (default #f(0 0 0)))
  (cond
   ((vectorp value) (float-vector (elt value 0) (elt value 1) (elt value 2)))
   ((consp value) (float-vector (elt value 0) (elt value 1) (elt value 2)))
   (t default)))

(defun jgt-plist-get (plist key &optional default)
  (let ((tail (member key plist)))
    (if (and tail (cdr tail)) (cadr tail) default)))

(defun jgt-to-rotation-matrix (value)
  (cond
   ((null value) nil)
   ((vectorp value) (rpy-matrix (elt value 0) (elt value 1) (elt value 2)))
   ((consp value) (rpy-matrix (elt value 0) (elt value 1) (elt value 2)))
   (t value)))

(defun jgt-to-origin-offset (value)
  (cond
   ((and (consp value)
         (or (member :pos value) (member :position value)
             (member :rot value) (member :rotation value)
             (member :rpy value) (member :rpy-deg value)))
    (let* ((has-pos (or (member :pos value) (member :position value)))
           (pos (if has-pos
                    (jgt-to-float-vector3
                     (or (jgt-plist-get value :pos nil)
                         (jgt-plist-get value :position nil))
                     #f(0 0 0))))
           (rot (or (jgt-to-rotation-matrix (jgt-plist-get value :rot nil))
                    (jgt-to-rotation-matrix (jgt-plist-get value :rotation nil))
                    (jgt-to-rotation-matrix (jgt-plist-get value :rpy nil))
                    (let ((rpy-deg (jgt-plist-get value :rpy-deg nil)))
                      (if rpy-deg
                          (jgt-to-rotation-matrix (scale (/ pi 180.0)
                                                         (jgt-to-float-vector3 rpy-deg))))))))
      (list :pos pos :rot rot)))
   ((and value (find-method value :worldpos))
    (list :pos (send value :worldpos) :rot (send value :worldrot)))
   (value
    (list :pos (jgt-to-float-vector3 value) :rot nil))
   (t nil)))

(defun jgt-normalize-offsets (arms origin-offsets)
  (cond
   ((null origin-offsets)
    (mapcar #'(lambda (arm) (cons arm nil)) arms))
   ((and (consp origin-offsets) (consp (car origin-offsets)))
    (mapcar #'(lambda (arm)
                (cons arm (jgt-to-origin-offset
                           (jgt-alist-get arm origin-offsets nil))))
            arms))
   (t
    (let ((offset (jgt-to-origin-offset origin-offsets)))
      (mapcar #'(lambda (arm) (cons arm offset)) arms)))))

(defun jgt-current-end-coords (arm)
  (send (send *jgt-robot* arm :end-coords) :copy-worldcoords))

(defun jgt-debug-make-arrow-alist (arms suffix)
  (mapcar #'(lambda (arm)
              (let ((arrow (arrow :name (format nil "~A-~A" arm suffix))))
                (cons arm arrow)))
          arms))

(defun jgt-debug-arrow-objects ()
  (append (mapcar #'cdr *jgt-debug-target-arrow-alist*)
          (mapcar #'cdr *jgt-debug-end-arrow-alist*)))

(defun jgt-debug-init-arrows ()
  (setq *jgt-debug-target-arrow-alist*
        (jgt-debug-make-arrow-alist *jgt-arms* "target"))
  (setq *jgt-debug-end-arrow-alist*
        (jgt-debug-make-arrow-alist *jgt-arms* "end"))
  (objects (append (list *jgt-robot*) (jgt-debug-arrow-objects))))

(defun jgt-debug-update-arrow (arrow coords)
  (when (and arrow coords)
    (send arrow :newcoords (send coords :copy-worldcoords))))

(defun jgt-debug-update-arrows ()
  (dolist (arm *jgt-arms*)
    (jgt-debug-update-arrow
     (jgt-alist-get arm *jgt-debug-target-arrow-alist*)
     (jgt-alist-get arm *jgt-target-coords-alist*))
    (jgt-debug-update-arrow
     (jgt-alist-get arm *jgt-debug-end-arrow-alist*)
     (jgt-current-end-coords arm))))

(defun jgt-debug-update-arrows-if-needed ()
  (if *jgt-debug-view* (jgt-debug-update-arrows)))

(defun jgt-apply-init-pose (init-pose)
  (cond
    ((null init-pose) nil)
    ((vectorp init-pose) (send *jgt-robot* :angle-vector init-pose))
    ((symbolp init-pose)
     (if (find-method *jgt-robot* init-pose)
         (send *jgt-robot* init-pose)
	 (ros::ros-warn "init-pose method is not found: ~A" init-pose)))
    ((functionp init-pose) (funcall init-pose *jgt-robot*))
    (t (ros::ros-warn "unsupported init-pose: ~A" init-pose)))
  (send *ri* :angle-vector (send *jgt-robot* :angle-vector) 1000)
  (send *ri* :wait-interpolation)
  )

(defun jgt-init-origins (origin-offsets)
  (let ((offset-alist (jgt-normalize-offsets *jgt-arms* origin-offsets)))
    (setq *jgt-origin-coords-alist* nil)
    (setq *jgt-target-coords-alist* nil)
    (dolist (arm *jgt-arms*)
      (let ((origin (jgt-current-end-coords arm))
            (offset (jgt-alist-get arm offset-alist nil)))
        (when offset
          (let ((pos (jgt-plist-get offset :pos nil))
                (rot (jgt-plist-get offset :rot nil)))
            (if pos (send origin :locate pos :world))
            (if rot (send origin :transform (make-coords :rot rot) :world))))
        (setq *jgt-origin-coords-alist*
              (jgt-alist-set arm origin *jgt-origin-coords-alist*))
        (setq *jgt-target-coords-alist*
              (jgt-alist-set arm (send origin :copy-worldcoords)
                             *jgt-target-coords-alist*))))))

(defun jgt-omni-state-cb (arm msg)
  (setq *jgt-absolute-input-alist*
        (jgt-alist-set
         arm
         (list
          (send msg :pose :position)
          (send msg :pose :orientation)
          (send msg :close_gripper)
          (send msg :locked))
         *jgt-absolute-input-alist*)))

(defun jgt-left-omni-state-cb (msg)
  (if *jgt-arms*
      (jgt-omni-state-cb (car *jgt-arms*) msg)))

(defun jgt-right-omni-state-cb (msg)
  (if (cdr *jgt-arms*)
      (jgt-omni-state-cb (cadr *jgt-arms*) msg)))

(defun jgt-relative-joy-cb (msg)
  (setq *jgt-relative-prev-joy* *jgt-relative-joy*)
  (setq *jgt-relative-joy* msg))

(defun jgt-vector-from-omni-pos (pos)
  (float-vector (send pos :y)
                (- 0 (send pos :x))
                (send pos :z)))

(defun jgt-rot-from-omni-quaternion (q)
  (m* *jgt-absolute-rotation-map*
      (quaternion2matrix
       (float-vector (send q :w) (send q :x) (send q :y) (send q :z)))))

(defun jgt-update-absolute-targets ()
  (dolist (arm *jgt-arms*)
    (let ((input (jgt-alist-get arm *jgt-absolute-input-alist*)))
      (when input
        (let ((pos (car input))
              (rot (cadr input))
              (locked (cadddr input))
              (origin (jgt-alist-get arm *jgt-origin-coords-alist*)))
          (when (and pos rot origin (not locked))
            (let* ((mapped-pos (jgt-vector-from-omni-pos pos))
                   (target-pos (v+ (send origin :worldpos)
                                   (scale *jgt-ratio* mapped-pos)))
                   (target-rot (m* (send origin :worldrot)
                                   (jgt-rot-from-omni-quaternion rot)))
                   (target (make-coords :pos target-pos :rot target-rot)))
              (setq *jgt-target-coords-alist*
                    (jgt-alist-set arm target *jgt-target-coords-alist*))
              (jgt-mark-updated arm))))))))

(defun jgt-deadzone (value)
  (if (> (abs value) *jgt-relative-deadzone*) value 0.0))

(defun jgt-button-on-p (joy index)
  (and joy
       (< index (length (send joy :buttons)))
       (> (elt (send joy :buttons) index) 0)))

(defun jgt-axis (joy index)
  (if (and joy (< index (length (send joy :axes))))
      (jgt-deadzone (elt (send joy :axes) index))
    0.0))

(defun jgt-select-relative-arm ()
  (when (and *jgt-relative-joy* *jgt-relative-prev-joy*)
    (cond
     ((and (jgt-button-on-p *jgt-relative-joy* 0)
           (not (jgt-button-on-p *jgt-relative-prev-joy* 0)))
      (setq *jgt-active-arm* (car *jgt-arms*))
      (ros::ros-info "active teleop arm: ~A" *jgt-active-arm*))
     ((and (jgt-button-on-p *jgt-relative-joy* 1)
           (not (jgt-button-on-p *jgt-relative-prev-joy* 1))
           (cdr *jgt-arms*))
      (setq *jgt-active-arm* (cadr *jgt-arms*))
      (ros::ros-info "active teleop arm: ~A" *jgt-active-arm*)))))

(defun jgt-update-relative-target ()
  (when *jgt-relative-joy*
    (jgt-select-relative-arm)
    (let* ((target (jgt-alist-get *jgt-active-arm* *jgt-target-coords-alist*))
           (axes (send *jgt-relative-joy* :axes)))
      (when target
        (let* ((x (jgt-axis *jgt-relative-joy* 0))
               (y (jgt-axis *jgt-relative-joy* 1))
               (z (jgt-axis *jgt-relative-joy* 2))
               (rr (jgt-axis *jgt-relative-joy* 3))
               (rp (jgt-axis *jgt-relative-joy* 4))
               (ry (jgt-axis *jgt-relative-joy* 5))
               (pos-diff (scale (* *jgt-ratio* *jgt-relative-translation-scale*)
                                (float-vector x y z)))
               (rot-diff (rpy-matrix
                          (* *jgt-relative-rotation-scale* ry)
                          (* *jgt-relative-rotation-scale* rp)
                          (* *jgt-relative-rotation-scale* rr)))
               (new-pos (v+ (send target :worldpos) pos-diff))
               (new-rot (send (send (make-coords :rot (send target :worldrot))
                                    :transform (make-coords :rot rot-diff) :world)
                              :worldrot)))
          (when (some #'(lambda (v) (> (abs v) 0.0)) (coerce axes cons))
            (setq *jgt-target-coords-alist*
                  (jgt-alist-set
                   *jgt-active-arm*
                   (make-coords :pos new-pos :rot new-rot)
                   *jgt-target-coords-alist*))
            (jgt-mark-updated *jgt-active-arm*)))))))

(defun jgt-update-targets ()
  (cond
   ((eq *jgt-device-type* :omni) (jgt-update-absolute-targets))
   ((or (eq *jgt-device-type* :spacenav) (eq *jgt-device-type* :joy))
    (jgt-update-relative-target))
   (t (ros::ros-warn "unknown device-type: ~A" *jgt-device-type*))))

(defun jgt-use-torso-args ()
  (unless (eq *jgt-use-torso* :unset) (list :use-torso *jgt-use-torso*)))

(defun jgt-solve-arm-ik (arm target)
  (apply #'send *jgt-robot* arm :inverse-kinematics target
         :rotation-axis *jgt-rotation-axis*
         :stop *jgt-stop*
         :revert-if-fail *jgt-revert-if-fail*
         (jgt-use-torso-args)))

(defun jgt-self-collision-ok-p ()
  (or (not *jgt-check-self-collision*)
      (not (find-method *jgt-robot* :self-collision-check))
      (not (send *jgt-robot* :self-collision-check))))

(defun jgt-run-ik-step ()
  (let ((prev-target-coords-alist (jgt-copy-coords-alist *jgt-target-coords-alist*))
        (failed-arms nil))
    (setq *jgt-updated-arms* nil)
    (jgt-update-targets)
    (dolist (arm *jgt-arms*)
      (let ((target (jgt-alist-get arm *jgt-target-coords-alist*)))
        (when target
          (unless (jgt-solve-arm-ik arm target)
            (setq failed-arms (cons arm failed-arms))))))
    (when (jgt-relative-device-p)
      (dolist (arm failed-arms)
        (when (member arm *jgt-updated-arms*)
          (let ((prev-target (jgt-alist-get arm prev-target-coords-alist)))
            (if prev-target
                (setq *jgt-target-coords-alist*
                      (jgt-alist-set arm prev-target *jgt-target-coords-alist*))))))))
  (send *ri* :angle-vector (send *jgt-robot* :angle-vector) *jgt-send-time* nil 0 :min-time 0)
  (jgt-debug-update-arrows-if-needed))

(defun jgt-subscribe-device
  (&key
   (device-type *jgt-device-type*)
   (left-omni-topic "/left_device/phantom/state")
   (right-omni-topic "/right_device/phantom/state")
   (spacenav-topic "/spacenav/joy")
   (joy-topic "/joy"))
  (cond
   ((eq device-type :omni)
    (ros::subscribe left-omni-topic omni_msgs::OmniState #'jgt-left-omni-state-cb 1)
    (if (cdr *jgt-arms*)
        (ros::subscribe right-omni-topic omni_msgs::OmniState #'jgt-right-omni-state-cb 1)))
   ((eq device-type :spacenav)
    (ros::subscribe spacenav-topic sensor_msgs::Joy #'jgt-relative-joy-cb 1))
   ((eq device-type :joy)
    (ros::subscribe joy-topic sensor_msgs::Joy #'jgt-relative-joy-cb 1))
   (t (error "unknown device-type: ~A" device-type))))

(defun jsk-generic-teleop
  (&key
   robot
   robot-interface
   (arms '(:larm :rarm))
   (device-type :omni)
   (init-pose :reset-pose)
   (origin-offsets nil)
   (ratio 1.0)
   (send-time 200)
   (rate 30)
   (use-torso :unset)
   (rotation-axis t)
   (stop 10)
   (revert-if-fail t)
   (check-self-collision t)
   (left-omni-topic "/left_device/phantom/state")
   (right-omni-topic "/right_device/phantom/state")
   (spacenav-topic "/spacenav/joy")
   (joy-topic "/joy")
   (relative-translation-scale 50.0)
   (relative-rotation-scale (/ pi 16.0))
   (relative-deadzone 0.2)
   (absolute-rotation-map *jgt-absolute-rotation-map*)
   (active-arm (car arms))
   (debug-view nil))
  (setq *jgt-robot* (or robot (if (boundp '*robot*) *robot* nil)))
  (if (null *jgt-robot*) (error "robot is nil. Pass :robot or bind *robot*."))
  (setq *jgt-arms* arms)
  (setq *jgt-device-type* device-type)
  (setq *jgt-ratio* ratio)
  (setq *jgt-send-time* send-time)
  (setq *jgt-use-torso* use-torso)
  (setq *jgt-rotation-axis* rotation-axis)
  (setq *jgt-stop* stop)
  (setq *jgt-revert-if-fail* revert-if-fail)
  (setq *jgt-check-self-collision* check-self-collision)
  (setq *jgt-debug-view* debug-view)
  (setq *jgt-relative-translation-scale* relative-translation-scale)
  (setq *jgt-relative-rotation-scale* relative-rotation-scale)
  (setq *jgt-relative-deadzone* relative-deadzone)
  (setq *jgt-absolute-rotation-map* absolute-rotation-map)
  (setq *jgt-active-arm* active-arm)
  (jgt-apply-init-pose init-pose)
  (jgt-init-origins origin-offsets)
  (if *jgt-debug-view* (jgt-debug-init-arrows))
  (jgt-subscribe-device
   :device-type device-type
   :left-omni-topic left-omni-topic
   :right-omni-topic right-omni-topic
   :spacenav-topic spacenav-topic
   :joy-topic joy-topic)
  (ros::rate rate)
  (ros::ros-info "jsk_generic_teleop started: device=~A arms=~A ratio=~A"
                 device-type arms ratio)
  (while (ros::ok)
    (ros::spin-once)
    (jgt-run-ik-step)
    (if *jgt-debug-view* (progn (send *ri* :wait-interpolation) (send *irtviewer* :draw-objects)))
    (ros::sleep)))

(provide :jsk-generic-teleop)
