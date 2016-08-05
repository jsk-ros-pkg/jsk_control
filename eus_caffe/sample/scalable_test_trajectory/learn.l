#!/usr/bin/env roseus

(require "package://eus_caffe/euslisp/eus-caffe.l")
(require "package://eus_caffe/euslisp/eus-caffe-db.l")
(require "package://eus_caffe/euslisp/eus-log.l")
;; (require "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknt-interface.l")
(require "package://baxtereus/baxter.l")
(require "models/arrow-object.l")
(require "package://eus_caffe/sample/scalable_test_trajectory/gen-solver.l")

(require "package://contact_behavior_generation/euslisp/util/partition-spline/partition-spline.lisp")

(defclass vector-coords
  :super propertied-object
  :slots (coords mode v-buf c-buf))
(defmethod vector-coords
  (:init (c
          &key ((:mode m) :coords))
         (setq coords c) (setq mode m)
         (setq c-buf (make-coords))
         (setq v-buf (instantiate float-vector (send self :length))))
  (:worldcoords
   nil
   (let* ((c (cond ((functionp coords) (funcall coords))
                   (t coords))))
     (cond
      ((not (find-method c :worldcoords))
       (warning-message 1 "[coords2vector] invalid coords format ~A~%" coords)
       (throw :vector-coords-exception t)))
     (send c :worldcoords)))
  ;;
  (:vector
   (&key
    (org-coords (make-coords))
    (offset-coords)
    (ret v-buf) (i 0))
   (let* ((_c (send self :worldcoords))
	  (c (if offset-coords (send (send _c :copy-worldcoords)
				     :transform offset-coords :world) _c)))
     (cond
      ((eq mode :coords)
       (let* (pos rot)
         (setq pos (send (send org-coords :worldcoords)
                         :difference-position c))
         (setq rot (send (send org-coords :worldcoords)
                         :difference-rotation c))
         (dotimes (j 3) (setf (aref ret (+ i j)) (* 1e-3 (aref pos j))))
         (dotimes (j 3) (setf (aref ret (+ 3 i j)) (aref rot j)))))
      ((eq mode :pos)
       (let* (pos)
         (setq pos (send (send org-coords :worldcoords)
                         :difference-position
                         c))
         (dotimes (j 3) (setf (aref ret (+ i j)) (* 1e-3 (aref pos j))))
         ;; (setq ret (scale 1e-3 pos))
         ))
      (t
       (warning-message 1 "[coords2vector] invalid mode ~A~%" mode)
       (throw :vector-coords-exception t)))
     ret))
  (:length
   (&optional vec ret)
   (setq ret
         (cond ((eq mode :coords) 6) ((eq mode :pos) 3)
               (t (warning-message 1 "[coords-vector] mode invalid~A~%" mode)
                  (throw :vector-coords-exception t))))
   (cond
    ((and vec (not (eq ret (length vec))))
     (warning-message 1 "[coords-vector] invalid length~A~%" vec)
     (throw :vector-coords-exception t)))
   ret)
  (:coords-from-vector
   (vec &key (org-coords (make-coords)) c)
   ;; check length
   (send self :length vec)
   ;;
   (cond
    ((eq mode :coords)
     (setq c (make-coords
              :pos (scale 1e+3 (subseq vec 0 3))
              :rot (matrix-exponent (subseq vec 3 6))))
     (setq c (send (send org-coords :copy-worldcoords)
                   :transform c :local))
     c)
    ((eq mode :pos)
     (setq c (make-coords :pos (scale 1e+3 vec)))
     (setq c (send (send org-coords :copy-worldcoords)
                   :transform c :local))
     (setq c (make-coords :pos (send c :worldpos)))
     c))
   c)
  (:nomethod
   (&rest args)
   (send* (send self :worldcoords) (car args) (cdr args)))
  (:transform-copy
   (c) (instance vector-coords :init (send (send self :copy-worldcoords) :transform c :world)))
  )

(defvar *robot* (baxter))
(defvar *joint-list1* (send *robot* :rarm :joint-list))

(defvar *arrow* (list (arrow)))
(defvar *goal-arrow* (copy-object *arrow*))
(defvar *ef-coords* (list (instance vector-coords :init (send (car *arrow*) :worldcoords))))
;; (defvar *ef-coords-length* (list 6))
(defvar *root-link* (car (send *robot* :links)))
(defvar *root-link-fix-coords-func* nil)

(defvar *learn-mode* :rarm)

(send-all (send *robot* :links) :set-val 'analysis-level :coords)
(send-all (send *robot* :joint-list) :set-val 'joint-min-max-table nil)
(send-all (send *robot* :joint-list) :set-val 'joint-min-max-target nil)
(objects (flatten (list *robot* *arrow* *goal-arrow*)))
(send *irtviewer* :change-background (float-vector 1 1 1))
(send *viewer* :draw-objects)

(defvar *sample-points*
  (let* ((dof (* 3 (length *joint-list1*)))
	 (org
          (min
           (round 1e+5)
           (expt (max (min (round (expt 1e+5 (/ 1.0 dof))) 4) 2) dof)))
	 (sep 32))
    (if (not (zerop (mod org sep)))
        (max 1 (round (* (ceiling (/ (* 1.0 org) sep)) sep)))
      org)))

(defvar *state-map*)
(defvar *task-map*)

;; mapcar '(lambda (ll) (send-all (flatten ll) :name)) *collidable-pairs*
(defvar *collidable-pairs*
  (let* ((all-link (union (flatten (send *robot* :collision-check-pairs)) nil))
	 (black-list '(("right_upper_shoulder" "right_upper_elbow_visual")
		       ("right_upper_elbow" "right_upper_forearm")
		       ("right_upper_elbow" "right_upper_forearm_visual")
		       ("right_upper_shoulder" "right_upper_elbow")
		       )))
    (remove nil
	    (mapcar
	     #'(lambda (name-name)
		 (let* ((n1 (car name-name)) (n2 (cadr name-name))
			(l1 (find-if #'(lambda (l) (string-equal (send l :name) n1)) all-link))
			(l2 (find-if #'(lambda (l) (string-equal (send l :name) n2)) all-link))
			(bl-check)
			)
		   (cond
		    ((or (not l1) (not l2)) nil)
		    ((setq bl-check (find-if #'(lambda (nn)
						 (or (and (string-equal (car nn) n1)
							  (string-equal (cadr nn) n2))
						     (and (string-equal (cadr nn) n1)
							  (string-equal (car nn) n2))))
					     black-list)) nil)
		    (t (cons l1 l2)))))
	     '(("base" "right_upper_forearm_visual") ("base" "right_upper_forearm") ("base" "right_lower_forearm") ("base" "right_wrist") ("torso" "right_upper_elbow_visual") ("torso" "right_upper_elbow") ("torso" "right_lower_elbow") ("torso" "right_upper_forearm_visual") ("torso" "right_upper_forearm") ("torso" "right_lower_forearm") ("torso" "right_wrist") ("torso" "right_hand") ("torso" "right_hand_range") ("torso" "right_hand_camera") ("torso" "right_hand_accelerometer") ("torso" "right_gripper_base") ("torso" "right_gripper") ("collision_head_link_2" "right_lower_elbow") ("collision_head_link_2" "right_upper_forearm_visual") ("collision_head_link_2" "right_upper_forearm") ("collision_head_link_1" "right_lower_elbow") ("collision_head_link_1" "right_upper_forearm_visual") ("collision_head_link_1" "right_upper_forearm") ("sonar_ring" "right_lower_elbow") ("sonar_ring" "right_upper_forearm_visual") ("sonar_ring" "right_upper_forearm") ("sonar_ring" "right_lower_forearm") ("sonar_ring" "right_wrist") ("sonar_ring" "right_hand") ("sonar_ring" "right_hand_accelerometer") ("sonar_ring" "right_gripper_base") ("right_arm_mount" "right_upper_forearm_visual") ("right_arm_mount" "right_upper_forearm") ("right_arm_mount" "right_lower_forearm") ("pedestal" "right_upper_forearm_visual") ("pedestal" "right_upper_forearm") ("pedestal" "right_lower_forearm") ("pedestal" "right_wrist") ("pedestal" "right_hand") ("pedestal" "right_hand_range") ("pedestal" "right_hand_camera") ("pedestal" "right_hand_accelerometer") ("pedestal" "right_gripper_base") ("pedestal" "right_gripper") ("left_arm_mount" "right_upper_forearm_visual") ("left_arm_mount" "right_upper_forearm") ("left_arm_mount" "right_lower_forearm") ("left_arm_mount" "right_wrist") ("head" "right_upper_elbow_visual") ("head" "right_upper_elbow") ("head" "right_lower_elbow") ("head" "right_upper_forearm_visual") ("head" "right_upper_forearm") ("head" "right_lower_forearm") ("head" "right_wrist") ("head" "right_hand") ("head" "right_hand_range") ("head" "right_hand_camera") ("head" "right_hand_accelerometer") ("head" "right_gripper_base") ("right_upper_shoulder" "right_upper_elbow_visual") ("right_upper_shoulder" "right_upper_elbow") ("right_upper_shoulder" "right_upper_forearm_visual") ("right_upper_shoulder" "right_upper_forearm") ("right_upper_shoulder" "right_lower_forearm") ("right_upper_shoulder" "right_wrist") ("right_upper_shoulder" "right_hand") ("right_upper_shoulder" "right_hand_range") ("right_upper_shoulder" "right_hand_camera") ("right_upper_shoulder" "right_hand_accelerometer") ("right_upper_shoulder" "right_gripper_base") ("right_upper_shoulder" "right_gripper") ("right_lower_shoulder" "right_lower_forearm") ("right_lower_shoulder" "right_wrist") ("right_lower_shoulder" "right_hand") ("right_lower_shoulder" "right_hand_range") ("right_lower_shoulder" "right_hand_camera") ("right_lower_shoulder" "right_hand_accelerometer") ("right_lower_shoulder" "right_gripper_base") ("right_lower_shoulder" "right_gripper") ("right_upper_elbow_visual" "right_upper_forearm_visual") ("right_upper_elbow_visual" "right_upper_forearm") ("right_upper_elbow_visual" "screen") ("right_upper_elbow_visual" "display") ("right_upper_elbow" "right_upper_forearm_visual") ("right_upper_elbow" "right_upper_forearm") ("right_upper_elbow" "screen") ("right_upper_elbow" "display") ("right_lower_elbow" "screen") ("right_lower_elbow" "display") ("right_upper_forearm_visual" "left_upper_shoulder") ("right_upper_forearm_visual" "left_lower_shoulder") ("right_upper_forearm_visual" "left_upper_elbow_visual") ("right_upper_forearm_visual" "left_upper_elbow") ("right_upper_forearm_visual" "screen") ("right_upper_forearm_visual" "display") ("right_upper_forearm" "left_upper_shoulder") ("right_upper_forearm" "left_lower_shoulder") ("right_upper_forearm" "left_upper_elbow_visual") ("right_upper_forearm" "left_upper_elbow") ("right_upper_forearm" "left_lower_elbow") ("right_upper_forearm" "screen") ("right_upper_forearm" "display") ("right_lower_forearm" "left_upper_shoulder") ("right_lower_forearm" "left_lower_shoulder") ("right_lower_forearm" "left_upper_elbow_visual") ("right_lower_forearm" "left_upper_elbow") ("right_lower_forearm" "left_lower_elbow") ("right_lower_forearm" "left_upper_forearm_visual") ("right_lower_forearm" "left_upper_forearm") ("right_lower_forearm" "screen") ("right_lower_forearm" "display") ("right_wrist" "left_upper_shoulder") ("right_wrist" "left_lower_shoulder") ("right_wrist" "left_upper_elbow_visual") ("right_wrist" "left_upper_elbow") ("right_wrist" "left_lower_elbow") ("right_wrist" "left_upper_forearm_visual") ("right_wrist" "left_upper_forearm") ("right_wrist" "left_wrist") ("right_wrist" "left_hand") ("right_wrist" "left_hand_camera") ("right_wrist" "screen") ("right_wrist" "display") ("right_hand" "left_upper_shoulder") ("right_hand" "left_lower_shoulder") ("right_hand" "left_upper_elbow_visual") ("right_hand" "left_upper_elbow") ("right_hand" "left_lower_elbow") ("right_hand" "left_upper_forearm_visual") ("right_hand" "left_upper_forearm") ("right_hand" "left_wrist") ("right_hand" "left_hand") ("right_hand" "screen") ("right_hand" "display") ("right_hand_range" "left_upper_shoulder") ("right_hand_range" "left_lower_shoulder") ("right_hand_range" "left_upper_elbow_visual") ("right_hand_range" "left_wrist") ("right_hand_range" "screen") ("right_hand_camera" "left_upper_shoulder") ("right_hand_camera" "left_lower_shoulder") ("right_hand_camera" "left_upper_elbow_visual") ("right_hand_camera" "left_upper_elbow") ("right_hand_camera" "left_upper_forearm_visual") ("right_hand_camera" "left_hand") ("right_hand_camera" "screen") ("right_hand_accelerometer" "left_upper_shoulder") ("right_hand_accelerometer" "left_lower_shoulder") ("right_hand_accelerometer" "screen") ("right_gripper_base" "left_upper_shoulder") ("right_gripper_base" "left_lower_shoulder") ("right_gripper_base" "left_upper_elbow_visual") ("right_gripper_base" "left_upper_elbow") ("right_gripper_base" "left_lower_elbow") ("right_gripper_base" "left_upper_forearm_visual") ("right_gripper_base" "left_upper_forearm") ("right_gripper_base" "screen") ("right_gripper_base" "display") ("right_gripper" "left_upper_shoulder") ("right_gripper" "left_lower_shoulder") ("right_gripper" "left_upper_elbow") ("right_gripper" "left_lower_elbow") ("right_gripper" "left_wrist") ("right_gripper" "screen"))))))

(defvar *trajectory*
  (let* ((dimension (length *joint-list1*))
	 (id-max 3)
	 (recursive-order (make-list dimension :initial-element 2))
	 (dim-list
	  (let ((id -1)) (mapcar #'(lambda (a) (incf id)) (make-list dimension)))))
    (instance partition-spline-vector
	      :init
	      :dimension dimension :id-max id-max
	      :x-min 0.0 :x-max 1.0 :recursive-order recursive-order)))


(defun gen-random-angle-vector
  (&optional (joint-list *joint-list1*))
  (dolist (j joint-list)
    (send j :joint-angle
          (-
           (+ (send j :min-angle)
              (* (random 1.0)
                 (- (send j :max-angle) (send j :min-angle))))
           (send j :joint-angle))
          :relative t)))

(defun set-angle-vector
  (&optional av (joint-list *joint-list1*))
  (map cons '(lambda (av j)
	       (send j :joint-angle
		     (- (rad2deg av) (send j :joint-angle))
		     :relative t))
       av joint-list))

(defun get-task-vector
  (&key
   (ef-coords *ef-coords*)
   (root-link *root-link*)
   (arrow *arrow*)
   (joint-list *joint-list1*)
   (robot *robot*)
   (offset-coords (make-list (length ef-coords)))
   (org-coords (send root-link :worldcoords))
   )
  (apply 'concatenate
         (cons float-vector
	       (append
		(mapcar #'(lambda (efc off)
			    (send efc :vector :org-coords org-coords :offset-coords off))
			ef-coords offset-coords)
		(list (map float-vector 'deg2rad (send-all joint-list :joint-angle)))))))

(defun get-ef-coords-from-task-vector
  (&key
   (ef-coords *ef-coords*)
   (vector (get-task-vector))
   (root-link *root-link*)
   (joint-list *joint-list1*)
   (robot *robot*)
   (org-coords (send root-link :worldcoords))
   (i 0) len
   ret)
  (dolist (efc ef-coords)
    (setq len (send efc :length))
    (push (send efc :coords-from-vector (subseq vector i (+ i len))
                :org-coords org-coords) ret)
    (setq i (+ i len)))
  (map cons
       #'(lambda (j av)
	   (send j :joint-angle
		 (- (rad2deg av) (send j :joint-angle))
		 :relative t))
       joint-list
       (subseq vector (- (length vector) (length joint-list))
	       (length vector)))
  (reverse ret))

(defun test-gen-collidable-pairs
  (&key
   (robot *robot*)
   (joint-list *joint-list1*)
   (draw? t)
   (max-cnt (if draw? 100 10000)))
  (let* ((all-pairs (send robot :collision-check-pairs))
	 (col-cnt (instantiate integer-vector (length all-pairs)))
	 ret)
    (warning-message 6 "[test-gen-collidable-pairs] start~%")
    (dotimes (i max-cnt)
      (if (eq 0 (mod i (/ max-cnt 100))) (warning-message 6 "  ~A%~%" (/ (* 100.0 i) max-cnt)))
      (gen-random-angle-vector joint-list)
      (if draw? (send *viewer* :draw-objects))
      (let* ((buf) (j 0))
	(dolist (ll all-pairs)
	  (setq buf (user::pqp-collision-check (car ll) (cdr ll)))
	  (if (plusp buf) (setf (aref col-cnt j) (+ (aref col-cnt j) 1)))
	  (incf j))))
    ;;
    (let* ((buf) (j 0))
      (dolist (ll all-pairs)
	(cond
	 ((eq max-cnt (aref col-cnt j))
	  (warning-message 1 " ~A always collided? ... skip~%" (send-all (flatten ll) :name)))
	 ((zerop (aref col-cnt j))
	  (warning-message 1 " ~A(~A) never collide~%" (send-all (flatten ll) :name) (aref col-cnt j)))
	 (t ;;(plusp (aref col-cnt j))
	  (warning-message 6 " ~A(~A) collidable~%" (send-all (flatten ll) :name) (aref col-cnt j))
	  (push ll buf)))
	(incf j))
      (setq *collidable-pairs* buf))))

(defun check-all-collision nil (send *robot* :self-collision-check :pairs *collidable-pairs*))
(defun random-test-check-all-collision
  (&optional
   (init (gen-random-angle-vector *joint-list1*)))
  (send *viewer* :draw-objects)
  (let* ((ret (send *robot* :self-collision-check :pairs *collidable-pairs*)))
    (send-all (flatten ret) :draw-on :flush nil)
    (send *viewer* :viewsurface :flush)
    ret)
  )

;; (let* ((a (integer-vector 0 0))) (gen-random-trajectory :draw? t :error-cnt a :debug? :no-message) (print a))
;; (let* ((max 1000) (tm (bench2 (dotimes (i max) (let* ((a (integer-vector 0 0))) (gen-random-trajectory :draw? nil :error-cnt a :debug? :no-message)))))) (format t "~A x ~A = ~Asec~%" (/ tm max) max tm))
(defun gen-random-trajectory
  (&rest args &key
	 (bspline *trajectory*)
	 (dimension (send bspline :dimension))
	 (id-max (send bspline :id-max))
	 (recursive-order (send bspline :recursive-order))
	 (dim-list
	  (let ((id -1)) (mapcar #'(lambda (a) (incf id)) (make-list dimension))))
	 ;;
	 (debug? t) (draw? debug?) (revert? t) (collide-check :first)
	 (error-cnt (instantiate integer-vector 2))
	 (joint-list *joint-list1*)
	 (x-min (send bspline :x-min)) (x-max (send bspline :x-max)) (x-mid (/ (+ x-min x-max) 2.0))
	 (start-pos (progn (gen-random-angle-vector joint-list)
			   (map float-vector 'deg2rad (send-all joint-list :joint-angle))))
	 (mid-pos (progn (gen-random-angle-vector joint-list)
			 (map float-vector 'deg2rad (send-all joint-list :joint-angle))))
	 (end-pos (progn (gen-random-angle-vector joint-list)
			 (map float-vector 'deg2rad (send-all joint-list :joint-angle))))
	 (pos-coeff-matrix
	  (matrix-append
	   (mapcar
	    #'(lambda (pos tm)
		(send bspline :calc-pos-constraints-coeff-matrix-for-gain-vector
		      :dim-list dim-list :tm tm))
	    (list start-pos mid-pos end-pos) (list x-min x-mid x-max))
	   '(1 0)))
	 (pos-vector
	  (concatenate float-vector start-pos mid-pos end-pos))
	 (gain-vector (transform (pseudo-inverse pos-coeff-matrix) pos-vector))
	 (gain-matrix (send bspline :convert-gain-vector-to-gain-matrix gain-vector))
	 ;;
	 check
	 &allow-other-keys
	 )
  (if debug?
      (setq check (apply #'check-trajectory :bspline bspline :joint-list joint-list :start-pos start-pos :mid-pos mid-pos :end-pos end-pos  :error-cnt error-cnt :debug? debug? :draw? draw? :revert? revert? :collide-check collide-check args)))
  (if (and debug? (not check)) nil bspline)
  )

(defun check-trajectory
  (&rest args &key
	 (bspline *trajectory*)
	 (joint-list *joint-list1*)
	 (x-min (send bspline :x-min)) (x-max (send bspline :x-max))
	 (x-mid (/ (+ x-min x-max) 2.0))
	 start-pos end-pos mid-pos
	 (_start-pos (or start-pos (map float-vector 'deg2rad (send-all joint-list :joint-angle))))
	 ;;
	 (x-step 0.1) (x-max-called)
	 (debug? t) (draw? debug?) (revert? t) (collide-check :first)
	 (error-cnt (instantiate integer-vector 2))
	 ;;
	 (root-link-fix-coords-func *root-link-fix-coords-func*)
	 ret
	 &allow-other-keys
	 )
  (let* ((pos0 (send bspline :calc x-min))
	 (pos1 (send bspline :calc x-mid))
	 (pos2 (send bspline :calc x-max))
	 (dif0 (if start-pos (norm (v- pos0 start-pos))))
	 (dif1 (if mid-pos (norm (v- pos1 mid-pos))))
	 (dif2 (if end-pos (norm (v- pos2 end-pos))))
	 (thre 1e-3))
    (if (not (eq debug? :no-message)) (warning-message 6 "[check-trajectory]::debug~%"))
    ;;
    (cond
     (start-pos
      (if (not (eq debug? :no-message))
	  (warning-message 6 "start-pos: ~A - ~A = ~A < ~A ?~%" pos0 start-pos dif0 thre))
      (if (> dif0 thre) (warning-message 1 "  -- start-pos not satisfied!!!~%"))))
    (cond
     (mid-pos
      (if (not (eq debug? :no-message))
	  (warning-message 6 "mid-pos:   ~A - ~A = ~A < ~A ?~%" pos1 mid-pos dif1 thre))
      (if (> dif1 thre) (warning-message 1 "  -- mid-pos   not satisfied!!!~%"))))
    (cond
     (end-pos
      (if (not (eq debug? :no-message))
	  (warning-message 6 "end-pos:   ~A - ~A = ~A < ~A ?~%" pos2 end-pos dif2 thre))
      (if (> dif2 thre) (warning-message 1 "  -- end-pos   not satisfied!!!~%"))))
    ;;
    (let* ((x x-min) (pos) col
	   (range-min (map float-vector 'deg2rad (send-all joint-list :min-angle)))
	   (range-max (map float-vector 'deg2rad (send-all joint-list :max-angle))))
      (cond
       ((not (eq debug? :no-message))
	(warning-message 6 "range&collision-check:~%")
	(warning-message 6 "  -- min: ~A~%" range-min)
	(warning-message 6 "  -- max: ~A~%" range-max)))
      (catch :collision-detected-message
	(while (<= x x-max)
	  (setq pos (send bspline :calc x))
	  (map cons #'(lambda (j av)
			(if (eq debug? :no-message)
			    (send j :joint-angle (- (rad2deg av) (send j :joint-angle)) :relative t)
			  (send j :joint-angle (rad2deg av))))
	       joint-list pos)
	  (if (functionp root-link-fix-coords-func) (funcall root-link-fix-coords-func))
	  (send-all (send-all joint-list :child-link) :worldcoords)
	  (send (car *arrow*) :newcoords
		(send (send *robot* :rarm :end-coords :worldcoords) :copy-worldcoords))
	  (send-all *arrow* :worldcoords)
	  (cond
	   (draw? (send *viewer* :draw-objects) (unix::usleep (* 100 1000))))
	  (if (not (eq debug? :no-message)) (warning-message 6 "  -- ~A: ~A E ?~%" x pos))
	  ;;
	  (cond
	   ((setq col (check-all-collision))
	    (setf (aref error-cnt 0) (+ (aref error-cnt 0) 1))
	    (cond (draw? (random-test-check-all-collision nil) (unix::usleep (* 100 1000))))
	    (if (not (eq debug? :no-message)) (warning-message 1 "  -- collision ~A~%" col))
	    (if (eq collide-check :first) (throw :collision-detected-message (setq ret :collide)))
	    ))
	  ;;
	  (setq x (+ x x-step))
	  (if (and (not x-max-called) (>= x x-max)) (progn (setq x-max-called t) (setq x x-max)))
	  ))
      ;;
      (cond
       (revert?
	(map cons '(lambda (av j) (send j :joint-angle
					(- (rad2deg av) (send j :joint-angle))
					:relative t))
	     _start-pos joint-list)
	(if (not (eq debug? :no-message)) (warning-message 6 "reverted~%"))))
      ;;
      (if (eq ret :collide) nil (get-task-vector))
      )))

(defun gen-random-valid-trajectory-vector
  (&rest args)
  (let* (ret)
    (while (not ret)
      (setq ret (apply #'gen-random-trajectory (append args (list :debug? :no-message :draw? nil :revert? t :collide-check :first)))))
    (send ret :gain-vector)))

(defun gen-random-traj-learning-data
  (&key (sample-points *sample-points*)
        (joint-list *joint-list1*)
        (ef-coords *ef-coords*)
	(control-points (send *trajectory* :id-max))
        (root-link *root-link*)
	;;
        (i 0)
        (draw? nil)
        (dump? t)
        (surfix (if dump?
                    (format nil ".~A" (unix-command-line "basename `pwd`"))))
        (gen-random-traj-vector-func 'gen-random-valid-trajectory-vector)
	 ;; #'(lambda (&rest args)
	 ;;     (let* ((errcnt (integer-vector 0 0))
	 ;; 	    (ret
	 ;; 	     (gen-random-trajectory :debug? :no-message :revert? t
	 ;; 				    :draw? nil :error-cnt errcnt)))
	 ;;       (if (and ret (zerop (aref errcnt 0)))
	 ;; 	   (send ret :gain-vector)))))
        ;;
        (len (length (get-task-vector)))
	(skip-cnt 0)
        &allow-other-keys
        )
  (setq *sample-points* sample-points)
  (cond
   ((not (eq (length *state-map*)
             (* control-points (length joint-list) sample-points)))
    (warning-message 1 "state-map alloc~%")
    (setq *state-map*
          (instantiate float-vector
                       (* control-points (length joint-list) sample-points)))))
  (cond
   ((not (eq (length *task-map*)
             (* len sample-points)))
    (warning-message 1 "task-map alloc~%")
    (setq *task-map*
          (instantiate float-vector (* len sample-points)))))
  ;;
  (let* (task traj)
    (while (< i sample-points)
      (if (and (> sample-points 100)
               (zerop (mod i (round (/ sample-points 100)))))
          (warning-message 3 "~A%~%" (/ i (round (/ sample-points 100)))))
      (setq traj nil)
      (if (functionp gen-random-traj-vector-func)
          (setq traj (funcall gen-random-traj-vector-func))
        (throw :invalid-gen-random-angle-vector-func nil))
      (cond
       ((and (vectorp traj)
	     (eq (length traj) (* control-points (length joint-list))))
	;;
	(dotimes (j (length traj))
	  (setf (aref *state-map* (+ (* i (length traj)) j)) (aref traj j)))
	;;
	(send-all (send-all joint-list :child-link) :worldcoords)
	(send-all ef-coords :worldcoords)
	(setq task (get-task-vector :ef-coords ef-coords :root-link root-link))
	(dotimes (j len) (setf (aref *task-map* (+ j (* i len))) (aref task j)))
	;;
	;; (random-check-traning-db :state (subseq *state-map* (* i (length traj)) (* (+ i 1) (length traj)))
	;; :task (subseq *task-map* (* i (length task)) (* (+ i 1) (length task))))
	;;
	(incf i)
	)
       (t (incf skip-cnt))
       ))
    (if dump? (dump-traj-learning-data :surfix surfix))
    (warning-message 6 "~A + ~A = ~A sampling~%" sample-points skip-cnt (+ sample-points skip-cnt))
    skip-cnt
    ))

;; random-check-traning-db :state (gen-random-valid-trajectory-vector) :task (progn (send-all (send-all *joint-list1* :child-link) :worldcoords) (send-all *ef-coords* :worldcoords) (get-task-vector :ef-coords *ef-coords* :root-link *root-link*))
(defun random-check-traning-db
  (&key
   (init
    (cond
     ((or (zerop (length *state-map*))
	  (zerop (length *task-map*)))
      (load-traj-learning-data)
      (setq *sample-points* ;;96)
	    (/ (length *state-map*) (* (send *trajectory* :id-max) (send *trajectory* :dimension)))))))
   (rpos (random *sample-points*))
   (stated (/ (length *state-map*) *sample-points*))
   (taskd (/ (length *task-map*) *sample-points*))
   (task-diff) (thre 1e-3)
   (state (subseq *state-map* (* (+ rpos 0) stated) (* (+ rpos 1) stated)))
   (task (subseq *task-map* (* (+ rpos 0) taskd) (* (+ rpos 1) taskd)))
   (target-task))
  (send *trajectory* :convert-gain-vector-to-gain-matrix (copy-seq state))
  (check-trajectory :start-pos (subseq task (- (length task) (length *joint-list1*)) (length task)))
  (setq target-task (get-task-vector))
  ;;
  (setq task-diff (v- task target-task))
  (if (> (norm task-diff) thre) (warning-message 1 "[random-check-traning-db] too large error~%"))
  task-diff)

(defun load-traj-learning-data
  (&key (root "teacher") (surfix ""))
  (setq *state-map* (cpplog::read-vector (format nil "~A/state-map.dat~A" root surfix)))
  (setq *task-map* (cpplog::read-vector (format nil "~A/task-map.dat~A" root surfix)))
  (length *task-map*))

(defun dump-traj-learning-data
  (&key (root "teacher") (surfix ""))
  (unix::system (format nil "mkdir -p ~A" root))
  (cpplog::write-vector (format nil "~A/state-map.dat~A" root surfix)
                        *state-map*)
  (cpplog::write-vector (format nil "~A/task-map.dat~A" root surfix)
                        *task-map*))

(defun traj-learn
  (&key (solver "traj_solver.prototxt")
        (solverstate "")
        &allow-other-keys)
  (cond
   ((or (zerop (length *state-map*))
        (zerop (length *task-map*)))
    (load-traj-learning-data)
    (setq *sample-points* ;;96)
          (/ (length *state-map*) (* 3 (length *joint-list1*))))))
  ;;
  (cond
   ((or (zerop (length *state-map*))
        (zerop (length *task-map*)))
    (warning-message 1 "zero training DB~%"))
   (t
    (caffe::learn
     :solver solver
     :solverstate solverstate
     :size *sample-points*
     :ddata *task-map*
     :idata *state-map*
     ))))

(defun unix-command-line
  (cmd)
  (let* ((p (piped-fork cmd))
         (ret (read-line p nil)))
    (close p)
    ret))

(defun gen-snapshot-prefix
  (solver)
  (unix-command-line
   (concatenate string
                (format nil "cat ~A" solver)
                " | "
                "grep \"snapshot_prefix\""
                " | "
                "sed -e \"s/^snapshot_prefix: \\\"\\\(.\\\+\\\)\\\"$/\\\1/g\""
                )))

(defun link-best-settings
  (solver
   &key
   (best-path solver)
   (best-snap (gen-snapshot-prefix best-path))
   (caffe-snap
    (unix-command-line
     (format nil "ls ~A*caffemodel -t | head -n 1" best-snap)))
   (solver-snap
    (unix-command-line
     (format nil "ls ~A*solverstate -t | head -n 1" best-snap)))
   (solver-lnk
    (format nil "ln -snf ~A traj_solver.best.prototxt" best-path))
   (caffe-lnk
    (format nil "ln -snf ~A traj.best.caffemodel" caffe-snap))
   (solverstate-lnk
    (format nil "ln -snf ~A traj.best.solverstate" solver-snap)))
  ;;
  (warning-message 3 (format nil "~A~%" solver-lnk))
  (if (string-equal solver "traj_solver.best.prototxt")
      (print 'skip)
    (unix::system solver-lnk))
  (warning-message 3 (format nil "~A~%" caffe-lnk))
  (unix::system caffe-lnk)
  (warning-message 3 (format nil "~A~%" solverstate-lnk))
  (unix::system solverstate-lnk)
  )

(defun traj-learn-with-all-solver
  (&rest args)
  (let* ((ret
          (sort
           (mapcar
            #'(lambda (solver)
                (cons
                 solver
                 (apply 'traj-learn :solver solver args)))
            (remove-if-not
             #'(lambda (str) (and (substringp "traj_solver" str)
                                  (substringp "prototxt" str)))
             (directory)))
           #'(lambda (a b) (< (cdr a) (cdr b)))))
         )
    (link-best-settings (car (car ret)))
    ret))

(defun traj-learn-best
  (&rest args &key (thre) &allow-other-keys)
  (let* (config state)
    ;; detect solver config
    (setq config
          (or (unix-command-line "ls traj_solver.best.prototxt")
              (unix-command-line "ls -t traj_solver*prototxt")))
    ;;
    ;; detect solver state
    (setq state
          (or (unix-command-line "ls traj.best.solverstate")
              (unix-command-line "ls -t traj*solverstate")
              ""))
    ;;
    (warning-message 3 "config: ~A, state: ~A~%" config state)
    ;;
    (cond
     ((not (and config state))
      (warning-message 1 "empty setting files~%")
      (throw :settings-not-found nil)))
    (warning-message 3 "regenrate random traning DB~%")
    (if (numberp thre)
	(warning-message 1 "gen-random-traj-learning-data, thre not supported~%"))
    ;; (gen-random-traj-learning-data-with-thre :draw? nil :thre thre))
    (gen-random-traj-learning-data)
    (warning-message 3 "update solver~%")
    (let* ((p (apply 'update-solver :input "traj_solver.best.prototxt" args)))
      (unix::system (format nil "ln -snf ~A traj_solver.best.prototxt" p)))
    (warning-message 3 "start learning ... ~%")
    (apply 'traj-learn :solver config :solverstate state args)
    ;; (traj-learn :solver solver :solverstate state)
    (warning-message 3 "done~%")
    (link-best-settings "traj_solver.best.prototxt")
    (warning-message 3 "evaluate network~%")
    (cpplog::change-output-stream "/dev/null")
    (let* ((ret (traj-random-test-min-average :draw? nil)))
      (cpplog::change-output-stream "")
      ret)
    ))

(defun traj-learn-best-loop
  (&rest args
         &key
         (lp 10)
         ;;
         (base_lr-gain 1.0)
         (eq-scale-thre 0.01)
         (base_lr-gain-update-scale 1.1)
         (vain-cnt 0)
         (vain-cnt-max 5) prev now
         (thre
          (progn
            (cpplog::change-output-stream "/dev/null")
            (let* ((ret (traj-random-test-min-average :draw? nil)))
              (cpplog::change-output-stream "")
              ret)))
         ;;
         &allow-other-keys)
  (let* ((ret))
    (push thre ret)
    (dotimes (i lp)
      (unix::gc)
      (push (apply 'traj-learn-best :base_lr-gain base_lr-gain :thre (car ret) args) ret)
      (setq prev now)
      (setq now (car ret))
      ;; update base_lr-gain
      (cond
       ((or (not (numberp now)) (not (numberp prev)))
        'skip)
       ((eps= now prev (* now eq-scale-thre))
        ;;(setq base_lr-gain (* base_lr-gain base_lr-gain-update-scale))
        (setq base_lr-gain base_lr-gain-update-scale)
        (incf vain-cnt)
        )
       ((eps< now prev (* now eq-scale-thre))
        (setq base_lr-gain 1.0)
        (decf vain-cnt)
        'nop
        )
       ((eps> now prev (* now eq-scale-thre))
        ;; (setq base_lr-gain (/ base_lr-gain base_lr-gain-update-scale))
        (setq base_lr-gain (/ 1.0 base_lr-gain-update-scale))
        (incf vain-cnt)
        ))
      (warning-message
       3 (format nil "~A~%" (unix-command-line "ls -l traj.best.caffemodel | sed -e \"s/ /\\\n/g\" | tail -1")))
      (warning-message
       3 (format nil "~A~%" (unix-command-line "ls -l traj.best.solverstate | sed -e \"s/ /\\\n/g\" | tail -1")))
      (warning-message 3 "update base_lr-gain: ~A, vain: ~A, now: ~A~%"
                       base_lr-gain vain-cnt now)
      ;; convergence
      (cond
       ((>= vain-cnt vain-cnt-max)
        (warning-message 3 "loop exceeded, ~A > ~A~%"
                         vain-cnt vain-cnt-max)
        (return-from nil nil)))
      )
    ;; (setq *ret* ret)
    ret))

(defun smart-initialize-net
  nil
  (let* ((caffemodel (unix-command-line "ls -t | grep -e \"caffemodel$\" | head -n 1")))
    (warning-message 6 "load caffe model ~A~%" caffemodel)
    (caffe::gen-test-net :netproto "predict_traj_net.prototxt"
                         :caffemodel caffemodel))
  )

(defun traj-random-test
  (&key (calc-forward 'caffe::calc-forward-double) ;;'caffe::memory-calc-forward)
        (draw? t) (revert? nil) (update-arrow-coords? t)
        tm
	(trajectory *trajectory*)
        (joint-list *joint-list1*)
        (ef-coords *ef-coords*)
        (root-link *root-link*)
        (root-link-fix-coords-func *root-link-fix-coords-func*)
        (random-state (gen-random-valid-trajectory-vector))
        (update-coords
         (progn (if (functionp root-link-fix-coords-func)
                    (funcall root-link-fix-coords-func))
                (send-all (send-all joint-list :child-link) :worldcoords)
                (send-all ef-coords :worldcoords)
		(mapcar '(lambda (ar ar2) (send ar :newcoords (send ar2 :copy-worldcoords)))
			*goal-arrow* *arrow*)
		(send-all *goal-arrow* :worldcoords)))
	(random-task (get-task-vector :ef-coords ef-coords :root-link root-link))
	_random-task
	;;
        (state (instantiate float-vector (* (send trajectory :id-max) (send trajectory :dimension))))
	(task nil) ;; (copy-seq random-task))
	(collide?)
        (initialize? (smart-initialize-net))
        (redundancy ;;(instantiate float-vector (- (caffe::get-blob-count "input") (length input))))
	 (map float-vector '(lambda (hoge) (random 1.0))
	      (make-list (- (caffe::get-blob-count "input") (length random-task)))))
	;;
	(after-callback-func)
	(evaluation-callback-func
	 '(lambda (target-state target-task state task collide?)
	    (if collide?
		*inf*
	      (sqrt (/ (norm2 (v- target-state state)) (length target-state)))
	      ;; (sqrt (/ (norm2 (v- target-task task)) (length target-task)))
	      ;; (sqrt (/ (norm2 (subseq (v- target-task task) 0 6)) 6))
	      )))
	evaled-value
        &allow-other-keys
        )
  (setq _random-task (concatenate float-vector random-task redundancy))
  (setq
   tm
   (bench2
    (apply calc-forward
           (list :isize (length _random-task) :input _random-task
                 :osize (length state) :output state))
    ))
  (send trajectory :convert-gain-vector-to-gain-matrix (copy-seq state))
  ;;
  (setq collide?
	(check-trajectory :bspline trajectory :joint-list joint-list
			  ;; :start-pos (subseq random-task (- (length random-task) (length joint-list))
			  ;; (length random-task))
			  :collide-check :first :draw? draw? :debug? :no-message
			  :root-link-fix-coords-func root-link-fix-coords-func))
  (setq task (or collide? (get-task-vector)))
  ;;
  (if evaluation-callback-func
      (setq evaled-value (funcall evaluation-callback-func random-state random-task state task (not collide?))))
  ;;
  (cond
   ((or (not (functionp after-callback-func))
	(funcall after-callback-func av c ef-coords))
    (list
     (cons :time tm)
     (cons :state state)
     (cons :redundancy (copy-seq redundancy))
     (cons :task-diff (v- random-task task))
     (cons :state-diff (v- random-state state))
     (cons :evaluation evaled-value)
     (cons :collide? (not collide?))
     ))))

(defun traj-random-test-min
  (&rest
   args
   &key
   ;;
   (draw? nil)
   (joint-list *joint-list1*)
   (ef-coords *ef-coords*)
   (root-link *root-link*)
   (random-state (gen-random-valid-trajectory-vector))
   (root-link-fix-coords-func *root-link-fix-coords-func*)
   (update-coords
    (progn (if (functionp root-link-fix-coords-func)
	       (funcall root-link-fix-coords-func))
	   (send-all (send-all joint-list :child-link) :worldcoords)
	   (send-all ef-coords :worldcoords)
	   (mapcar '(lambda (ar ar2) (send ar :newcoords (send ar2 :copy-worldcoords)))
		   *goal-arrow* *arrow*)
	   (send-all *goal-arrow* :worldcoords)))
   (random-task (get-task-vector :ef-coords ef-coords :root-link root-link))
   (initialize? (smart-initialize-net))
   ;;
   (evaluation-func
    '(lambda (val soa)
       (< (cdr (assoc :evaluation val))
	  (cdr (assoc :evaluation soa)))))
   (ret nil) (step-max 5000)
   best
   &allow-other-keys
   )
  (cpplog::change-output-stream "/dev/null")
  (dotimes (i step-max)
    (push (apply 'traj-random-test :draw? draw? :joint-list joint-list :ef-coords ef-coords :root-link root-link :random-state random-state :update-coords update-coords :random-task random-task :initialize? nil :root-link-fix-coords-func root-link-fix-coords-func args) ret)
    (if (not best) (setq best (car ret))
      (if (funcall evaluation-func (car ret) best) (setq best (car ret)))))
  (cpplog::change-output-stream "")
  best)

(defun traj-random-test-min-average
  (&rest args &key (lp 100) (step-max 300) (variance (float-vector 0)) &allow-other-keys)
  (let* ((ret) avr var)
    (smart-initialize-net)
    (dotimes (i lp)
      (push (apply 'traj-random-test-min :initialize? nil :step-max step-max args) ret))
    (setq ret (remove nil ret))
    ;;
    (setq avr (/ (apply '+ (mapcar '(lambda (d) (cdr (assoc :evaluation d))) ret))
		 (length ret)))
    (setq var (mapcar #'(lambda (d) (- avr (cdr (assoc :evaluation d)))) ret))
    (setq var (apply '+ (mapcar '(lambda (d) (* d d)) var)))
    (setq var (sqrt (/ var (length ret))))
    (setf (aref variance 0) var)
    avr))

#|
