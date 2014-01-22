(require "package://hrpsys_gazebo_atlas/euslisp/atlas-model.l")
(require "package://atlas_prog/models/atlas-hook-model.l")

(defvar *robot* (atlas-with-hook))
(defvar *atlas* *robot*)

;; (defmethod robot-model
;;   (:set-end-coords
;;    (&key
;;     (key :rarm)
;;     (coords (send self key :end-coords :copy-worldcoords)))
;;    (let* ((now (send self key :end-coords))
;; 	  (root (send now :parent)))
;;      (send root :dissoc now)
;;      (case key
;; 	   (:rarm 
;; 	    (setq rarm-end-coords
;; 		  (make-cascoords :init :link-list
;; 				  :parent root
;; 				  :coords coords
;; 				  :name (send now :name)))
;; 	    (send root :assoc rarm-end-coords)
;; 	    )
;; 	   (:larm
;; 	    (setq larm-end-coords
;; 		  (make-cascoords :init :link-list
;; 				  :parent root
;; 				  :coords coords
;; 				  :name (send now :name)))
;; 	    (send root :assoc larm-end-coords)
;; 	    )
;; 	   (:rleg
;; 	    (setq rleg-end-coords
;; 		  (make-cascoords :init :link-list
;; 				  :parent root
;; 				  :coords coords
;; 				  :name (send now :name)))
;; 	    (send root :assoc rleg-end-coords)
;; 	    )
;; 	   (:lleg
;; 	    (setq lleg-end-coords
;; 		  (make-cascoords :init :link-list
;; 				  :parent root
;; 				  :coords coords
;; 				  :name (send now :name)))
;; 	    (send root :assoc lleg-end-coords)
;; 	    )
;; 	   ))))

(defun set-end-coords
  (key coords
       &key
       (now-av (copy-object (send *robot* :angle-vector)))
       (now-c (copy-object (send *robot* :copy-worldcoords)))
       buf ret)
  (if (find-method *robot* :old-reset-manip-pose)
      (send *robot* :old-reset-manip-pose)
    (send *robot* :reset-manip-pose))
  (send *robot* :newcoords (make-coords))
  (send *robot* key :end-coords :worldcoords)
  (let ((trans
	 (send (send *robot* key :end-coords)
	       :transformation
	       (copy-object coords))))
    (send (send *robot* key :end-coords) :transform trans))
  (send *robot* key :end-coords :worldcoords)
  (send *robot* :angle-vector now-av)
  (send *robot* :newcoords now-c)
  (send *robot* key :end-coords :worldcoords)
  (send *robot* key :end-coords))

;; (defun set-end-coords
;;   (key coords
;;        &key
;;        (now-av (copy-object (send *robot* :angle-vector)))
;;        (now-c (copy-object (send *robot* :copy-worldcoords)))
;;        buf ret)
;;   (send *robot* :reset-manip-pose)
;;   (send *robot* :newcoords (make-coords))
;;   (send *robot* key :end-coords :worldcoords)
;;   ;;
;;   (send *robot* :set-end-coords
;; 	:key key
;; 	:coords coords)
;;   ;;
;;   (send *robot* key :end-coords :worldcoords)
;;   (send *robot* :angle-vector now-av)
;;   (send *robot* :newcoords now-c)
;;   (send *robot* key :end-coords :worldcoords)
;;   (send *robot* key :end-coords))

;; reset-manip-pose & pelvis 0-coords
(defvar *limb-key* '(:rarm :larm :rleg :lleg))
(defvar *switch-endcoords-collections*
  (list
   (list
    (make-coords
     :pos (float-vector 474.777 -267.564 376.567)
     :rpy (float-vector 7.748074e-06 -0.002922 -1.494227e-06))
    (make-coords
     :pos (float-vector 474.779 267.726 376.562)
     :rpy (float-vector -7.472436e-06 -0.00291 2.071643e-06))
    (make-coords
     :pos (float-vector 24.7508 -88.999 -794.516)
     :rpy (float-vector -3.582311e-09 -0.002932 1.221736e-06))
    (make-coords
     :pos (float-vector 24.7513 89.001 -794.515)
     :rpy (float-vector -3.582311e-09 -0.002932 1.221736e-06))) ;; default
   (list
    (make-coords
     :pos (float-vector 535.86 -263.914 367.043)
     :rpy (float-vector -3.14159 -0.307211 1.57079))
    (make-coords
     :pos (float-vector 535.862 264.076 367.037)
     :rpy (float-vector -4.551172e-06 0.308817 1.57079))
    (make-coords
     :pos (float-vector 24.7508 -88.999 -794.516)
     :rpy (float-vector -3.582311e-09 -0.002932 1.221736e-06))
    (make-coords
     :pos (float-vector 24.7513 89.001 -794.515)
     :rpy (float-vector -3.582311e-09 -0.002932 1.221736e-06)))
   (list
    (make-coords
     :pos (float-vector 664.777 -297.564 436.567)
     :rpy (float-vector 7.748074e-06 -0.002922 -1.494227e-06))
    (make-coords
     :pos (float-vector 664.779 297.726 436.562)
     :rpy (float-vector -7.472436e-06 -0.00291 2.071643e-06))
    (make-coords
     :pos (float-vector 24.7508 -88.999 -794.516)
     :rpy (float-vector -3.582311e-09 -0.002932 1.221736e-06))
    (make-coords
     :pos (float-vector 24.7513 89.001 -794.515)
     :rpy (float-vector -3.582311e-09 -0.002932 1.221736e-06)))
   ))
(defvar *now-end-coords-id* 0)

(defun seq-end-coords
  (&key
   (now-av (copy-object (send *robot* :angle-vector)))
   (now-c (copy-object (send *robot* :copy-worldcoords)))
   buf
   ret
   )
  (if (find-method *robot* :old-reset-manip-pose)
      (send *robot* :old-reset-manip-pose)
    (send *robot* :reset-manip-pose))
  (send *robot* :newcoords (make-coords))
  (setq
   ret
   (cons
    'list
    (mapcar
     #'(lambda (k)
	 (setq buf (send *robot* k :end-coords :copy-worldcoords))
	 (list 'make-coords
	       :pos (send buf :worldpos)
	       :rpy (coerce (car (send buf :rpy-angle)) float-vector)))
     *limb-key*)))
  (send *robot* :angle-vector now-av)
  (send *robot* :newcoords now-c)
  ret)

(defun switch-end-coords
  (&key
   (id 0))
  (cond
   ((eq id *now-end-coords-id*)
    nil)
   ((< id (length *switch-endcoords-collections*))
    (setq *now-end-coords-id* id)
    (mapcar
     #'set-end-coords
     *limb-key*
     (nth id *switch-endcoords-collections*)))))



#|

(defun setup-end-coords ()
  (unless (boundp '*setup-end-coords-for-hku-flag*)
    ;; right
    (let ((end-coords (make-coords))
	  (arm :rarm)
	  ;;(palm-link :right_palm_lk)
	  (palm-link :right_palm_left_camera_optical_frame_lk)
	  (axis (float-vector 0 0 1))
	  (rad 2.23))
      ;;(rad 1.57))
      ;;(rad 0))
      (send end-coords :rotate rad axis)
      (send end-coords :translate (float-vector 30 -250 45) :world)
      (let ((new-end-coords
	     (send (send *atlas* :r_hand_lk) :copy-worldcoords)))
	;;(send (send (send *atlas* :hand arm) palm-link) :copy-worldcoords)))
	(send new-end-coords :transform end-coords :local)
	(let ((trans (send (send *atlas* arm :end-coords :copy-worldcoords)
			   :transformation new-end-coords)))
	  (send (send *atlas* arm :end-coords) :transform trans))
	))
    ;; left
    (let ((end-coords (make-coords))
	  (arm :larm)
	  ;;(palm-link :right_palm_lk)
	  (palm-link :left_palm_left_camera_optical_frame_lk)
	  (axis (float-vector 0 0 1))
	  (rad 0.91))
      (send end-coords :rotate rad axis)
      (send end-coords :translate (float-vector 30 250 45) :world)
      (let ((new-end-coords
	     (send (send *atlas* :l_hand_lk) :copy-worldcoords)))
	;;(send (send (send *atlas* :hand arm) palm-link) :copy-worldcoords)))
	(send new-end-coords :transform end-coords :local)
	(let ((trans (send (send *atlas* arm :end-coords) :transformation
			   new-end-coords)))
	  (send (send *atlas* arm :end-coords) :transform trans))
	))
    )
  (setq *setup-end-coords-for-hku-flag* t)
  )
