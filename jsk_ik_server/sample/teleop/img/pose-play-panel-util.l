#!/usr/bin/env roseus

;; (progn (jsk) (rbrain) (require "package://euslisp/jskeus/irteus/demo/sample-robot-model.l") (setq *robot* (instance sample-robot :init)) (pickview :no-menu t) (objects (list *robot*)))

(send *pickview* :viewer :viewsurface :bg-color #F(0.9 0.9 0.9 0))

(defun dump-pose-data
  (&key
   (name "test")
   (path (format nil "~A.jpg" name))
   (links
    (remove-if
     #'(lambda (l)
	 (not (find-method (send l :joint) :joint-angle)))
     (flatten
      (append
       (mapcar
	#'(lambda (k) (send *robot* k :links))
	'(:rarm :larm :torso :head :rleg :lleg))
       (send *robot* :arms :hand :links)))))
   )
  (send *viewer* :draw-objects)
  (send *viewer* :viewsurface :write-to-jpg-file path)
  (list
   'list
   (list 'cons :name name)
   (list 'cons :path path)
   (list 'cons :joint_states
	 (list
	  'list
	  (list 'cons :name
		(cons
		 'list
		 (mapcar #'(lambda (n) (format nil "~A" n)) (send-all links :name))))
	  (list 'cons :position
		(cons
		 'list
		 (mapcar #'deg2rad
			 (send-all (send-all links :joint) :joint-angle))))))))
