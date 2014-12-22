(defun inverse-reachablity-with-given-coords
  (robot base-coords-list
	 &rest args ;;same with the command sent for inverse-kinematics
	 )
  (let* ((ik-command-index (or (position :inverse-kinematics args) (position :fullbody-inverse-kinematics args)))
	 (target-coords (elt args (+ ik-command-index 1)))
	 (initial-angle-vector (send robot :angle-vector))
	 (target-coords-transformed nil)
	 (result-coords-list)
	 )
    (dotimes (i (length base-coords-list))
      (if (consp target-coords)
	  (progn 
	    (dotimes (j (length target-coords))
	      
	      (setq target-coords-transformed (append target-coords-transformed (send (send (send (elt target-coords j) :copy-worldcoords) :transformation (elt base-coords-list i) :local) :transformation (make-coords) :local)))
	      ))
	(setq target-coords-transformed (send (send (send target-coords :copy-worldcoords) :transformation (elt base-coords-list i) :local) :transformation (make-coords) :local))
	)
      (setf (elt args (+ ik-command-index 1)) target-coords-transformed)
      (if (forward-message-to robot args)
	  (setq result-coords-list (append result-coords-list (list (elt base-coords-list i))))
	)
      (send robot :angle-vector initial-angle-vector)	   
      )
    result-coords-list
    )
  )

(defun make-coords-list-for-inverse-reachablity ; by args sent to go-pos
  (&key (max-x 0.5) (min-x -0.5) (dx 0.2) (max-y 0.5) (min-y -0.5) (dy 0.2) (max-d 30) (min-d -30) (dd 15))
  (let ((output-coords-list nil))
    (dotimes (ix (+ 1 (round (/ (- max-x min-x) dx))))
      (let ((x-index (+ min-x (* ix dx))))
	(dotimes (iy (+ 1 (round (/ (- max-y min-y) dy))))
	  (let ((y-index (+ min-y (* iy dy))))
	    (dotimes (id (+ 1 (round (/ (- max-d min-d) dd))))
	      (let ((d-index (+ min-d (* id dd))))
		(setq output-coords-list (append output-coords-list (list 
								     (make-coords :pos (float-vector (* x-index 1000) (* y-index 1000) 0)
										  :rpy (float-vector (deg2rad d-index) 0 0)))))
		))
	    ))
	))
    output-coords-list
    ))

