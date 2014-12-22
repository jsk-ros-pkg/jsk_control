(defun inverse-reachablity-with-given-coords
  (robot base-coords-list
	 &rest args ;;same with the command sent for inverse-kinematics
	 )
  (let* ((ik-command-index (position :inverse-kinematics args))
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