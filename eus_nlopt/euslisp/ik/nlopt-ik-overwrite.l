;#-:jsk (jsk)
;#-:rbrain-basic (rbrain)

(require "nlopt-ik.l")

(defmethod cascaded-link
  (:inverse-kinematics
   (target-coords
    &rest
    args
    &key
    (algorithm L_BFGS)
    &allow-other-keys)
   (setq algorithm (mod algorithm OPTIMIZE_EUS_IK))
   (send (instance*
	  nlopt-ik
	  :init
	  (append
	   (list :target-coords target-coords)
	   (list :algorithm algorithm)
	   args))
	 :optimize)
   )
  )

#|

(defmethod robot-model
  (:fullbody-inverse-kinematics
   (target-coords
    &rest
    args
    &key
    (optimize-command :optimize)
    &allow-other-keys)
   (send (eval
	  (append
	   (list 'instance 'nlopt-ik :init
		 :target-coords 'target-coords)
	   (mapcar
	    #'(lambda (v) (if (listp v) (list 'quote v) v))
	    args)))
	 optimize-command)
   )
  )
