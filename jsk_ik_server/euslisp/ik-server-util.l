(ros::load-ros-manifest "jsk_ik_server")

(defmethod cascaded-link
  (:cog-convergence-check
   (centroid-thre target-centroid-pos &optional centroid-offset-func)
   (let ((cdiff
	  (send self :difference-cog-position target-centroid-pos centroid-offset-func)))
     (cond
      ((numberp centroid-thre) (> centroid-thre (norm cdiff)))
      ((functionp centroid-thre) (funcall centroid-thre cdiff))
      ((vectorp centroid-thre)
       (v< (map float-vector #'abs cdiff) centroid-thre))
      ((and (class centroid-thre)
	    (find-method centroid-thre :cog-convergence-check))
       (send centroid-thre :cog-convergence-check cdiff)))))
  )

(defmethod cascaded-link
  (:fullbody-inverse-kinematics-with-error
   (target-coords
    &rest
    args
    &key
    (revert-if-fail t)
    link-list move-target
    rotation-axis translation-axis thre rthre
    target-centroid-pos centroid-thre centroid-offset-func
    &allow-other-keys)
   (let* ((initial-av (copy-object (send self :angle-vector)))
	  (inital-coords (copy-object (send self :worldcoords)))
	  (target-coords
	   (mapcar #'(lambda (x)
		       (if (functionp x) (funcall x) x))
		   target-coords))
	  dif-pos dif-rot success cdiff
	  )
     (if (atom (car link-list)) (setq link-list (list link-list)))
     (if (atom move-target) (setq move-target (list move-target)))
     (if (atom target-coords) (setq target-coords (list target-coords)))
     (if (atom rotation-axis) (setq rotation-axis (list rotation-axis)))
     (if (atom translation-axis) (setq translation-axis (list translation-axis)))
     (if (atom thre) (setq thre (list thre)))
     (if (atom rthre) (setq rthre (list rthre)))
     (send* self :fullbody-inverse-kinematics target-coords
	    :revert-if-fail nil args)
     (setq dif-pos
	   (mapcar #'(lambda (mv tc trans-axis)
		       (send mv :difference-position tc
			     :translation-axis trans-axis))
		   move-target target-coords translation-axis))
     (setq dif-rot
	   (mapcar #'(lambda (mv tc rot-axis)
		       (send mv :difference-rotation tc
			     :rotation-axis rot-axis))
		   move-target target-coords rotation-axis))
     (cond
      (target-centroid-pos
       (setq cdiff
	     (send self :difference-cog-position
		   target-centroid-pos centroid-offset-func)))
      (t (setq cdiff 0)))
     (setq success
	   (send self :ik-convergence-check
		 t dif-pos dif-rot
		 rotation-axis translation-axis thre rthre
		 centroid-thre target-centroid-pos centroid-offset-func))
     (cond
      ((and (not success) revert-if-fail)
       (send self :angle-vector initial-av)
       (send self :newcoords inital-coords)
       (list
	(cons :dif-pos dif-pos)
	(cons :dif-rot dif-rot)
	(cons :thre thre)
	(cons :rthre rthre)
	(cons :cog-dif cdiff)
	(cons :cthre centroid-thre))
       )
      (t (send self :angle-vector))))))

(defmethod ik-server
  (:make-foot-convex
  (&key
   (foot-key '(:rleg :lleg))
   (force? nil))
  ;;
  (when (position nil (mapcar #'(lambda (l) (send robot l)) foot-key))
    (return-from :make-foot-convex nil))
  ;;
  (format t "[foot-convex] ")
  (cond
   ((or
     force?
     (null foot-convex)
     ;; (and *fix-limb-coords*
     ;; 	  (flatten
     ;; 	   (mapcar
     ;; 	    #'(lambda (k)
     ;; 		(or
     ;; 		 (not (assoc k *fix-limb-coords*))
     ;; 		 (> (norm
     ;; 		     (send
     ;; 		      (send robot k :end-coords :worldcoords)
     ;; 		      :difference-position
     ;; 		      (cdr (assoc k *fix-limb-coords*))))
     ;; 		    50)
     ;; 		 (> (norm
     ;; 		     (send
     ;; 		      (send robot k :end-coords :worldcoords)
     ;; 		      :difference-rotation
     ;; 		      (cdr (assoc k *fix-limb-coords*))))
     ;; 		    (deg2rad 10))))
     ;; 	    foot-key))))
     )
    (format t "update~%")
    (setq foot-convex
	  (make-prism
	   (quickhull
	    (mapcar
	     #'(lambda (v)
		 (concatenate float-vector
			      (subseq v 0 2)
			      (list
			       (+
				-10
				(aref
				 (send robot :rleg :end-coords :worldpos)
				 2)))))
	     (flatten
	      (mapcar
	       #'(lambda (k)
		   (send-all
		    (send
		     (send robot k :end-coords :parent)
		     :faces)
		    :vertices))
	       foot-key))))
	   5))
    (send foot-convex :set-color #F(0 0 1))
    ;; (setq *fix-limb-coords*
    ;; 	  (append
    ;; 	   (remove-if
    ;; 	    #'(lambda (kf) (find (car kf) '(:rleg :lleg)))
    ;; 	    *fix-limb-coords*)
    ;; 	   (mapcar
    ;; 	    #'(lambda (k)
    ;; 		(cons
    ;; 		 k
    ;; 		 (send robot k :end-coords :copy-worldcoords)))
    ;; 	    foot-key)))
    (gl::transparent foot-convex 0.3)
    )
   (t (format t "reject~%"))))
  (:make-centroid-object
   ()
   (setq centroid-object (make-cylinder 20 400))
   (send centroid-object :set-color #F(1 1 0)))
  ;;
  (:convex-pos-list
   nil
   (if (find-method foot-convex :faces)
       (union
	(send
	 (find-if
	  #'(lambda (f)
	      (find :bottom (send f :id)))
	  (send foot-convex :faces))
	 :vertices)
	nil)))
  ;;
  (:fix-centroid-object
   (&optional
    (centroid (send robot :centroid))
    (leg-h (scale 0.5
		  (v+
		   (if (send robot :rleg) (send robot :rleg :end-coords :worldpos) (float-vector 0 0 0))
		   (if (send robot :lleg) (send robot :lleg :end-coords :worldpos) (float-vector 0 0 0))
		   ))))
   (if (and centroid-object (find-method centroid-object :newcoords))
       (send centroid-object
	     :newcoords
	     (make-coords
	      :pos (v+
		    #F(0 0 -200)
		    (concatenate float-vector
				 (subseq centroid 0 2)
				 (list (+ 10 (aref leg-h 2))))))))
   )
  ;;
  (:cog-convergence-check
   (diff
    &optional
    (convex (send self :convex-pos-list))
    (center
     (if (find-method robot :rleg)
	 (scale
	  0.5
	  (apply
	   #'v+
	   (mapcar
	    #'(lambda (k)
		(send robot k :end-coords :worldpos))
	    '(:rleg :lleg))))
       (send robot :worldpos))))
   ;;(print convex)
   (let* ((diff (concatenate float-vector diff (list 0)))
	  (target (v+ diff center))
	  (t-vec (mapcar
		  #'(lambda (v) (v- target v))
		  convex))
	  (rotate
	   (mapcar
	    #'(lambda (v1 v2) (v* v1 v2))
	    t-vec (append (cdr t-vec) (list (car t-vec)))))
	  )
     (eq
      1
      (length
       (union
	nil
	(mapcar
	 #'(lambda (v) (minusp (aref v 2)))
	 rotate))))))
  )

(defmethod ik-server
  (:fullbody-ik-main
   (&rest
    args
    &key
    (target-limb '(:rarm :larm :rleg :lleg))
    (move-target
     (mapcar
      #'(lambda (k)
	  (send robot k :end-coords))
      target-limb))
    (link-list
     (mapcar
      #'(lambda (mt)
	  (send robot :link-list
		(send mt :parent)))
      move-target))
    (target-coords (send-all move-target :copy-worldcoords))
    (target-centroid-pos
     (scale
      0.5
      (apply #'v+
	     (mapcar
	      #'(lambda (k)
		  (if (send robot k)
		      (send robot k :end-coords
			    :worldpos)
		    (send (make-coords) :worldpos)))
	      '(:rleg :lleg)))))
    (min #F(-1000 -1000 -1000 -200 -200 -200))
    (max #F(1000 1000 1000 200 200 200))
    (min-vec min) ;; for mm
    (max-vec max)
    (thre (mapcar #'(lambda (a) 4) move-target))
    (rthre (mapcar #'(lambda (a) (deg2rad 3)) move-target))
    (translation-axis (mapcar #'(lambda (a) t) move-target))
    (rotation-axis (mapcar #'(lambda (a) t) move-target))
    (centroid-thre self)
    ;;
    (initial-av (copy-seq (send robot :angle-vector)))
    (initial-coords (copy-object (send robot :worldcoords)))
    (debug-view :no-message)
    (collision-pair)
    (collision-avoidance-link-pair collision-pair)
    &allow-other-keys
    )
   (let (ret)
     (setq target-coords ;; for mm
	   (mapcar
	    #'(lambda (c m k)
		(or c
		    ;;(cdr (assoc k *fix-limb-coords*))
		    (send m :copy-worldcoords)))
	    target-coords move-target target-limb))
     ;;
     (setq
      ret
      (send* robot
	     :fullbody-inverse-kinematics-with-error
	     (append
	      (list
	       target-coords
	       :move-target move-target
	       :link-list link-list
	       :target-centroid-pos target-centroid-pos
	       :centroid-offset-func
	       #'(lambda nil
		   (x::window-main-one)
		   (send robot :centroid))
	       :centroid-thre centroid-thre
	       ;;
	       :collision-avoidance-link-pair
	       collision-avoidance-link-pair
	       :min min-vec
	       :max max-vec
	       :thre thre
	       :rthre rthre
	       :translation-axis translation-axis
	       :rotation-axis rotation-axis
	       ;;
	       :dump-command nil
	       :debug-view debug-view
	       ;;:debug-view :no-message
	       :warnp nil)
	      args)))
     ;;
     (when collision-avoidance-link-pair
       (setq collision-avoidance-link-pair
	     (send robot :self-collision-check :pairs
		   (mapcar
		    #'(lambda (ll) (cons (car ll) (cadr ll)))
		    collision-avoidance-link-pair))))
     (when collision-avoidance-link-pair
       (format t "[collision check] ~A~%"
	       collision-avoidance-link-pair))
     ;;
     (cond
      ((and (not collision-avoidance-link-pair) (vectorp ret))
       (format t "[ik-main] success !!~%")
       (send self :fix-centroid-object)
       (if viewer (send viewer :draw-objects))
       (if (or
	    (null success-state-collection)
	    (> (norm (v- ret initial-av)) 1))
	   (setq success-state-collection
		 (subseq
		  (cons
		   (list (cons :angle-vector (copy-seq ret))
			 (cons :worldcoords
			       (copy-object
				(send robot :worldcoords))))
		   success-state-collection)
		  0 500)))
       )
      (t
       (format t "[ik-main] fail!!~%")
       (send robot :angle-vector initial-av)
       (send robot :newcoords initial-coords)
       (if viewer (send viewer :draw-objects))
       (print ret)
       (setq ret nil)))
     ret
     ))
  )
