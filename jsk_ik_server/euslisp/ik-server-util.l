(ros::load-ros-manifest "jsk_ik_server")

(defmethod cascaded-link
  (:cog-convergence-check
   (centroid-thre target-centroid-pos
		  &key
		  (centroid-offset-func)
		  (translation-axis :z)
		  (update-mass-properties t))
   (let ((cdiff
	  (send self :difference-cog-position target-centroid-pos
		:centroid-offset-func centroid-offset-func
		:translation-axis translation-axis
		:update-mass-properties update-mass-properties)))
     (cond
      ((numberp centroid-thre) (> centroid-thre (norm cdiff)))
      ((functionp centroid-thre) (funcall centroid-thre cdiff))
      ((vectorp centroid-thre) (v< (map float-vector #'abs cdiff) centroid-thre))
      ((vectorp centroid-thre)
       (v< (map float-vector #'abs cdiff) centroid-thre))
      ((and (class centroid-thre)
  	    (find-method centroid-thre :cog-convergence-check))
       (send centroid-thre :cog-convergence-check cdiff)))))
  )
;; (:cog-convergence-check
;;  (centroid-thre target-centroid-pos &optional centroid-offset-func)
;;  (let ((cdiff
;; 	  (send self :difference-cog-position target-centroid-pos centroid-offset-func)))
;;    (cond
;;     ((numberp centroid-thre) (> centroid-thre (norm cdiff)))
;;     ((functionp centroid-thre) (funcall centroid-thre cdiff))
;;     ((vectorp centroid-thre)
;;      (v< (map float-vector #'abs cdiff) centroid-thre))
;;     ((and (class centroid-thre)
;; 	    (find-method centroid-thre :cog-convergence-check))
;;      (send centroid-thre :cog-convergence-check cdiff)))))
;; )

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
     (print 1)
     (send* self :fullbody-inverse-kinematics
	    target-coords
	    (append
	     (list :revert-if-fail nil)
	     args))
     (print 2)
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
		   target-centroid-pos
		   :centroid-offset-func centroid-offset-func)))
      (t (setq cdiff 0)))
     (setq success
	   (send self :ik-convergence-check
		 t dif-pos dif-rot
		 rotation-axis translation-axis thre rthre
		 centroid-thre target-centroid-pos centroid-offset-func :z))
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
   (support-links support-links)
   (support-link-target-transform-coords
    (make-list (length support-links) :initial-element (make-coords)))
   (support-vertices
    (mapcar
     #'(lambda (l c)
	 (let ((l2 (copy-object l)))
	   (send l2 :transform c :local)
	   (send l2 :worldcoords)
	   (flatten (send-all (send l2 :faces) :vertices))))
     support-links support-link-target-transform-coords))
   (support-height
    (mapcar #'(lambda (hl) (apply #'min hl))
	    (mapcar #'(lambda (vl)
			(mapcar #'(lambda (v) (aref v 2)) vl))
		    support-vertices)))
   (force? nil))
  ;;
  ;;(when (position nil (mapcar #'(lambda (l) (send robot l)) foot-key))
  ;;(return-from :make-foot-convex nil))
  (when (null support-links) (return-from :make-foot-convex nil))
  ;;
  (format t "[foot-convex] offset=~A~%" support-link-target-transform-coords)
  (format t "[foot-convex] ")
  (cond
   ((or force? (null foot-convex))
    (format t "update~%")
    (setq foot-convex
	  (make-prism
	   (quickhull
	    (flatten
	     (mapcar
	      #'(lambda (vl h)
		  (mapcar
		   #'(lambda (v)
		       (float-vector
			(aref v 0)
			(aref v 1)
			(- h 10)))
		   vl))
	      support-vertices
	      support-height)))
	   5))
    (send foot-convex :set-color #F(0 0 1))
    (gl::transparent foot-convex 0.3)
    (send foot-convex :worldcoords)
    (setq support-centroid-pos (copy-object (send foot-convex :centroid)))
    )
   (t (format t "reject~%"))))
  (:make-centroid-object
   ()
   (setq centroid-object (make-cylinder 20 400))
   (send centroid-object :set-color #F(1 1 0)))
  ;;
  (:convex-pos-list
   nil
   ;;(send self :make-foot-convex :force? t)
   (if (and foot-convex (find-method foot-convex :faces))
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
    (leg-h
     (cond
      ((null support-links) #F(0 0 0))
      ((> (length support-links) 1)
       (scale (/ 1.0 (length support-links))
	      (reduce #'v+ (send-all support-links :worldpos))))
      (t (send (car support-links) :worldpos)))))
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
    (center support-centroid-pos))
   ;;(print convex)
   (if viewer (x::window-main-one))
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
    (target-centroid-pos support-centroid-pos)
    (min #F(-1000 -1000 -1000 -200 -200 -200))
    (max #F(1000 1000 1000 200 200 200))
    (min-vec min) ;; for mm
    (max-vec max)
    (stop 15)
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
    (avoid-collision-distance 10)
    (avoid-collision-null-gain 0.3)
    (avoid-collision-joint-gain 0.3)
    &allow-other-keys
    )
   (format t "[target-centroid-pos] check ~A~%" target-centroid-pos)
   (if (not (vectorp target-centroid-pos))
       (format t "CAUSION!!! target-centroid-pos is not a vector~%"))
   (let (ret)
     (setq target-coords ;; for mm
	   (mapcar
	    #'(lambda (c m) ;; k)
		(or c
		    ;;(cdr (assoc k *fix-limb-coords*))
		    (send m :copy-worldcoords)))
	    target-coords move-target));; target-limb))
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
	       :centroid-thre centroid-thre
	       ;;
	       :collision-avoidance-link-pair
	       collision-avoidance-link-pair
	       :avoid-collision-distance avoid-collision-distance
	       :avoid-collision-null-gain avoid-collision-null-gain
	       :avoid-collision-joint-gain avoid-collision-joint-gain
	       :min min-vec
	       :max max-vec
	       :stop stop
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
