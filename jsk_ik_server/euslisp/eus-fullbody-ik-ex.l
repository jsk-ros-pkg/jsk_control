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

(defmethod robot-model
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

  (when (position nil (mapcar #'(lambda (l) (send *robot* l)) foot-key))
    (return-from :make-foot-convex nil))

  (format t "[foot-convex] ")
  (cond
   ((or
     force?
     (null *foot-convex*)
     (and *fix-limb-coords*
	  (flatten
	   (mapcar
	    #'(lambda (k)
		(or
		 (not (assoc k *fix-limb-coords*))
		 (> (norm
		     (send
		      (send *robot* k :end-coords :worldcoords)
		      :difference-position
		      (cdr (assoc k *fix-limb-coords*))))
		    50)
		 (> (norm
		     (send
		      (send *robot* k :end-coords :worldcoords)
		      :difference-rotation
		      (cdr (assoc k *fix-limb-coords*))))
		    (deg2rad 10))))
	    foot-key))))
    (format t "update~%")
    (setq *foot-convex*
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
				 (send *robot* :rleg :end-coords :worldpos)
				 2)))))
	     (flatten
	      (mapcar
	       #'(lambda (k)
		   (send-all
		    (send
		     (send *robot* k :end-coords :parent)
		     :faces)
		    :vertices))
	       foot-key))))
	   5))
    (send *foot-convex* :set-color #F(0 0 1))
    (setq *fix-limb-coords*
	  (append
	   (remove-if
	    #'(lambda (kf) (find (car kf) '(:rleg :lleg)))
	    *fix-limb-coords*)
	   (mapcar
	    #'(lambda (k)
		(cons
		 k
		 (send *robot* k :end-coords :copy-worldcoords)))
	    foot-key)))
    (gl::transparent *foot-convex* 0.3)
    )
   (t (format t "reject~%"))))

  (:make-centroid-object
  ()
  (setq *centroid-object* (make-cylinder 20 400))
  (send *centroid-object* :set-color #F(1 1 0)))

  (:convex-pos-list
  nil
  (if (find-method *foot-convex* :faces)
      (union
       (send
	(find-if
	 #'(lambda (f)
	     (find :bottom (send f :id)))
	 (send *foot-convex* :faces))
	:vertices)
       nil)))

  (:fix-centroid-object
  (&optional
   (centroid (send *robot* :centroid))
   (leg-h (scale 0.5
		 (v+
		  (if (send *robot* :rleg) (send *robot* :rleg :end-coords :worldpos) (float-vector 0 0 0))
		  (if (send *robot* :lleg) (send *robot* :lleg :end-coords :worldpos) (float-vector 0 0 0))
		 ))))
  (if (and *centroid-object* (find-method *centroid-object* :newcoords))
      (send *centroid-object*
	    :newcoords
	    (make-coords
	     :pos (v+
		   #F(0 0 -200)
		   (concatenate float-vector
				(subseq centroid 0 2)
				(list (+ 10 (aref leg-h 2))))))))
  )

  (:cog-convergence-check
   (diff
    &optional
    (convex (send self :convex-pos-list))
    (center
     (if (find-method *robot* :rleg)
	 (scale
	  0.5
	  (apply
	   #'v+
	   (mapcar
	    #'(lambda (k)
		(send *robot* k :end-coords :worldpos))
	    '(:rleg :lleg))))
       (send *robot* :worldpos))))
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
   (real? nil)
   (real-feedback? nil)
   (rotate? t)
   (translate? t)
   ;; meta
   (prev-state
    (list
     (cons :angle-vector
	   (copy-object (send *robot* :angle-vector)))
     (cons :worldcoords
	   (copy-object (send *robot* :worldcoords)))))
   (target-limb '(:rarm :larm :rleg :lleg))
   (target-coords (make-list (length target-limb)))
   (thre (make-list 4 :initial-element 5))
   (rthre (make-list 4 :initial-element (deg2rad 3)))
   (left-support? t)
   (move-target
    (mapcar
     #'(lambda (k)
	 (send *robot* k :end-coords))
     target-limb))
   (link-list
    (mapcar
     #'(lambda (k)
	 (send *robot* :link-list
	       (send *robot* k :end-coords :parent)
	       (if (find k nil);'(:rarm :larm))
		   (send *robot* k :root-link))))
     target-limb))
   (target-centroid-pos
    (scale
     0.5
     (apply #'v+
	    (mapcar
	     #'(lambda (k)
		 (if (send *robot* k)
		     (send *robot* k :end-coords
			   :worldpos)
		   (send (make-coords) :worldpos)))
	     '(:rleg :lleg)))))
   (additional-weight-list
    (mapcar
     #'(lambda (k gain)
	 (list (send *robot* :torso k :child-link)
	       gain))
     (list :waist-r :waist-p :waist-y)
     (list 1e-3 1e-2 1e-2)))
   (root-link-virtual-joint-weight
    #F(0.1 1.5 1 0.08 0.08 0.08))
    ;;(float-vector 0.01 0.01 1 0.01 0.01 0.01))
   (min-vec #F(-1000 -1000 -1000 -200 -200 -200))
   (max-vec #F(1000 1000 1000 200 200 200))
   (centroid-thre self)
   (rotation-axis
    (mapcar
     #'(lambda (k)
	 (case k
	   (:rarm rotate?)
	   (:larm left-support?)
	   (t t)))
     target-limb))
   (translation-axis
    (mapcar
     #'(lambda (k)
	 (case k
	   (:rarm translate?)
	   (:larm left-support?)
	   (t t)))
     target-limb))
   ;;
   (initial-av (copy-seq (send *robot* :angle-vector)))
   (initial-coords (copy-object (send *robot* :worldcoords)))
   (debug-view :no-message)
   (stop 50)
   (collision? t)
   (collision-pair
    (apply
     #'append
     (mapcar
      #'(lambda (l)
	  (remove-if 
	   #'null
	  (list (if (find-method *robot* :utorso_lk) (list l (send *robot* :utorso_lk)))
		(if (find-method *robot* :r_uleg_lk) (list l (send *robot* :r_uleg_lk)))
		(if (find-method *robot* :l_uleg_lk) (list l (send *robot* :l_uleg_lk))))))
      (remove-if
       #'(lambda (l)
	   (or (eq (send l :parent)
		   (if (find-method *robot* :utorso_lk) (send *robot* :utorso_lk)))
	       (eq (send (send l :parent) :parent)
		   (if (find-method *robot* :utorso_lk) (send *robot* :utorso_lk)))))
       (apply
	#'append
	(mapcar
	 #'(lambda (k) (send *robot* k :links))
	 (remove-if
	  #'(lambda (k) (find k '(:rleg :lleg)))
	  target-limb)))))))
   (cog-gain 0.1)
   &allow-other-keys
   )
  (let (ret)
    (cond
     ((and real-feedback?
	   (boundp '*ri*) *ri*)
      (leg-coords-fix-from-real
       :init-grasp? nil
       :solve-ik? nil)))
    (setq target-coords
	  (mapcar
	   #'(lambda (c m k)
	       (or c
		   (cdr (assoc k *fix-limb-coords*))
		   (send m :copy-worldcoords)))
	   target-coords move-target target-limb))
    ;;
    (setq
     ret
     (send* *robot*
	    :fullbody-inverse-kinematics-with-error
	    (append
	     (list
	      target-coords
	      :move-target move-target
	      :link-list link-list
	      :rotation-axis rotation-axis
	      :translation-axis translation-axis
	      :target-centroid-pos target-centroid-pos
	      :centroid-offset-func
	      #'(lambda nil
		  (x::window-main-one)
		  (send *robot* :centroid))
	      :cog-gain cog-gain
	      :additional-weight-list additional-weight-list
	      :stop stop
	      :thre thre
	      :rthre rthre
	      :centroid-thre centroid-thre
	      ;;
	      :collision-avoidance-link-pair collision-pair
	      :root-link-virtual-joint-weight
	      root-link-virtual-joint-weight
	      :min min-vec
	      :max max-vec
	      :avoid-collision-distance 10
	      :avoid-collision-null-gain 0.3
	      :avoid-collision-joint-gain 0.3
	      ;;
	      :dump-command nil
	      :debug-view debug-view
	      ;;:debug-view :no-message
	      :warnp nil)
	     args)))
    ;;
    (when collision?
      (setq collision?
	    (send *robot* :self-collision-check :pairs
		  (mapcar
		   #'(lambda (ll) (cons (car ll) (cadr ll)))
		   collision-pair))))
    (when collision? (format t "[collision check] ~A~%" collision?))
    ;;
    (cond
     ((and (not collision?) (vectorp ret))
      (format t "[ik-main] success !!~%")
      (send self :fix-centroid-object)
      (if viewer (send viewer :draw-objects))
      (cond
       ((and real? (boundp '*ri*) *ri*)
	(let ((interpolate-time 1000))
	  (cond
	   ((> (norm (v- (send *ri* :state :potentio-vector)
			 (send *robot* :angle-vector))) 100)
	    (setq interpolate-time 4000)))
	  (model2real :time interpolate-time :wait t)
	  (unix::usleep (round (* 1000 1000)))
					;(read-line)
	  )))
      (if viewer (send viewer :draw-objects))
      (if (or
	   (null *success-state-collection*)
	   (> (norm
	       (v- ret
		   (cdr (assoc :angle-vector prev-state))))
	      1))
	  (setq *success-state-collection*
		(subseq
		 (cons
		  (list (cons :angle-vector (copy-seq ret))
			(cons :worldcoords
			      (copy-object
			       (send *robot* :worldcoords))))
		  *success-state-collection*)
		 0 500)))
      )
     (t
      (format t "[ik-main] fail!!~%")
      (send *robot* :angle-vector initial-av)
      (send *robot* :newcoords initial-coords)
      (if viewer (send viewer :draw-objects))
      (print ret)
      (setq ret nil)))
    ret
    ))

  (:init-fullbody-ik-main
  ()
  (send (send *robot* :utorso_lk) :make-pqpmodel :fat 30)
  (setq *success-state-collection* nil)
  ;; (send self :make-foot-convex)
  (send self :make-centroid-object))
  )

(defmethod ik-server
  (:fullbody-ik-with-collision
   (arm goal-in-world &rest args)
   (apply
    #'send self :fullbody-ik-main
    (append
     args
     (list
      :target-limb (mapcan #'(lambda (limb) (if (send *robot* limb) (list limb))) (list arm :lleg :rleg))
      :target-coords (mapcan #'(lambda (limb target) (if (send *robot* limb) (list target))) (list arm :lleg :rleg) (list goal-in-world nil nil))
      :collision? t
      ;;:centroid-thre 100
      :root-link-virtual-joint-weight
      (float-vector 0.02 0.02 0.1 0.01 0.04 0.04)
      :thre (mapcan #'(lambda (limb) (if (send *robot* limb) (list 5))) (list arm :lleg :rleg))
      :cog-gain 0.7
      ;;:target-centroid-pos nil
      ;;:stop 100
      :rotation-axis (mapcan #'(lambda (limb) (if (send *robot* limb) (list t))) (list arm :lleg :rleg))))))

  (:torso-ik-with-collision
   (arm goal-in-world
	&rest args
	&key
	(waist-gain '(0.1 0.1 0.1))
	(root-link-virtual-joint-weight
	 (float-vector 0 0 0 0 0 0))
	(max
	 (map float-vector
	      #'*
	      root-link-virtual-joint-weight
	      #F(1000 1000 1000 200 200 200)))
	(min
	 (scale
	  -1
	  (map float-vector
	       #'*
	       root-link-virtual-joint-weight
	       #F(1000 1000 1000 200 200 200))))
	&allow-other-keys)
   (apply
    #'send self :fullbody-ik-main
    (append
     args
     (list
      :target-limb (mapcan #'(lambda (limb) (if (send *robot* limb) (list limb))) (list arm :lleg :rleg))
      :target-coords (mapcan #'(lambda (limb target) (if (send *robot* limb) (list target))) (list arm :lleg :rleg) (list goal-in-world nil nil))
      :additional-weight-list
      (mapcar
       #'(lambda (k gain)
	   (list (send *robot* :torso k :child-link)
		 gain))
       (list :waist-r :waist-p :waist-y)
       waist-gain)
      ;;(list 1e-3 1e-3 1e-3))
      :root-link-virtual-joint-weight
      root-link-virtual-joint-weight
      :min-vec min
      :max-vec max
      :collision? t
      :dif-pos-ratio 1.0
      :dif-rot-ratio 0.00500
      :manipulability-gain 0 ;0.00500
      ;;:centroid-thre 200
      ;;:target-centroid-pos nil
      :thre (mapcan #'(lambda (limb) (if (send *robot* limb) (list 5))) (list arm :lleg :rleg))
      ;;:stop 100
      :rotation-axis (mapcan #'(lambda (limb) (if (send *robot* limb) (list t))) (list arm :lleg :rleg))))))

  (:normal-ik-with-collision
   (arm goal-in-world &rest args)
   (let* ((w
	   (send-all (send *robot* :joint-list)
		     :max-joint-velocity))
	  (vel-limit
	   (send-all (send *robot* :joint-list)
		     :max-joint-velocity 0.2))
	  (ret
	   (apply
	    #'send self :fullbody-ik-main
	    (append
	     args
	     (list
	      :target-limb (mapcan #'(lambda (limb) (if (send *robot* limb) (list limb))) (list arm :lleg :rleg))
	      :target-coords (mapcan #'(lambda (limb target) (if (send *robot* limb) (list target))) (list arm :lleg :rleg) (list goal-in-world nil nil))
	      ;;:additional-weight-list
	      ;;(if (find-method *robot* :torso :waist-r)
	      ;;(append
	      ;;(mapcar
	      ;;#'(lambda (k gain)
	      ;;(list (send *robot* :torso k :child-link)
	      ;;gain))
	      ;;(list :waist-r :waist-p :waist-y)
	      ;;(make-list 3 :initial-element 0))))
	      ;; :additional-nspace-list
	      ;; (mapcar
	      ;;  #'(lambda (k)
	      ;; 	(list (send *atlas* arm k :child-link)
	      ;; 	      #'(lambda nil (- (random 1.0) 2.0))))
	      ;;  '(:elbow-p))
	      ;;(list 1e-3 1e-3 1e-3))
	      :root-link-virtual-joint-weight
	      (coerce (make-list 6 :initial-element 0) float-vector)
	      :min-vec (float-vector 0 0 0 0 0 0)
	      :max-vec (float-vector 0 0 0 0 0 0)
	      :collision? t
	      :centroid-thre 1e+10000
					;:stop 100
	      ;;:dif-pos-ratio 1.0
	      ;;:dif-rot-ratio 0.00500
	      ;;:manipulability-gain 0.00500
	      :thre (mapcan #'(lambda (limb) (if (send *robot* limb) (list 5))) (list arm :lleg :rleg))
	      :rthre (mapcan #'(lambda (limb) (if (send *robot* limb) (list (deg2rad 1)))) (list arm :lleg :rleg))
	      :rotation-axis (mapcan #'(lambda (limb) (if (send *robot* limb) (list t))) (list arm :lleg :rleg)))))))
     (mapcar
      #'(lambda (j g) (send j :max-joint-velocity g))
      (send *robot* :joint-list) w)
     ret))
  )
