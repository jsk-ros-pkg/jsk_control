
(defmethod ik-server
  (:old-ik-service-cb
   (req)
   (let* ((res (send req :response))
	  (arm (if (or (substringp "left_arm" (send req :ik_request :group_name))
		       (substringp "larm" (send req :ik_request :group_name))
		       (substringp "lhand" (send req :ik_request :group_name)))
		   :larm
		 :rarm))
	  (torsop (substringp "torso" (send req :ik_request :group_name)))
	  (torso2p (substringp "torso2" (send req :ik_request :group_name)))
	  (torso3p (substringp "torso3" (send req :ik_request :group_name)))
	  (fullbodyp (substringp "fullbody" (send req :ik_request :group_name)))
	  (opt-str (send (send (send req :ik_request) :constraints) :name))
	  (opt
	   (cond
	    ((and (stringp opt-str)
		  (plusp (length opt-str))
		  (listp (setq opt-str (read-from-string opt-str))))
	     opt-str)
	    (t (list :update-convex t)))))
     ;; taking joint state into account
     (cond
      ((and (send req :ik_request :robot_state)
	    (send req :ik_request :robot_state :joint_state)
	    (send req :ik_request :robot_state :joint_state :name)
	    (send req :ik_request :robot_state :joint_state :position)
	    (= (length (send req :ik_request :robot_state :joint_state :name))
	       (length (send req :ik_request :robot_state :joint_state :position))))
       (dotimes (i (length (send req :ik_request :robot_state :joint_state :name)))
	 (let ((joint-name (elt (send req :ik_request :robot_state :joint_state :name) i)))
	   (let ((j (find joint-name (send robot :joint-list)
			  :test #'string= :key #'(lambda (x) (send x :name)))))
	     (if j
		 (send j :joint-angle
		       (rad2deg
			(elt
			 (send req :ik_request :robot_state :joint_state :position)
			 i)))
	       (warn "we cannot find ~A~%" joint-name)
	       )))))
      (t (send robot :reset-manip-pose)))
     (format t "[ik-server] options: ~A~%" opt)
     (format t "            target: limb=~A, torso=~A:~A:~A, fullbody=~A~%"
	     arm torsop torso2p torso3p fullbodyp)
     (send robot :fix-leg-to-coords (make-coords) :both)
     (cond
      ((cadr (member :update-convex opt))
       (send self :make-foot-convex :force? t)
       (cond
	(viewer
	 (send viewer :objects
	       (list robot centroid-object foot-convex arrow))
	 (send viewer :draw-objects)))))
     (let ((goal (send req :ik_request :pose_stamped))
	   ;; check that goal is malformed or not
	   (q (float-vector (send req :ik_request :pose_stamped :pose :orientation :w)
			    (send req :ik_request :pose_stamped :pose :orientation :x)
			    (send req :ik_request :pose_stamped :pose :orientation :y)
			    (send req :ik_request :pose_stamped :pose :orientation :z))))
       (send res :error_code :val moveit_msgs::MoveItErrorCodes::*FAILURE*)
       (cond
	((not (eps= (norm q) 1.0))
	 (ros::ros-error "Invalid quotanion detected ~A~%" q)
	 res)
	((not (equal root-link-frame-id (send goal :header :frame_id)))
	 (ros::ros-error "root-link(~A) and frame_id(~A) must be same" root-link-frame-id (send goal :header :frame_id))
	 res)
	((let* ((goal-in-pelvis (send *tf* :transform-pose root-link-frame-id goal))
		(goal-in-world
		 (send (send robot :copy-worldcoords)
		       :transform goal-in-pelvis))
		(draw-arrow
		 (send arrow :newcoords goal-in-world))
		(ik-ret (cond
			 (fullbodyp
			  (apply
			   #'send self :fullbody-ik-with-collision
			   (append
			    (list arm goal-in-world)
			    opt))
			  )
			 (torso2p
			  (apply
			   #'send self :torso-ik-with-collision
			   (append
			    (list arm goal-in-world)
			    (list :root-link-virtual-joint-weight
				  (float-vector 0.02 0.02 0.1 0.0 0.0 0.04))
			    opt)))
			 (torso3p
			  (apply
			   #'send self :torso-ik-with-collision
			   (append
			    (list arm goal-in-world)
			    (list :waist-gain '(0 0 1))
			    opt)))
			 (torsop
			  (apply
			   #'send self :torso-ik-with-collision
			   (append
			    (list arm goal-in-world)
			    opt)))
			 (t
			  (apply
			   #'send self :normal-ik-with-collision
			   (append
			    (list arm goal-in-world)
			    opt))))))
	   ;; (print (list 'original goal))
	   ;; (print (list 'original
	   ;;              (send goal :header :frame_id)
	   ;;              (send goal :pose :position :x)
	   ;;              (send goal :pose :position :y)
	   ;;              (send goal :pose :position :z)
	   ;;              (send goal :pose :orientation :x)
	   ;;              (send goal :pose :orientation :y)
	   ;;              (send goal :pose :orientation :z)
	   ;;              (send goal :pose :orientation :w)))
	   ;; (print (list 'goal-in-pelvis goal-in-pelvis))
	   ;; (print (list 'goal-in-world goal-in-world))
	   (vectorp ik-ret))
	 ;;(format t "[ik-server] SUCCESS!!~%")
	 (send res :error_code :val moveit_msgs::MoveItErrorCodes::*SUCCESS*)
	 (if viewer (send viewer :draw-objects))
	 (send res :solution :joint_state :name
	       (send-all (send robot :joint-list) :name))
	 (send res :solution :joint_state :position
	       (map float-vector #'deg2rad (send robot :angle-vector)))
	 res)
	(t res))))
   )
  )

(defmethod ik-server
  (:old-fullbody-ik-main
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
	    (copy-object (send robot :angle-vector)))
      (cons :worldcoords
	    (copy-object (send robot :worldcoords)))))
    (target-limb '(:rarm :larm :rleg :lleg))
    (target-coords (make-list (length target-limb)))
    (thre (make-list 4 :initial-element 5))
    (rthre (make-list 4 :initial-element (deg2rad 3)))
    (left-support? t)
    (move-target
     (mapcar
      #'(lambda (k)
	  (send robot k :end-coords))
      target-limb))
    (link-list
     (mapcar
      #'(lambda (k)
	  (send robot :link-list
		(send robot k :end-coords :parent)
		(if (find k nil);'(:rarm :larm))
		    (send robot k :root-link))))
      target-limb))
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
    (additional-weight-list
     (mapcar
      #'(lambda (k gain)
	  (list (send robot :torso k :child-link)
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
    (initial-av (copy-seq (send robot :angle-vector)))
    (initial-coords (copy-object (send robot :worldcoords)))
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
	    (list (if (find-method robot :utorso_lk) (list l (send robot :utorso_lk)))
		  (if (find-method robot :r_uleg_lk) (list l (send robot :r_uleg_lk)))
		  (if (find-method robot :l_uleg_lk) (list l (send robot :l_uleg_lk))))))
       (remove-if
	#'(lambda (l)
	    (or (eq (send l :parent)
		    (if (find-method robot :utorso_lk) (send robot :utorso_lk)))
		(eq (send (send l :parent) :parent)
		    (if (find-method robot :utorso_lk) (send robot :utorso_lk)))))
	(apply
	 #'append
	 (mapcar
	  #'(lambda (k) (send robot k :links))
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
	       :rotation-axis rotation-axis
	       :translation-axis translation-axis
	       :target-centroid-pos target-centroid-pos
	       :centroid-offset-func
	       #'(lambda nil
		   (x::window-main-one)
		   (send robot :centroid))
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
	     (send robot :self-collision-check :pairs
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
			  (send robot :angle-vector))) 100)
	     (setq interpolate-time 4000)))
	   (model2real :time interpolate-time :wait t)
	   (unix::usleep (round (* 1000 1000)))
					;(read-line)
	   )))
       (if viewer (send viewer :draw-objects))
       (if (or
	    (null success-state-collection)
	    (> (norm
		(v- ret
		    (cdr (assoc :angle-vector prev-state))))
	       1))
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

(defmethod ik-server
  (:fullbody-ik-with-collision
   (arm goal-in-world &rest args)
   (apply
    #'send self :old-fullbody-ik-main
    (append
     args
     (list
      :target-limb (mapcan #'(lambda (limb) (if (send robot limb) (list limb))) (list arm :lleg :rleg))
      :target-coords (mapcan #'(lambda (limb target) (if (send robot limb) (list target))) (list arm :lleg :rleg) (list goal-in-world nil nil))
      :collision? t
      ;;:centroid-thre 100
      :root-link-virtual-joint-weight
      (float-vector 0.02 0.02 0.1 0.01 0.04 0.04)
      :thre (mapcan #'(lambda (limb) (if (send robot limb) (list 5))) (list arm :lleg :rleg))
      :cog-gain 0.7
      ;;:target-centroid-pos nil
      ;;:stop 100
      :rotation-axis (mapcan #'(lambda (limb) (if (send robot limb) (list t))) (list arm :lleg :rleg))))))
  ;;
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
    #'send self :old-fullbody-ik-main
    (append
     args
     (list
      :target-limb (mapcan #'(lambda (limb) (if (send robot limb) (list limb))) (list arm :lleg :rleg))
      :target-coords (mapcan #'(lambda (limb target) (if (send robot limb) (list target))) (list arm :lleg :rleg) (list goal-in-world nil nil))
      :additional-weight-list
      (mapcar
       #'(lambda (k gain)
	   (list (send robot :torso k :child-link)
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
      :thre (mapcan #'(lambda (limb) (if (send robot limb) (list 5))) (list arm :lleg :rleg))
      ;;:stop 100
      :rotation-axis (mapcan #'(lambda (limb) (if (send robot limb) (list t))) (list arm :lleg :rleg))))))
  ;;
  (:normal-ik-with-collision
   (arm goal-in-world &rest args)
   (let* ((w
	   (send-all (send robot :joint-list)
		     :max-joint-velocity))
	  (vel-limit
	   (send-all (send robot :joint-list)
		     :max-joint-velocity 0.2))
	  (ret
	   (apply
	    #'send self :old-fullbody-ik-main
	    (append
	     args
	     (list
	      :target-limb (mapcan #'(lambda (limb) (if (send robot limb) (list limb))) (list arm :lleg :rleg))
	      :target-coords (mapcan #'(lambda (limb target) (if (send robot limb) (list target))) (list arm :lleg :rleg) (list goal-in-world nil nil))
	      ;;:additional-weight-list
	      ;;(if (find-method robot :torso :waist-r)
	      ;;(append
	      ;;(mapcar
	      ;;#'(lambda (k gain)
	      ;;(list (send robot :torso k :child-link)
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
	      :thre (mapcan #'(lambda (limb) (if (send robot limb) (list 5))) (list arm :lleg :rleg))
	      :rthre (mapcan #'(lambda (limb) (if (send robot limb) (list (deg2rad 1)))) (list arm :lleg :rleg))
	      :rotation-axis (mapcan #'(lambda (limb) (if (send robot limb) (list t))) (list arm :lleg :rleg)))))))
     (mapcar
      #'(lambda (j g) (send j :max-joint-velocity g))
      (send robot :joint-list) w)
     ret))
  )

