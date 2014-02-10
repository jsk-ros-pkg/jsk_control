
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
	   (let ((j (find joint-name (send *robot* :joint-list)
			  :test #'string= :key #'(lambda (x) (send x :name)))))
	     (if j
		 (send j :joint-angle
		       (rad2deg
			(elt
			 (send req :ik_request :robot_state :joint_state :position)
			 i)))
	       (warn "we cannot find ~A~%" joint-name)
	       )))))
      (t (send *robot* :reset-manip-pose)))
     (format t "[ik-server] options: ~A~%" opt)
     (format t "            target: limb=~A, torso=~A:~A:~A, fullbody=~A~%"
	     arm torsop torso2p torso3p fullbodyp)
     (send *robot* :fix-leg-to-coords (make-coords) :both)
     (cond
      ((cadr (member :update-convex opt))
       (send self :make-foot-convex :force? t)
       (cond
	(viewer
	 (send viewer :objects
	       (list *robot* *centroid-object* *foot-convex* *arrow*))
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
	((not (equal *root-link* (send goal :header :frame_id)))
	 (ros::ros-error "root-link(~A) and frame_id(~A) must be same" *root-link* (send goal :header :frame_id))
	 res)
	((let* ((goal-in-pelvis (send *tf* :transform-pose *root-link* goal))
		(goal-in-world
		 (send (send *robot* :copy-worldcoords)
		       :transform goal-in-pelvis))
		(draw-arrow
		 (send *arrow* :newcoords goal-in-world))
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
	       (send-all (send *robot* :joint-list) :name))
	 (send res :solution :joint_state :position
	       (map float-vector #'deg2rad (send *robot* :angle-vector)))
	 res)
	(t res))))
   )
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
  ;;
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
