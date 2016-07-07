(ros::load-ros-manifest "jsk_footstep_msgs")

(defclass footstep-corrector
  :super propertied-object
  :slots (footsteps execute-flag listener
                    init-transform
                    map-frame odom-frame)
  )

(defmethod footstep-corrector
  (:init
   (&rest args &key ((:map-frame map) "map") ((:odom-frame odom) "odom_init"))
   (send self :clear-footsteps)
   (setq listener (instance ros::transform-listener :init))
   (setq map-frame map)
   (setq odom-frame odom)
   )
  (:register-footsteps-from-msg
   (msg)
   (when execute-flag
     (ros::ros-warn "footsteps are now executed")
     (return)
     )
   (send self :clear-footsteps)
   (let ((footstep-msg-list (send msg :footsteps))
         footstep-coords new-footsteps)
     (dolist (footstep-msg footstep-msg-list)
       (setq footstep-coords (ros::tf-pose->coords (send footstep-msg :pose)))
       (cond 
        ((equal (send footstep-msg :leg) jsk_footstep_msgs::Footstep::*LLEG*)
         (send footstep-coords :name :lleg))
        ((equal (send footstep-msg :leg) jsk_footstep_msgs::Footstep::*RLEG*)
         (send footstep-coords :name :rleg))
        )
       (push footstep-coords new-footsteps)
       )
     ;; register initial transform
     (send listener :wait-for-transform map-frame odom-frame (send msg :header :stamp) 1.0)
     (setq init-transform (send listener :lookup-transform map-frame odom-frame (send msg :header :stamp)))
     ;; retrun footsteps
     (setq footsteps (reverse new-footsteps))
     (ros::ros-warn "Finished registering footsteps")
     )
   )
  (:correct-footstep
   (raw-footstep &key (dx 0) (dy 0) (dz 0) (droll 0) (dpitch 0) (dyaw 0) (wrt :world))
   (let (new-footstep)
     (setq new-footstep (send raw-footstep :copy-worldcoords))
     (send new-footstep :transform
           (make-coords :pos (float-vector dx dy dz)
                        :rpy (float-vector dyaw dpitch droll))
           wrt)
     (send new-footstep :name (send raw-footstep :name))
     new-footstep
     )
   )
  (:check-footstep-collision
   (current next)
   ;; fix next footstep to avoid collision
   ;; return original next footstep if current and next would not cause collision 
   (let (current-foot next-foot)
     (setq current-foot (make-cube 235 135 10))
     (setq next-foot (make-cube 235 135 10))
     (send current-foot :move-to (send current :coords) :world)
     (send next-foot :move-to (send next :coords) :world)
     (let ((dy 0))
       (while (pqp-collision-check-objects (list current-foot) (list next-foot))
	 (cond
	  ((equal (send next :name) :lleg)
	   (setq dy (+ 1 dy))
	   (send next-foot :translate #f( 0 1 0) :local))
	  ((equal (send next :name) :rleg)
	   (setq dy (- dy 1))
	   (send next-foot :translate #f( 0 -1 0) :local))
	  )
	 )
       (send next :translate (float-vector 0 dy 0 ) :local)
       )
     )
   next
   )
  (:execute-footstep-once
   (&key (use-correction t))
   (let (current next)     
     (cond 
      ((< (length footsteps) 1)
       (ros::ros-warn "footstep list is empty")
       (send self :clear-footsteps)
       )
      ((= (length footsteps) 1)
       (ros::ros-warn "last footstep")
       (setq execute-flag t)
       (setq current (pop footsteps))
       (send *ri* :set-foot-steps-no-wait (list current current)) ;; footstep list should contain first and target foot steps
       (send self :clear-footsteps) ;; executing footstep is finished (footstep is empty)
       ) 
      ((> (length footsteps) 1)
       (ros::ros-warn "remaining footstep: ~A" (length footsteps))
       (setq execute-flag t)
       (setq current (pop footsteps))
       (setq next (pop footsteps))
       (when use-correction
         (let (current-transform diff-transform dpos dypr)
           (setq current-transform (send listener :lookup-transform map-frame odom-frame (ros::time 0)))
           (setq diff-transform (send current-transform :transformation init-transform))
           (setq dpos (send diff-transform :pos))
           (setq dypr (car (send diff-transform :rpy-angle))) ;; rpy-angle is yaw pitch roll arrangement
           (setq next (send self :correct-footstep next :dx (elt dpos 0) :dy (elt dpos 1) :dyaw (elt dypr 0))) ;; only consider 2d plane
           
           (ros::ros-warn "dx: ~A dy: ~A dz: ~A dyaw: ~A dpitch: ~A droll: ~A"
                          (elt dpos 0) (elt dpos 1) (elt dpos 2) (elt dypr 0) (elt dypr 1) (elt dypr 2))
           )
         )
       (setq next (send self :check-footstep-collision current next))
       (send *ri* :set-foot-steps-no-wait (list current next))
       (push next footsteps) ;; back to footstep list to use on next execution       
       )
      )
     current ;; return latest footstep (for debug usage)
     )
   )
  (:clear-footsteps
   ()
   (setq footsteps (list))
   (setq execute-flag nil)
   (setq init-transform nil)
   (ros::ros-warn "footsteps are cleared")
   )
  (:get-remaining-footsteps
   ()
   footsteps
   )
  (:get-execute-flag
   ()
   execute-flag
   )  
  )
