;; -*- indent-tabs-mode: nil -*-

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

(defun fullbody-inverse-kinematics-with-standcoords
    (robot
     coords-list
     &rest args
     &key
     (rotation-axis nil)
     (arm :rarm)
     (limb arm)
     (limbs (list limb))
     (move-target (mapcar #'(lambda (l) (send robot l :end-coords)) limbs))
     (link-list (mapcar #'(lambda (m) (send *robot* :link-list (send m :parent))) move-target))
     (root-link-virtual-joint-weight #f(0.1 0.1 0 0 0 0.05)) ;; for moving foot with root
     (stop 30)
     (debug nil)
     (warnp t)
     (move-centroid t)
     &allow-other-keys)
  (unless (consp coords-list) (setq coords-list (list coords-list)))
  (dotimes (i (length coords-list))
    (unless (consp (elt coords-list i)) (setf (elt coords-list i) (list (elt coords-list i)))) ;; have to be list of list
    )
  (unless rotation-axis (setq rotation-axis (make-list (length move-target) :initial-element t)))
  (let (
        standpos
        angle-vector-list
        (coords-list-length (length coords-list))
        (success nil)
        )
    (dotimes (i coords-list-length) (setq angle-vector-list (append angle-vector-list (list (send robot :angle-vector)))))
    (block
     :ik-loop
     (dotimes (i stop)
       (let ((success-loop t))
         (dotimes (j coords-list-length)
           (send robot :angle-vector (elt angle-vector-list j))
           (prog1
               (let ((rest-args-for-ik
                      (append
                       (list
                        :link-list link-list
                        :move-target move-target
                        :target-centroid-pos nil
                        :root-link-virtual-joint-weight root-link-virtual-joint-weight
                        :rotation-axis rotation-axis
                        ) args)
                      ))
                 (send* robot :fullbody-inverse-kinematics
                        (send-all (elt coords-list j) :copy-worldcoords)
                        :stop 2
                        :revert-if-fail nil
                        :debug-view (if debug :no-message nil)
                        rest-args-for-ik
                        )
                 (setq success-loop
                       (and
                        success-loop
                        (send* robot :fullbody-inverse-kinematics
                               (send-all (elt coords-list j) :copy-worldcoords)
                               :stop 1
                               :revert-if-fail t
                               :debug-view nil
                               :warnp (and warnp (equal i (- stop 1)))
                               rest-args-for-ik
                               )
                        ))
                 )
             )
           (setf (elt angle-vector-list j) (send robot :angle-vector))
           )
         (if success-loop
             (progn
               (setq success t)
               (return-from :ik-loop :ik-succeeded)
               )
           )
         )
       )
      )
    (if (not success) 
        (progn
          (warn "fullbody-inverse-kinematics-with-standcoords failed.~%")
          (return-from fullbody-inverse-kinematics-with-standcoords nil))
      )
    (setq standpos (send robot :foot-midcoords))
    (if move-centroid
        (dotimes (i coords-list-length)
          (send robot :angle-vector (elt angle-vector-list i))
          (send robot :fix-leg-to-coords standpos)
          (unless (send robot :move-centroid-on-foot :both (append (list :rarm :larm :rleg :lleg))) ;; do not move
            (warn "fullbody-inverse-kinematics-with-standcoords failed because of move-centroid.~%")
            (return-from fullbody-inverse-kinematics-with-standcoords nil))
          (setf (elt angle-vector-list i) (send robot :angle-vector))
          )
      )
    (list standpos angle-vector-list)
    )
  )
