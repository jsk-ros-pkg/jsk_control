#-:jsk (jsk)
#-:rbrain-basic (rbrain)

(require "nlopt.l")
(defvar *robot* (hrp2-simple-detail))

(defvar *move-target* (list (send *robot* :rarm :end-coords)))
(defvar *links*
  (mapcar #'(lambda (c) (send *robot* :link-list (send c :parent))) *move-target*))
(defvar *weights* (make-list (length *move-target*) :initial-element 1))
(defvar *all-links* (reduce #'union *links*))
(defvar *target-coords*
  (list (make-coords :pos #F(700 0 1000) :rot (rotation-matrix (deg2rad -90) :x))))
(defvar *debug-view* nil);:no-message)

(defun add-6dof-joint
  (&key
   (robot *robot*)
   (link-list
    (if (atom (car *links*)) (list *links*) *links*))
   (min (float-vector -1500 -1500  -1500 -200 -200 -200))
   (max (float-vector 1500 1500 1500 200 200 200))
   (joint-args (list :min min :max max))
   (joint-class 6dof-joint)
   (vlink (instance bodyset-link :init (make-cascoords)
		    :bodies (list (make-cube 150 10 400))
		    :name 'virtual-link
		    :weight 0 :centroid (float-vector 0 0 0)
		    :inertia-tensor (make-matrix 3 3)))
   (rlink (car (send robot :links))))
  (send-message rlink bodyset-link :add-joint
		(instance* (or joint-class 6dof-joint) :init
			   :child-link robot
			   :parent-link vlink
			   joint-args))
  (send rlink :add-parent-link vlink)
  (send vlink :add-child-links rlink)
  (list
   (cons :6dof-links
	 (mapcar #'(lambda (l) (cons rlink l)) link-list))
   (cons :del-6dof-links
	 (eval
	  (list 'function
		(list 'lambda nil
		      (list 'send rlink :del-joint)
		      (list 'send rlink :del-parent-link)
		      (list 'send vlink :del-child-link rlink)))))))

(defun vector-eus2nlopt
  (vl &key
      (vector-scale 1e-3)
      (scala-scale (deg2rad 1)))
  (apply
   #'concatenate
   (cons
    float-vector
    (mapcar
     #'(lambda (v) (cond ((vectorp v) (scale vector-scale v))
			 (t (list (* scala-scale v)))))
     vl))))

(defun vector-nlopt2eus
  (v &key
     (link-list *all-links*)
     (joint-list (send-all link-list :joint))
     (vector-scale 1e+3)
     (scala-scale (rad2deg 1))
     (index 0) buf ret)
  (mapcar
   #'(lambda (j)
       (setq buf (send j :joint-angle))
       (cond
	((vectorp buf)
	 (scale vector-scale (subseq v index (setq index (+ index (length buf))))))
	(t (* scala-scale (aref v (- (incf index) 1))))))
   joint-list))

(defun-c-callable endcoords-diff ((v1 :string) (v2 :string)) :integer
  (let (buf (tmp 0))
    (mapcar
     #'(lambda (j v) (send j :joint-angle v))
     (send-all *all-links* :joint)
     (vector-nlopt2eus v1 :link-list *all-links*))
    (if *debug-view* (send *pickview* :draw-objects))
    (x::window-main-one)
    (setf (aref v2 0)
	  (apply
	   #'+
	   (mapcar
	    #'(lambda (w m c)
		(setq buf (concatenate float-vector
				       (scale 1e-3
					      (send m :difference-position c))
				       (send m :difference-rotation c)))
		(* w (norm2 buf)))
	    *weights* *move-target* *target-coords*)))
    )
  0)

(defun-c-callable jacobian ((v1 :string) (v2 :string)) :integer
  (let (dx J buf)
    (mapcar
     #'(lambda (j v) (send j :joint-angle v))
     (send-all *all-links* :joint)
     (vector-nlopt2eus v1 :link-list *all-links*))
    (if *debug-view* (send *pickview* :draw-objects))
    (x::window-main-one)
    (my-copy-matrix
     (reduce
      #'v+
      (append
       (list (scale 0 v2) (scale 0 v2))
       (mapcar
	#'(lambda (w l m c)
	    (setq J (send *robot* :calc-jacobian-from-link-list l
			  :move-target m
			  :translation-axis '(t)
			  :rotation-axis '(t)))
	    (setq dx
		  (concatenate float-vector
			       (scale 1e-3
				      (send m :difference-position c))
			       (send m :difference-rotation c)))
	    (setq buf
		  (mapcar
		   #'cons
		   l
		   (vector-nlopt2eus
		    (transform (transpose J) (scale (* -2.0 w) dx))
		    :link-list l)))
	    (vector-eus2nlopt
	     (mapcar
	      #'(lambda (al)
		  (let* ((val (cdr (assoc al buf)))
			 (val0 (send (send al :joint) :joint-angle)))
		    (cond
		     ((and val (vectorp val)) (scale 1e-3 val))
		     (val val)
		     ((vectorp val0) (scale 0 val0))
		     (t 0))))
	      *all-links*
	      )))
	*weights* *links* *move-target* *target-coords*)))
     v2))
  0)

(defun fullbody-ik-test
  (&key
   (draw?
    (cond
     ((not (and (boundp '*pickview*) *pickview*))
      (pickview :no-menu t)
      (objects *robot*))))
   (target-limb (list :rarm :larm :rleg :lleg))
   (move-target (mapcar #'(lambda (k) (send *robot* k :end-coords)) target-limb))
   (link-list
    (mapcar
     #'(lambda (k c)
	 (send *robot* :link-list (send c :parent)))
	       ;(if (find-method *robot* k) (send *robot* k :root-link))))
     target-limb
     move-target))
   (all-links (reduce #'union (cons nil link-list)))
   (weights (make-list (length move-target) :initial-element 1.0))
   (eus? nil) (reset? t) (move-base? t) (alg SLSQP)
   (x0 (progn
	 (cond
	  (reset?
	   (send *robot* :reset-pose)
	   (send *robot* :newcoords (make-coords :pos (float-vector 0 0 700)))
	   (send *pickview* :draw-objects)))
	 (cond
	  ((and (not eus?) move-base?)
	   (setq move-base? (add-6dof-joint :link-list link-list))
	   (setq link-list (cdr (assoc :6dof-links move-base?)))
	   (setq all-links (reduce #'union (cons nil link-list)))))
	 (vector-eus2nlopt (send-all (send-all all-links :joint) :joint-angle))))
   (target-coords ;*target-coords*)
    (mapcar #'(lambda (k c)
		(cond
		 ((find k '(:rleg :lleg)) (copy-object (send c :worldcoords)))
		 (t
		  (make-coords :pos (v+ (float-vector -900 (aref (send c :worldpos) 1) 0)
					(random-vector 0.0))
			       :rpy (map cons #'+
					 (random-vector 0.0))))))
	    target-limb move-target))
   ret
   )
  (setq *target-coords* target-coords)
  (setq *links* link-list)
  (setq *all-links* all-links)
  (setq *move-target* move-target)
  (setq *weights* weights)
  (format
   t
   "TIME: ~A~%"
   (bench2
    (setq
     ret
     (cond
      (eus?
       (send *robot* :fullbody-inverse-kinematics
	     *target-coords*
	     :target-centroid-pos nil
	     :debug-view *debug-view*
	     :stop 100
	     :link-list *links*
	     :revert-if-fail nil
	     :move-target *move-target*))
      (t (nlopt-optmize :alg alg
			:x0 x0
			:x-min
			(vector-eus2nlopt (send-all (send-all all-links :joint) :min-angle))
			:x-max
			(vector-eus2nlopt (send-all (send-all all-links :joint) :max-angle))
			:f 'endcoords-diff :df 'jacobian
			:g 'nop :dg 'nop
			:h 'nop :dh 'nop
			:ftol 1e-8 :xtol 1e-8 :eqthre 1e-3
			:m-x (length x0)
			:m-g 0 :m-h 0))))))
  (send *pickview* :draw-objects)
  (cond ((and (not eus?) move-base?) (funcall (cdr (assoc :del-6dof-links move-base?)))))
  ret)

;(fullbody-ik-test :target-limb '(:rarm :larm :rleg :lleg) :move-base? t :eus? nil)
;(fullbody-ik-test :target-limb '(:rarm) :move-base? nil)
